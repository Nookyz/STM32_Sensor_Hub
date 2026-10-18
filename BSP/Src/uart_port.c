#include "uart_port.h"
#include "ringbuf.h"

#include <string.h>

#define RX_BUF_SIZE 128u /* Power of two */
#define TX_BUF_SIZE UART_PORT_TX_BUF_SIZE
#define TX_MASK (TX_BUF_SIZE - 1u)
#define TX_WAIT_TIMEOUT_MS 100u /* Longest wait for free space in one write */

/* Indices are 16 bit, and one slot always stays free */
_Static_assert((TX_BUF_SIZE & (TX_BUF_SIZE - 1u)) == 0u, "TX buffer size must be a power of two");
_Static_assert(TX_BUF_SIZE <= 32768u, "TX buffer too large for 16-bit indices");

static UART_HandleTypeDef *s_huart;
static volatile uart_stats_t s_stats;

/* ---- RX: one byte per interrupt into a ring buffer (DMA comes in step 5b) ---- */
static RingBuf s_rx_ring;
static uint8_t s_rx_storage[RX_BUF_SIZE];
static uint8_t s_rx_byte; /* Target of the 1-byte IT receive */

/* ---- TX: the main loop owns head, the DMA-complete interrupt owns tail ---- */
static uint8_t s_tx_buf[TX_BUF_SIZE];
static volatile uint16_t s_tx_head;    /* Next byte to be written */
static volatile uint16_t s_tx_tail;    /* Next byte to be sent */
static volatile uint16_t s_tx_dma_len; /* Size of the chunk the DMA is sending */
static volatile bool s_tx_busy;        /* A DMA transfer is in flight */

static status_t map_hal(HAL_StatusTypeDef hs)
{
  switch (hs)
  {
  case HAL_OK:
    return STATUS_OK;
  case HAL_TIMEOUT:
    return STATUS_ERR_TIMEOUT;
  case HAL_BUSY:
    return STATUS_ERR_BUSY;
  default:
    return STATUS_ERR_BUS;
  }
}

static inline uint32_t irq_save(void)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static inline void irq_restore(uint32_t primask)
{
  __set_PRIMASK(primask);
}

/* One slot stays unused so that head == tail always means "empty" */
static inline uint16_t tx_free(void)
{
  return (uint16_t)((s_tx_tail - s_tx_head - 1u) & TX_MASK);
}

/* Start the next DMA chunk if the channel is idle and data is waiting.
 * The caller must have interrupts masked, or be the DMA-complete ISR. */
static void tx_kick(void)
{
  while (!s_tx_busy)
  {
    const uint16_t head = s_tx_head;
    const uint16_t tail = s_tx_tail;
    if (head == tail)
    {
      return;
    }

    /* One contiguous run: up to the write index, or up to the buffer end */
    const uint16_t len = (head > tail) ? (uint16_t)(head - tail)
                                       : (uint16_t)(TX_BUF_SIZE - tail);

    /* Set busy first: the completion ISR must find a consistent state */
    s_tx_dma_len = len;
    s_tx_busy = true;
    if (HAL_UART_Transmit_DMA(s_huart, &s_tx_buf[tail], len) != HAL_OK)
    {
      /* Could not start: discard the chunk so the queue never stalls */
      s_tx_busy = false;
      s_tx_tail = (uint16_t)((tail + len) & TX_MASK);
      s_stats.tx_errors++;
    }
  }
}

status_t uart_port_init(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return STATUS_ERR_PARAM;
  }
  status_t st = ringbuf_init(&s_rx_ring, s_rx_storage, RX_BUF_SIZE);
  if (st != STATUS_OK)
  {
    return st;
  }

  s_tx_head = 0u;
  s_tx_tail = 0u;
  s_tx_dma_len = 0u;
  s_tx_busy = false;

  s_huart = huart;
  return map_hal(HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1u));
}

bool uart_port_read(uint8_t *byte)
{
  return (byte != NULL) && ringbuf_get(&s_rx_ring, byte);
}

status_t uart_port_write(const uint8_t *data, uint16_t len)
{
  if ((s_huart == NULL) || (data == NULL))
  {
    return STATUS_ERR_PARAM;
  }

  const uint32_t t0 = HAL_GetTick();
  bool waited = false;

  while (len > 0u)
  {
    const uint16_t room = tx_free();

    if (room == 0u)
    {
      if (!waited)
      {
        s_stats.tx_waits++;
        waited = true;
      }
      if ((HAL_GetTick() - t0) >= TX_WAIT_TIMEOUT_MS)
      {
        s_stats.tx_dropped += len;
        return STATUS_ERR_TIMEOUT;
      }
      /* Safety net: make sure the DMA is running while we wait */
      const uint32_t pm = irq_save();
      tx_kick();
      irq_restore(pm);
      continue;
    }

    /* Copy outside the critical section: the area between head and tail
     * belongs to this function until the new head is published below */
    const uint16_t n = (len < room) ? len : room;
    const uint16_t head = s_tx_head;
    const uint16_t first = (n < (TX_BUF_SIZE - head)) ? n : (uint16_t)(TX_BUF_SIZE - head);
    memcpy(&s_tx_buf[head], data, first);
    memcpy(&s_tx_buf[0], data + first, (size_t)(n - first));

    /* Publish the data and start the DMA as one step, so the completion
     * ISR cannot declare the queue empty in between */
    const uint32_t pm = irq_save();
    s_tx_head = (uint16_t)((head + n) & TX_MASK);
    tx_kick();
    const uint16_t fill = (uint16_t)((s_tx_head - s_tx_tail) & TX_MASK);
    irq_restore(pm);

    if (fill > s_stats.tx_peak)
    {
      s_stats.tx_peak = fill;
    }

    data += n;
    len = (uint16_t)(len - n);
  }
  return STATUS_OK;
}

void uart_port_get_stats(uart_stats_t *out)
{
  if (out != NULL)
  {
    out->rx_overflow = s_stats.rx_overflow;
    out->rx_hw_errors = s_stats.rx_hw_errors;
    out->tx_waits = s_stats.tx_waits;
    out->tx_dropped = s_stats.tx_dropped;
    out->tx_errors = s_stats.tx_errors;
    out->tx_peak = s_stats.tx_peak;
  }
}

/* ---- HAL callbacks (run in ISR context, keep them short) ---- */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((s_huart == NULL) || (huart->Instance != s_huart->Instance))
  {
    return;
  }
  if (!ringbuf_put(&s_rx_ring, s_rx_byte))
  {
    s_stats.rx_overflow++;
  }
  /* Re-arm immediately to avoid losing the next byte */
  (void)HAL_UART_Receive_IT(huart, &s_rx_byte, 1u);
}

/* Called when the last byte of a DMA chunk has left the shift register */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((s_huart == NULL) || (huart->Instance != s_huart->Instance))
  {
    return;
  }
  s_tx_tail = (uint16_t)((s_tx_tail + s_tx_dma_len) & TX_MASK);
  s_tx_busy = false;
  tx_kick(); /* The HAL state is already READY here, so a new transfer can start */
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((s_huart == NULL) || (huart->Instance != s_huart->Instance))
  {
    return;
  }

  const uint32_t err = huart->ErrorCode;

  if ((err & (HAL_UART_ERROR_PE | HAL_UART_ERROR_NE |
              HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE)) != 0u)
  {
    s_stats.rx_hw_errors++;
  }

  if (((err & HAL_UART_ERROR_DMA) != 0u) && s_tx_busy)
  {
    /* The DMA transfer died and the HAL has already stopped it:
     * drop that chunk and carry on with the rest */
    s_tx_tail = (uint16_t)((s_tx_tail + s_tx_dma_len) & TX_MASK);
    s_tx_busy = false;
    s_stats.tx_errors++;
    tx_kick();
  }

  /* After ORE the HAL aborts RX, so re-arm. In the other cases the receive
   * stays armed and this call returns HAL_BUSY, which is harmless. */
  (void)HAL_UART_Receive_IT(huart, &s_rx_byte, 1u);
}