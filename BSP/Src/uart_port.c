#include "uart_port.h"
#include "ringbuf.h"

#include <string.h>

#define RX_DMA_SIZE 128u  /* Circular DMA buffer: half/full events every 64 bytes */
#define RX_RING_SIZE 512u /* Power of two. Holds data while the main loop is busy */
#define TX_BUF_SIZE UART_PORT_TX_BUF_SIZE
#define TX_MASK (TX_BUF_SIZE - 1u)
#define TX_WAIT_TIMEOUT_MS 100u /* Longest wait for free space in one write */

/* Indices are 16 bit, and one slot always stays free */
_Static_assert((TX_BUF_SIZE & (TX_BUF_SIZE - 1u)) == 0u, "TX buffer size must be a power of two");
_Static_assert(TX_BUF_SIZE <= 32768u, "TX buffer too large for 16-bit indices");
_Static_assert((RX_RING_SIZE & (RX_RING_SIZE - 1u)) == 0u, "RX ring size must be a power of two");

static UART_HandleTypeDef *s_huart;
static volatile uart_stats_t s_stats;

/* ---- RX: the DMA fills s_rx_dma in a circle, the ISR copies new bytes into a ring ---- */
static RingBuf s_rx_ring;
static uint8_t s_rx_storage[RX_RING_SIZE];
static volatile uint8_t s_rx_dma[RX_DMA_SIZE]; /* Written by the DMA */
static uint16_t s_rx_pos;                      /* Next DMA byte to copy (ISR only) */

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

/* ---- RX ---- */

static HAL_StatusTypeDef rx_start(void)
{
  return HAL_UARTEx_ReceiveToIdle_DMA(s_huart, (uint8_t *)s_rx_dma, RX_DMA_SIZE);
}

/* Copy everything between our read position and `end`, the DMA write position.
 * `end` may equal RX_DMA_SIZE, which means "up to the end of the buffer".
 * Runs in ISR context only. A lap by the DMA without an interrupt in between
 * cannot be detected, but half and full events come every 64 bytes (5.5 ms),
 * far more often than any interrupt delay in this firmware. */
static void rx_drain(uint16_t end)
{
  const uint16_t stop = (end >= RX_DMA_SIZE) ? 0u : end;

  while (s_rx_pos != stop)
  {
    s_stats.rx_bytes++;
    if (!ringbuf_put(&s_rx_ring, s_rx_dma[s_rx_pos]))
    {
      s_stats.rx_overflow++;
    }
    s_rx_pos++;
    if (s_rx_pos >= RX_DMA_SIZE)
    {
      s_rx_pos = 0u;
    }
  }
}

/* ---- TX ---- */

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

/* ---- Public API ---- */

status_t uart_port_init(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return STATUS_ERR_PARAM;
  }
  /* A missing DMA link would crash inside the HAL, so refuse it here */
  if ((huart->hdmatx == NULL) || (huart->hdmarx == NULL))
  {
    return STATUS_ERR_PARAM;
  }
  status_t st = ringbuf_init(&s_rx_ring, s_rx_storage, RX_RING_SIZE);
  if (st != STATUS_OK)
  {
    return st;
  }

  s_tx_head = 0u;
  s_tx_tail = 0u;
  s_tx_dma_len = 0u;
  s_tx_busy = false;
  s_rx_pos = 0u;

  s_huart = huart;
  return map_hal(rx_start());
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
    out->rx_bytes = s_stats.rx_bytes;
    out->rx_events = s_stats.rx_events;
    out->rx_overflow = s_stats.rx_overflow;
    out->rx_hw_errors = s_stats.rx_hw_errors;
    out->tx_waits = s_stats.tx_waits;
    out->tx_dropped = s_stats.tx_dropped;
    out->tx_errors = s_stats.tx_errors;
    out->tx_peak = s_stats.tx_peak;
    out->rx_restarts = s_stats.rx_restarts;
  }
}

/* ---- HAL callbacks (run in ISR context, keep them short) ---- */

/* Called on IDLE, on half and on full DMA buffer. Size is the DMA write
 * position (RX_DMA_SIZE for the full-buffer event). */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if ((s_huart == NULL) || (huart->Instance != s_huart->Instance))
  {
    return;
  }
  s_stats.rx_events++;
  rx_drain(Size);

  /* Circular DMA keeps the reception running. If the HAL has ended it (DMA in
   * Normal mode), start a new one so the input never goes deaf. Bytes that
   * arrive during the restart gap are lost, so Circular stays the right mode. */
  if (huart->RxState == HAL_UART_STATE_READY)
  {
    s_stats.rx_restarts++;
    s_rx_pos = 0u;
    (void)rx_start();
  }
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

  /* A TX DMA failure: the HAL has already stopped the transfer (state READY)
   * while we still think it is running. Drop that chunk and carry on. */
  if (((err & HAL_UART_ERROR_DMA) != 0u) && s_tx_busy &&
      (huart->gState == HAL_UART_STATE_READY))
  {
    s_tx_tail = (uint16_t)((s_tx_tail + s_tx_dma_len) & TX_MASK);
    s_tx_busy = false;
    s_stats.tx_errors++;
    tx_kick();
  }

  /* The HAL aborts a DMA receive on any UART error. RX state READY means it
   * has stopped: salvage bytes that already sit in the DMA buffer (the abort
   * leaves the DMA counter untouched), then restart from index 0. */
  if (huart->RxState == HAL_UART_STATE_READY)
  {
    const uint16_t left = (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);
    rx_drain((uint16_t)(RX_DMA_SIZE - left));
    s_rx_pos = 0u;
    (void)rx_start();
  }
}