#include "uart_port.h"
#include "ringbuf.h"

#define RX_BUF_SIZE 128u /* Power of two */
#define TX_TIMEOUT_MS 100u

static UART_HandleTypeDef *s_huart;
static RingBuf s_rx_ring;
static uint8_t s_rx_storage[RX_BUF_SIZE];
static uint8_t s_rx_byte; /* Target of the 1-byte IT receive */
static volatile uart_stats_t s_stats;

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
  if (len == 0u)
  {
    return STATUS_OK;
  }
  return map_hal(HAL_UART_Transmit(s_huart, (uint8_t *)data, len, TX_TIMEOUT_MS));
}

void uart_port_get_stats(uart_stats_t *out)
{
  if (out != NULL)
  {
    out->rx_overflow = s_stats.rx_overflow;
    out->rx_hw_errors = s_stats.rx_hw_errors;
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

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((s_huart == NULL) || (huart->Instance != s_huart->Instance))
  {
    return;
  }
  s_stats.rx_hw_errors++;
  /* After ORE the HAL aborts RX, so re-arm. For FE/NE/PE the receive stays
   * armed and this call just returns HAL_BUSY, which is harmless. */
  (void)HAL_UART_Receive_IT(huart, &s_rx_byte, 1u);
}