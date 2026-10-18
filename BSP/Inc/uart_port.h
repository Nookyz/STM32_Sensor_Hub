#ifndef UART_PORT_H
#define UART_PORT_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "status.h"

/* TX ring size in bytes. Must be a power of two (checked at compile time).
 * 2048 bytes is about 180 ms of output at 115200 baud. */
#define UART_PORT_TX_BUF_SIZE 2048u

typedef struct
{
  uint32_t rx_overflow;  /* Bytes dropped: RX ring buffer was full */
  uint32_t rx_hw_errors; /* ORE / FE / NE / PE events */
  uint32_t tx_waits;     /* write() calls that had to wait for free space */
  uint32_t tx_dropped;   /* Bytes lost: no space within the wait timeout */
  uint32_t tx_errors;    /* DMA start or transfer failures (chunk discarded) */
  uint32_t tx_peak;      /* Highest TX buffer fill seen, in bytes */
} uart_stats_t;

/** Bind the driver to a HAL handle and start interrupt-driven RX.
 *  The USART2_TX DMA stream must be configured in CubeMX. */
status_t uart_port_init(UART_HandleTypeDef *huart);

/** Non-blocking: fetch one received byte. Returns false if none. */
bool uart_port_read(uint8_t *byte);

/** Queue data for transmission and return. Blocks only while the TX buffer
 *  is full, for at most 100 ms. STATUS_ERR_TIMEOUT means some bytes were
 *  dropped (counted in tx_dropped). Main loop only, not for ISRs. */
status_t uart_port_write(const uint8_t *data, uint16_t len);

void uart_port_get_stats(uart_stats_t *out);

#endif /* UART_PORT_H */