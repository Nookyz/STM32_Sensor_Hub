#ifndef UART_PORT_H
#define UART_PORT_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "status.h"

typedef struct
{
  uint32_t rx_overflow;  /* Bytes dropped: ring buffer was full */
  uint32_t rx_hw_errors; /* ORE / FE / NE / PE events */
} uart_stats_t;

/** Bind the driver to a HAL handle and start interrupt-driven RX. */
status_t uart_port_init(UART_HandleTypeDef *huart);

/** Non-blocking: fetch one received byte. Returns false if none. */
bool uart_port_read(uint8_t *byte);

/** Blocking TX with timeout (replaced by DMA later). */
status_t uart_port_write(const uint8_t *data, uint16_t len);

void uart_port_get_stats(uart_stats_t *out);

#endif /* UART_PORT_H */