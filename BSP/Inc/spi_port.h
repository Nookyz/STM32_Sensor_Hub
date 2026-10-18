#ifndef SPI_PORT_H
#define SPI_PORT_H

#include <stdint.h>
#include "main.h"
#include "status.h"

typedef struct
{
  uint32_t errors; /* Timeouts and bus errors */
} spi_stats_t;

/** Bind the driver to a HAL handle. Call after MX_SPI1_Init(). */
status_t spi_port_init(SPI_HandleTypeDef *hspi);

/** Blocking transmit with timeout. Chip select is handled by the caller. */
status_t spi_port_write(const uint8_t *data, uint16_t len);

/** Blocking receive with timeout. The buffer's old contents are clocked out
 *  on MOSI (HAL behaviour), so slaves must ignore input in this phase. */
status_t spi_port_read(uint8_t *data, uint16_t len);

/** Actual SCK frequency in Hz, 0 before spi_port_init(). */
uint32_t spi_port_get_clock_hz(void);

void spi_port_get_stats(spi_stats_t *out);

#endif /* SPI_PORT_H */