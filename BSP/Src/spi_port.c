#include "spi_port.h"

#define SPI_TIMEOUT_MS 50u /* Generous: 4 KB at 5 MHz takes ~7 ms */

static SPI_HandleTypeDef *s_hspi;
static spi_stats_t s_stats;

static status_t finish(HAL_StatusTypeDef hs)
{
  switch (hs)
  {
  case HAL_OK:
    return STATUS_OK;
  case HAL_BUSY:
    s_stats.errors++;
    return STATUS_ERR_BUSY;
  case HAL_TIMEOUT:
    s_stats.errors++;
    return STATUS_ERR_TIMEOUT;
  default:
    s_stats.errors++;
    return STATUS_ERR_BUS;
  }
}

status_t spi_port_init(SPI_HandleTypeDef *hspi)
{
  if (hspi == NULL)
  {
    return STATUS_ERR_PARAM;
  }
  s_hspi = hspi;
  return STATUS_OK;
}

status_t spi_port_write(const uint8_t *data, uint16_t len)
{
  if ((s_hspi == NULL) || (data == NULL) || (len == 0u))
  {
    return STATUS_ERR_PARAM;
  }
  return finish(HAL_SPI_Transmit(s_hspi, (uint8_t *)data, len, SPI_TIMEOUT_MS));
}

status_t spi_port_read(uint8_t *data, uint16_t len)
{
  if ((s_hspi == NULL) || (data == NULL) || (len == 0u))
  {
    return STATUS_ERR_PARAM;
  }
  return finish(HAL_SPI_Receive(s_hspi, data, len, SPI_TIMEOUT_MS));
}

void spi_port_get_stats(spi_stats_t *out)
{
  if (out != NULL)
  {
    *out = s_stats;
  }
}