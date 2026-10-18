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

uint32_t spi_port_get_clock_hz(void)
{
  if (s_hspi == NULL)
  {
    return 0u;
  }
  /* The prescaler field is a 3-bit code: code n divides the bus clock by 2^(n+1).
   * SPI1 sits on APB2, SPI2 and SPI3 on APB1. */
  const uint32_t shift = (s_hspi->Init.BaudRatePrescaler >> SPI_CR1_BR_Pos) + 1u;
  const uint32_t pclk = (s_hspi->Instance == SPI1) ? HAL_RCC_GetPCLK2Freq()
                                                   : HAL_RCC_GetPCLK1Freq();
  return pclk >> shift;
}