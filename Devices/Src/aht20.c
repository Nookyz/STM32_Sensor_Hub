#include "aht20.h"
#include "i2c_port.h"
#include "main.h"

#define AHT20_POWERUP_MS 40u  /* Datasheet: wait after power-up */
#define AHT20_CAL_MS 10u      /* Datasheet: wait after the init command */
#define AHT20_MEASURE_MS 80u  /* Typical conversion time */
#define AHT20_RECHECK_MS 5u   /* Re-read period while still busy */
#define AHT20_TIMEOUT_MS 150u /* Give up if busy for this long */

#define AHT20_STATUS_BUSY 0x80u
#define AHT20_STATUS_CAL 0x08u
#define AHT20_FRAME_LEN 7u /* status + 5 data bytes + CRC */

static const uint8_t k_cmd_init[3] = {0xBEu, 0x08u, 0x00u};
static const uint8_t k_cmd_measure[3] = {0xACu, 0x33u, 0x00u};

/* CRC-8, polynomial 0x31, init 0xFF (as in the AHT20 datasheet) */
static uint8_t crc8(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0xFFu;
  while (len-- > 0u)
  {
    crc ^= *data++;
    for (uint8_t i = 0u; i < 8u; i++)
    {
      crc = ((crc & 0x80u) != 0u) ? (uint8_t)((crc << 1) ^ 0x31u)
                                  : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

status_t aht20_init(aht20_t *dev)
{
  if (dev == NULL)
  {
    return STATUS_ERR_PARAM;
  }
  dev->state = AHT20_IDLE;

  /* HAL tick starts near power-up, so wait only for the remainder */
  const uint32_t now = HAL_GetTick();
  if (now < AHT20_POWERUP_MS)
  {
    HAL_Delay(AHT20_POWERUP_MS - now);
  }

  uint8_t status;
  status_t st = i2c_port_read(AHT20_I2C_ADDR, &status, 1u);
  if (st != STATUS_OK)
  {
    return st;
  }

  if ((status & AHT20_STATUS_CAL) == 0u)
  {
    st = i2c_port_write(AHT20_I2C_ADDR, k_cmd_init, sizeof(k_cmd_init));
    if (st != STATUS_OK)
    {
      return st;
    }
    HAL_Delay(AHT20_CAL_MS);

    st = i2c_port_read(AHT20_I2C_ADDR, &status, 1u);
    if (st != STATUS_OK)
    {
      return st;
    }
    /* The AHT20 has no ID register. A calibration flag that stays clear
     * means a faulty or different device at this address. */
    if ((status & AHT20_STATUS_CAL) == 0u)
    {
      return STATUS_ERR_ID;
    }
  }
  return STATUS_OK;
}

status_t aht20_start(aht20_t *dev)
{
  if (dev == NULL)
  {
    return STATUS_ERR_PARAM;
  }
  if (dev->state != AHT20_IDLE)
  {
    return STATUS_ERR_BUSY;
  }

  status_t st = i2c_port_write(AHT20_I2C_ADDR, k_cmd_measure, sizeof(k_cmd_measure));
  if (st != STATUS_OK)
  {
    return st;
  }

  dev->t_start = HAL_GetTick();
  dev->t_next = dev->t_start + AHT20_MEASURE_MS;
  dev->state = AHT20_MEASURING;
  return STATUS_OK;
}

status_t aht20_poll(aht20_t *dev, aht20_sample_t *out, bool *ready)
{
  if ((dev == NULL) || (out == NULL) || (ready == NULL))
  {
    return STATUS_ERR_PARAM;
  }
  *ready = false;

  if (dev->state != AHT20_MEASURING)
  {
    return STATUS_OK;
  }

  const uint32_t now = HAL_GetTick();
  if ((int32_t)(now - dev->t_next) < 0)
  {
    return STATUS_OK; /* Conversion not finished yet */
  }

  uint8_t buf[AHT20_FRAME_LEN];
  status_t st = i2c_port_read(AHT20_I2C_ADDR, buf, AHT20_FRAME_LEN);
  if (st != STATUS_OK)
  {
    dev->state = AHT20_IDLE;
    return st;
  }

  if ((buf[0] & AHT20_STATUS_BUSY) != 0u)
  {
    if ((now - dev->t_start) > AHT20_TIMEOUT_MS)
    {
      dev->state = AHT20_IDLE;
      return STATUS_ERR_TIMEOUT;
    }
    dev->t_next = now + AHT20_RECHECK_MS; /* Do not hammer the bus */
    return STATUS_OK;
  }

  dev->state = AHT20_IDLE;
  if (crc8(buf, AHT20_FRAME_LEN - 1u) != buf[AHT20_FRAME_LEN - 1u])
  {
    return STATUS_ERR_CRC;
  }

  /* 20-bit humidity, then 20-bit temperature, packed MSB first */
  const uint32_t raw_h = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) |
                         ((uint32_t)buf[3] >> 4);
  const uint32_t raw_t = (((uint32_t)buf[3] & 0x0Fu) << 16) |
                         ((uint32_t)buf[4] << 8) | (uint32_t)buf[5];

  /* hum  = raw * 100 %  / 2^20 -> in 0.01 %: raw * 10000 / 2^20 = raw * 625 / 2^16
   * temp = raw * 200 C / 2^20 - 50 -> in 0.01 C: raw * 625 / 2^15 - 5000
   * The factor 625 keeps everything in 32 bits (max raw * 625 < 2^32). */
  out->hum_c100 = (raw_h * 625u + (1u << 15)) >> 16;
  out->temp_c100 = (int32_t)((raw_t * 625u + (1u << 14)) >> 15) - 5000;

  *ready = true;
  return STATUS_OK;
}