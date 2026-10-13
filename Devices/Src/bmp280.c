#include "bmp280.h"
#include "i2c_port.h"
#include "main.h"

/* Registers */
#define BMP280_REG_CALIB 0x88u /* 24 bytes of trim data */
#define BMP280_REG_ID 0xD0u
#define BMP280_REG_RESET 0xE0u
#define BMP280_REG_STATUS 0xF3u
#define BMP280_REG_CTRL_MEAS 0xF4u
#define BMP280_REG_CONFIG 0xF5u
#define BMP280_REG_DATA 0xF7u /* press[3] then temp[3] */

#define BMP280_CHIP_ID 0x58u /* BME280 would report 0x60 */
#define BMP280_RESET_CMD 0xB6u
#define BMP280_ST_MEASURING 0x08u
#define BMP280_ST_IM_UPDATE 0x01u

#define BMP280_CALIB_LEN 24u
#define BMP280_DATA_LEN 6u

/* ctrl_meas: osrs_t x1 (001), osrs_p x4 (011), mode forced (01).
 * This is the "standard resolution" setting from the datasheet. */
#define BMP280_CTRL_FORCED 0x2Du
#define BMP280_CONFIG_NOFILT 0x00u /* IIR filter off, standby unused in forced mode */

#define BMP280_RESET_WAIT_MS 3u /* Datasheet: startup time 2 ms */
#define BMP280_NVM_TIMEOUT_MS 10u
#define BMP280_MEASURE_MS 15u /* Max conversion time for these settings is ~13 ms */
#define BMP280_RECHECK_MS 2u
#define BMP280_TIMEOUT_MS 50u

#define BMP280_ADC_SKIPPED 0x80000 /* Reset value: measurement was skipped */

static inline uint16_t u16le(const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline int16_t s16le(const uint8_t *p)
{
  return (int16_t)u16le(p);
}

static void parse_calib(bmp280_calib_t *c, const uint8_t *b)
{
  c->T1 = u16le(&b[0]);
  c->T2 = s16le(&b[2]);
  c->T3 = s16le(&b[4]);
  c->P1 = u16le(&b[6]);
  c->P2 = s16le(&b[8]);
  c->P3 = s16le(&b[10]);
  c->P4 = s16le(&b[12]);
  c->P5 = s16le(&b[14]);
  c->P6 = s16le(&b[16]);
  c->P7 = s16le(&b[18]);
  c->P8 = s16le(&b[20]);
  c->P9 = s16le(&b[22]);
}

/* Bosch reference compensation, 32-bit. Returns 0.01 degC and updates
 * t_fine, which the pressure formula needs. */
static int32_t compensate_temp(const bmp280_calib_t *c, int32_t adc_t, int32_t *t_fine)
{
  const int32_t var1 = (((adc_t >> 3) - ((int32_t)c->T1 << 1)) * (int32_t)c->T2) >> 11;
  const int32_t d = (adc_t >> 4) - (int32_t)c->T1;
  const int32_t var2 = (((d * d) >> 12) * (int32_t)c->T3) >> 14;

  *t_fine = var1 + var2;
  return (*t_fine * 5 + 128) >> 8;
}

/* Bosch reference compensation, 64-bit. Returns Pascals, or 0 if the
 * calibration data would cause a division by zero. */
static uint32_t compensate_press(const bmp280_calib_t *c, int32_t adc_p, int32_t t_fine)
{
  int64_t var1 = (int64_t)t_fine - 128000;
  int64_t var2 = var1 * var1 * (int64_t)c->P6;
  var2 += (var1 * (int64_t)c->P5) << 17;
  var2 += (int64_t)c->P4 << 35;
  var1 = ((var1 * var1 * (int64_t)c->P3) >> 8) + ((var1 * (int64_t)c->P2) << 12);
  var1 = (((int64_t)1 << 47) + var1) * (int64_t)c->P1 >> 33;

  if (var1 == 0)
  {
    return 0u;
  }

  int64_t p = 1048576 - adc_p;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = ((int64_t)c->P9 * (p >> 13) * (p >> 13)) >> 25;
  var2 = ((int64_t)c->P8 * p) >> 19;
  p = ((p + var1 + var2) >> 8) + ((int64_t)c->P7 << 4);

  /* p is Pascals in Q24.8 format: round and drop the fraction */
  return (uint32_t)((p + 128) >> 8);
}

status_t bmp280_init(bmp280_t *dev, uint8_t addr7)
{
  if ((dev == NULL) || (addr7 > 0x7Fu))
  {
    return STATUS_ERR_PARAM;
  }
  dev->addr = addr7;
  dev->state = BMP280_IDLE;

  uint8_t id;
  status_t st = i2c_port_mem_read(addr7, BMP280_REG_ID, &id, 1u);
  if (st != STATUS_OK)
  {
    return st;
  }
  if (id != BMP280_CHIP_ID)
  {
    return STATUS_ERR_ID;
  }

  /* Soft reset puts the sensor in a known state regardless of what a
   * previous firmware run left behind */
  const uint8_t reset = BMP280_RESET_CMD;
  st = i2c_port_mem_write(addr7, BMP280_REG_RESET, &reset, 1u);
  if (st != STATUS_OK)
  {
    return st;
  }
  HAL_Delay(BMP280_RESET_WAIT_MS);

  /* Wait until the NVM copy into the registers is finished */
  const uint32_t t0 = HAL_GetTick();
  uint8_t status;
  do
  {
    st = i2c_port_mem_read(addr7, BMP280_REG_STATUS, &status, 1u);
    if (st != STATUS_OK)
    {
      return st;
    }
    if ((status & BMP280_ST_IM_UPDATE) == 0u)
    {
      break;
    }
  } while ((HAL_GetTick() - t0) < BMP280_NVM_TIMEOUT_MS);

  if ((status & BMP280_ST_IM_UPDATE) != 0u)
  {
    return STATUS_ERR_TIMEOUT;
  }

  uint8_t buf[BMP280_CALIB_LEN];
  st = i2c_port_mem_read(addr7, BMP280_REG_CALIB, buf, BMP280_CALIB_LEN);
  if (st != STATUS_OK)
  {
    return st;
  }
  parse_calib(&dev->calib, buf);

  /* All-zero or all-one trim words mean a floating or dead bus */
  if ((dev->calib.T1 == 0u) || (dev->calib.T1 == 0xFFFFu) ||
      (dev->calib.P1 == 0u) || (dev->calib.P1 == 0xFFFFu))
  {
    return STATUS_ERR_ID;
  }

  const uint8_t cfg = BMP280_CONFIG_NOFILT;
  return i2c_port_mem_write(addr7, BMP280_REG_CONFIG, &cfg, 1u);
}

status_t bmp280_start(bmp280_t *dev)
{
  if (dev == NULL)
  {
    return STATUS_ERR_PARAM;
  }
  if (dev->state != BMP280_IDLE)
  {
    return STATUS_ERR_BUSY;
  }

  const uint8_t ctrl = BMP280_CTRL_FORCED;
  status_t st = i2c_port_mem_write(dev->addr, BMP280_REG_CTRL_MEAS, &ctrl, 1u);
  if (st != STATUS_OK)
  {
    return st;
  }

  dev->t_start = HAL_GetTick();
  dev->t_next = dev->t_start + BMP280_MEASURE_MS;
  dev->state = BMP280_MEASURING;
  return STATUS_OK;
}

status_t bmp280_poll(bmp280_t *dev, bmp280_sample_t *out, bool *ready)
{
  if ((dev == NULL) || (out == NULL) || (ready == NULL))
  {
    return STATUS_ERR_PARAM;
  }
  *ready = false;

  if (dev->state != BMP280_MEASURING)
  {
    return STATUS_OK;
  }

  const uint32_t now = HAL_GetTick();
  if ((int32_t)(now - dev->t_next) < 0)
  {
    return STATUS_OK; /* Conversion not finished yet */
  }

  uint8_t status;
  status_t st = i2c_port_mem_read(dev->addr, BMP280_REG_STATUS, &status, 1u);
  if (st != STATUS_OK)
  {
    dev->state = BMP280_IDLE;
    return st;
  }

  if ((status & BMP280_ST_MEASURING) != 0u)
  {
    if ((now - dev->t_start) > BMP280_TIMEOUT_MS)
    {
      dev->state = BMP280_IDLE;
      return STATUS_ERR_TIMEOUT;
    }
    dev->t_next = now + BMP280_RECHECK_MS;
    return STATUS_OK;
  }

  /* Burst read: the sensor latches all six bytes together */
  uint8_t d[BMP280_DATA_LEN];
  dev->state = BMP280_IDLE;
  st = i2c_port_mem_read(dev->addr, BMP280_REG_DATA, d, BMP280_DATA_LEN);
  if (st != STATUS_OK)
  {
    return st;
  }

  /* 20-bit values, MSB first, the lowest nibble of the third byte is unused */
  const int32_t adc_p = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | ((int32_t)d[2] >> 4);
  const int32_t adc_t = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | ((int32_t)d[5] >> 4);

  /* No valid conversion landed in the data registers */
  if ((adc_p == BMP280_ADC_SKIPPED) || (adc_t == BMP280_ADC_SKIPPED))
  {
    return STATUS_ERR_BUS;
  }

  int32_t t_fine;
  out->temp_c100 = compensate_temp(&dev->calib, adc_t, &t_fine);
  out->press_pa = compensate_press(&dev->calib, adc_p, t_fine);
  if (out->press_pa == 0u)
  {
    return STATUS_ERR_ID; /* Bad calibration data */
  }

  *ready = true;
  return STATUS_OK;
}