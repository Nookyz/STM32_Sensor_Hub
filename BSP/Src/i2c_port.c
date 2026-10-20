#include "i2c_port.h"
#include "perf.h"

#define I2C_TIMEOUT_MS 20u
#define PROBE_TIMEOUT_MS 5u
#define RECOVERY_CLOCKS 9u /* Enough to flush any byte a slave is sending */

/* Must match the I2C1 pinout in CubeMX */
#define I2C_SCL_PORT GPIOB
#define I2C_SCL_PIN GPIO_PIN_6
#define I2C_SDA_PORT GPIOB
#define I2C_SDA_PIN GPIO_PIN_7

static I2C_HandleTypeDef *s_hi2c;
static i2c_stats_t s_stats;
static uint32_t s_max_cycles; /* Longest successful HAL transfer */

static inline uint16_t addr8(uint8_t addr7)
{
  return (uint16_t)((uint16_t)addr7 << 1); /* HAL expects the shifted address */
}

/* Rough bit-bang delay. Accuracy does not matter: slower than 100 kHz is fine. */
static void bitbang_delay(void)
{
  for (volatile uint32_t i = 0u; i < 200u; i++)
  {
    __NOP();
  }
}

status_t i2c_port_recover(void)
{
  if (s_hi2c == NULL)
  {
    return STATUS_ERR_PARAM;
  }
  s_stats.recoveries++;

  /* Release the pins from the peripheral (HAL MspDeInit) */
  if (HAL_I2C_DeInit(s_hi2c) != HAL_OK)
  {
    return STATUS_ERR_BUS;
  }

  /* Open-drain GPIO: the module pull-ups bring the lines high */
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = I2C_SCL_PIN | I2C_SDA_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(I2C_SCL_PORT, &gpio);
  HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
  bitbang_delay();

  /* A slave holding SDA low is mid-byte: clock until it lets go */
  for (uint32_t i = 0u; i < RECOVERY_CLOCKS; i++)
  {
    if (HAL_GPIO_ReadPin(I2C_SDA_PORT, I2C_SDA_PIN) == GPIO_PIN_SET)
    {
      break;
    }
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
    bitbang_delay();
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    bitbang_delay();
  }

  /* Generate a STOP condition: SDA rises while SCL is high */
  HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET);
  bitbang_delay();
  HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
  bitbang_delay();
  HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
  bitbang_delay();

  /* Hand the pins back and re-init (HAL MspInit sets the alternate function) */
  HAL_GPIO_DeInit(I2C_SCL_PORT, I2C_SCL_PIN);
  HAL_GPIO_DeInit(I2C_SDA_PORT, I2C_SDA_PIN);
  if (HAL_I2C_Init(s_hi2c) != HAL_OK)
  {
    return STATUS_ERR_BUS;
  }

  /* Still low after the sequence: hardware fault (short, dead slave) */
  return STATUS_OK;
}

/* Convert a HAL result to status_t. Real bus failures trigger recovery,
 * NACK does not: it only means "no such device or device not ready". */
static status_t finish(HAL_StatusTypeDef hs)
{
  if (hs == HAL_OK)
  {
    return STATUS_OK;
  }

  const uint32_t err = s_hi2c->ErrorCode;
  status_t st;

  if (hs == HAL_BUSY)
  {
    st = STATUS_ERR_BUSY;
  }
  else if ((err & HAL_I2C_ERROR_AF) != 0u)
  {
    s_stats.nacks++;
    return STATUS_ERR_NACK;
  }
  else if ((hs == HAL_TIMEOUT) || ((err & HAL_I2C_ERROR_TIMEOUT) != 0u))
  {
    st = STATUS_ERR_TIMEOUT;
  }
  else
  {
    st = STATUS_ERR_BUS; /* BERR, ARLO, OVR */
  }

  s_stats.errors++;
  (void)i2c_port_recover();
  return st;
}

/* Time one HAL transfer. Only successful transfers count toward the maximum:
 * a failed one is bounded by I2C_TIMEOUT_MS plus bus recovery. */
static status_t timed_finish(HAL_StatusTypeDef hs, uint32_t t0)
{
  const uint32_t dt = perf_cycles() - t0;
  if ((hs == HAL_OK) && (dt > s_max_cycles))
  {
    s_max_cycles = dt;
  }
  return finish(hs);
}

status_t i2c_port_init(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == NULL)
  {
    return STATUS_ERR_PARAM;
  }
  s_hi2c = hi2c;
  return STATUS_OK;
}

status_t i2c_port_probe(uint8_t addr7)
{
  if ((s_hi2c == NULL) || (addr7 > 0x7Fu))
  {
    return STATUS_ERR_PARAM;
  }

  HAL_StatusTypeDef hs = HAL_I2C_IsDeviceReady(s_hi2c, addr8(addr7), 1u, PROBE_TIMEOUT_MS);
  if (hs == HAL_OK)
  {
    return STATUS_OK;
  }

  /* An empty address answers with NACK: expected during a scan, so it is
   * neither counted nor treated as a bus failure */
  if ((hs == HAL_ERROR) &&
      ((s_hi2c->ErrorCode & (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_ARLO)) == 0u))
  {
    return STATUS_ERR_NACK;
  }
  return finish(hs);
}

status_t i2c_port_write(uint8_t addr7, const uint8_t *data, uint16_t len)
{
  if ((s_hi2c == NULL) || (data == NULL) || (len == 0u) || (addr7 > 0x7Fu))
  {
    return STATUS_ERR_PARAM;
  }
  const uint32_t t0 = perf_cycles();
  const HAL_StatusTypeDef hs = HAL_I2C_Master_Transmit(s_hi2c, addr8(addr7),
                                                       (uint8_t *)data, len, I2C_TIMEOUT_MS);
  return timed_finish(hs, t0);
}

status_t i2c_port_read(uint8_t addr7, uint8_t *data, uint16_t len)
{
  if ((s_hi2c == NULL) || (data == NULL) || (len == 0u) || (addr7 > 0x7Fu))
  {
    return STATUS_ERR_PARAM;
  }
  const uint32_t t0 = perf_cycles();
  const HAL_StatusTypeDef hs = HAL_I2C_Master_Receive(s_hi2c, addr8(addr7),
                                                      data, len, I2C_TIMEOUT_MS);
  return timed_finish(hs, t0);
}

status_t i2c_port_mem_write(uint8_t addr7, uint8_t reg, const uint8_t *data, uint16_t len)
{
  if ((s_hi2c == NULL) || (data == NULL) || (len == 0u) || (addr7 > 0x7Fu))
  {
    return STATUS_ERR_PARAM;
  }
  const uint32_t t0 = perf_cycles();
  const HAL_StatusTypeDef hs = HAL_I2C_Mem_Write(s_hi2c, addr8(addr7), reg,
                                                 I2C_MEMADD_SIZE_8BIT,
                                                 (uint8_t *)data, len, I2C_TIMEOUT_MS);
  return timed_finish(hs, t0);
}

status_t i2c_port_mem_read(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len)
{
  if ((s_hi2c == NULL) || (data == NULL) || (len == 0u) || (addr7 > 0x7Fu))
  {
    return STATUS_ERR_PARAM;
  }
  const uint32_t t0 = perf_cycles();
  const HAL_StatusTypeDef hs = HAL_I2C_Mem_Read(s_hi2c, addr8(addr7), reg,
                                                I2C_MEMADD_SIZE_8BIT,
                                                data, len, I2C_TIMEOUT_MS);
  return timed_finish(hs, t0);
}

void i2c_port_get_stats(i2c_stats_t *out)
{
  if (out != NULL)
  {
    *out = s_stats;
    out->max_xfer_us = perf_cycles_to_us(s_max_cycles);
  }
}

void i2c_port_reset_max(void)
{
  s_max_cycles = 0u;
}
