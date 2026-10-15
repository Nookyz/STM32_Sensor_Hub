#include "w25q128.h"
#include "spi_port.h"
#include "main.h"

#include <stdbool.h>

#define CMD_RELEASE_PD 0xABu /* Wake from deep power-down, harmless otherwise */
#define CMD_JEDEC_ID 0x9Fu
#define CMD_READ_STATUS1 0x05u
#define CMD_READ_DATA 0x03u /* 24-bit address, any length, up to ~50 MHz */

#define SR1_BUSY 0x01u

#define WINBOND_ID 0xEFu
#define CAP_CODE_MIN 0x11u /* 128 KB */
#define CAP_CODE_MAX 0x18u /* 16 MB: the limit of 3-byte addressing */

#define POWERUP_MS 10u /* tVSL: Vcc stable to first command */

static uint32_t s_capacity;

static inline void cs_low(void)
{
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
}

static inline void cs_high(void)
{
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

/* One chip-select frame: command bytes out, then optional bytes in.
 * CS is released on every path, so an error never leaves the flash selected. */
static status_t transaction(const uint8_t *cmd, uint8_t cmd_len,
                            uint8_t *rx, uint16_t rx_len)
{
  cs_low();
  status_t st = spi_port_write(cmd, cmd_len);
  if ((st == STATUS_OK) && (rx_len > 0u))
  {
    st = spi_port_read(rx, rx_len);
  }
  cs_high();
  return st;
}

status_t w25q_init(w25q_info_t *info)
{
  s_capacity = 0u;
  cs_high(); /* Idle level, regardless of the CubeMX default */

  /* Give the chip time to settle after power-up (tick counts from reset) */
  const uint32_t now = HAL_GetTick();
  if (now < POWERUP_MS)
  {
    HAL_Delay(POWERUP_MS - now);
  }

  const uint8_t wake = CMD_RELEASE_PD;
  status_t st = transaction(&wake, 1u, NULL, 0u);
  if (st != STATUS_OK)
  {
    return st;
  }
  HAL_Delay(1u); /* tRES1 is 3 us, the shortest HAL delay is plenty */

  const uint8_t cmd = CMD_JEDEC_ID;
  uint8_t id[3];
  st = transaction(&cmd, 1u, id, sizeof(id));
  if (st != STATUS_OK)
  {
    return st;
  }

  const bool valid = (id[0] == WINBOND_ID) &&
                     (id[2] >= CAP_CODE_MIN) && (id[2] <= CAP_CODE_MAX);
  if (valid)
  {
    s_capacity = 1UL << id[2]; /* The capacity code is log2 of the size in bytes */
  }

  if (info != NULL)
  {
    info->manufacturer = id[0];
    info->mem_type = id[1];
    info->capacity_code = id[2];
    info->capacity_bytes = s_capacity;
  }
  return valid ? STATUS_OK : STATUS_ERR_ID;
}

uint32_t w25q_capacity(void)
{
  return s_capacity;
}

status_t w25q_read_status(uint8_t *sr1)
{
  if (sr1 == NULL)
  {
    return STATUS_ERR_PARAM;
  }
  const uint8_t cmd = CMD_READ_STATUS1;
  return transaction(&cmd, 1u, sr1, 1u);
}

status_t w25q_wait_ready(uint32_t timeout_ms)
{
  const uint32_t t0 = HAL_GetTick();

  for (;;)
  {
    uint8_t sr;
    status_t st = w25q_read_status(&sr);
    if (st != STATUS_OK)
    {
      return st;
    }
    if ((sr & SR1_BUSY) == 0u)
    {
      return STATUS_OK;
    }
    /* An absent chip floats MISO high (0xFF), which also lands here */
    if ((HAL_GetTick() - t0) >= timeout_ms)
    {
      return STATUS_ERR_TIMEOUT;
    }
  }
}

status_t w25q_read(uint32_t addr, uint8_t *buf, uint16_t len)
{
  if ((buf == NULL) || (len == 0u))
  {
    return STATUS_ERR_PARAM;
  }
  if (s_capacity == 0u)
  {
    return STATUS_ERR_ID;
  }
  /* Written as a subtraction so that addr + len can never overflow */
  if ((addr >= s_capacity) || ((uint32_t)len > (s_capacity - addr)))
  {
    return STATUS_ERR_PARAM;
  }

  /* While an erase or program runs, the chip ignores read commands and
   * the data would be garbage, so refuse instead of returning it */
  uint8_t sr;
  status_t st = w25q_read_status(&sr);
  if (st != STATUS_OK)
  {
    return st;
  }
  if ((sr & SR1_BUSY) != 0u)
  {
    return STATUS_ERR_BUSY;
  }

  const uint8_t cmd[4] = {
      CMD_READ_DATA,
      (uint8_t)(addr >> 16),
      (uint8_t)(addr >> 8),
      (uint8_t)addr,
  };
  return transaction(cmd, sizeof(cmd), buf, len);
}