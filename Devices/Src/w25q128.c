#include <stdbool.h>
#include "w25q128.h"
#include "spi_port.h"
#include "main.h"

#define CMD_RELEASE_PD 0xABu /* Wake from deep power-down, harmless otherwise */
#define CMD_JEDEC_ID 0x9Fu

#define WINBOND_ID 0xEFu
#define CAP_CODE_MIN 0x11u /* 128 KB */
#define CAP_CODE_MAX 0x1Au /* 64 MB */

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