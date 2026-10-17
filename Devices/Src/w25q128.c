#include "w25q128.h"
#include "spi_port.h"
#include "main.h"

#include <stdbool.h>

#define CMD_RELEASE_PD 0xABu /* Wake from deep power-down, harmless otherwise */
#define CMD_JEDEC_ID 0x9Fu
#define CMD_READ_STATUS1 0x05u
#define CMD_READ_DATA 0x03u /* 24-bit address, any length */
#define CMD_WRITE_ENABLE 0x06u
#define CMD_PAGE_PROGRAM 0x02u
#define CMD_SECTOR_ERASE 0x20u /* 4 KB */
#define CMD_CHIP_ERASE 0xC7u

#define SR1_BUSY 0x01u
#define SR1_WEL 0x02u /* Write enable latch */

#define WINBOND_ID 0xEFu
#define CAP_CODE_MIN 0x11u /* 128 KB */
#define CAP_CODE_MAX 0x18u /* 16 MB: the limit of 3-byte addressing */

#define POWERUP_MS 10u /* tVSL: Vcc stable to first command */

/* Datasheet maximums are 3 ms and 400 ms, with some margin on top */
#define PAGE_PROGRAM_TIMEOUT_MS 10u
#define SECTOR_ERASE_TIMEOUT_MS 500u

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
 * CS is released on every path, so an error never leaves the flash selected.
 * Erase and program start on the rising edge of CS, i.e. in cs_high(). */
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

static void build_cmd(uint8_t out[4], uint8_t op, uint32_t addr)
{
  out[0] = op;
  out[1] = (uint8_t)(addr >> 16);
  out[2] = (uint8_t)(addr >> 8);
  out[3] = (uint8_t)addr;
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

status_t w25q_init(w25q_info_t *info)
{
  cs_high(); /* Idle level, regardless of the CubeMX default */

  /* Give the chip time to settle after power-up (tick counts from reset) */
  const uint32_t now = HAL_GetTick();
  if (now < POWERUP_MS)
  {
    HAL_Delay(POWERUP_MS - now);
  }

  /* During an erase or program the chip ignores everything but status
   * reads, so detection would return garbage. Keep the known capacity and
   * report busy. 0xFF is a floating MISO line, not a busy chip. */
  uint8_t sr;
  status_t st = w25q_read_status(&sr);
  if (st != STATUS_OK)
  {
    return st;
  }
  if (((sr & SR1_BUSY) != 0u) && (sr != 0xFFu))
  {
    return STATUS_ERR_BUSY;
  }

  s_capacity = 0u;

  const uint8_t wake = CMD_RELEASE_PD;
  st = transaction(&wake, 1u, NULL, 0u);
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

/* ---- Guards shared by read, program and erase ---- */

/* Written as a subtraction so that addr + len can never overflow */
static status_t check_range(uint32_t addr, uint32_t len)
{
  if (len == 0u)
  {
    return STATUS_ERR_PARAM;
  }
  if (s_capacity == 0u)
  {
    return STATUS_ERR_ID;
  }
  if ((addr >= s_capacity) || (len > (s_capacity - addr)))
  {
    return STATUS_ERR_PARAM;
  }
  return STATUS_OK;
}

/* While an erase or program runs, the chip ignores read, erase and program
 * commands. Refuse up front instead of returning garbage or losing a command. */
static status_t require_idle(void)
{
  if (s_capacity == 0u)
  {
    return STATUS_ERR_ID;
  }
  uint8_t sr;
  status_t st = w25q_read_status(&sr);
  if (st != STATUS_OK)
  {
    return st;
  }
  return ((sr & SR1_BUSY) != 0u) ? STATUS_ERR_BUSY : STATUS_OK;
}

/* Erase and program commands are ignored silently unless WEL is set,
 * so confirm the latch before sending them */
static status_t write_enable(void)
{
  const uint8_t cmd = CMD_WRITE_ENABLE;
  status_t st = transaction(&cmd, 1u, NULL, 0u);
  if (st != STATUS_OK)
  {
    return st;
  }

  uint8_t sr;
  st = w25q_read_status(&sr);
  if (st != STATUS_OK)
  {
    return st;
  }
  return ((sr & SR1_WEL) != 0u) ? STATUS_OK : STATUS_ERR_BUS;
}

/* ---- Read and verify ---- */

status_t w25q_read(uint32_t addr, uint8_t *buf, uint16_t len)
{
  if (buf == NULL)
  {
    return STATUS_ERR_PARAM;
  }
  status_t st = check_range(addr, len);
  if (st != STATUS_OK)
  {
    return st;
  }
  st = require_idle();
  if (st != STATUS_OK)
  {
    return st;
  }

  uint8_t cmd[4];
  build_cmd(cmd, CMD_READ_DATA, addr);
  return transaction(cmd, sizeof(cmd), buf, len);
}

/* Compare flash with expected data, or with 0xFF when expected is NULL.
 * Works in small chunks so it needs almost no stack. */
static status_t compare(uint32_t addr, const uint8_t *expected, uint16_t len)
{
  uint8_t chunk[32];
  uint16_t done = 0u;

  while (done < len)
  {
    uint16_t n = (uint16_t)(len - done);
    if (n > sizeof(chunk))
    {
      n = sizeof(chunk);
    }

    status_t st = w25q_read(addr + done, chunk, n);
    if (st != STATUS_OK)
    {
      return st;
    }
    for (uint16_t i = 0u; i < n; i++)
    {
      const uint8_t want = (expected != NULL) ? expected[done + i] : 0xFFu;
      if (chunk[i] != want)
      {
        return STATUS_ERR_VERIFY;
      }
    }
    done = (uint16_t)(done + n);
  }
  return STATUS_OK;
}

status_t w25q_verify(uint32_t addr, const uint8_t *expected, uint16_t len)
{
  if ((expected == NULL) || (len == 0u))
  {
    return STATUS_ERR_PARAM;
  }
  return compare(addr, expected, len);
}

status_t w25q_check_blank(uint32_t addr, uint16_t len)
{
  if (len == 0u)
  {
    return STATUS_ERR_PARAM;
  }
  return compare(addr, NULL, len);
}

/* ---- Program and erase ---- */

/* The caller guarantees that the data does not cross a page boundary */
static status_t page_program(uint32_t addr, const uint8_t *data, uint16_t len)
{
  status_t st = write_enable();
  if (st != STATUS_OK)
  {
    return st;
  }

  uint8_t cmd[4];
  build_cmd(cmd, CMD_PAGE_PROGRAM, addr);

  /* Command and data must share one CS frame */
  cs_low();
  st = spi_port_write(cmd, sizeof(cmd));
  if (st == STATUS_OK)
  {
    st = spi_port_write(data, len);
  }
  cs_high(); /* The program cycle starts here */
  if (st != STATUS_OK)
  {
    return st;
  }

  return w25q_wait_ready(PAGE_PROGRAM_TIMEOUT_MS);
}

status_t w25q_write(uint32_t addr, const uint8_t *data, uint16_t len)
{
  if (data == NULL)
  {
    return STATUS_ERR_PARAM;
  }
  status_t st = check_range(addr, len);
  if (st != STATUS_OK)
  {
    return st;
  }
  st = require_idle();
  if (st != STATUS_OK)
  {
    return st;
  }

  while (len > 0u)
  {
    /* At the end of a page the chip wraps the address back to the page
     * start and overwrites the first bytes, so cut at every boundary */
    uint16_t chunk = (uint16_t)(W25Q_PAGE_SIZE - (addr & (W25Q_PAGE_SIZE - 1u)));
    if (chunk > len)
    {
      chunk = len;
    }

    st = page_program(addr, data, chunk);
    if (st != STATUS_OK)
    {
      return st;
    }
    addr += chunk;
    data += chunk;
    len = (uint16_t)(len - chunk);
  }
  return STATUS_OK;
}

status_t w25q_sector_erase(uint32_t addr)
{
  if ((addr % W25Q_SECTOR_SIZE) != 0u)
  {
    return STATUS_ERR_PARAM;
  }
  status_t st = check_range(addr, W25Q_SECTOR_SIZE);
  if (st != STATUS_OK)
  {
    return st;
  }
  st = require_idle();
  if (st != STATUS_OK)
  {
    return st;
  }
  st = write_enable();
  if (st != STATUS_OK)
  {
    return st;
  }

  uint8_t cmd[4];
  build_cmd(cmd, CMD_SECTOR_ERASE, addr);
  st = transaction(cmd, sizeof(cmd), NULL, 0u);
  if (st != STATUS_OK)
  {
    return st;
  }
  return w25q_wait_ready(SECTOR_ERASE_TIMEOUT_MS);
}

status_t w25q_chip_erase_start(void)
{
  status_t st = require_idle();
  if (st != STATUS_OK)
  {
    return st;
  }
  st = write_enable();
  if (st != STATUS_OK)
  {
    return st;
  }

  /* No wait: the chip erases on its own and reports it through BUSY */
  const uint8_t cmd = CMD_CHIP_ERASE;
  return transaction(&cmd, 1u, NULL, 0u);
}