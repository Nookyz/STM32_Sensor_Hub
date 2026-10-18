#include "app_cmds.h"
#include "main.h"
#include "shell.h"
#include "uart_port.h"
#include "i2c_port.h"
#include "sensor_service.h"
#include "spi_port.h"
#include "w25q128.h"
#include "flash_selftest.h"
#include "data_log.h"
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "perf.h"

/* ---- Sensor log ---- */
#define LOG_DUMP_DEFAULT 5u
#define LOG_DUMP_MAX 20u

#define FLASH_READ_MAX 256u /* Bytes per 'flash read', keeps output short */
#define FLASH_WRITE_MAX 16u /* Bytes per 'flash write': the shell line is only 63 chars */

/* ---- Background flash scan ---- */
#define SCAN_LIST_MAX 16u       /* Non-blank sectors printed one by one */
#define SCAN_PROGRESS_STEP 512u /* Sectors between progress lines */

static uint8_t s_wr_buf[FLASH_WRITE_MAX];

/* Objects the commands operate on, set once in app_cmds_init() */
static const Led *s_led;

/* Static, not on the stack: the CubeMX default stack is only 1 KB and
 * vsnprintf below already needs a good part of it */
static uint8_t s_rd_buf[FLASH_READ_MAX];

typedef struct
{
  bool active;
  bool dirty;        /* Current sector holds a byte other than 0xFF */
  uint32_t addr;     /* Next address to read */
  uint32_t nonblank; /* Sectors that are not blank so far */
  uint32_t t_start;
} flash_scan_t;

static flash_scan_t s_scan;

/* Known sensor addresses, to label scan results */
typedef struct
{
  uint8_t addr;
  const char *name;
} KnownDev;

static const KnownDev s_known[] = {
    {0x38u, "AHT20"},
    {0x76u, "BMP280"}, /* SDO tied to GND */
    {0x77u, "BMP280"}, /* SDO tied to VCC */
};

static const char *known_name(uint8_t addr)
{
  for (size_t i = 0u; i < sizeof(s_known) / sizeof(s_known[0]); i++)
  {
    if (s_known[i].addr == addr)
    {
      return s_known[i].name;
    }
  }
  return "unknown";
}

static const char *clock_source(void)
{
  switch (__HAL_RCC_GET_SYSCLK_SOURCE())
  {
  case RCC_SYSCLKSOURCE_STATUS_HSI:
    return "HSI";
  case RCC_SYSCLKSOURCE_STATUS_HSE:
    return "HSE";
  case RCC_SYSCLKSOURCE_STATUS_PLLCLK:
    return ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) != 0u) ? "PLL(HSE)" : "PLL(HSI)";
  default:
    return "?";
  }
}

/* ---- Command handlers ---- */

static status_t cmd_status(int argc, char *argv[])
{
  (void)argc;
  (void)argv;

  uart_stats_t us;
  i2c_stats_t is;
  uart_port_get_stats(&us);
  i2c_port_get_stats(&is);

  shell_printf("uptime : %lu ms\r\n", (unsigned long)HAL_GetTick());
  shell_printf("led    : %s\r\n", led_is_on(s_led) ? "on" : "off");
  shell_printf("uart   : rx_overflow=%lu hw_errors=%lu\r\n",
               (unsigned long)us.rx_overflow,
               (unsigned long)us.rx_hw_errors);
  shell_printf("uart tx: waits=%lu dropped=%lu errors=%lu peak=%lu/%u\r\n",
               (unsigned long)us.tx_waits, (unsigned long)us.tx_dropped,
               (unsigned long)us.tx_errors, (unsigned long)us.tx_peak,
               (unsigned)UART_PORT_TX_BUF_SIZE);
  shell_printf("uart rx: bytes=%lu events=%lu restarts=%lu\r\n",
               (unsigned long)us.rx_bytes, (unsigned long)us.rx_events,
               (unsigned long)us.rx_restarts);
  shell_printf("i2c    : errors=%lu nacks=%lu recoveries=%lu\r\n",
               (unsigned long)is.errors,
               (unsigned long)is.nacks,
               (unsigned long)is.recoveries);
  spi_stats_t ss;
  spi_port_get_stats(&ss);
  shell_printf("spi    : errors=%lu\r\n", (unsigned long)ss.errors);
  shell_printf("clock  : %s, sys %lu Hz, spi %lu Hz\r\n", clock_source(),
               (unsigned long)HAL_RCC_GetSysClockFreq(),
               (unsigned long)spi_port_get_clock_hz());
  return STATUS_OK;
}

/* perf | perf reset */
static status_t cmd_perf(int argc, char *argv[])
{
  if (argc == 2)
  {
    if (strcmp(argv[1], "reset") != 0)
    {
      return STATUS_ERR_PARAM;
    }
    perf_reset();
    i2c_port_reset_max();
    shell_printf("perf counters reset\r\n");
    return STATUS_OK;
  }
  if (argc != 1)
  {
    return STATUS_ERR_PARAM;
  }

  perf_loop_t l;
  i2c_stats_t is;
  perf_loop_get(&l);
  i2c_port_get_stats(&is);

  shell_printf("loop   : max %lu us, avg %lu.%lu us, %lu passes\r\n",
               (unsigned long)l.max_us,
               (unsigned long)(l.avg_x10_us / 10u), (unsigned long)(l.avg_x10_us % 10u),
               (unsigned long)l.passes);
  shell_printf("i2c    : longest transfer %lu us\r\n", (unsigned long)is.max_xfer_us);
  return STATUS_OK;
}

static status_t cmd_led(int argc, char *argv[])
{
  if (argc != 2)
  {
    return STATUS_ERR_PARAM; /* Shell prints the usage string */
  }
  if (strcmp(argv[1], "on") == 0)
  {
    return led_on(s_led);
  }
  if (strcmp(argv[1], "off") == 0)
  {
    return led_off(s_led);
  }
  if (strcmp(argv[1], "toggle") == 0)
  {
    return led_toggle(s_led);
  }
  return STATUS_ERR_PARAM;
}

static status_t cmd_i2c(int argc, char *argv[])
{
  if ((argc != 2) || (strcmp(argv[1], "scan") != 0))
  {
    return STATUS_ERR_PARAM;
  }

  unsigned found = 0u;
  /* 0x00-0x07 and 0x78-0x7F are reserved by the I2C specification */
  for (uint8_t addr = 0x08u; addr <= 0x77u; addr++)
  {
    status_t st = i2c_port_probe(addr);
    if (st == STATUS_OK)
    {
      shell_printf("  0x%02X  %s\r\n", (unsigned)addr, known_name(addr));
      found++;
    }
    else if (st != STATUS_ERR_NACK)
    {
      shell_printf("bus error at 0x%02X\r\n", (unsigned)addr);
      return st;
    }
  }
  shell_printf("%u device(s) found\r\n", found);
  return STATUS_OK;
}

/* Split a value in 0.01 units into sign, whole and fractional parts,
 * so that values like -0.50 print correctly */
static void split_c100(int32_t v, const char **sign, uint32_t *whole, uint32_t *frac)
{
  const uint32_t a = (uint32_t)((v < 0) ? -v : v);
  *sign = (v < 0) ? "-" : "";
  *whole = a / 100u;
  *frac = a % 100u;
}

static status_t cmd_sensor(int argc, char *argv[])
{
  if ((argc != 2) || (strcmp(argv[1], "read") != 0))
  {
    return STATUS_ERR_PARAM;
  }

  sensor_data_t d;
  sensor_service_get(&d);

  const uint32_t now = HAL_GetTick();
  const char *sg;
  uint32_t w, f;

  if (d.aht_ok)
  {
    split_c100(d.aht_temp_c100, &sg, &w, &f);
    shell_printf("aht20  : %s%lu.%02lu C, %lu.%02lu %%RH (%lu ms ago)\r\n",
                 sg, (unsigned long)w, (unsigned long)f,
                 (unsigned long)(d.aht_hum_c100 / 100u),
                 (unsigned long)(d.aht_hum_c100 % 100u),
                 (unsigned long)(now - d.aht_stamp_ms));
  }
  else
  {
    shell_printf("aht20  : no data (errors=%lu)\r\n", (unsigned long)d.aht_errors);
  }

  if (d.bmp_ok)
  {
    split_c100(d.bmp_temp_c100, &sg, &w, &f);
    shell_printf("bmp280 : %s%lu.%02lu C, %lu.%02lu hPa (%lu ms ago)\r\n",
                 sg, (unsigned long)w, (unsigned long)f,
                 (unsigned long)(d.bmp_press_pa / 100u),
                 (unsigned long)(d.bmp_press_pa % 100u),
                 (unsigned long)(now - d.bmp_stamp_ms));
  }
  else
  {
    shell_printf("bmp280 : no data (errors=%lu)\r\n", (unsigned long)d.bmp_errors);
  }
  return STATUS_OK;
}

/* Parse "123" or "0x7B". Rejects empty strings, junk and 32-bit overflow. */
static bool parse_u32(const char *s, uint32_t *out)
{
  uint32_t base = 10u;
  uint32_t v = 0u;

  if ((s[0] == '0') && ((s[1] == 'x') || (s[1] == 'X')))
  {
    base = 16u;
    s += 2;
  }
  if (*s == '\0')
  {
    return false;
  }

  for (; *s != '\0'; s++)
  {
    uint32_t d;
    if ((*s >= '0') && (*s <= '9'))
    {
      d = (uint32_t)(*s - '0');
    }
    else if ((base == 16u) && (*s >= 'a') && (*s <= 'f'))
    {
      d = (uint32_t)(*s - 'a') + 10u;
    }
    else if ((base == 16u) && (*s >= 'A') && (*s <= 'F'))
    {
      d = (uint32_t)(*s - 'A') + 10u;
    }
    else
    {
      return false;
    }
    if (v > (UINT32_MAX - d) / base)
    {
      return false; /* v * base + d would overflow */
    }
    v = v * base + d;
  }
  *out = v;
  return true;
}

/* "00F000: 12 34 ...  |ascii|" for up to 16 bytes. Built by hand into one
 * string: one UART write per line instead of one per byte. */
static void dump_line(uint32_t addr, const uint8_t *p, uint8_t n)
{
  static const char hexd[] = "0123456789ABCDEF";
  char line[80];
  char *o = line;

  for (int s = 20; s >= 0; s -= 4)
  { /* 24-bit address */
    *o++ = hexd[(addr >> s) & 0xFu];
  }
  *o++ = ':';

  for (uint8_t i = 0u; i < 16u; i++)
  {
    *o++ = ' ';
    if (i < n)
    {
      *o++ = hexd[p[i] >> 4];
      *o++ = hexd[p[i] & 0x0Fu];
    }
    else
    {
      *o++ = ' '; /* Pad the last short line */
      *o++ = ' ';
    }
  }

  *o++ = ' ';
  *o++ = ' ';
  for (uint8_t i = 0u; i < n; i++)
  {
    *o++ = ((p[i] >= 0x20u) && (p[i] <= 0x7Eu)) ? (char)p[i] : '.';
  }
  *o++ = '\r';
  *o++ = '\n';
  *o = '\0';

  shell_printf("%s", line);
}

/* Parse a hex string such as "deadbeef" into bytes. Returns the byte count,
 * or -1 for an odd length, a non-hex character or more than max bytes. */
static int parse_hex_bytes(const char *s, uint8_t *out, size_t max)
{
  const size_t n = strlen(s);
  if ((n == 0u) || ((n & 1u) != 0u) || ((n / 2u) > max))
  {
    return -1;
  }

  for (size_t i = 0u; i < n; i++)
  {
    const char c = s[i];
    uint8_t v;
    if ((c >= '0') && (c <= '9'))
    {
      v = (uint8_t)(c - '0');
    }
    else if ((c >= 'a') && (c <= 'f'))
    {
      v = (uint8_t)(c - 'a' + 10);
    }
    else if ((c >= 'A') && (c <= 'F'))
    {
      v = (uint8_t)(c - 'A' + 10);
    }
    else
    {
      return -1;
    }

    if ((i & 1u) == 0u)
    {
      out[i / 2u] = (uint8_t)(v << 4);
    }
    else
    {
      out[i / 2u] |= v;
    }
  }
  return (int)(n / 2u);
}

static void print_flash_busy(void)
{
  shell_printf("flash busy: erase or program in progress\r\n");
}

static status_t flash_info(void)
{
  /* Re-detect every time, so a flash wired after boot is picked up */
  w25q_info_t info;
  const status_t st = w25q_init(&info);

  if (st == STATUS_ERR_BUSY)
  {
    shell_printf("status   : busy (erase or program in progress)\r\n");
    return STATUS_OK;
  }
  if ((st != STATUS_OK) && (st != STATUS_ERR_ID))
  {
    return st; /* SPI failure, nothing to print */
  }

  shell_printf("JEDEC ID : %02X %02X %02X\r\n",
               (unsigned)info.manufacturer, (unsigned)info.mem_type,
               (unsigned)info.capacity_code);

  if (st == STATUS_OK)
  {
    shell_printf("vendor   : Winbond\r\n");
    shell_printf("capacity : %lu KB\r\n", (unsigned long)(info.capacity_bytes / 1024u));

    uint8_t sr;
    if (w25q_read_status(&sr) == STATUS_OK)
    {
      shell_printf("status   : 0x%02X (%s)\r\n", (unsigned)sr,
                   ((sr & 0x01u) != 0u) ? "busy" : "ready");
    }
    return STATUS_OK;
  }

  /* Hints for the two most common wiring faults */
  if ((info.manufacturer == 0xFFu) && (info.mem_type == 0xFFu))
  {
    shell_printf("no answer: MISO floats high. Check VCC, CS and MISO wiring\r\n");
  }
  else if ((info.manufacturer == 0x00u) && (info.mem_type == 0x00u))
  {
    shell_printf("no answer: MISO stuck low. Check power and MISO/MOSI swap\r\n");
  }
  else
  {
    shell_printf("unknown chip, expected Winbond (EF)\r\n");
  }
  return st;
}

static status_t flash_read(int argc, char *argv[])
{
  uint32_t addr;
  uint32_t len;

  if ((argc != 4) || !parse_u32(argv[2], &addr) || !parse_u32(argv[3], &len))
  {
    return STATUS_ERR_PARAM;
  }
  if ((len == 0u) || (len > FLASH_READ_MAX))
  {
    shell_printf("length must be 1..%u\r\n", (unsigned)FLASH_READ_MAX);
    return STATUS_ERR_PARAM;
  }
  if (w25q_capacity() == 0u)
  {
    shell_printf("flash not detected, run 'flash info'\r\n");
    return STATUS_ERR_ID;
  }

  status_t st = w25q_read(addr, s_rd_buf, (uint16_t)len);
  if (st == STATUS_ERR_PARAM)
  {
    shell_printf("out of range: flash size is %lu bytes\r\n",
                 (unsigned long)w25q_capacity());
    return st;
  }
  if (st == STATUS_ERR_BUSY)
  {
    print_flash_busy();
    return st;
  }
  if (st != STATUS_OK)
  {
    return st;
  }

  for (uint32_t off = 0u; off < len; off += 16u)
  {
    const uint32_t left = len - off;
    dump_line(addr + off, &s_rd_buf[off], (uint8_t)((left < 16u) ? left : 16u));
  }
  return STATUS_OK;
}

/* flash erase <addr>|chip [confirm] */
static status_t flash_erase(int argc, char *argv[])
{
  if ((argc < 3) || (argc > 4))
  {
    return STATUS_ERR_PARAM;
  }
  const bool confirmed = (argc == 4);
  if (confirmed && (strcmp(argv[3], "confirm") != 0))
  {
    return STATUS_ERR_PARAM;
  }
  if (w25q_capacity() == 0u)
  {
    shell_printf("flash not detected, run 'flash info'\r\n");
    return STATUS_ERR_ID;
  }

  status_t st;

  if (strcmp(argv[2], "chip") == 0)
  {
    if (!confirmed)
    {
      shell_printf("refused: this erases the WHOLE flash. Add 'confirm' to proceed\r\n");
      return STATUS_OK;
    }
    st = w25q_chip_erase_start();
    if (st == STATUS_OK)
    {
      shell_printf("chip erase started (up to 200 s). 'flash info' shows when it is done\r\n");
    }
  }
  else
  {
    uint32_t addr;
    if (!parse_u32(argv[2], &addr))
    {
      return STATUS_ERR_PARAM;
    }
    if ((addr >= w25q_capacity()) || ((addr % W25Q_SECTOR_SIZE) != 0u))
    {
      shell_printf("address must be a multiple of %u, below %lu\r\n",
                   (unsigned)W25Q_SECTOR_SIZE, (unsigned long)w25q_capacity());
      return STATUS_ERR_PARAM;
    }
    if (!confirmed)
    {
      shell_printf("refused: this erases %u bytes at 0x%06lX. Add 'confirm' to proceed\r\n",
                   (unsigned)W25Q_SECTOR_SIZE, (unsigned long)addr);
      return STATUS_OK;
    }

    const uint32_t t0 = HAL_GetTick();
    st = w25q_sector_erase(addr);
    if (st == STATUS_OK)
    {
      shell_printf("erased %u bytes at 0x%06lX (%lu ms)\r\n",
                   (unsigned)W25Q_SECTOR_SIZE, (unsigned long)addr,
                   (unsigned long)(HAL_GetTick() - t0));
    }
  }

  if (st == STATUS_ERR_BUSY)
  {
    print_flash_busy();
  }
  return st;
}

/* flash write <addr> <hex> [confirm] */
static status_t flash_write(int argc, char *argv[])
{
  if ((argc < 4) || (argc > 5))
  {
    return STATUS_ERR_PARAM;
  }
  const bool confirmed = (argc == 5);
  if (confirmed && (strcmp(argv[4], "confirm") != 0))
  {
    return STATUS_ERR_PARAM;
  }

  uint32_t addr;
  if (!parse_u32(argv[2], &addr))
  {
    return STATUS_ERR_PARAM;
  }
  const int len = parse_hex_bytes(argv[3], s_wr_buf, FLASH_WRITE_MAX);
  if (len < 0)
  {
    shell_printf("data: even number of hex digits, 1..%u bytes\r\n", (unsigned)FLASH_WRITE_MAX);
    return STATUS_ERR_PARAM;
  }

  /* Programming can only clear bits: a non-blank target would end up with
   * a mix of old and new data. This check also validates the range. */
  status_t st = w25q_check_blank(addr, (uint16_t)len);
  if (st == STATUS_ERR_VERIFY)
  {
    shell_printf("area is not erased: erase the sector first\r\n");
    return st;
  }
  if (st == STATUS_ERR_PARAM)
  {
    shell_printf("out of range: flash size is %lu bytes\r\n",
                 (unsigned long)w25q_capacity());
    return st;
  }
  if (st == STATUS_ERR_ID)
  {
    shell_printf("flash not detected, run 'flash info'\r\n");
    return st;
  }
  if (st == STATUS_ERR_BUSY)
  {
    print_flash_busy();
    return st;
  }
  if (st != STATUS_OK)
  {
    return st;
  }

  if (!confirmed)
  {
    shell_printf("refused: this writes %d bytes at 0x%06lX. Add 'confirm' to proceed\r\n",
                 len, (unsigned long)addr);
    return STATUS_OK;
  }

  const uint32_t t0 = HAL_GetTick();
  st = w25q_write(addr, s_wr_buf, (uint16_t)len);
  if (st == STATUS_OK)
  {
    st = w25q_verify(addr, s_wr_buf, (uint16_t)len);
  }
  if (st == STATUS_ERR_VERIFY)
  {
    shell_printf("verify failed: read-back differs from the written data\r\n");
    return st;
  }
  if (st != STATUS_OK)
  {
    return st;
  }

  shell_printf("wrote %d bytes at 0x%06lX, verified (%lu ms)\r\n",
               len, (unsigned long)addr, (unsigned long)(HAL_GetTick() - t0));
  return STATUS_OK;
}

/* flash scan | flash scan stop */
static status_t flash_scan(int argc, char *argv[])
{
  if (argc == 3)
  {
    if (strcmp(argv[2], "stop") != 0)
    {
      return STATUS_ERR_PARAM;
    }
    if (s_scan.active)
    {
      s_scan.active = false;
      shell_printf("scan stopped at 0x%06lX\r\n", (unsigned long)s_scan.addr);
    }
    else
    {
      shell_printf("no scan running\r\n");
    }
    return STATUS_OK;
  }
  if (argc != 2)
  {
    return STATUS_ERR_PARAM;
  }
  if (s_scan.active)
  {
    shell_printf("scan already running, 'flash scan stop' cancels it\r\n");
    return STATUS_OK;
  }
  if (w25q_capacity() == 0u)
  {
    shell_printf("flash not detected, run 'flash info'\r\n");
    return STATUS_ERR_ID;
  }

  s_scan = (flash_scan_t){.active = true, .t_start = HAL_GetTick()};
  shell_printf("scan started: %lu sectors, about 30 s, other commands keep working\r\n",
               (unsigned long)(w25q_capacity() / W25Q_SECTOR_SIZE));
  return STATUS_OK;
}

void app_cmds_poll(void)
{
  if (!s_scan.active)
  {
    return;
  }

  /* One 256-byte chunk per call keeps every main-loop pass short.
   * s_rd_buf is shared with 'flash read', which is safe: both run to
   * completion in the main loop and never overlap. */
  const status_t st = w25q_read(s_scan.addr, s_rd_buf, FLASH_READ_MAX);
  if (st != STATUS_OK)
  {
    /* A running chip erase lands here as STATUS_ERR_BUSY */
    shell_printf("scan aborted at 0x%06lX, error %d\r\n",
                 (unsigned long)s_scan.addr, (int)st);
    s_scan.active = false;
    return;
  }

  for (uint32_t i = 0u; i < FLASH_READ_MAX; i++)
  {
    if (s_rd_buf[i] != 0xFFu)
    {
      s_scan.dirty = true;
      break;
    }
  }
  s_scan.addr += FLASH_READ_MAX;

  /* A sector is complete every 4096 bytes */
  if ((s_scan.addr % W25Q_SECTOR_SIZE) == 0u)
  {
    if (s_scan.dirty)
    {
      s_scan.nonblank++;
      if (s_scan.nonblank <= SCAN_LIST_MAX)
      {
        shell_printf("  0x%06lX not blank\r\n",
                     (unsigned long)(s_scan.addr - W25Q_SECTOR_SIZE));
      }
      s_scan.dirty = false;
    }
    const uint32_t done = s_scan.addr / W25Q_SECTOR_SIZE;
    if (((done % SCAN_PROGRESS_STEP) == 0u) && (s_scan.addr < w25q_capacity()))
    {
      shell_printf("  %lu/%lu sectors\r\n", (unsigned long)done,
                   (unsigned long)(w25q_capacity() / W25Q_SECTOR_SIZE));
    }
  }

  if (s_scan.addr >= w25q_capacity())
  {
    s_scan.active = false;
    shell_printf("scan done: %lu sectors, %lu not blank, %lu ms\r\n",
                 (unsigned long)(w25q_capacity() / W25Q_SECTOR_SIZE),
                 (unsigned long)s_scan.nonblank,
                 (unsigned long)(HAL_GetTick() - s_scan.t_start));
    if (s_scan.nonblank > SCAN_LIST_MAX)
    {
      shell_printf("only the first %u non-blank sectors were listed\r\n",
                   (unsigned)SCAN_LIST_MAX);
    }
  }
}

/* flash test [confirm] */
static status_t flash_test(int argc, char *argv[])
{
  if (argc > 3)
  {
    return STATUS_ERR_PARAM;
  }
  const bool confirmed = (argc == 3);
  if (confirmed && (strcmp(argv[2], "confirm") != 0))
  {
    return STATUS_ERR_PARAM;
  }

  const uint32_t addr = flash_selftest_addr();
  if (addr == 0u)
  {
    shell_printf("flash not detected, run 'flash info'\r\n");
    return STATUS_ERR_ID;
  }
  if (!confirmed)
  {
    shell_printf("refused: this erases the sector at 0x%06lX. Add 'confirm' to proceed\r\n",
                 (unsigned long)addr);
    return STATUS_OK;
  }

  shell_printf("testing sector 0x%06lX, shell is busy until done\r\n", (unsigned long)addr);

  flash_selftest_t r;
  const status_t st = flash_selftest_run(&r);
  if (st == STATUS_OK)
  {
    shell_printf("PASS: erase %lu ms, write %u B in %lu ms, verify %lu ms\r\n",
                 (unsigned long)r.erase_ms, (unsigned)r.test_len,
                 (unsigned long)r.write_ms, (unsigned long)r.verify_ms);
  }
  else
  {
    shell_printf("FAIL at '%s': error %d\r\n", r.failed_step, (int)st);
  }
  return st;
}

static status_t cmd_flash(int argc, char *argv[])
{
  if (argc < 2)
  {
    /* One short message per call: shell_printf output is size-limited */
    shell_printf("flash info\r\n");
    shell_printf("flash read <addr> <len>\r\n");
    shell_printf("flash erase <addr>|chip [confirm]\r\n");
    shell_printf("flash write <addr> <hex> [confirm]\r\n");
    shell_printf("flash scan [stop]\r\n");
    shell_printf("flash test [confirm]\r\n");
    return STATUS_OK;
  }
  if (strcmp(argv[1], "info") == 0)
  {
    return (argc == 2) ? flash_info() : STATUS_ERR_PARAM;
  }
  if (strcmp(argv[1], "read") == 0)
  {
    return flash_read(argc, argv);
  }
  if (strcmp(argv[1], "erase") == 0)
  {
    return flash_erase(argc, argv);
  }
  if (strcmp(argv[1], "write") == 0)
  {
    return flash_write(argc, argv);
  }
  if (strcmp(argv[1], "scan") == 0)
  {
    return flash_scan(argc, argv);
  }
  if (strcmp(argv[1], "test") == 0)
  {
    return flash_test(argc, argv);
  }
  return STATUS_ERR_PARAM;
}

static void print_log_record(const log_record_t *r)
{
  char a[24];
  char b[28];
  const char *sg;
  uint32_t w, f;

  if (r->aht_ok)
  {
    split_c100(r->aht_temp_c100, &sg, &w, &f);
    snprintf(a, sizeof(a), "%s%lu.%02lu C %lu.%02lu%%", sg,
             (unsigned long)w, (unsigned long)f,
             (unsigned long)(r->aht_hum_c100 / 100u),
             (unsigned long)(r->aht_hum_c100 % 100u));
  }
  else
  {
    strcpy(a, "--");
  }

  if (r->bmp_ok)
  {
    split_c100(r->bmp_temp_c100, &sg, &w, &f);
    snprintf(b, sizeof(b), "%s%lu.%02lu C %lu.%02lu hPa", sg,
             (unsigned long)w, (unsigned long)f,
             (unsigned long)(r->bmp_press_pa / 100u),
             (unsigned long)(r->bmp_press_pa % 100u));
  }
  else
  {
    strcpy(b, "--");
  }

  shell_printf("#%lu %lu ms | A: %s | B: %s\r\n",
               (unsigned long)r->seq, (unsigned long)r->stamp_ms, a, b);
}

static status_t log_save(void)
{
  sensor_data_t d;
  sensor_service_get(&d);

  if (!d.aht_ok && !d.bmp_ok)
  {
    shell_printf("no sensor data yet, check 'sensor read'\r\n");
    return STATUS_OK;
  }

  uint32_t idx;
  const status_t st = data_log_append(&d, HAL_GetTick(), &idx);
  if (st == STATUS_ERR_FULL)
  {
    shell_printf("log is full (%u records), 'log clear confirm' erases it\r\n",
                 (unsigned)DATA_LOG_MAX_RECORDS);
  }
  else if (st == STATUS_OK)
  {
    shell_printf("saved record #%lu (aht20 %s, bmp280 %s)\r\n", (unsigned long)idx,
                 d.aht_ok ? "ok" : "no data", d.bmp_ok ? "ok" : "no data");
  }
  return st;
}

static status_t log_status(void)
{
  uint32_t count;
  const status_t st = data_log_count(&count);
  if (st != STATUS_OK)
  {
    return st;
  }

  shell_printf("records : %lu / %u\r\n", (unsigned long)count, (unsigned)DATA_LOG_MAX_RECORDS);
  shell_printf("area    : 0x%06lX - 0x%06lX, %u bytes per record\r\n",
               (unsigned long)DATA_LOG_START_ADDR,
               (unsigned long)(DATA_LOG_START_ADDR + (DATA_LOG_SECTORS * W25Q_SECTOR_SIZE) - 1u),
               (unsigned)DATA_LOG_RECORD_SIZE);
  return STATUS_OK;
}

/* log dump [n]: the last n records, oldest first */
static status_t log_dump(int argc, char *argv[])
{
  uint32_t n = LOG_DUMP_DEFAULT;

  if (argc == 3)
  {
    if (!parse_u32(argv[2], &n) || (n == 0u) || (n > LOG_DUMP_MAX))
    {
      shell_printf("n must be 1..%u\r\n", (unsigned)LOG_DUMP_MAX);
      return STATUS_ERR_PARAM;
    }
  }
  else if (argc != 2)
  {
    return STATUS_ERR_PARAM;
  }

  uint32_t count;
  status_t st = data_log_count(&count);
  if (st != STATUS_OK)
  {
    return st;
  }
  if (count == 0u)
  {
    shell_printf("log is empty\r\n");
    return STATUS_OK;
  }

  for (uint32_t i = (count > n) ? (count - n) : 0u; i < count; i++)
  {
    log_record_t r;
    st = data_log_read(i, &r);
    if (st == STATUS_ERR_CRC)
    {
      shell_printf("#%lu bad record (CRC)\r\n", (unsigned long)i);
    }
    else if (st != STATUS_OK)
    {
      return st;
    }
    else
    {
      print_log_record(&r);
    }
  }
  return STATUS_OK;
}

/* log clear [confirm] */
static status_t log_clear(int argc, char *argv[])
{
  if (argc > 3)
  {
    return STATUS_ERR_PARAM;
  }
  const bool confirmed = (argc == 3);
  if (confirmed && (strcmp(argv[2], "confirm") != 0))
  {
    return STATUS_ERR_PARAM;
  }

  uint32_t count;
  status_t st = data_log_count(&count);
  if (st != STATUS_OK)
  {
    return st;
  }
  if (count == 0u)
  {
    shell_printf("log is already empty\r\n");
    return STATUS_OK;
  }
  if (!confirmed)
  {
    shell_printf("refused: this erases %lu record(s). Add 'confirm' to proceed\r\n",
                 (unsigned long)count);
    return STATUS_OK;
  }

  uint32_t sectors = 0u;
  const uint32_t t0 = HAL_GetTick();
  st = data_log_clear(&sectors);
  shell_printf("erased %lu sector(s) in %lu ms\r\n", (unsigned long)sectors,
               (unsigned long)(HAL_GetTick() - t0));
  return st;
}

static status_t cmd_log(int argc, char *argv[])
{
  if (argc < 2)
  {
    shell_printf("log save\r\n");
    shell_printf("log status\r\n");
    shell_printf("log dump [n]\r\n");
    shell_printf("log clear [confirm]\r\n");
    return STATUS_OK;
  }
  if (w25q_capacity() == 0u)
  {
    shell_printf("flash not detected, run 'flash info'\r\n");
    return STATUS_ERR_ID;
  }

  if (strcmp(argv[1], "save") == 0)
  {
    return (argc == 2) ? log_save() : STATUS_ERR_PARAM;
  }
  if (strcmp(argv[1], "status") == 0)
  {
    return (argc == 2) ? log_status() : STATUS_ERR_PARAM;
  }
  if (strcmp(argv[1], "dump") == 0)
  {
    return log_dump(argc, argv);
  }
  if (strcmp(argv[1], "clear") == 0)
  {
    return log_clear(argc, argv);
  }
  return STATUS_ERR_PARAM;
}

/* const: the table stays in Flash. It must come after all handlers.
 * To add a command: write a handler above and add one row here. */
static const ShellCmd s_cmds[] = {
    {"status", "status", "show uptime and error counters", cmd_status},
    {"led", "led on|off|toggle", "control the LED", cmd_led},
    {"i2c", "i2c scan", "list devices on the I2C bus", cmd_i2c},
    {"sensor", "sensor read", "print the latest sensor data", cmd_sensor},
    {"flash", "flash <subcommand>", "flash tools, run 'flash'", cmd_flash}, // info|read|erase|write
    {"log", "log <subcommand>", "save sensor data to flash, run 'log'", cmd_log},
    {"perf", "perf [reset]", "loop timing statistics", cmd_perf},
};

status_t app_cmds_init(const Led *led)
{
  if (led == NULL)
  {
    return STATUS_ERR_PARAM;
  }
  s_led = led;
  shell_init(s_cmds, sizeof(s_cmds) / sizeof(s_cmds[0]));
  return STATUS_OK;
}
