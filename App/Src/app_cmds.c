#include "app_cmds.h"
#include "main.h"
#include "shell.h"
#include "uart_port.h"
#include "i2c_port.h"
#include "sensor_service.h"
#include "spi_port.h"
#include "w25q128.h"

#include <stddef.h>
#include <string.h>

#define FLASH_READ_MAX 256u /* Bytes per 'flash read', keeps output short */

/* Objects the commands operate on, set once in app_cmds_init() */
static const Led *s_led;

/* Static, not on the stack: the CubeMX default stack is only 1 KB and
 * vsnprintf below already needs a good part of it */
static uint8_t s_rd_buf[FLASH_READ_MAX];

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
  shell_printf("i2c    : errors=%lu nacks=%lu recoveries=%lu\r\n",
               (unsigned long)is.errors,
               (unsigned long)is.nacks,
               (unsigned long)is.recoveries);
  spi_stats_t ss;
  spi_port_get_stats(&ss);
  shell_printf("spi    : errors=%lu\r\n", (unsigned long)ss.errors);
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

static status_t flash_info(void)
{
  /* Re-detect every time, so a flash wired after boot is picked up */
  w25q_info_t info;
  const status_t st = w25q_init(&info);

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

static status_t cmd_flash(int argc, char *argv[])
{
  if (argc < 2)
  {
    return STATUS_ERR_PARAM;
  }
  if (strcmp(argv[1], "info") == 0)
  {
    return (argc == 2) ? flash_info() : STATUS_ERR_PARAM;
  }
  if (strcmp(argv[1], "read") == 0)
  {
    return flash_read(argc, argv);
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
    {"flash", "flash info|read <addr> <len>", "flash ID and hex dump", cmd_flash},
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
