#include "app_cmds.h"
#include "main.h"
#include "shell.h"
#include "uart_port.h"
#include "i2c_port.h"

#include <stddef.h>
#include <string.h>

/* Objects the commands operate on, set once in app_cmds_init() */
static const Led *s_led;

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

/* const: the table stays in Flash. It must come after all handlers.
 * To add a command: write a handler above and add one row here. */
static const ShellCmd s_cmds[] = {
    {"status", "status", cmd_status},
    {"led", "led on|off|toggle", cmd_led},
    {"i2c", "i2c scan", cmd_i2c},
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