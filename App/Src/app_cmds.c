#include "app_cmds.h"
#include "main.h"
#include "shell.h"
#include "uart_port.h"

#include <stddef.h>
#include <string.h>

/* Objects the commands operate on, set once in app_cmds_init() */
static const Led *s_led;

/* ---- Command handlers ---- */

static status_t cmd_status(int argc, char *argv[])
{
  (void)argc;
  (void)argv;

  uart_stats_t st;
  uart_port_get_stats(&st);

  shell_printf("uptime : %lu ms\r\n", (unsigned long)HAL_GetTick());
  shell_printf("led    : %s\r\n", led_is_on(s_led) ? "on" : "off");
  shell_printf("uart   : rx_overflow=%lu hw_errors=%lu\r\n",
               (unsigned long)st.rx_overflow,
               (unsigned long)st.rx_hw_errors);
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

/* const: the table stays in Flash. To add a command: write a handler
 * above and add one row here. */
static const ShellCmd s_cmds[] = {
    {"status", "status", cmd_status},
    {"led", "led on|off|toggle", cmd_led},
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