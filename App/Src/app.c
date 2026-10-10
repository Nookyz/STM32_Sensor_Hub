#include "app.h"
#include "main.h"
#include "status.h"
#include "led.h"
#include "button.h"

/* Hardware description lives here, pin names come from CubeMX labels */
static const Led g_led = {
    .port = LED_GPIO_Port,
    .pin = LED_Pin,
    .active_high = true,
};

static Button g_btn_user = {
    .port = BTN_GPIO_Port,
    .pin = BTN_Pin,
    .active_low = true, /* Button to GND, internal pull-up */
    .debounce_ms = 30u,
};

void app_init(void)
{
  /* Start with the LED off regardless of the CubeMX default level */
  if (led_init(&g_led) != STATUS_OK)
  {
    Error_Handler();
  }
  if (button_init(&g_btn_user) != STATUS_OK)
  {
    Error_Handler();
  }
}

void app_loop(void)
{
  /* Returns true once per press, so one press toggles the LED once */
  if (button_update(&g_btn_user))
  {
    (void)led_toggle(&g_led);
  }
}