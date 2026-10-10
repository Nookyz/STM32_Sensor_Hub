#include "led.h"

static inline bool led_valid(const Led *led)
{
  return (led != NULL) && (led->port != NULL) && (led->pin != 0u);
}

/* Map logical state to pin level without branching on polarity:
 * level is HIGH when (on == active_high), otherwise LOW. */
static inline GPIO_PinState led_level(const Led *led, bool on)
{
  return (on == led->active_high) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

status_t led_init(const Led *led)
{
  /* CubeMX default output level may mean "on" for active-low LEDs */
  return led_off(led);
}

status_t led_on(const Led *led)
{
  if (!led_valid(led))
  {
    return STATUS_ERR_PARAM;
  }
  HAL_GPIO_WritePin(led->port, led->pin, led_level(led, true));
  return STATUS_OK;
}

status_t led_off(const Led *led)
{
  if (!led_valid(led))
  {
    return STATUS_ERR_PARAM;
  }
  HAL_GPIO_WritePin(led->port, led->pin, led_level(led, false));
  return STATUS_OK;
}

status_t led_toggle(const Led *led)
{
  if (!led_valid(led))
  {
    return STATUS_ERR_PARAM;
  }
  /* Polarity-independent: HAL toggles via the output data register */
  HAL_GPIO_TogglePin(led->port, led->pin);
  return STATUS_OK;
}

bool led_is_on(const Led *led)
{
  if (!led_valid(led))
  {
    return false;
  }
  /* IDR reflects the actual pin level, even for push-pull outputs */
  return (HAL_GPIO_ReadPin(led->port, led->pin) == GPIO_PIN_SET) == led->active_high;
}