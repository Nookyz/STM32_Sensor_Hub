#include "button.h"

static inline bool btn_valid(const Button *btn)
{
  return (btn != NULL) && (btn->port != NULL) && (btn->pin != 0u);
}

/* Return true when pressed, regardless of wiring polarity:
 * pressed = (pin is HIGH) XOR active_low */
static inline bool read_pressed(const Button *btn)
{
  bool high = (HAL_GPIO_ReadPin(btn->port, btn->pin) == GPIO_PIN_SET);
  return high != btn->active_low;
}

status_t button_init(Button *btn)
{
  if (!btn_valid(btn))
  {
    return STATUS_ERR_PARAM;
  }

  /* Start from the real pin state so a held button at boot
   * does not generate a false press event */
  btn->raw_prev = read_pressed(btn);
  btn->stable = btn->raw_prev;
  btn->t_change = HAL_GetTick();
  return STATUS_OK;
}

bool button_update(Button *btn)
{
  if (!btn_valid(btn))
  {
    return false;
  }

  const bool raw = read_pressed(btn);
  const uint32_t now = HAL_GetTick();

  /* Raw level changed: restart the stability timer */
  if (raw != btn->raw_prev)
  {
    btn->raw_prev = raw;
    btn->t_change = now;
  }

  /* Fast path: nothing to confirm */
  if (raw == btn->stable)
  {
    return false;
  }

  /* Unsigned subtraction stays correct across tick overflow */
  if ((now - btn->t_change) >= btn->debounce_ms)
  {
    btn->stable = raw;
    return raw; /* Event only on the press edge, not on release */
  }
  return false;
}

bool button_is_pressed(const Button *btn)
{
  return btn_valid(btn) && btn->stable;
}