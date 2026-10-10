#ifndef BUTTON_H
#define BUTTON_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "status.h"

typedef struct
{
  /* Configuration: fill before button_init() */
  GPIO_TypeDef *port;
  uint16_t pin;
  bool active_low; /* true: pressed = pin LOW (button to GND) */
  uint32_t debounce_ms;

  /* Internal state: do not touch from application code */
  bool raw_prev;     /* Last raw reading */
  bool stable;       /* Debounced state, true = pressed */
  uint32_t t_change; /* Tick of the last raw change */
} Button;

/** Capture the initial state. Call once after GPIO init. */
status_t button_init(Button *btn);

/** Poll the button. Returns true once per press (on the pressed edge). */
bool button_update(Button *btn);

/** Return the debounced state, true = pressed. */
bool button_is_pressed(const Button *btn);

#endif /* BUTTON_H */