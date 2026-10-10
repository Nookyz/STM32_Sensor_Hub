#ifndef LED_H
#define LED_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "status.h"

/* LED descriptor. Keep instances const so they live in Flash. */
typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
  bool active_high; /* true: pin HIGH turns the LED on */
} Led;

/** Force the LED off. Call once after GPIO init. */
status_t led_init(const Led *led);

status_t led_on(const Led *led);
status_t led_off(const Led *led);
status_t led_toggle(const Led *led);

/** Return true if the LED is currently lit (read from the pin). */
bool led_is_on(const Led *led);

#endif /* LED_H */