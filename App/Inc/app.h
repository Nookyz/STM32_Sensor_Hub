#ifndef APP_H
#define APP_H

/** Initialize application modules. Call once after HAL/BSP init. */
void app_init(void);

/** Run one non-blocking application cycle. Call from the main loop. */
void app_loop(void);

#endif /* APP_H */