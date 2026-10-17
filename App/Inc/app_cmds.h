#ifndef APP_CMDS_H
#define APP_CMDS_H

#include "status.h"
#include "led.h"

/** Register shell commands and start the shell.
 *  Call after uart_port_init(). The led pointer must stay valid forever
 *  (pass a static object). */
status_t app_cmds_init(const Led *led);

/** Advance background work started by shell commands (flash scan).
 *  Non-blocking, call it from the main loop on every iteration. */
void app_cmds_poll(void);

#endif /* APP_CMDS_H */