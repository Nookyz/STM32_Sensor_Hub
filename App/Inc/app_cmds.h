#ifndef APP_CMDS_H
#define APP_CMDS_H

#include "status.h"
#include "led.h"

/** Register shell commands and start the shell.
 *  Call after uart_port_init(). The led pointer must stay valid forever
 *  (pass a static object). */
status_t app_cmds_init(const Led *led);

#endif /* APP_CMDS_H */