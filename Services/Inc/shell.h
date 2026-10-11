#ifndef SHELL_H
#define SHELL_H

#include <stddef.h>
#include "status.h"

#define SHELL_LINE_MAX 64u /* Max input line length including NUL */
#define SHELL_ARGS_MAX 4   /* Max tokens per command */

/* Handler gets tokens in argv. Return STATUS_ERR_PARAM to print usage. */
typedef status_t (*ShellHandler)(int argc, char *argv[]);

typedef struct
{
  const char *name;  /* Lowercase command name */
  const char *usage; /* Short usage, e.g. "led on|off|toggle" */
  ShellHandler handler;
} ShellCmd;

/** Register a const command table and print the first prompt.
 *  "help" is built in. */
void shell_init(const ShellCmd *cmds, size_t count);

/** Non-blocking: process a bounded number of received bytes. */
void shell_poll(void);

/** printf-style output to the UART (not reentrant, main loop only). */
void shell_printf(const char *fmt, ...);

#endif /* SHELL_H */