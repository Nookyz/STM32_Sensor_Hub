#include "shell.h"
#include "uart_port.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define SHELL_RX_BUDGET 32u /* Max bytes handled per poll to bound loop time */
#define SHELL_OUT_MAX 96u   /* Max size of one formatted message */

static const ShellCmd *s_cmds;
static size_t s_count;
static char s_line[SHELL_LINE_MAX];
static uint8_t s_len;
static bool s_overflow; /* Current line exceeded the buffer */
static bool s_prev_cr;  /* Swallow LF that follows CR (CRLF terminals) */

static void out(const char *s)
{
  (void)uart_port_write((const uint8_t *)s, (uint16_t)strlen(s));
}

void shell_printf(const char *fmt, ...)
{
  static char buf[SHELL_OUT_MAX];
  va_list ap;

  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (n > 0)
  {
    uint16_t len = (n < (int)sizeof(buf)) ? (uint16_t)n : (uint16_t)(sizeof(buf) - 1u);
    (void)uart_port_write((const uint8_t *)buf, len);
  }
}

static void prompt(void)
{
  out("> ");
}

static void print_help(void)
{
  out("Commands:\r\n");
  out("  help                 show this list\r\n");
  for (size_t i = 0u; i < s_count; i++)
  {
    shell_printf("  %-20s %s\r\n", s_cmds[i].usage, s_cmds[i].desc);
  }
}

/* Split the line in place on spaces. Returns argc or -1 if too many tokens. */
static int tokenize(char *line, char *argv[], int max)
{
  int argc = 0;
  char *p = line;

  while (*p != '\0')
  {
    while (*p == ' ')
    {
      *p++ = '\0';
    }
    if (*p == '\0')
    {
      break;
    }
    if (argc >= max)
    {
      return -1;
    }
    argv[argc++] = p;
    while ((*p != '\0') && (*p != ' '))
    {
      p++;
    }
  }
  return argc;
}

static void execute_one(char *line)
{
  char *argv[SHELL_ARGS_MAX];

  /* Lowercase once so commands are case-insensitive */
  for (char *p = line; *p != '\0'; p++)
  {
    if ((*p >= 'A') && (*p <= 'Z'))
    {
      *p = (char)(*p + ('a' - 'A'));
    }
  }

  int argc = tokenize(line, argv, SHELL_ARGS_MAX);
  if (argc < 0)
  {
    out("error: too many arguments\r\n");
    return;
  }
  if (argc == 0)
  {
    return;
  }
  if (strcmp(argv[0], "help") == 0)
  {
    print_help();
    return;
  }

  for (size_t i = 0u; i < s_count; i++)
  {
    if (strcmp(argv[0], s_cmds[i].name) == 0)
    {
      status_t st = s_cmds[i].handler(argc, argv);
      if (st == STATUS_ERR_PARAM)
      {
        shell_printf("usage: %s\r\n", s_cmds[i].usage);
      }
      else if (st != STATUS_OK)
      {
        shell_printf("error: code %d\r\n", (int)st);
      }
      return;
    }
  }
  out("unknown command, type 'help'\r\n");
}

/* Run several commands separated by ';' on one line. Each one starts only
 * after the previous handler has returned. */
static void execute(char *line)
{
  char *seg = line;

  for (char *p = line;; p++)
  {
    if ((*p == ';') || (*p == '\0'))
    {
      const bool last = (*p == '\0');
      *p = '\0'; /* Close this segment, the tokenizer stops here */
      execute_one(seg);
      if (last)
      {
        break;
      }
      seg = p + 1;
    }
  }
}

static void on_char(uint8_t c)
{
  if ((c == '\r') || (c == '\n'))
  {
    if ((c == '\n') && s_prev_cr)
    {
      s_prev_cr = false; /* Second half of CRLF */
      return;
    }
    s_prev_cr = (c == '\r');

    out("\r\n");
    if (s_overflow)
    {
      out("error: line too long\r\n");
    }
    else
    {
      s_line[s_len] = '\0';
      execute(s_line);
    }
    s_len = 0u;
    s_overflow = false;
    prompt();
    return;
  }
  s_prev_cr = false;

  if ((c == 0x08u) || (c == 0x7Fu))
  { /* Backspace / DEL */
    if (s_len > 0u)
    {
      s_len--;
      out("\b \b");
    }
    return;
  }

  if ((c >= 0x20u) && (c <= 0x7Eu))
  { /* Printable ASCII only */
    if ((s_len < (SHELL_LINE_MAX - 1u)) && !s_overflow)
    {
      s_line[s_len++] = (char)c;
      (void)uart_port_write(&c, 1u); /* Echo */
    }
    else
    {
      s_overflow = true;
    }
  }
  /* Other control bytes are ignored */
}

void shell_init(const ShellCmd *cmds, size_t count)
{
  s_cmds = cmds;
  s_count = count;
  s_len = 0u;
  s_overflow = false;
  s_prev_cr = false;
  out("\r\nSensor Hub shell. Type 'help'.\r\n");
  prompt();
}

void shell_poll(void)
{
  uint8_t c;
  for (uint32_t i = 0u; i < SHELL_RX_BUDGET; i++)
  {
    if (!uart_port_read(&c))
    {
      break;
    }
    on_char(c);
  }
}