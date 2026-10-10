#ifndef STATUS_H
#define STATUS_H

typedef enum
{
  STATUS_OK = 0,
  STATUS_ERR_PARAM, /* Invalid argument (NULL, out of range) */
  STATUS_ERR_TIMEOUT,
  STATUS_ERR_BUS,
  STATUS_ERR_BUSY,
  STATUS_ERR_ID /* Unexpected device ID */
} status_t;

#endif /* STATUS_H */