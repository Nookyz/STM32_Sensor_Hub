#ifndef STATUS_H
#define STATUS_H

typedef enum
{
  STATUS_OK = 0,
  STATUS_ERR_PARAM, /* Invalid argument (NULL, out of range) */
  STATUS_ERR_TIMEOUT,
  STATUS_ERR_BUS, /* Bus error or arbitration lost */
  STATUS_ERR_BUSY,
  STATUS_ERR_ID,  /* Unexpected device ID */
  STATUS_ERR_NACK /* Device did not acknowledge */
} status_t;

#endif /* STATUS_H */