#ifndef W25Q128_H
#define W25Q128_H

#include <stdint.h>
#include "status.h"

typedef struct
{
  uint8_t manufacturer;    /* 0xEF = Winbond */
  uint8_t mem_type;        /* 0x40 or 0x70 depending on the W25Q128 variant */
  uint8_t capacity_code;   /* 0x18 = 2^24 bytes = 16 MB */
  uint32_t capacity_bytes; /* 0 if the ID was not recognized */
} w25q_info_t;

/** Wake the chip, read and validate the JEDEC ID. Safe to call repeatedly.
 *  If info is not NULL it receives the raw ID even when validation fails,
 *  so the caller can print it. Blocks for about 2-12 ms.
 *  Returns STATUS_ERR_ID if no Winbond flash answers. */
status_t w25q_init(w25q_info_t *info);

/** Capacity found by the last successful w25q_init(), 0 if none. */
uint32_t w25q_capacity(void);

#endif /* W25Q128_H */