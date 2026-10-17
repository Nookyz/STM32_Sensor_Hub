#ifndef W25Q128_H
#define W25Q128_H

#include <stdint.h>
#include "status.h"

#define W25Q_PAGE_SIZE 256u    /* Largest single program operation */
#define W25Q_SECTOR_SIZE 4096u /* Smallest erasable unit */

typedef struct
{
  uint8_t manufacturer;    /* 0xEF = Winbond */
  uint8_t mem_type;        /* 0x40 or 0x70 depending on the W25Q128 variant */
  uint8_t capacity_code;   /* 0x18 = 2^24 bytes = 16 MB */
  uint32_t capacity_bytes; /* 0 if the ID was not recognized */
} w25q_info_t;

/** Wake the chip, read and validate the JEDEC ID. Safe to call repeatedly.
 *  If info is not NULL it receives the raw ID even when validation fails.
 *  Returns STATUS_ERR_BUSY (info untouched) while an erase or program runs,
 *  because the chip answers only status reads in that time.
 *  Returns STATUS_ERR_ID if no Winbond flash answers. */
status_t w25q_init(w25q_info_t *info);

/** Capacity found by the last successful w25q_init(), 0 if none. */
uint32_t w25q_capacity(void);

/** Read status register 1. Bit 0 is BUSY (erase or program in progress). */
status_t w25q_read_status(uint8_t *sr1);

/** Poll BUSY until the chip is ready. Always reads the status at least once.
 *  Returns STATUS_ERR_TIMEOUT if it stays busy longer than timeout_ms. */
status_t w25q_wait_ready(uint32_t timeout_ms);

/** Read len bytes starting at addr. The range is validated against the
 *  detected capacity before the bus is touched.
 *  STATUS_ERR_PARAM: NULL, len == 0 or range outside the flash.
 *  STATUS_ERR_ID:    no flash detected, call w25q_init() first.
 *  STATUS_ERR_BUSY:  an erase or program is still running. */
status_t w25q_read(uint32_t addr, uint8_t *buf, uint16_t len);

/** Program len bytes. The target must be erased (0xFF) first: programming
 *  can only clear bits. Splits the data at 256-byte page boundaries and
 *  blocks until done (about 1 ms per page). Same errors as w25q_read(). */
status_t w25q_write(uint32_t addr, const uint8_t *data, uint16_t len);

/** Compare flash contents with expected. STATUS_ERR_VERIFY on mismatch. */
status_t w25q_verify(uint32_t addr, const uint8_t *expected, uint16_t len);

/** STATUS_OK if every byte in the range is 0xFF, STATUS_ERR_VERIFY if not. */
status_t w25q_check_blank(uint32_t addr, uint16_t len);

/** Erase the 4 KB sector at addr (must be sector aligned). Blocks until
 *  done: typically 45 ms, at most 400 ms. */
status_t w25q_sector_erase(uint32_t addr);

/** Start a full chip erase and return immediately. It takes 40 s typically,
 *  200 s at most. Until it finishes, all other operations return
 *  STATUS_ERR_BUSY. */
status_t w25q_chip_erase_start(void);

#endif /* W25Q128_H */