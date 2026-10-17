#ifndef FLASH_SELFTEST_H
#define FLASH_SELFTEST_H

#include <stdint.h>
#include "status.h"

typedef struct
{
  uint32_t test_addr;      /* Sector used by the test */
  uint32_t erase_ms;       /* First sector erase */
  uint32_t write_ms;       /* Programming of the test pattern */
  uint32_t verify_ms;      /* Read-back of the test pattern */
  uint16_t test_len;       /* Pattern length in bytes */
  const char *failed_step; /* NULL when every step passed */
  status_t status;         /* Error code of the failed step */
} flash_selftest_t;

/** Address of the sector the test uses (the last one), 0 if no flash
 *  was detected. */
uint32_t flash_selftest_addr(void);

/** Run the self-test. DESTROYS the contents of that sector. Blocks for
 *  well under a second. Returns STATUS_OK if every step passed, otherwise
 *  the error of the first failed step, named in res->failed_step. */
status_t flash_selftest_run(flash_selftest_t *res);

#endif /* FLASH_SELFTEST_H */