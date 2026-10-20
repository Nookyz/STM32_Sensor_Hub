#ifndef ERASE_JOB_H
#define ERASE_JOB_H

#include <stdbool.h>
#include <stdint.h>
#include "status.h"

typedef struct
{
  bool chip;       /* true: whole chip, false: a run of sectors */
  status_t status; /* STATUS_OK, or the error that stopped the job */
  uint32_t sectors_done;
  uint32_t sectors_total; /* 0 for a chip erase */
  uint32_t elapsed_ms;
} erase_result_t;

typedef struct
{
  bool active;
  bool chip;
  uint32_t sectors_done;
  uint32_t sectors_total;
  uint32_t elapsed_ms;
} erase_progress_t;

/* Called from erase_job_poll() (main loop context, not an ISR) when a job ends */
typedef void (*erase_done_cb_t)(const erase_result_t *result);

/** Reset the service. on_done may be NULL. */
void erase_job_init(erase_done_cb_t on_done);

/** Erase count sectors from first_addr (sector aligned) in the background.
 *  With descending set, the highest sector goes first, so an interrupted job
 *  leaves a contiguous prefix of data. The first erase command is sent before
 *  this function returns, so a bad range or a busy chip is reported here.
 *  STATUS_ERR_BUSY if a job is already running. */
status_t erase_job_start_range(uint32_t first_addr, uint32_t count, bool descending);

/** Start a full chip erase (40 s typical, 200 s at most) in the background. */
status_t erase_job_start_chip(void);

bool erase_job_active(void);
void erase_job_get_progress(erase_progress_t *out);

/** Advance the job without blocking. Call from the scheduler every ~5 ms. */
void erase_job_poll(void);

#endif /* ERASE_JOB_H */