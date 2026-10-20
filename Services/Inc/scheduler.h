#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stddef.h>
#include <stdint.h>
#include "status.h"

#define SCHED_MAX_TASKS 8u

typedef void (*sched_fn_t)(void);

/* One table row. Keep the table const so it stays in Flash. */
typedef struct
{
  const char *name;   /* Up to 7 characters, shown by 'tasks' */
  uint32_t period_ms; /* 0 = run on every pass of the main loop */
  sched_fn_t fn;      /* Must return quickly: the scheduler is cooperative */
} sched_task_t;

typedef struct
{
  const char *name;
  uint32_t period_ms;
  uint32_t runs;
  uint32_t avg_x10_us;  /* Average run time in 0.1 us units */
  uint32_t max_run_us;  /* Longest single run */
  uint32_t max_late_ms; /* Largest delay behind the schedule (1 ms resolution) */
  uint32_t resyncs;     /* Times a whole period was missed and the schedule restarted */
} sched_info_t;

/** Register a task table and schedule every task for an immediate first run.
 *  The table must stay valid forever (pass a static const array). */
status_t sched_init(const sched_task_t *tasks, size_t count);

/** Run every task that is due. Call on every pass of the main loop. */
void sched_run(void);

size_t sched_count(void);

/** Fill in the statistics of task number index. */
status_t sched_get_info(size_t index, sched_info_t *out);

/** Clear run-time statistics, the schedule itself is kept. */
void sched_reset_stats(void);

#endif /* SCHEDULER_H */