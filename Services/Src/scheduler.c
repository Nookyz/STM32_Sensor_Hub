#include "scheduler.h"
#include "perf.h"
#include "main.h"

#include <string.h>

typedef struct
{
  uint32_t next; /* Tick of the next run */
  uint32_t runs;
  uint64_t total_cycles; /* 64 bit: the sum would overflow 32 bits within a minute */
  uint32_t max_cycles;
  uint32_t max_late_ms;
  uint32_t resyncs;
} task_state_t;

static const sched_task_t *s_tasks;
static size_t s_count;
static task_state_t s_state[SCHED_MAX_TASKS];

status_t sched_init(const sched_task_t *tasks, size_t count)
{
  if ((tasks == NULL) || (count == 0u) || (count > SCHED_MAX_TASKS))
  {
    return STATUS_ERR_PARAM;
  }
  for (size_t i = 0u; i < count; i++)
  {
    if (tasks[i].fn == NULL)
    {
      return STATUS_ERR_PARAM;
    }
  }

  s_tasks = tasks;
  s_count = count;
  memset(s_state, 0, sizeof(s_state));

  const uint32_t now = HAL_GetTick();
  for (size_t i = 0u; i < count; i++)
  {
    s_state[i].next = now;
  }
  return STATUS_OK;
}

void sched_run(void)
{
  for (size_t i = 0u; i < s_count; i++)
  {
    const sched_task_t *t = &s_tasks[i];
    task_state_t *s = &s_state[i];

    if (t->period_ms != 0u)
    {
      /* Read the tick for every task: an earlier task may have run long */
      const uint32_t now = HAL_GetTick();
      if ((int32_t)(now - s->next) < 0)
      {
        continue; /* Not due yet, wrap-safe compare */
      }

      const uint32_t late = now - s->next;
      if (late > s->max_late_ms)
      {
        s->max_late_ms = late;
      }

      /* Advance from the planned time, not from "now", so the period
       * does not drift. If a whole period was missed, restart from now
       * instead of running the task in a burst to catch up. */
      s->next += t->period_ms;
      if ((int32_t)(now - s->next) >= 0)
      {
        s->resyncs++;
        s->next = now + t->period_ms;
      }
    }

    const uint32_t t0 = perf_cycles();
    t->fn();
    const uint32_t dt = perf_cycles() - t0;

    s->runs++;
    s->total_cycles += dt;
    if (dt > s->max_cycles)
    {
      s->max_cycles = dt;
    }
  }
}

size_t sched_count(void)
{
  return s_count;
}

status_t sched_get_info(size_t index, sched_info_t *out)
{
  if ((out == NULL) || (index >= s_count))
  {
    return STATUS_ERR_PARAM;
  }

  const task_state_t *s = &s_state[index];

  out->name = s_tasks[index].name;
  out->period_ms = s_tasks[index].period_ms;
  out->runs = s->runs;
  out->max_run_us = perf_cycles_to_us(s->max_cycles);
  out->max_late_ms = s->max_late_ms;
  out->resyncs = s->resyncs;

  out->avg_x10_us = 0u;
  if (s->runs > 0u)
  {
    const uint32_t avg_cycles = (uint32_t)(s->total_cycles / s->runs);
    out->avg_x10_us = (uint32_t)(((uint64_t)avg_cycles * 10000000u) / SystemCoreClock);
  }
  return STATUS_OK;
}

void sched_reset_stats(void)
{
  for (size_t i = 0u; i < s_count; i++)
  {
    s_state[i] = (task_state_t){.next = s_state[i].next};
  }
}