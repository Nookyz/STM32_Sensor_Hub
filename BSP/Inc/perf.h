#ifndef PERF_H
#define PERF_H

#include <stdint.h>
#include "main.h"

typedef struct
{
  uint32_t max_us;     /* Longest main-loop pass since reset */
  uint32_t avg_x10_us; /* Average pass in 0.1 us units */
  uint32_t passes;     /* Passes counted (wraps after days, use perf reset) */
} perf_loop_t;

/** Enable the DWT cycle counter. Call once at startup. */
void perf_init(void);

/** CPU cycle counter, wraps every ~51 s at 84 MHz. Use unsigned subtraction
 *  to get an interval. */
static inline uint32_t perf_cycles(void)
{
  return DWT->CYCCNT;
}

/** Convert a cycle count to microseconds using the real core clock. */
uint32_t perf_cycles_to_us(uint32_t cycles);

/** Call once per main-loop pass, always at the same place. The time between
 *  two calls is one pass, so a blocking call anywhere shows up here. */
void perf_loop_tick(void);

void perf_loop_get(perf_loop_t *out);
void perf_reset(void);

#endif /* PERF_H */