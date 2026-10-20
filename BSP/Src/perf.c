#include "perf.h"

#include <stdbool.h>

static uint32_t s_prev;
static bool s_have_prev;
static uint32_t s_max;   /* Cycles */
static uint64_t s_total; /* Cycles, 64 bit so it never overflows in practice */
static uint32_t s_passes;

void perf_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* Trace block on */
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t perf_cycles_to_us(uint32_t cycles)
{
  return (uint32_t)(((uint64_t)cycles * 1000000u) / SystemCoreClock);
}

void perf_loop_tick(void)
{
  const uint32_t now = DWT->CYCCNT;

  if (s_have_prev)
  {
    const uint32_t d = now - s_prev; /* Correct across counter wrap */
    if (d > s_max)
    {
      s_max = d;
    }
    s_total += d;
    s_passes++;
  }
  s_prev = now;
  s_have_prev = true;
}

void perf_loop_get(perf_loop_t *out)
{
  if (out == NULL)
  {
    return;
  }
  out->max_us = perf_cycles_to_us(s_max);
  out->passes = s_passes;

  /* Divide first: total * 1e7 would overflow 64 bits after about 6 hours */
  out->avg_x10_us = 0u;
  if (s_passes > 0u)
  {
    const uint32_t avg_cycles = (uint32_t)(s_total / s_passes);
    out->avg_x10_us = (uint32_t)(((uint64_t)avg_cycles * 10000000u) / SystemCoreClock);
  }
}

void perf_reset(void)
{
  s_max = 0u;
  s_total = 0u;
  s_passes = 0u;
}