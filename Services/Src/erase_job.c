#include "erase_job.h"
#include "w25q128.h"
#include "main.h"

#define SECTOR_TIMEOUT_MS 500u  /* Datasheet maximum is 400 ms */
#define CHIP_TIMEOUT_MS 250000u /* Datasheet maximum is 200 s */
#define SR1_BUSY 0x01u

typedef struct
{
  bool active;
  bool chip;
  bool descending;     /* Erase the highest sector first */
  uint32_t first_addr; /* Lowest address of the range */
  uint32_t total;      /* Sectors in the job, 0 for a chip erase */
  uint32_t done;       /* Sectors finished */
  uint32_t t_job;      /* Tick at the start of the job */
  uint32_t t_step;     /* Tick when the current erase command was sent */
} job_t;

static job_t s_job;
static erase_done_cb_t s_on_done;

static uint32_t sector_addr(const job_t *j, uint32_t n)
{
  const uint32_t idx = j->descending ? (j->total - 1u - n) : n;
  return j->first_addr + (idx * W25Q_SECTOR_SIZE);
}

static void finish(status_t st)
{
  const erase_result_t r = {
      .chip = s_job.chip,
      .status = st,
      .sectors_done = s_job.done,
      .sectors_total = s_job.total,
      .elapsed_ms = HAL_GetTick() - s_job.t_job,
  };

  s_job.active = false; /* Before the callback: it may start the next job */
  if (s_on_done != NULL)
  {
    s_on_done(&r);
  }
}

void erase_job_init(erase_done_cb_t on_done)
{
  s_job = (job_t){0};
  s_on_done = on_done;
}

status_t erase_job_start_range(uint32_t first_addr, uint32_t count, bool descending)
{
  if (s_job.active)
  {
    return STATUS_ERR_BUSY;
  }
  if ((count == 0u) || ((first_addr % W25Q_SECTOR_SIZE) != 0u))
  {
    return STATUS_ERR_PARAM;
  }

  const uint32_t cap = w25q_capacity();
  if (cap == 0u)
  {
    return STATUS_ERR_ID;
  }
  /* Written as a division so that the end address can never overflow */
  if ((first_addr >= cap) || (count > ((cap - first_addr) / W25Q_SECTOR_SIZE)))
  {
    return STATUS_ERR_PARAM;
  }

  s_job = (job_t){
      .descending = descending,
      .first_addr = first_addr,
      .total = count,
      .t_job = HAL_GetTick(),
  };

  const status_t st = w25q_sector_erase_start(sector_addr(&s_job, 0u));
  if (st != STATUS_OK)
  {
    return st;
  }
  s_job.t_step = HAL_GetTick();
  s_job.active = true;
  return STATUS_OK;
}

status_t erase_job_start_chip(void)
{
  if (s_job.active)
  {
    return STATUS_ERR_BUSY;
  }

  s_job = (job_t){.chip = true, .t_job = HAL_GetTick()};

  const status_t st = w25q_chip_erase_start();
  if (st != STATUS_OK)
  {
    return st;
  }
  s_job.t_step = s_job.t_job;
  s_job.active = true;
  return STATUS_OK;
}

bool erase_job_active(void)
{
  return s_job.active;
}

void erase_job_get_progress(erase_progress_t *out)
{
  if (out == NULL)
  {
    return;
  }
  out->active = s_job.active;
  out->chip = s_job.chip;
  out->sectors_done = s_job.done;
  out->sectors_total = s_job.total;
  out->elapsed_ms = s_job.active ? (HAL_GetTick() - s_job.t_job) : 0u;
}

void erase_job_poll(void)
{
  if (!s_job.active)
  {
    return;
  }

  uint8_t sr;
  status_t st = w25q_read_status(&sr);
  if (st != STATUS_OK)
  {
    finish(st);
    return;
  }
  if (sr == 0xFFu)
  {
    finish(STATUS_ERR_BUS); /* MISO floats high: the chip is gone */
    return;
  }

  if ((sr & SR1_BUSY) != 0u)
  {
    const uint32_t limit = s_job.chip ? CHIP_TIMEOUT_MS : SECTOR_TIMEOUT_MS;
    if ((HAL_GetTick() - s_job.t_step) > limit)
    {
      finish(STATUS_ERR_TIMEOUT);
    }
    return;
  }

  /* The current erase has finished */
  if (s_job.chip)
  {
    finish(STATUS_OK);
    return;
  }
  s_job.done++;
  if (s_job.done >= s_job.total)
  {
    finish(STATUS_OK);
    return;
  }

  st = w25q_sector_erase_start(sector_addr(&s_job, s_job.done));
  if (st != STATUS_OK)
  {
    finish(st);
    return;
  }
  s_job.t_step = HAL_GetTick();
}