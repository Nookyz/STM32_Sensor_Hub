#include "sensor_service.h"
#include "aht20.h"
#include "bmp280.h"
#include "main.h"

#include <string.h>

#define SAMPLE_PERIOD_MS 1000u        /* Pause between measurements */
#define RETRY_PERIOD_MS 5000u         /* Re-init interval for an offline sensor */
#define BMP_ADDR BMP280_ADDR_SDO_HIGH /* Matches the module wiring */

typedef enum
{
  SENS_OFFLINE = 0, /* Not initialized or failed, retry periodically */
  SENS_IDLE,        /* Waiting for the next sample time */
  SENS_BUSY,        /* Measurement in progress */
} sens_state_t;

static aht20_t s_aht;
static sens_state_t s_aht_state;
static uint32_t s_aht_next; /* Tick of the next action */

static bmp280_t s_bmp;
static sens_state_t s_bmp_state;
static uint32_t s_bmp_next;

static sensor_data_t s_data;

/* Wrap-safe "deadline reached" check */
static inline bool due(uint32_t now, uint32_t deadline)
{
  return (int32_t)(now - deadline) >= 0;
}

/* ---- AHT20 ---- */

/* A sensor that worked has failed: drop its data and schedule a re-init */
static void aht_fail(uint32_t now)
{
  s_data.aht_ok = false;
  s_data.aht_errors++;
  s_aht_state = SENS_OFFLINE;
  s_aht_next = now + RETRY_PERIOD_MS;
}

static void aht_step(void)
{
  const uint32_t now = HAL_GetTick();

  switch (s_aht_state)
  {
  case SENS_OFFLINE:
    if (!due(now, s_aht_next))
    {
      return;
    }
    if (aht20_init(&s_aht) == STATUS_OK)
    {
      s_aht_state = SENS_IDLE;
      s_aht_next = now;
    }
    else
    {
      s_aht_next = now + RETRY_PERIOD_MS; /* Not counted as a new error */
    }
    break;

  case SENS_IDLE:
    if (!due(now, s_aht_next))
    {
      return;
    }
    if (aht20_start(&s_aht) == STATUS_OK)
    {
      s_aht_state = SENS_BUSY;
    }
    else
    {
      aht_fail(now);
    }
    break;

  case SENS_BUSY:
  {
    aht20_sample_t sample;
    bool ready;
    if (aht20_poll(&s_aht, &sample, &ready) != STATUS_OK)
    {
      aht_fail(now);
    }
    else if (ready)
    {
      s_data.aht_temp_c100 = sample.temp_c100;
      s_data.aht_hum_c100 = sample.hum_c100;
      s_data.aht_stamp_ms = now;
      s_data.aht_ok = true;
      s_aht_state = SENS_IDLE;
      s_aht_next = now + SAMPLE_PERIOD_MS;
    }
    break;
  }
  }
}

/* ---- BMP280 ---- */

static void bmp_fail(uint32_t now)
{
  s_data.bmp_ok = false;
  s_data.bmp_errors++;
  s_bmp_state = SENS_OFFLINE;
  s_bmp_next = now + RETRY_PERIOD_MS;
}

static void bmp_step(void)
{
  const uint32_t now = HAL_GetTick();

  switch (s_bmp_state)
  {
  case SENS_OFFLINE:
    if (!due(now, s_bmp_next))
    {
      return;
    }
    if (bmp280_init(&s_bmp, BMP_ADDR) == STATUS_OK)
    {
      s_bmp_state = SENS_IDLE;
      s_bmp_next = now;
    }
    else
    {
      s_bmp_next = now + RETRY_PERIOD_MS;
    }
    break;

  case SENS_IDLE:
    if (!due(now, s_bmp_next))
    {
      return;
    }
    if (bmp280_start(&s_bmp) == STATUS_OK)
    {
      s_bmp_state = SENS_BUSY;
    }
    else
    {
      bmp_fail(now);
    }
    break;

  case SENS_BUSY:
  {
    bmp280_sample_t sample;
    bool ready;
    if (bmp280_poll(&s_bmp, &sample, &ready) != STATUS_OK)
    {
      bmp_fail(now);
    }
    else if (ready)
    {
      s_data.bmp_temp_c100 = sample.temp_c100;
      s_data.bmp_press_pa = sample.press_pa;
      s_data.bmp_stamp_ms = now;
      s_data.bmp_ok = true;
      s_bmp_state = SENS_IDLE;
      s_bmp_next = now + SAMPLE_PERIOD_MS;
    }
    break;
  }
  }
}

/* ---- Public API ---- */

void sensor_service_init(void)
{
  memset(&s_data, 0, sizeof(s_data));
  const uint32_t now = HAL_GetTick();

  s_aht_state = SENS_OFFLINE;
  s_aht_next = now; /* Try to bring the sensors up right away */
  s_bmp_state = SENS_OFFLINE;
  s_bmp_next = now;
}

void sensor_service_poll(void)
{
  /* Each sensor has its own state machine, so one failing sensor
   * never stops the other */
  aht_step();
  bmp_step();
}

void sensor_service_get(sensor_data_t *out)
{
  if (out != NULL)
  {
    *out = s_data;
  }
}