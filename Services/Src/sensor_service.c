#include "sensor_service.h"
#include "aht20.h"
#include "main.h"

#include <string.h>

#define SAMPLE_PERIOD_MS 1000u /* Pause between measurements */
#define RETRY_PERIOD_MS 5000u  /* Re-init interval for an offline sensor */

typedef enum
{
  AHT_OFFLINE = 0, /* Not initialized or failed, retry periodically */
  AHT_IDLE,        /* Waiting for the next sample time */
  AHT_BUSY,        /* Measurement in progress */
} aht_state_t;

static aht20_t s_aht;
static aht_state_t s_aht_state;
static uint32_t s_aht_next; /* Tick of the next action */
static sensor_data_t s_data;

/* Wrap-safe "deadline reached" check */
static inline bool due(uint32_t now, uint32_t deadline)
{
  return (int32_t)(now - deadline) >= 0;
}

/* A sensor that worked has failed: drop its data and schedule a re-init */
static void aht_fail(uint32_t now)
{
  s_data.aht_ok = false;
  s_data.aht_errors++;
  s_aht_state = AHT_OFFLINE;
  s_aht_next = now + RETRY_PERIOD_MS;
}

static void aht_step(void)
{
  const uint32_t now = HAL_GetTick();

  switch (s_aht_state)
  {
  case AHT_OFFLINE:
    if (!due(now, s_aht_next))
    {
      return;
    }
    if (aht20_init(&s_aht) == STATUS_OK)
    {
      s_aht_state = AHT_IDLE;
      s_aht_next = now;
    }
    else
    {
      s_aht_next = now + RETRY_PERIOD_MS; /* Not counted as a new error */
    }
    break;

  case AHT_IDLE:
    if (!due(now, s_aht_next))
    {
      return;
    }
    if (aht20_start(&s_aht) == STATUS_OK)
    {
      s_aht_state = AHT_BUSY;
    }
    else
    {
      aht_fail(now);
    }
    break;

  case AHT_BUSY:
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
      s_aht_state = AHT_IDLE;
      s_aht_next = now + SAMPLE_PERIOD_MS;
    }
    break;
  }
  }
}

void sensor_service_init(void)
{
  memset(&s_data, 0, sizeof(s_data));
  s_aht_state = AHT_OFFLINE;
  s_aht_next = HAL_GetTick(); /* Try to bring the sensor up right away */
}

void sensor_service_poll(void)
{
  aht_step();
}

void sensor_service_get(sensor_data_t *out)
{
  if (out != NULL)
  {
    *out = s_data;
  }
}