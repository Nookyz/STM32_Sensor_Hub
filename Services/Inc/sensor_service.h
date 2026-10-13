#ifndef SENSOR_SERVICE_H
#define SENSOR_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  bool aht_ok;           /* Last measurement succeeded, data is valid */
  int32_t aht_temp_c100; /* 0.01 degC */
  uint32_t aht_hum_c100; /* 0.01 %RH */
  uint32_t aht_stamp_ms; /* Tick of the last good sample */
  uint32_t aht_errors;   /* Failures after the sensor was online */
} sensor_data_t;

/** Reset state. The first poll tries to bring the sensors up. */
void sensor_service_init(void);

/** Non-blocking: call from the main loop every iteration. */
void sensor_service_poll(void);

/** Copy the latest snapshot (main loop only, no locking). */
void sensor_service_get(sensor_data_t *out);

#endif /* SENSOR_SERVICE_H */