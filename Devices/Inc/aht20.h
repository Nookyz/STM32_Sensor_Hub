#ifndef AHT20_H
#define AHT20_H

#include <stdbool.h>
#include <stdint.h>
#include "status.h"

#define AHT20_I2C_ADDR 0x38u

typedef enum
{
  AHT20_IDLE = 0,
  AHT20_MEASURING,
} aht20_state_t;

typedef struct
{
  int32_t temp_c100; /* Temperature in 0.01 degC (2535 = 25.35 C) */
  uint32_t hum_c100; /* Relative humidity in 0.01 %RH */
} aht20_sample_t;

typedef struct
{
  aht20_state_t state;
  uint32_t t_start; /* Tick when the measurement was triggered */
  uint32_t t_next;  /* Tick of the next status check */
} aht20_t;

/** Wait for power-up, check the calibration flag, calibrate if needed.
 *  Blocks up to ~50 ms, so call it at startup or on a rare retry only.
 *  Returns STATUS_ERR_NACK if the device is absent. */
status_t aht20_init(aht20_t *dev);

/** Trigger a measurement. Returns STATUS_ERR_BUSY if one is in progress. */
status_t aht20_start(aht20_t *dev);

/** Advance the state machine without blocking. Sets *ready to true once
 *  when a new sample has been stored in *out. */
status_t aht20_poll(aht20_t *dev, aht20_sample_t *out, bool *ready);

#endif /* AHT20_H */