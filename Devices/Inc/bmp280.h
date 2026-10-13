#ifndef BMP280_H
#define BMP280_H

#include <stdbool.h>
#include <stdint.h>
#include "status.h"

#define BMP280_ADDR_SDO_LOW 0x76u  /* SDO tied to GND */
#define BMP280_ADDR_SDO_HIGH 0x77u /* SDO tied to VCC */

typedef enum
{
  BMP280_IDLE = 0,
  BMP280_MEASURING,
} bmp280_state_t;

/* Factory trim values read from the sensor NVM (datasheet naming) */
typedef struct
{
  uint16_t T1;
  int16_t T2, T3;
  uint16_t P1;
  int16_t P2, P3, P4, P5, P6, P7, P8, P9;
} bmp280_calib_t;

typedef struct
{
  int32_t temp_c100; /* 0.01 degC (2535 = 25.35 C) */
  uint32_t press_pa; /* Pascals (100812 = 1008.12 hPa) */
} bmp280_sample_t;

typedef struct
{
  bmp280_calib_t calib;
  uint8_t addr; /* 7-bit I2C address */
  bmp280_state_t state;
  uint32_t t_start; /* Tick when the measurement was triggered */
  uint32_t t_next;  /* Tick of the next status check */
} bmp280_t;

/** Check the chip ID, reset, load calibration and configure the filter.
 *  Blocks for roughly 3-13 ms, so call it at startup or on a rare retry.
 *  Returns STATUS_ERR_NACK if absent, STATUS_ERR_ID if the ID is wrong. */
status_t bmp280_init(bmp280_t *dev, uint8_t addr7);

/** Trigger one forced-mode measurement. STATUS_ERR_BUSY if one is running. */
status_t bmp280_start(bmp280_t *dev);

/** Advance the state machine without blocking. Sets *ready to true once
 *  when a new sample has been stored in *out. */
status_t bmp280_poll(bmp280_t *dev, bmp280_sample_t *out, bool *ready);

#endif /* BMP280_H */