#ifndef DATA_LOG_H
#define DATA_LOG_H

#include <stdbool.h>
#include <stdint.h>
#include "status.h"
#include "w25q128.h"
#include "sensor_service.h"

#define DATA_LOG_RECORD_SIZE 32u
#define DATA_LOG_FIRST_SECTOR 1u /* Sector 0 is left alone */
#define DATA_LOG_SECTORS 16u
#define DATA_LOG_START_ADDR (DATA_LOG_FIRST_SECTOR * W25Q_SECTOR_SIZE)
#define DATA_LOG_PER_SECTOR (W25Q_SECTOR_SIZE / DATA_LOG_RECORD_SIZE)
#define DATA_LOG_MAX_RECORDS (DATA_LOG_SECTORS * DATA_LOG_PER_SECTOR)

typedef struct
{
  uint32_t seq;      /* Record number, equals its index */
  uint32_t stamp_ms; /* Uptime when it was saved */
  bool aht_ok;
  bool bmp_ok;
  int32_t aht_temp_c100; /* 0.01 degC, 0 if the sensor had no data */
  uint32_t aht_hum_c100; /* 0.01 %RH */
  int32_t bmp_temp_c100; /* 0.01 degC */
  uint32_t bmp_press_pa; /* Pascals */
} log_record_t;

/** Number of used records (valid or damaged), found by binary search. */
status_t data_log_count(uint32_t *count);

/** Append one record built from a sensor snapshot. Blocks for a few ms.
 *  STATUS_ERR_FULL when all slots are used. The write is verified. */
status_t data_log_append(const sensor_data_t *d, uint32_t now_ms, uint32_t *index_out);

/** Read record number index. STATUS_ERR_CRC for a damaged record. */
status_t data_log_read(uint32_t index, log_record_t *out);

/** Erase every used sector, last one first. Blocks about 100 ms per sector. */
status_t data_log_clear(uint32_t *sectors_erased);

#endif /* DATA_LOG_H */