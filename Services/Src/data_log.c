#include "data_log.h"
#include "crc16.h"
#include "erase_job.h"

#include <string.h>

/* On-flash record, little endian, 32 bytes. The record is never all 0xFF
 * (version byte), so "blank" always means "not written yet".
 *   0  u32 seq             16  u32 bmp_press_pa
 *   4  u32 stamp_ms        20  10 reserved bytes, zero
 *   8  s16 aht_temp_c100   30  u16 crc16 over bytes 0..29
 *  10  u16 aht_hum_c100
 *  12  s16 bmp_temp_c100
 *  14  u8  flags (bit0 aht ok, bit1 bmp ok)
 *  15  u8  format version */
#define REC_CRC_OFFSET 30u
#define REC_VERSION 1u
#define FLAG_AHT_OK 0x01u
#define FLAG_BMP_OK 0x02u

static inline uint32_t record_addr(uint32_t index)
{
  /* 32-byte records never cross a 256-byte page, so one program is enough */
  return DATA_LOG_START_ADDR + (index * DATA_LOG_RECORD_SIZE);
}

static void put_u16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
  put_u16(p, (uint16_t)v);
  put_u16(p + 2, (uint16_t)(v >> 16));
}

static uint16_t get_u16(const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t get_u32(const uint8_t *p)
{
  return (uint32_t)get_u16(p) | ((uint32_t)get_u16(p + 2) << 16);
}

/* Value ranges (temperature -40..125 C in 0.01 units, humidity 0..10000)
 * fit the 16-bit fields without clipping */
static void encode(uint8_t b[DATA_LOG_RECORD_SIZE], const log_record_t *r)
{
  memset(b, 0, DATA_LOG_RECORD_SIZE);
  put_u32(&b[0], r->seq);
  put_u32(&b[4], r->stamp_ms);
  put_u16(&b[8], (uint16_t)(int16_t)r->aht_temp_c100);
  put_u16(&b[10], (uint16_t)r->aht_hum_c100);
  put_u16(&b[12], (uint16_t)(int16_t)r->bmp_temp_c100);
  b[14] = (uint8_t)((r->aht_ok ? FLAG_AHT_OK : 0u) | (r->bmp_ok ? FLAG_BMP_OK : 0u));
  b[15] = REC_VERSION;
  put_u32(&b[16], r->bmp_press_pa);
  put_u16(&b[REC_CRC_OFFSET], crc16_ccitt(b, REC_CRC_OFFSET));
}

static status_t decode(const uint8_t b[DATA_LOG_RECORD_SIZE], log_record_t *r)
{
  if ((b[15] != REC_VERSION) ||
      (crc16_ccitt(b, REC_CRC_OFFSET) != get_u16(&b[REC_CRC_OFFSET])))
  {
    return STATUS_ERR_CRC;
  }
  r->seq = get_u32(&b[0]);
  r->stamp_ms = get_u32(&b[4]);
  r->aht_temp_c100 = (int16_t)get_u16(&b[8]);
  r->aht_hum_c100 = get_u16(&b[10]);
  r->bmp_temp_c100 = (int16_t)get_u16(&b[12]);
  r->aht_ok = (b[14] & FLAG_AHT_OK) != 0u;
  r->bmp_ok = (b[14] & FLAG_BMP_OK) != 0u;
  r->bmp_press_pa = get_u32(&b[16]);
  return STATUS_OK;
}

static status_t is_blank(uint32_t index, bool *blank)
{
  const status_t st = w25q_check_blank(record_addr(index), DATA_LOG_RECORD_SIZE);
  if (st == STATUS_OK)
  {
    *blank = true;
    return STATUS_OK;
  }
  if (st == STATUS_ERR_VERIFY)
  {
    *blank = false; /* Written, or torn by a power loss */
    return STATUS_OK;
  }
  return st;
}

/* Blank records form a suffix of the area, so the first blank one can be
 * found by bisection. A torn record is not blank and is skipped over. */
static status_t find_end(uint32_t *count)
{
  uint32_t lo = 0u;
  uint32_t hi = DATA_LOG_MAX_RECORDS;

  while (lo < hi)
  {
    const uint32_t mid = lo + ((hi - lo) / 2u);
    bool blank;
    const status_t st = is_blank(mid, &blank);
    if (st != STATUS_OK)
    {
      return st;
    }
    if (blank)
    {
      hi = mid;
    }
    else
    {
      lo = mid + 1u;
    }
  }
  *count = lo;
  return STATUS_OK;
}

status_t data_log_count(uint32_t *count)
{
  if (count == NULL)
  {
    return STATUS_ERR_PARAM;
  }
  return find_end(count);
}

status_t data_log_append(const sensor_data_t *d, uint32_t now_ms, uint32_t *index_out)
{
  if (d == NULL)
  {
    return STATUS_ERR_PARAM;
  }

  uint32_t idx;
  status_t st = find_end(&idx);
  if (st != STATUS_OK)
  {
    return st;
  }
  if (idx >= DATA_LOG_MAX_RECORDS)
  {
    return STATUS_ERR_FULL;
  }

  /* A sensor without data is stored as zeros, the flags tell them apart */
  const log_record_t r = {
      .seq = idx,
      .stamp_ms = now_ms,
      .aht_ok = d->aht_ok,
      .bmp_ok = d->bmp_ok,
      .aht_temp_c100 = d->aht_ok ? d->aht_temp_c100 : 0,
      .aht_hum_c100 = d->aht_ok ? d->aht_hum_c100 : 0u,
      .bmp_temp_c100 = d->bmp_ok ? d->bmp_temp_c100 : 0,
      .bmp_press_pa = d->bmp_ok ? d->bmp_press_pa : 0u,
  };

  uint8_t buf[DATA_LOG_RECORD_SIZE];
  encode(buf, &r);

  const uint32_t addr = record_addr(idx);
  st = w25q_write(addr, buf, DATA_LOG_RECORD_SIZE);
  if (st == STATUS_OK)
  {
    st = w25q_verify(addr, buf, DATA_LOG_RECORD_SIZE);
  }
  if ((st == STATUS_OK) && (index_out != NULL))
  {
    *index_out = idx;
  }
  return st;
}

status_t data_log_read(uint32_t index, log_record_t *out)
{
  if ((out == NULL) || (index >= DATA_LOG_MAX_RECORDS))
  {
    return STATUS_ERR_PARAM;
  }

  uint8_t buf[DATA_LOG_RECORD_SIZE];
  const status_t st = w25q_read(record_addr(index), buf, DATA_LOG_RECORD_SIZE);
  if (st != STATUS_OK)
  {
    return st;
  }
  return decode(buf, out);
}

status_t data_log_clear_start(uint32_t *sectors)
{
  uint32_t count;
  const status_t st = find_end(&count);
  if (st != STATUS_OK)
  {
    return st;
  }

  const uint32_t used = (count == 0u) ? 0u
                                      : (((count - 1u) / DATA_LOG_PER_SECTOR) + 1u);
  if (sectors != NULL)
  {
    *sectors = used;
  }
  if (used == 0u)
  {
    return STATUS_OK;
  }
  return erase_job_start_range(DATA_LOG_START_ADDR, used, true);
}