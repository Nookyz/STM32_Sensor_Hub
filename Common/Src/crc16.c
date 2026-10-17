#include "crc16.h"

uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFFu;

  if (data == NULL)
  {
    return crc;
  }
  while (len-- > 0u)
  {
    crc ^= (uint16_t)((uint16_t)*data++ << 8);
    for (uint8_t i = 0u; i < 8u; i++)
    {
      crc = ((crc & 0x8000u) != 0u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                    : (uint16_t)(crc << 1);
    }
  }
  return crc;
}