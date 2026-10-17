#ifndef CRC16_H
#define CRC16_H

#include <stddef.h>
#include <stdint.h>

/** CRC-16/CCITT-FALSE: polynomial 0x1021, init 0xFFFF, no reflection, no
 *  final XOR. Check value: the ASCII string "123456789" gives 0x29B1. */
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

#endif /* CRC16_H */