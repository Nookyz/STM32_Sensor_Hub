#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdbool.h>
#include <stdint.h>
#include "status.h"

/* Lock-free single-producer / single-consumer byte ring buffer.
 * Producer = ISR (put), consumer = main loop (get). */
typedef struct
{
  uint8_t *buf;
  uint16_t mask;          /* size - 1, size must be a power of two */
  volatile uint16_t head; /* Written only by the producer */
  volatile uint16_t tail; /* Written only by the consumer */
} RingBuf;

/** Size must be a power of two (2..32768). */
status_t ringbuf_init(RingBuf *rb, uint8_t *storage, uint16_t size);

/** Returns false if the buffer is full (byte is dropped). */
bool ringbuf_put(RingBuf *rb, uint8_t byte);

/** Returns false if the buffer is empty. */
bool ringbuf_get(RingBuf *rb, uint8_t *byte);

#endif /* RINGBUF_H */