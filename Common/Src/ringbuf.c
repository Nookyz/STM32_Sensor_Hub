#include "ringbuf.h"
#include "main.h" /* __DMB() */

status_t ringbuf_init(RingBuf *rb, uint8_t *storage, uint16_t size)
{
  if ((rb == NULL) || (storage == NULL) || (size < 2u) ||
      ((size & (size - 1u)) != 0u))
  {
    return STATUS_ERR_PARAM;
  }
  rb->buf = storage;
  rb->mask = (uint16_t)(size - 1u);
  rb->head = 0u;
  rb->tail = 0u;
  return STATUS_OK;
}

bool ringbuf_put(RingBuf *rb, uint8_t byte)
{
  const uint16_t head = rb->head;
  const uint16_t next = (uint16_t)((head + 1u) & rb->mask);

  if (next == rb->tail)
  {
    return false; /* Full: one slot stays unused */
  }
  rb->buf[head] = byte;
  __DMB(); /* Data must be visible before the index moves */
  rb->head = next;
  return true;
}

bool ringbuf_get(RingBuf *rb, uint8_t *byte)
{
  const uint16_t tail = rb->tail;

  if (tail == rb->head)
  {
    return false; /* Empty */
  }
  *byte = rb->buf[tail];
  __DMB(); /* Finish the read before freeing the slot */
  rb->tail = (uint16_t)((tail + 1u) & rb->mask);
  return true;
}