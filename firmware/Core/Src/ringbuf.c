#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "ringbuf.h"

void ring_init (ringbuf_t *rb, uint8_t *storage, uint16_t size)
{
	rb->buf = storage;
	rb->size = size;
	rb->head = 0;
	rb->tail = 0;
}

uint16_t ring_count  (ringbuf_t *rb)
{
	return (rb->head + rb->size - rb->tail) & (rb->size - 1);
}

bool ring_is_full(ringbuf_t *rb)
{

	return ((rb->head+1)&(rb->size -1)) == rb->tail;
}

bool ring_put (ringbuf_t *rb, uint8_t byte)
{
	if(ring_is_full(rb))	return false;
	rb->buf[rb->head] = byte;
	rb->head = (rb->head+1)&(rb->size-1);
	return true;
}

bool ring_get (ringbuf_t *rb, uint8_t *byte)
{
	if(rb->head == rb->tail) return false;
	else
	{
		*byte = rb->buf[rb->tail];
		rb->tail = (rb->tail+1)&(rb->size-1);
		return true;
	}
}




