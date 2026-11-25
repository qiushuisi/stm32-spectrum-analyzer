#ifndef __RINGBUF_H
#define __RINGBUF_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t          *buf;     // 外部给的存储区
    uint16_t          size;    // 必须是 2 的幂
    volatile uint16_t head;    // 写指针，只有生产者改
    volatile uint16_t tail;    // 读指针，只有消费者改
} ringbuf_t;

void     ring_init   (ringbuf_t *rb, uint8_t *storage, uint16_t size);
bool     ring_put    (ringbuf_t *rb, uint8_t byte);
bool     ring_get    (ringbuf_t *rb, uint8_t *byte);
uint16_t ring_count  (ringbuf_t *rb);
bool     ring_is_full(ringbuf_t *rb);

#endif
