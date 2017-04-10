#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "samd/usb_samd.h"

#define BUFFER_SIZE 256

#define STREAM_QUEUE_SIZE 8
#define STREAM_QUEUE_SIZE_MASK 0x07
#define STREAM_MIN_LENGTH_START 4

struct stream_item {
    USB_ALIGN uint8_t buffer[BUFFER_SIZE];
};
struct stream_queue {
    USB_ALIGN struct stream_item items[STREAM_QUEUE_SIZE];
    uint32_t head;
    uint32_t tail;
    struct stream_item *head_item;
    struct stream_item *tail_item;
    bool running;
};

void stream_queue_init_module(void);
void stream_queue_init(struct stream_queue *);

static inline uint8_t stream_queue_length(struct stream_queue *queue) {
    return queue->head - queue->tail;
}

void stream_enqueue_prep_begin(struct stream_queue *);
static inline struct stream_item * stream_enqueue_exec_begin(struct stream_queue *queue) {
    return queue->head_item;
}

struct stream_item * stream_enqueue_begin(struct stream_queue *);
void stream_enqueue_end(struct stream_queue *);

void stream_dequeue_prep_begin(struct stream_queue *);
static inline struct stream_item * stream_dequeue_exec_begin(struct stream_queue *queue) {
    return queue->tail_item;
}

struct stream_item * stream_dequeue_begin(struct stream_queue *);
void stream_dequeue_end(struct stream_queue *);

// struct stream_queue_spi {
//   struct stream_queue *queue;
//   uint8_t dma;
//   uint8_t sercom;
//   uint8_t length;
// };
//
// void stream_spi_start_rx(struct stream_queue_spi *);
// void stream_spi_end_rx(struct stream_queue_spi *);
// void stream_spi_start_tx(struct stream_queue_spi *);
// void stream_spi_end_tx(struct stream_queue_spi *);
