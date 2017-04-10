#include "stream.h"

struct stream_item dummy_item;
struct stream_item empty_item;

void stream_queue_init_module() {
    memset(&empty_item, 0, sizeof(dummy_item));
    memset(&empty_item, 0, sizeof(empty_item));
}

void stream_queue_init(struct stream_queue *queue) {
    memset(queue, 0, sizeof(*queue));
    *queue = (struct stream_queue) {
        .items = {},
        .head = 0,
        .tail = 0,
        .running = false,
    };
}

void stream_enqueue_prep_begin(struct stream_queue *queue) {
    if (stream_queue_length(queue) >= STREAM_QUEUE_SIZE) {
        queue->tail += 1;
    }
    queue->head_item = &queue->items[queue->head & STREAM_QUEUE_SIZE_MASK];
}

struct stream_item * stream_enqueue_begin(struct stream_queue *queue) {
    stream_enqueue_prep_begin(queue);
    return stream_enqueue_exec_begin(queue);
}

void stream_enqueue_end(struct stream_queue *queue) {
    queue->head += 1;
    if (stream_queue_length(queue) > STREAM_QUEUE_SIZE) {
        queue->tail += 1;
    }
}

void stream_dequeue_prep_begin(struct stream_queue *queue) {
    uint32_t length = stream_queue_length(queue);
    if (length >= STREAM_MIN_LENGTH_START) {
        queue->running = true;
    }
    else if (length == 0) {
        queue->running = false;
    }

    if (queue->running) {
        queue->tail_item = &queue->items[queue->tail & STREAM_QUEUE_SIZE_MASK];
    }
    else {
        queue->tail_item = &empty_item;
    }
}

struct stream_item * stream_dequeue_begin(struct stream_queue *queue) {
    stream_dequeue_prep_begin(queue);
    return stream_dequeue_exec_begin(queue);
}

void stream_dequeue_end(struct stream_queue *queue) {
    if (queue->running) {
        queue->tail += 1;
    }
}
