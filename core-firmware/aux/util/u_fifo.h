#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifndef MIN
#define MIN(a, b) (a < b ? a : b)
#endif

// generic ring buffer //

// the fifo is implemented with two isolated safety domains: the consumer and the producer
// it is safe to enter both of these domains concurently but entering one domain concurently
// is unsafe.

struct u_fifo {
    uint8_t *buf;
    uint32_t buf_size;
    uint32_t head, tail;
};

// returns the number of bytes currently free in a ring buffer.
static inline uint32_t fifo_get_free(const struct u_fifo *f) {
    return (f->head < f->tail) ? (f->tail - f->head) : f->buf_size - (f->head - f->tail);
}

// returns the number of available bytes stored in the ring buffer.
static inline uint32_t fifo_get_available(const struct u_fifo *f) {
    return f->buf_size - fifo_get_free(f);
}

static void fifo_acquire_unsafe(struct u_fifo *f, uint32_t size) {
    uint32_t until_wrap = f->buf_size - f->head;
    f->head = size >= until_wrap ? size - until_wrap : f->head + size;
}

// try to reserve a set amount of bytes from the free range. Returns false if not enough space is
// available, true otherwise.
static bool fifo_acquire(struct u_fifo *f, uint32_t size) {
    // check if range fits in
    if (fifo_get_free(f) < size)
        return false;

    fifo_acquire_unsafe(f, size);
    return true;
}

static void fifo_release_unsafe(struct u_fifo *f, uint32_t size) {
    uint32_t until_wrap = f->buf_size - f->tail;
    f->tail = size >= until_wrap ? size - until_wrap : f->tail + size;
}

// try to free a set amount of bytes from the fifo, discarding data in the process. Returns false
// if not enough bytes are buffered, true otherwise.
static bool fifo_release(struct u_fifo *f, uint32_t size) {
    // check if enough bytes are present
    if (fifo_get_available(f) < size)
        return false;

    fifo_release_unsafe(f, size);
    return true;
}

// try to push a range of bytes to a ring buffer. Returns false if not enough space is available
// and no data is inserted, true otherwise.
static bool fifo_push(struct u_fifo *f, const void *src, uint32_t size) {
    // check if range fits in
    if (fifo_get_free(f) < size)
        return false;

    // push range until the end of ring
    uint32_t n = MIN(f->buf_size - f->head, size);
    uint32_t next_head = f->head + n;
    memcpy(&f->buf[f->head], src, n);

    // check if we need to wrap
    if (next_head == f->buf_size) {
        // wrap and push the second half
        next_head = /* 0 + */ size - n;
        memcpy(&f->buf[0], src + n, size - n);
    }

    f->head = next_head;
    return true;
}

// push a range of bytes to a ring buffer. This function will block until there's enough space
// for the new range.
static void fifo_push_blocking(struct u_fifo *f, const void *src, uint32_t size) {
    while (fifo_get_free(f) < size) {
        // yield();
    }

    fifo_push(f, src, size);
}

// try to pop a range of bytes from a ring buffer. Returns false if not enough bytes are present and
// no data is poped, true otherwise.
static bool fifo_pop(struct u_fifo *f, void *dst, uint32_t size) {
    // check if enough bytes are present
    if (fifo_get_available(f) < size)
        return false;

    // read data until the end of ring
    uint32_t n = MIN(f->buf_size - f->tail, size);
    uint32_t next_tail = f->tail - n;
    memcpy(dst, &f->buf[f->tail], n);

    // check if we need to wrap
    if (next_tail == f->buf_size) {
        // wrap and read rest of data
        next_tail = /* 0 + */ size - n;
        memcpy(dst + n, &f->buf[0], size - n);
    }

    f->tail = next_tail;
    return true;
}

// pop a range of bytes from a ring buffer. Blocks until there's enough data to pop
// the whole range.
static void fifo_pop_blocking(struct u_fifo *f, void *dst, uint32_t size) {
    while (fifo_get_available(f) < size) {
        // yeild();
    }

    fifo_pop(f, dst, size);
}

// peeks at a range of bytes stored inside the ring buffer. Returns false if the requested range
// isn't available in its entirety and no data is copied, true otherwise.
static bool fifo_peek(const struct u_fifo *f, void *dst, uint32_t size, uint32_t peek_offset) {
    // check the requested range is valid
    uint32_t avail = fifo_get_available(f);
    if (avail < peek_offset || avail - peek_offset < size)
        return false;

    uint32_t peek_tail = f->tail + peek_offset;
    if (peek_tail >= f->buf_size)
        peek_tail -= f->buf_size;

    // read data until the end of ring
    uint32_t n = MIN(f->buf_size - peek_tail, size);
    memcpy(dst, &f->buf[peek_tail], n);

    // check if we need to wrap
    if (n != size) {
        // wrap and read rest of data
        memcpy(dst + n, &f->buf[0], size - n);
    }

    return true;
}

// emplaces a range of bytes inside the "free" section of the ring buffer. This can be combined
// with fifo_acquire to construct safe pushes from more than a single continuous memory range.
//
// Returns false if the requested range does not fit inside the free buffer section in its entirety
// and no data is written, true otherwise.
static bool fifo_place(const struct u_fifo *f, const void *src, uint32_t size, uint32_t place_offset) {
    // check the requested range is valid
    uint32_t avail = fifo_get_free(f);
    if (avail < place_offset || avail - place_offset < size)
        return false;

    uint32_t place_head = f->head + place_offset;
    if (place_head >= f->buf_size)
        place_head -= f->buf_size;

    // write data until the end of ring
    uint32_t n = MIN(f->buf_size - place_head, size);
    memcpy(&f->buf[place_head], src, n);

    // check of need to wrap
    if (n != size) {
        // wrap and write rest of data
        memcpy(&f->buf[0], src + n, size - n);
    }

    return true;
}
