#pragma once

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

#include <hardware/sync/spin_lock.h>
#include <util/u_fifo.h>

struct pl_rx_header {
    uint8_t credit_return, chan_idx;
    uint8_t __padding, xfer_count;
};

struct pl_tx_header {
    uint8_t credit_return, chan_idx;
    uint8_t xfer_count0 /*used by tx*/, xfer_count1 /*used by rx*/;
};

typedef void (*pl_rx_cb)(uint32_t link, uint32_t chan, uint32_t n_bytes);
typedef void (*pl_tx_cb)();

struct pl_link_perf {
    atomic_uint tx_bytes;
    atomic_uint rx_bytes;

    atomic_uint tx_stalls;

    atomic_uint rx_irqs;
    atomic_uint rx_proc_irqs;
};

struct pl_link {
    // link state //

    atomic_uint available_credit;
    atomic_uint credit_to_return;

    atomic_uint tx_chan_available;

    uint32_t active_chan;
    uint32_t active_pak_size;

#ifdef PL_ENABLE_STATS
    struct pl_link_perf perf;
#endif

    // link resources //

    struct u_fifo tx_fifos[16];
    struct u_fifo rx_fifo;

    pl_rx_cb rx_cb;
    pl_tx_cb tx_cb;

    uint32_t rx_dma_chan;
    uint32_t tx_dma_chan;
    spin_lock_t *tx_lock;

    uint32_t chan_count;
};

struct pl_link_config {
    uint32_t chan_count; // number of tx channels requested
    uint32_t pin_base;   // base pin for the pico-link bus
    bool is_initial;     // whether to start as rx or tx on init
                         // TODO: rework into pl_arbiter

    pl_rx_cb rx_cb; // receive callback. called for every received packet by the link.
    pl_tx_cb tx_cb; // transmit callback. called after a packet has been sent, useful for stream pacing. (optional)

    // backing buffers for internal tx buffers for each link channel
    // only [chan_count] buffers are used
    uint8_t *tx_bufs[16];
    uint32_t tx_buf_size;

    // backing buffers for the internal rx buffer
    uint8_t *rx_buf;
    uint32_t rx_buf_size;
};

// defines a pico-link usable backing buffer.
#define DEFINE_LINK_BUFFER(name, size) static uint8_t name[size] __attribute__((aligned(size)));

// initialize a pio block as a pico-link interface. pl_init_link
// has to be called as well for an active link to be created.
void pl_init(uint32_t pio_index);

// initialize a link on an idle pio sm. the link will become active
// after this function returns. returns the link index usable for the rest of the api.
uint32_t pl_init_link(uint32_t pio_index, uint32_t pio_sm, const struct pl_link_config *config);

// fetch the link-specific performance counters and store them into [buf], optionally also reset them.
// this function will return valid data only if the driver was compiled with PL_ENABLE_PERF
void pl_perf(uint32_t link, struct pl_link_perf *buf, bool reset);

// push a packet to a channel transmit queue, the link will schedule the packet based
// on the channel prioroty. this call will block until enough space is present in the
// internal transmit queue for the packet.
void pl_tx(uint32_t link, uint32_t chan, const void *src, uint32_t size);

// read a range from a packet inside the internal receive buffer, this can only be called from a
// receive callback (rx_cb) context, otherwise it's UB. a pl_rx call does not discard any data and
// reading from the same range muliple times will yield identical output (under the same rx_cb context).
// returns the number of bytes actually read.
uint32_t pl_rx(uint32_t link, void *dst, uint32_t max_size, uint32_t offset);
