#include "pl.h"
#include "util/u_fifo.h"
#include <hardware/sync/spin_lock.h>
#include <pl_phy.pio.h>

#include <hardware/dma.h>
#include <hardware/irq.h>
#include <hardware/pio.h>
#include <hardware/sync.h>

#include <assert.h>
#include <stdatomic.h>
#include <stdio.h>

#define RX_PROC_IRQ SPARE_IRQ_0

// link state storage
// one per pio block per sm
static struct pl_link lks[3 * 4];

// a bitset of which links in [lks] are active
static atomic_uint lk_enabled;

static atomic_uint lk_tx_idle;
static atomic_uint lk_rx_irq;

// helpers //

static uint32_t pl_size_to_xc(const struct pl_link *lk, uint32_t size) {
    return (size - 2) * 2 - 1;
}

static uint32_t pl_xc_to_size(const struct pl_link *lk, uint32_t transfer_count) {
    return (transfer_count + 1) / 2 + 2;
}

// phy interrupts //

static void pl_link_tx_zl(uint32_t idx) {
    struct pl_link *lk = &lks[idx];

    struct pl_tx_header pak = {
        .credit_return = MIN(lk->credit_to_return, 255),
        .chan_idx = 0,                       // mock channel, rx_cb won't be called anyway
        .xfer_count0 = pl_size_to_xc(lk, 4), // no data
        .xfer_count1 = pl_size_to_xc(lk, 4),
    };

    atomic_fetch_sub(&lk->available_credit, sizeof(struct pl_tx_header));
    atomic_fetch_sub(&lk->credit_to_return, pak.credit_return);

    pio_sm_put_blocking(PIO_INSTANCE(idx / 4), idx % 4, *(uint32_t *)&pak);
}

static void pl_link_feed_tx(uint32_t idx, bool only_if_idle) {
    struct pl_link *lk = &lks[idx];

    // == citical section start ==

    uint32_t irq = spin_lock_blocking(lk->tx_lock);

    if (only_if_idle && !(lk_tx_idle & (1u << idx))) {
        goto unlock;
    }

    // load ready link channels for transmition
    uint32_t ready_chans = atomic_load(&lk->tx_chan_available);

    // no data buffered, transmit a 0-length packet or put link into idle
    if (!ready_chans) {
        if (atomic_load(&lk->credit_to_return) > 128 && sizeof(struct pl_tx_header) <= lk->available_credit) {
            pl_link_tx_zl(idx);
        }

        atomic_fetch_or(&lk_tx_idle, 1u << idx);
        goto unlock;
    }

    // select a channel to activate based on descending priority
    uint32_t chan = __builtin_ctz(ready_chans);

    // check if link has enough credits to transmit, idle if not
    struct pl_tx_header *pak = (struct pl_tx_header *)&lk->tx_fifos[chan].buf[lk->tx_fifos[chan].tail]; // a hacky way to extract the next packet header from the fifo
    uint32_t pak_size = pl_xc_to_size(lk, pak->xfer_count0);

    if (pak_size > atomic_load(&lk->available_credit)) {
        atomic_fetch_or(&lk_tx_idle, 1u << idx);
        goto unlock;
    }

    atomic_fetch_and(&lk_tx_idle, ~(1u << idx));
    spin_unlock(lk->tx_lock, irq);

    // == citical section end ==

    // fill-in packet header fields
    pak->credit_return = MIN(atomic_load(&lk->credit_to_return), 255);

    // update link and submit transfer to phy and dma
    lk->active_chan = chan;
    lk->active_pak_size = pak_size;

    atomic_fetch_sub(&lk->credit_to_return, pak->credit_return);
    atomic_fetch_sub(&lk->available_credit, pak_size);

    dma_channel_transfer_from_buffer_now(lk->tx_dma_chan, pak, pak_size / sizeof(uint32_t));
    return;

unlock:
    spin_unlock(lk->tx_lock, irq);
    return;
}

static void pl_link_tx_irq(uint32_t idx) {
    struct pl_link *lk = &lks[idx];

    // check if this link finished a transfer and needs processing
    if (!dma_channel_get_irq0_status(lk->tx_dma_chan))
        return;

    // ackknowledge interrupt and release transfered memory from tx fifo
    dma_channel_acknowledge_irq0(lk->tx_dma_chan);
    fifo_release_unsafe(&lk->tx_fifos[lk->active_chan], lk->active_pak_size);

    // safely mark the current channel as finished _if_ no data was added
    atomic_fetch_xor(&lk->tx_chan_available, 1u << lk->active_chan);
    if (fifo_get_available(&lk->tx_fifos[lk->active_chan]))
        atomic_fetch_or(&lk->tx_chan_available, 1u << lk->active_chan);

    // note: lock should never block, only prevents manual feeds from missing idle bits
    pl_link_feed_tx(idx, false);

    // notify user of the finished packet
    if (lk->tx_cb)
        lk->tx_cb();
}

static void pl_tx_irq() {
    for (uint32_t en = lk_enabled, i = __builtin_ctz(en); en; en ^= (1u << i), i = __builtin_ctz(en)) {
        pl_link_tx_irq(i);
    }
}

static void pl_rx_irq(uint32_t pio) {
    for (uint32_t irqs = PIO_INSTANCE(pio)->irq, i = __builtin_ctz(irqs); irqs; irqs ^= (1u << i), i = __builtin_ctz(irqs)) {
        struct pl_link *lk = &lks[3 * pio + i];

        pio_interrupt_clear(PIO_INSTANCE(pio), i);

        // a hacky way to extract the next packet header from the free part of the fifo
        // (a pl_rx_header is 4 bytes so it can be assumed to never be split / wrapped)
        struct pl_rx_header *pak = (struct pl_rx_header *)&lk->rx_fifo.buf[lk->rx_fifo.head];
        uint32_t pak_size = pl_xc_to_size(lk, pak->xfer_count);

        // mark the newly transfered packet as available for reading
        // and raise the lower-priority rx process irq

        fifo_acquire_unsafe(&lk->rx_fifo, pak_size);
        atomic_fetch_add(&lk->available_credit, pak->credit_return);

        atomic_fetch_or(&lk_rx_irq, 1u << (3 * pio + i));
        irq_set_pending(RX_PROC_IRQ);
    }
}

static void pl_rx_irq0() {
    pl_rx_irq(0);
}

static void pl_rx_irq1() {
    pl_rx_irq(1);
}

static void pl_rx_irq2() {
    pl_rx_irq(2);
}

static void pl_rx_proc_link(uint32_t idx) {
    struct pl_link *lk = &lks[idx];
    bool trigger_tx = false;

    for (; fifo_get_available(&lk->rx_fifo);) {
        struct pl_rx_header h;
        fifo_peek(&lk->rx_fifo, &h, sizeof(h), 0);
        uint32_t pak_size = pl_xc_to_size(lk, h.xfer_count);

        if (h.credit_return) {
            trigger_tx = true;
        }

        if (pak_size - sizeof(h) && lk->rx_cb) {
            // notify the user with the packet
            lk->rx_cb(h.chan_idx, &h);
        }

        fifo_release_unsafe(&lk->rx_fifo, pak_size);
        atomic_fetch_add(&lk->credit_to_return, pak_size);

        // FIXME: failsafe to break out if corrupt
    }

    // trigger a zero-length tx if too many credits are buffered to avoid stalling
    if (lk->credit_to_return > 128)
        trigger_tx = true;

    // restart transmision if new credits were received
    if (trigger_tx) {
        pl_link_feed_tx(idx, true);
    }
}

static void pl_rx_proc_irq() {
    uint32_t links_to_proc = atomic_load(&lk_rx_irq);
    atomic_fetch_xor(&lk_rx_irq, links_to_proc);

    for (uint32_t bits = links_to_proc, i = __builtin_ctz(bits); bits; bits ^= (1u << i), i = __builtin_ctz(bits)) {
        pl_rx_proc_link(i);
    }
}

// driver api //

void pl_init(uint32_t pio_idx) {
    static bool initial_init = true;

    if (initial_init) {
        irq_set_exclusive_handler(RX_PROC_IRQ, pl_rx_proc_irq);
        irq_set_enabled(RX_PROC_IRQ, true);

        // FIXME: handle dma irqs better
        // irq_add_shared_handler(DMA_IRQ_0, pl_tx_irq, 200);
        irq_set_exclusive_handler(DMA_IRQ_0, pl_tx_irq);
        irq_set_enabled(DMA_IRQ_0, true);

        initial_init = false;
    }

    PIO pio = pio_get_instance(pio_idx);
    int res = pio_add_program_at_offset(pio, &pl_phy_program, 0);
    assert(res >= 0);

    const void *rx_irq_table[] = {pl_rx_irq0, pl_rx_irq1, pl_rx_irq2};

    uint32_t pl_irq = pio_get_irq_num(pio, 0);
    irq_set_exclusive_handler(pl_irq, rx_irq_table[pio_idx]);
    irq_set_priority(pl_irq, 0x70);
    irq_set_enabled(pl_irq, true);
}

uint32_t pl_init_link(uint32_t pio_idx, uint32_t pio_sm, const struct pl_link_config *config) {
    PIO pio = pio_get_instance(pio_idx);
    uint32_t lk_idx = pio_idx * 4 + pio_sm;
    struct pl_link *lk = &lks[lk_idx];

    // validate configuration

    assert((lk_enabled & (1u << lk_idx)) == 0 && "link already initialized");
    assert(pio_sm <= 4 && "out-of-bounds sm idx");
    assert(config->chan_count > 0 && config->chan_count <= 16 && "invalid channel count");

    uint32_t tx_size_log2 = __builtin_ctz(config->tx_buf_size);
    assert(1u << tx_size_log2 == config->tx_buf_size && "tx_buf_size must be a power of 2");

    for (uint32_t chan = 0; chan < config->chan_count; chan++) {
        assert((uintptr_t)config->tx_bufs[chan] % config->tx_buf_size == 0 && "tx_buf must always be aligned to tx_buf_size");
    }

    uint32_t rx_size_log2 = __builtin_ctz(config->rx_buf_size);
    assert(1u << rx_size_log2 == config->rx_buf_size && "rx_buf_size must be a power of 2");
    assert((uintptr_t)config->rx_buf % config->rx_buf_size == 0 && "rx_buf must always be aligned to rx_buf_size");

    // init and start-up link phy (TODO: setup a software arbiter and replace is_initial)
    pl_phy_program_init(pio, pio_sm, 0, config->pin_base, config->is_initial);

    // init link state

    *lk = (struct pl_link){
        .available_credit = config->rx_buf_size, // assuming both sides match

        .tx_cb = config->tx_cb,
        .rx_cb = config->rx_cb,
        .tx_dma_chan = dma_claim_unused_channel(true),
        .rx_dma_chan = dma_claim_unused_channel(true),

        .chan_count = config->chan_count,
    };

    // TODO: probably replace spin locks entirely
    uint32_t lock_idx = spin_lock_claim_unused(true);
    lk->tx_lock = spin_lock_init(lock_idx);

    for (uint32_t chan = 0; chan < config->chan_count; chan++) {
        lk->tx_fifos[chan] = (struct u_fifo){
            .buf = config->tx_bufs[chan],
            .buf_size = config->tx_buf_size,
        };
    }

    lk->rx_fifo = (struct u_fifo){
        .buf = config->rx_buf,
        .buf_size = config->rx_buf_size,
    };

    // setup dma and interrupts

    dma_channel_config c = dma_channel_get_default_config(lk->rx_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_write_increment(&c, true);
    channel_config_set_read_increment(&c, false);
    channel_config_set_ring(&c, true, tx_size_log2);
    channel_config_set_dreq(&c, PIO_DREQ_NUM(pio, pio_sm, false));

    dma_channel_configure(lk->rx_dma_chan, &c, config->rx_buf, &pio->rxf[pio_sm], dma_encode_endless_transfer_count(), true);

    c = dma_channel_get_default_config(lk->tx_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_write_increment(&c, false);
    channel_config_set_read_increment(&c, true);
    channel_config_set_ring(&c, false, rx_size_log2);
    channel_config_set_dreq(&c, PIO_DREQ_NUM(pio, pio_sm, true));

    dma_channel_set_irq0_enabled(lk->tx_dma_chan, true);
    dma_channel_configure(lk->tx_dma_chan, &c, &pio->txf[pio_sm], NULL, 0, false);

    pio_set_irqn_source_enabled(pio, 0, pis_interrupt0 + pio_sm, true);

    // set link enable bit for irq handlers
    atomic_fetch_or(&lk_tx_idle, 1u << lk_idx);
    atomic_fetch_or(&lk_enabled, 1u << lk_idx);

    return lk_idx;
}

void pl_tx(uint32_t link, uint32_t chan, const void *src, uint32_t size) {
    assert(link < 3 * 4 && "out-of-bounds link index");
    assert(lk_enabled & (1u << link) && "link has not been initialized");

    struct pl_link *lk = &lks[link];
    assert(chan < lk->chan_count && "out-of-bounds link channel");

    // block until enough space is freed up in the channel tx fifo

    uint32_t pak_size = size + sizeof(struct pl_tx_header);
    assert(pak_size % 4 == 0 && "packet size is not 4-byte aligned");

    while (size > fifo_get_free(&lk->tx_fifos[chan])) {
        // __wfi(); wait for an rx irq
    }

    // setup and push packet to channel tx fifo

    uint8_t xc = pl_size_to_xc(lk, pak_size);
    struct pl_tx_header h = {
        .chan_idx = chan,
        // .credit_return filled in pl_link_feed_tx
        .xfer_count0 = xc,
        .xfer_count1 = xc,
    };

    fifo_place(&lk->tx_fifos[chan], &h, sizeof(h), 0);
    fifo_place(&lk->tx_fifos[chan], src, size, sizeof(h));
    fifo_acquire_unsafe(&lk->tx_fifos[chan], pak_size);

    // flag the channel as available for the tx scheduler
    atomic_fetch_or(&lk->tx_chan_available, 1u << chan);

    // try to schedule a transmit if link is idle
    pl_link_feed_tx(link, true);
}

uint32_t pl_rx(uint32_t link, const struct pl_rx_header *header, void *dst, uint32_t max_size) {
    struct pl_link *lk = &lks[link];
    uint32_t size = MIN(pl_xc_to_size(lk, header->xfer_count), max_size);

    fifo_peek(&lk->rx_fifo, dst, size, 0);
    return size;
}
