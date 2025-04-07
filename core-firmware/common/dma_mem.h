#pragma once

#include <hardware/dma.h>
#include <hardware/platform_defs.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* memset */

// as memset supplies the value to write by-value, we have to store it for the duration of the transfer
static uint32_t memset_v_buf[NUM_DMA_CHANNELS];

static inline void dma_memset8(uint32_t ch, void *dst, uint8_t v, size_t count) {
    dma_channel_config c = dma_channel_get_default_config(ch);

    channel_config_set_write_increment(&c, true);
    channel_config_set_read_increment(&c, false);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    
    dma_channel_wait_for_finish_blocking(ch);

    memset_v_buf[ch] = v;
    dma_channel_configure(
        ch,
        &c,
        dst,
        &memset_v_buf[ch],
        count,
        true);
}

static inline void dma_memset16(uint32_t ch, void *dst, uint16_t v, size_t count) {
    dma_channel_config c = dma_channel_get_default_config(ch);

    channel_config_set_write_increment(&c, true);
    channel_config_set_read_increment(&c, false);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    
    dma_channel_wait_for_finish_blocking(ch);

    memset_v_buf[ch] = v;
    dma_channel_configure(
        ch,
        &c,
        dst,
        &memset_v_buf[ch],
        count,
        true);
}

static inline void dma_memset32(uint32_t ch, void *dst, uint32_t v, size_t count) {
    dma_channel_config c = dma_channel_get_default_config(ch);

    channel_config_set_write_increment(&c, true);
    channel_config_set_read_increment(&c, false);
    // channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    
    dma_channel_wait_for_finish_blocking(ch);

    memset_v_buf[ch] = v;
    dma_channel_configure(
        ch,
        &c,
        dst,
        &memset_v_buf[ch],
        count,
        true);
}

/* memcpy */

static inline void dma_memcpy8(uint32_t ch, void *dst, void *src, size_t count) {
    dma_channel_config c = dma_channel_get_default_config(ch);

    channel_config_set_write_increment(&c, true);
    // channel_config_set_read_increment(&c, true);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);

    dma_channel_wait_for_finish_blocking(ch);
    dma_channel_configure(
        ch,
        &c,
        dst,
        &src,
        count,
        true);
}

static inline void dma_memcpy16(uint32_t ch, void *dst, void *src, size_t count) {
    dma_channel_config c = dma_channel_get_default_config(ch);

    channel_config_set_write_increment(&c, true);
    // channel_config_set_read_increment(&c, true);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    
    dma_channel_wait_for_finish_blocking(ch);
    dma_channel_configure(
        ch,
        &c,
        dst,
        &src,
        count,
        true);
}

static inline void dma_memcpy32(uint32_t ch, void *dst, void *src, size_t count) {
    dma_channel_config c = dma_channel_get_default_config(ch);

    channel_config_set_write_increment(&c, true);
    // channel_config_set_read_increment(&c, true);
    // channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    
    dma_channel_wait_for_finish_blocking(ch);
    dma_channel_configure(
        ch,
        &c,
        dst,
        &src,
        count,
        true);
}