#pragma once

#include <stdint.h>

/* cmdbuf command encodings */

enum cmd_type {
    cmd_type_end = 0,

    cmd_type_clear,
    cmd_type_present,

    cmd_type_xfer_to_device,
    cmd_type_xfer_to_host,
    // cmd_type_xfer_local,

    cmd_type_write_cbuf,
    cmd_type_push_cbuf,
    cmd_type_push_gstate,

    cmd_type_draw,
};

struct cmd_fb_clear {
    enum cmd_type ptype;
};

struct cmd_fb_present {
    enum cmd_type ptype;
};

struct cmd_xfer {
    enum cmd_type ptype;

    uint8_t xfer_idx;

    uint32_t xfer_size;
    uint32_t *vram_addr;
};

struct cmd_write_cbuf {
    enum cmd_type ptype;

    uint16_t size;
    uint16_t offset;

    uint32_t src_addr;
};

struct cmd_push_cbuf {
    enum cmd_type ptype;

    uint8_t size;
    uint16_t offset;

    /* inline cbuf data (4-byte aligned) */
};

struct cmd_push_gstate {
    enum cmd_type ptype;

    uint8_t size;
    uint8_t offset;

    /* inline gstate data (4-byte aligned) */
};

struct cmd_gcs_draw {
    enum cmd_type ptype;
    uint8_t bits;

    uint32_t prim_count;
    uint32_t index_base;
};

/* usb immediate command encodings */

enum ucmd_type {
    ucmd_type_sync = 0,

    ucmd_type_xfer_to_device,
    ucmd_type_xfer_to_host,

    ucmd_type_enqueue,
};

struct ucmd_xfer_from_device {
    enum ucmd_type ptype;
    uint8_t xfer_idx;
};

struct ucmd_xfer_from_host {
    enum ucmd_type ptype;

    uint32_t xfer_size;
    uint32_t *vram_addr;
};

struct ucmd_enqueue {
    enum ucmd_type ptype;

    uint32_t cmdbuf;
    // fin_fence;
};
