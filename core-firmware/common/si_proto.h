#pragma once

#include <stdint.h>

#define MAX_SCS_PACKET_SIZE 1024
#define MAX_SCS_DBG_SIZE 63

/* shader interface cmd and xfer packet layouts */

enum si_packet_type {
    si_class_scs = 0,
    si_type_vbatch = si_class_scs,
    si_type_rbatch,
    si_type_finished,
    si_type_ld_cbuf,
    si_type_ld_cbuf_inline,

    si_type_flip,

    si_class_unordered = 16,
    si_type_vram_fetch = si_class_unordered,
    si_type_flash,
    si_type_dbg,
};

/* scs / gcs cmds */

// dispatch a vertex batch - shades a range of vertices and computes its v2f state
struct scs_vertex_batch {
    enum si_packet_type type;

    uint8_t v2f_idx;

    uint8_t primitive_count;
    uint32_t index_base;
};

// dispatch a raster batch - shades a range of fragments from current v2f state
struct scs_raster_batch {
    enum si_packet_type type;

    uint8_t v2f_idx;
};

// TODO: scs feedback

// batch finished - just a (mostly) state-less feedback to broker that a batch finished execution (used for SU <> broker sync)
struct scs_batch_finished {
    enum si_packet_type type;

    uint8_t v2f_idx;
};

// load constant buffer (range) - overrides a part of the local cbuf
struct scs_ld_cbuf {
    enum si_packet_type type;

    uint16_t range_offset;
    uint16_t range_size;

    // vramptr_t range_src;
};

// load constant buffer (range) - overrides a part of the local cbuf (with inline data in packet)
struct scs_ld_cbuf_inline {
    enum si_packet_type type;

    uint16_t range_size;
    uint32_t range_offset;

    /* cbuf range data follows */
};

/* xfer / vram packets */

struct si_dbg_packet {
    enum si_packet_type type;
    char dbg_message[MAX_SCS_DBG_SIZE];
};
