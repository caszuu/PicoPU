#pragma once
#include <stdint.h>

#define CONSTANT_BUFFER_SIZE 1024 * 64 
#define PROG_BIN_BUFFER_SIZE 1024 * 64

/* top-level shader unit state buffers */

struct chip_state {
    uint8_t cbuf[CONSTANT_BUFFER_SIZE];
    uint8_t prog_buf[PROG_BIN_BUFFER_SIZE];
};

extern struct chip_state chip_state;

void enter_scs();

/* scs proto */

enum scs_cmd_type {
    scs_null = 0,
    
    scs_type_ld_cbuf,
    scs_type_ld_bin,
    scs_type_disp_bin,

    // temp. trigger baked in gcs pipeline
    scs_type_disp_gcs,
    scs_type_flash,

    // TODO: flush, inval, wait
};

/* load constant buffer (range) - overrides a part of the local cbuf */
struct __attribute__((packed)) scs_ld_cbuf {
    enum scs_cmd_type type;

    uint16_t range_offset;
    uint16_t range_size;

    /* cbuf range data follows */
};

/* load program binary - overrides a part of the chip prog bin buffer */
struct __attribute__((packed)) scs_ld_bin {
    enum scs_cmd_type type;

    uint16_t bin_buf_offset;
    uint16_t bin_size;

    /* bin data follows */
};

/* dispatch binary - takes a chip prog binary buffer based offset to a entry point and enters it */
struct __attribute__((packed)) scs_disp_bin {
    enum scs_cmd_type type;

    uint16_t entry_buf_offset;
};