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
