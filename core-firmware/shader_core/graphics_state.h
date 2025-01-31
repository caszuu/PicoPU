#pragma once

#include <common/gcs_proto.h>
#include <common/ex_simd.h>
#include <common/picopu_types.h>

#include "pico/stdio.h"
#include <stdio.h>

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define MAX_VERTEX_OUTPUT_STRIDE 24

#define MAX_CONSTANT_BUFFER_SIZE 65536
#define MAX_BOUND_SAMPLERS 8

/* helper global funcs */

void send_ready();
void send_dbg(struct gcs_dbg *p);

#ifndef PICOPU_SINGLE_CHIP

#define put_buffer(buf, size) \
    stdio_put_string((const char *)buf, size, false, false);

#define format_dbg(...)                                                  \
    do {                                                                 \
        struct gcs_dbg __p = {gcs_type_dbg};                             \
        snprintf(__p.dbg_message, sizeof(__p.dbg_message), __VA_ARGS__); \
        /* send_dbg(&__p); */                                            \
    } while (false)

#else

#define put_buffer(buf, size) \
    tud_vendor_write

#define format_dbg(...)

#endif

/* fixed-function entry points */

void process_vertex_stream(struct gcs_vs_header *stream);
void process_fragment_stream(struct gcs_fs_header *stream);
