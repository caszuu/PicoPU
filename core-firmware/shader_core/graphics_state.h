#pragma once

#include "../common/cluster_bus.h"
#include "../common/ex_simd.h"
#include "../common/picopu_types.h"

#include "pico/stdio.h"
#include <stdio.h>

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define MAX_VERTEX_OUTPUT_STRIDE 24
#define MAX_VERTICES_PER_STREAM 32

#define MAX_CONSTANT_BUFFER_SIZE 65536
#define MAX_BOUND_SAMPLERS 8

/* gcs state */

struct gcs_state {
    screen_axis_t fb_extent[2];
    float view_transform_params[3][2];

    // struct gcs_sampler_state samplers[MAX_BOUND_SAMPLERS];

    primitive_mode_t rasterizer_mode;
    uint8_t rast_verts_per_prim[2];
};

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

// void bind_pipeline(struct gcs_gp_bind_header *gp);
// void configure_pipeline(struct gcs_gp_conf *conf);
void process_vertex_stream(struct gcs_vs_header *stream);
void process_fragment_stream(struct gcs_fs_header *stream);

void enter_graphics_state();

/* single chip test more */

// starts a mock broker infinitely dispatching test frame draws
// this is used to test shader firmware with a single chip
void start_mock_broker();
