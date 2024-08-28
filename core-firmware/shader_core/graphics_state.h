#pragma once

#include "../common/cluster_bus.h"
#include "../common/picopu_types.h"

#include "pico/stdio.h"
#include <stdio.h>

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define MAX_VERTEX_OUTPUT_STRIDE 24
#define MAX_VERTICES_PER_STREAM 32

#define MAX_CONSTANT_BUFFER_SIZE 65536

extern screen_axis_t fb_extent[2];
extern float view_transform_params[3][2];

extern uint8_t *vertex_mcode;
extern uint8_t *fragment_mcode;

extern uint8_t const_buffer[MAX_CONSTANT_BUFFER_SIZE];

extern uint8_t vertex_stride;
extern uint8_t output_vertex_stride;
extern uint8_t gp_verts_per_prim[2];

/* helper global funcs */

void send_ready();
void send_dbg(struct gcs_dbg *p);

#define put_buffer(buf, size) \
    stdio_put_string((const char *)buf, size, false, false);

#define format_dbg(...)                                                  \
    do {                                                                 \
        struct gcs_dbg __p = {gcs_type_dbg};                             \
        snprintf(__p.dbg_message, sizeof(__p.dbg_message), __VA_ARGS__); \
        send_dbg(&__p);                                                  \
    } while (false)

/* fixed-function entry points */

void configure_pipeline(struct gcs_gp_conf_header *conf);
void process_vertex_stream(struct gcs_vs_header *stream);
void process_fragment_stream(struct gcs_fs_header *stream);

void enter_graphics_state(struct gcs_begin *conf);
