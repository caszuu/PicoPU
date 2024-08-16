#pragma once

#include "../common/picopu_types.h"
#include "../common/cluster_bus.h"

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

// #define MAX_CONSTANT_BUFFERS 4
#define MAX_VERTEX_OUTPUT_STRIDE 24
#define MAX_VERTICES_PER_STREAM 32

extern screen_axis_t fb_extent[2];

extern uint8_t* vertex_mcode;
extern uint8_t* fragment_mcode;

extern uint8_t* const_buffer;

extern uint8_t vertex_stride;
extern uint8_t output_vertex_stride;
extern primitive_mode_t prim_mode;

void send_ready();
void send_dbg(struct gcs_dbg* p);

#define format_dbg(...) do { struct gcs_dbg p = { gcs_type_dbg }; snprintf(p.dbg_message, sizeof(p.dbg_message), __VA_ARGS__); send_dbg(&p); } while(false)

void configure_pipeline(struct gcs_gp_conf_header* conf);
void assign_vertex_stream(struct gcs_vs_header* stream);
void assign_fragment_stream(struct gcs_fs_header* stream);

void enter_graphics_state();