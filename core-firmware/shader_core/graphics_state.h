#pragma once

#include "../common/picopu_types.h"
#include "../common/cluster_bus.h"

// #define MAX_CONSTANT_BUFFERS 4
#define MAX_VERTEX_OUTPUT_STRIDE 8

extern screen_axis_t fb_extent[2];

extern uint8_t* vertex_mcode;
extern uint8_t* fragment_mcode;

// uint8_t* const_buffers[MAX_CONSTANT_BUFFERS];
extern uint8_t* const_buffer;

extern uint8_t vertex_stride;
extern uint8_t output_vertex_stride;
extern primitive_mode_t prim_mode;

void enter_graphics_state(struct gcs_begin* info);
