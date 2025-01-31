#pragma once

#include "picopu_types.h"
#include <shader_core/gcs/common.h>
#include <stdint.h>

#define MAX_SHADER_DBG_SIZE 512

/* gcs cbuf state */

struct __attribute__((packed)) gcs_state {
    screen_axis_t fb_extent[2];
    float viewport_transform_params[3][2];

    // struct gcs_sampler_state samplers[MAX_BOUND_SAMPLERS];

    primitive_mode_t rasterizer_mode;
};

/* gcs broker to shader broadcasts */

enum gcs_types {
    gcs_type_vs = 0,
    gcs_type_fs,

    gcs_type_ready = 16,
    gcs_type_po,
    gcs_type_fo,

    gcs_type_dbg = 31,
};
typedef uint8_t gcs_type_t;

// vertex stream, streams multiple vertex assemblies on which the vertex stage
// and vertex post-processing should be run
struct __attribute__((packed)) gcs_vs_header {
    gcs_type_t type;

    uint8_t prim_count;   // *must* be a multiple of 2
    
    uint16_t __padding;
    uint32_t base_vertex; // for gl_VertexID

    /* [vertex_count] of inline vertex assemblies follows */
};

// fragment stream, streams multiple primitives + shading range on which the
// fragment shader should be run
struct __attribute__((packed)) gcs_fs_header {
    gcs_type_t type;

    uint8_t prim_count;             // limited by MAX_PRIMS_PER_FSTREAM
    screen_axis_t shading_range[4]; // min_x, min_y, max_x, max_y; must be even to align with 2x2 rasterizer tiles

    uint16_t __padding;

    /* primitive vert count * [prim_count] of [clip_point]s */
    /* primitive vert count * [prim_count] of vertex output assemblies */
};

/* gcs shader to broker packets */

// signals that the shader chip is ready to receive next command streams - chip state: awaiting
// the broker needs to wait for this packet after gcs_begin, gcs_gp_conf and gcs_cb commands
struct __attribute__((packed)) gcs_ready {
    gcs_type_t type;
};

// primitive output, streams vertex processed primitives back to broker with
// their raster line counts - chip state: awaiting
struct __attribute__((packed)) gcs_po_header {
    gcs_type_t type;

    uint8_t primitive_count;
    int32_t shading_area[4]; // box area which the prims collectively cover (min x, min y, max x, max y)

    /* [primitive_count] * primitive vert count of vertex output assemblies (prog specific) */
};

// fragment output, streams a number of aligned fragment tiles to be patched to
// the fb - chip state: busy
//
// fragment tiles are then patched along the x axis,
// for row steps or skipped tiles, new fo streams will be created
//
// note: receiving this header *doesn't* inidcate that a fragment stream is
// finished!
// note: all color tiles passed to fragment output *must* already be
// pre-blended by the shader chip
struct __attribute__((packed)) gcs_fo_header {
    gcs_type_t type;

    uint8_t tile_count;
    uint16_t fb_base[2]; // start coords of the first tile in fb (in pixels)

    // uint32_t instru_systick;

    /* [tile_count] of uint16_t coverage masks */
    /* [tile_count] of [color_tile]s follow */
    /* [tile_count] of [depth_tile]s follow */
};

// a message packet for debug info
struct gcs_dbg {
    gcs_type_t type;
    char dbg_message[MAX_SHADER_DBG_SIZE];
};
