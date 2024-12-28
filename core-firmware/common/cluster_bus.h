#pragma once

// structs describing all commands transfered over the cluster-local bus

#include "picopu_types.h"
#include <stdint.h>

#define CHIPS_PER_CLUSTER 4
#define MAX_GCS_PACKET_SIZE 65536
#define MAX_SHADER_DBG_SIZE 512

#define SHADER_CHIP_ID 0 // TODO: generate by cmake

/* common structs */

#define RENDER_QUAD_SIZE 2 // note: size of one axis

struct color_tile {
    // FIXME: format dep
    struct rgba_color c[RENDER_QUAD_SIZE * 2];
};

struct depth_tile {
    depth_t d[RENDER_QUAD_SIZE * 2];
};

/* gcs broker to shader broadcasts */

enum gcs_types {
    gcs_type_begin = 0,
    gcs_type_vs,
    gcs_type_fs,

    gcs_type_ready = 16,
    gcs_type_po,
    gcs_type_fo,

    gcs_type_fault = 32,
    gcs_type_dbg,
};
typedef uint8_t gcs_type_t;

// vertex stream, streams multiple vertex assemblies on which the vertex stage
// and vertex post-processing should be run
struct gcs_vs_header {
    gcs_type_t type;

    uint32_t base_vertex;    // for gl_VertexID
    uint8_t primitive_count; // *must* be a multiple of 2

    /* [vertex_count] of inline vertex assemblies follows */
};

#define MAX_PRIMS_PER_FSTREAM 128

// fragment stream, streams multiple primitives + shading range on which the
// fragment shader should be run
struct __attribute__((packed)) gcs_fs_header {
    gcs_type_t type;

    screen_axis_t shading_range[4]; // min_x, min_y, max_x, max_y; must be even to align with 2x2 rasterizer tiles
    uint8_t prim_count;             // limited by MAX_PRIMS_PER_FSTREAM

    /* primitive vert count * [prim_count] of [clip_point]s */
    /* primitive vert count * [prim_count] of vertex output assemblies */
};

/* gcs shader to broker packets */

// signals that the shader chip is ready to receive next command streams - chip state: awaiting
// the broker needs to wait for this packet after gcs_begin, gcs_gp_conf and gcs_cb commands
struct gcs_ready {
    gcs_type_t type;
};

// report a shader execution or shader core fault, no matter the [fault] the
// shader core is now in a sleep state and will need a power cycle to reset it
struct gcs_fault {
    gcs_type_t type;

    enum fault_id {
        out_of_memory = 0,
        watchdog_trigger,
        invalid_chip_state,
    };

    enum fault_id fault;
};

// primitive output, streams vertex processed primitives back to broker with
// their raster line counts - chip state: awaiting
struct gcs_po_header {
    gcs_type_t type;
    uint8_t primitive_count;

    // area which the prim covers (min x, min y, max x, max y)
    rast_int_t shading_area[4];

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
    uint32_t fb_index_base; // start index of the first tile in fb

    /* ([tile_count] / 2) + ([tile_count] % 2) or (4 bits per tile) of uint8_t coverage masks */
    /* [tile_count] of [color_tile]s follow */
    /* [tile_count] of [depth_tile]s follow (for late depth tests) */
};

// a message packet for debug info
struct gcs_dbg {
    gcs_type_t type;
    char dbg_message[MAX_SHADER_DBG_SIZE];
};

/* transfer shader command packets */

struct tc_read_req { /* by shader */
    res_addr_t addr;
    trans_size_t size;
    uint8_t req_chip_id;
};

struct tc_read { /* by broker */
    uint8_t req_chip_id;

    /* [size] of bytes follows */
};

struct tc_write_req { /* by shader */
    res_addr_t addr;
    trans_size_t size;
    uint8_t req_chip_id;
};

struct tc_write_ready { /* by broker */
    uint8_t req_chip_id;
};

/* ccs broker to shader broadcasts */

// compute command streams maybe in far future...
