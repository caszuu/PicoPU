#pragma once

#include <common/ex_simd.h>
#include <stdint.h>

/* common types */

#define GUARD_BAND_MIN (-32768)
#define GUARD_BAND_MAX (32767)

// a vertex in clip space
struct clip_point {
    int32_t x, y;
    float z;

    v2f32 uv; // vertex texture coords
};

enum primitive_mode {
    e_prim_null = 0,
    e_prim_point,
    e_prim_line,
    e_prim_trig,
};
typedef enum primitive_mode primitive_mode_t;

#define RASTER_TILE_SIZE 4
#define RASTER_QUAD_SIZE 2

#define MAX_VERTICES_PER_BATCH 60
#define MAX_TRIGS_PER_BATCH (MAX_VERTICES_PER_BATCH / 3)

/* gcs structs and intermediate buffers */

struct gcs_gstate {
    // pipeline binds //
    uint8_t *vbuf;
    uint8_t *ibuf;

    uint8_t *fb_c0;
    uint8_t *fb_zs;

    // pipeline state //
    uint16_t fb_extent[2];
    float viewport_transform_params[3][2];

    // struct gcs_sampler_state samplers[MAX_BOUND_SAMPLERS];

    primitive_mode_t rasterizer_mode;
};

struct gcs_v2f_state {
    uint32_t prim_count;
    struct clip_point clip_buf[MAX_VERTICES_PER_BATCH];

    int32_t shading_range[4];
};

// temp.
struct demo_cbuf {
    m4f32 view_mat;
    m4f32 norm_mat;

    v4f32 light_dir;
    v4f32 col;
};
