#pragma once
#include <util/si_proto.h>

enum primitive_mode {
    e_prim_null = 0,
    e_prim_point,
    e_prim_line,
    e_prim_trig,
};
typedef enum primitive_mode primitive_mode_t;

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

#define MAX_VERTICES_PER_BATCH 32 * 3 // 96
#define MAX_TRIGS_PER_BATCH 32

void gcs_init();

void gcs_vertex_batch(struct scs_vertex_batch *b);
void gcs_raster_batch(struct scs_raster_batch *b);
