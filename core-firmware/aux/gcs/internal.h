#pragma once

#include "unit.h"

#include <hardware/sync.h>
#include <stdint.h>
#include <util/u_simd.h>

/* common types */

#define GUARD_BAND_MIN (-32768)
#define GUARD_BAND_MAX (32767)

// a vertex in clip space
struct clip_point {
    int32_t x, y;
    float z, w;

    float u, v;
};

#define RASTER_TILE_SIZE 4
#define RASTER_QUAD_SIZE 2

/* gcs intermediate buffers */

struct gcs_v2f_state {
    uint32_t prim_count;
    struct clip_point clip_buf[MAX_VERTICES_PER_BATCH];
};

extern struct gcs_v2f_state v2f;
extern spin_lock_t *locks[2];
