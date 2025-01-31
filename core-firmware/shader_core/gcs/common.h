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
};

#define RASTER_TILE_SIZE 4
#define RASTER_QUAD_SIZE 2

#define MAX_VERTICES_PER_VSTREAM 32
#define MAX_PRIMS_PER_FSTREAM 128