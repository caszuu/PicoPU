#pragma once

// the following types are shared between all picopu chips and *always* much match

#include <stdint.h>

typedef uint32_t res_addr_t;
typedef uint32_t res_size_t;

typedef uint16_t trans_size_t;
typedef uint16_t screen_axis_t;

struct rgba_color {
    uint8_t r, g, b, a;
};

typedef float depth_t;

enum primitive_mode {
    e_primm_null = 0,
    e_primm_point,
    e_primm_line,
    e_prim_trig,
};
typedef enum primitive_mode primitive_mode_t;

typedef int32_t rast_int_t;
#define GUARD_BAND_MIN (-32768)
#define GUARD_BAND_MAX (32767)

// a vertex in clip space
struct clip_point {
    rast_int_t x, y;
    depth_t d;
};
