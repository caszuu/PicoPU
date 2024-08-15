#pragma once

// the following types are shared between all picopu chips and *always* much match

// #include "pico/stdlib.h"
#include <stdint.h>

typedef uint32_t res_addr_t;
typedef uint32_t res_size_t;

typedef uint16_t trans_size_t;
typedef uint16_t screen_axis_t;

struct rgba_color {
    uint8_t r, g, b, a;  
};

typedef uint32_t depth_t;

enum primitive_mode {
    e_primm_null = 0,
    e_primm_point,
    e_primm_line,
    e_prim_trig,
};
typedef enum primitive_mode primitive_mode_t;

// a vertex in clip space
struct clip_point {
    int32_t x, y; // can be out of screen-bounds, need bigger range
    depth_t d;
};
