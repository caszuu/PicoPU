#pragma once

#include <stdint.h>

// explicit simd types: **undefined** behaviour on overflow
//   or other operations then additions, substractions and bitwise
//   note: bit-shifts might also be possible with explicit masking

// basic usage: use .v[] for value access (copy in/out) and .o for operations on the simd lane as a whole
//              see raster_stage.c for example usage

typedef union {
    uint8_t v[4];
    uint32_t o;
} u8_x4_simd;

typedef union {
    uint8_t v[2];
    uint16_t o;
} u8_x2_simd;

typedef union {
    uint16_t v[4];
    uint64_t o;
} u16_x4_simd;

typedef union {
    uint16_t v[2];
    uint32_t o;
} u16_x2_simd;

// signed variants, same constrains apply, *only* add, sub, bitwise ops, no overflow or underflow

/* typedef union {
    int8_t v[4];
    uint32_t o;
} i8_x4_simd;

typedef union {
    int8_t v[2];
    uint16_t o;
} i8_x2_simd;

typedef union {
    int16_t v[4];
    uint64_t o;
} i16_x4_simd;

typedef union {
    int16_t v[2];
    uint32_t o;
} i16_x2_simd; */
