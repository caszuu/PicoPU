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

// inverse two's-compliments helper types

typedef u8_x4_simd iu8_x4_simd;
typedef u8_x2_simd iu8_x2_simd;
typedef u16_x4_simd iu16_x4_simd;
typedef u16_x2_simd iu16_x2_simd;

// signed variants, same constrains apply, *only* add, sub, bitwise ops, no overflow or underflow

// note: signed types are disabled because the "overflow" condition applies to the binary representation
//       which means that crossing the 0 <-> -1 barrier was technically *and* practically undefined behaviour

// to make use of signed-like arithmetic, look at raster_stage.c with it's "inverse" two's-compliment repr

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
