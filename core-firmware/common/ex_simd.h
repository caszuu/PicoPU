#pragma once

#include <stdint.h>

// test types for explicit (lying) simd types, **undefined** behaviour on overflow
// or other operations then additions, substractions and bitwise

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

typedef union {
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
} i16_x2_simd;
