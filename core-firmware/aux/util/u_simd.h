#pragma once

#include <math.h>
#include <stdint.h>

typedef int32_t v2i32 __attribute__((vector_size(8)));
typedef uint32_t v2u32 __attribute__((vector_size(8)));

typedef int32_t v4i32 __attribute__((vector_size(16)));
typedef uint32_t v4u32 __attribute__((vector_size(16)));

typedef int16_t v2i16 __attribute__((vector_size(4)));
typedef uint16_t v2u16 __attribute__((vector_size(4)));

typedef int16_t v4i16 __attribute__((vector_size(8)));
typedef uint16_t v4u16 __attribute__((vector_size(8)));

typedef int8_t v2i8 __attribute__((vector_size(2)));
typedef uint8_t v2u8 __attribute__((vector_size(2)));

typedef int8_t v4i8 __attribute__((vector_size(4)));
typedef uint8_t v4u8 __attribute__((vector_size(4)));

typedef float v2f32 __attribute__((vector_size(8)));
typedef float v4f32 __attribute__((vector_size(16)));

typedef float m2f32 __attribute__((vector_size(2 * 2 * sizeof(float))));
typedef float m4f32 __attribute__((vector_size(4 * 4 * sizeof(float))));

/* clang-format off */

static inline v4f32 simd_mul4(m4f32 a, v4f32 b) {
    return (v4f32){
        a[0 ]*b[0 ] + a[4 ]*b[1 ] + a[8 ]*b[2 ] + a[12]*b[3 ],
        a[1 ]*b[0 ] + a[5 ]*b[1 ] + a[9 ]*b[2 ] + a[13]*b[3 ],
        a[2 ]*b[0 ] + a[6 ]*b[1 ] + a[10]*b[2 ] + a[14]*b[3 ],
        a[3 ]*b[0 ] + a[7 ]*b[1 ] + a[11]*b[2 ] + a[15]*b[3 ],
    };
}

static inline m4f32 simd_mmul4(m4f32 a, m4f32 b) {
    return (m4f32){
        a[0 ]*b[0 ] + a[4 ]*b[1 ] + a[8 ]*b[2 ] + a[12]*b[3 ],
        a[1 ]*b[0 ] + a[5 ]*b[1 ] + a[9 ]*b[2 ] + a[13]*b[3 ],
        a[2 ]*b[0 ] + a[6 ]*b[1 ] + a[10]*b[2 ] + a[14]*b[3 ],
        a[3 ]*b[0 ] + a[7 ]*b[1 ] + a[11]*b[2 ] + a[15]*b[3 ],

        a[0 ]*b[4 ] + a[4 ]*b[5 ] + a[8 ]*b[6 ] + a[12]*b[7 ],
        a[1 ]*b[4 ] + a[5 ]*b[5 ] + a[9 ]*b[6 ] + a[13]*b[7 ],
        a[2 ]*b[4 ] + a[6 ]*b[5 ] + a[10]*b[6 ] + a[14]*b[7 ],
        a[3 ]*b[4 ] + a[7 ]*b[5 ] + a[11]*b[6 ] + a[15]*b[7 ],

        a[0 ]*b[8 ] + a[4 ]*b[9 ] + a[8 ]*b[10] + a[12]*b[11],
        a[1 ]*b[8 ] + a[5 ]*b[9 ] + a[9 ]*b[10] + a[13]*b[11],
        a[2 ]*b[8 ] + a[6 ]*b[9 ] + a[10]*b[10] + a[14]*b[11],
        a[3 ]*b[8 ] + a[7 ]*b[9 ] + a[11]*b[10] + a[15]*b[11],

        a[0 ]*b[12] + a[4 ]*b[13] + a[8 ]*b[14] + a[12]*b[15],
        a[1 ]*b[12] + a[5 ]*b[13] + a[9 ]*b[14] + a[13]*b[15],
        a[2 ]*b[12] + a[6 ]*b[13] + a[10]*b[14] + a[14]*b[15],
        a[3 ]*b[12] + a[7 ]*b[13] + a[11]*b[14] + a[15]*b[15],
    };
}

static inline float simd_dot4(v4f32 a, v4f32 b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

static inline float simd_sqlen4(v4f32 a) {
    return a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
}

static inline float simd_len4(v4f32 a) {
    return sqrtf(simd_sqlen4(a));
}

static inline v4f32 simd_smul4(v4f32 a, float b) {
    return (v4f32){
        a[0] * b,
        a[1] * b,
        a[2] * b,
        a[3] * b,
    };
}

/* clang-format on */
