#pragma once

#include <stdint.h>

typedef int32_t v2i32 __attribute__((vector_size(8)));
typedef uint32_t v2u32 __attribute__((vector_size(8)));

typedef int32_t v4i32 __attribute__((vector_size(16)));
typedef uint32_t v4u32 __attribute__((vector_size(16)));

typedef int16_t v2i16 __attribute__((vector_size(4)));
typedef uint16_t v2u16 __attribute__((vector_size(4)));

typedef int16_t v4i16 __attribute__((vector_size(8)));
typedef uint16_t v4u16 __attribute__((vector_size(8)));

typedef float v2f32 __attribute__((vector_size(8)));
typedef float v4f32 __attribute__((vector_size(16)));
