#pragma once

#include <chip.h>
#include <common/ex_simd.h>

#include <hardware/interp.h>
#include <math.h>
#include <stdlib.h>

// WIP sampler and texel fetching routine implementations

struct sampler_state {
    v2u32 texture_extent;
    uint8_t *texture_vram_addr;
};

/* sampler routines */

static inline v4u8 sample_bilinear_rgba8(v2f32 interp_params, v4u8 texel0, v4u8 texel1, v4u8 texel2, v4u8 texel3) {
    // interp two horizontal samples

    interp0->accum[1] = interp_params[0] * 255;
    v4u32 horiz_samples;

    for (uint32_t chan = 0; chan < 4; chan++) {
        interp0->base01 = (uint32_t)(texel0[chan]) | (uint32_t)(texel1[chan]) << 16;
        horiz_samples[chan] = interp0->peek[1];
    }

    for (uint32_t chan = 0; chan < 4; chan++) {
        interp0->base01 = (uint32_t)(texel2[chan]) | (uint32_t)(texel3[chan]) << 16;
        horiz_samples[chan] |= interp0->peek[1] << 16;
    }

    // interp final sample

    interp0->accum[1] = interp_params[1] * 255;

    v4u8 sample;
    for (uint32_t chan = 0; chan < 4; chan++) {
        interp0->base01 = horiz_samples[chan];
        sample[chan] = interp0->peek[1];
    }

    return sample;
}

/* wrapping handlers */

// PERF TODO: currently wrap ops have relatively large perf hit
// TODO: probably rework to wrap with texel coords

static inline void wrap_uv_repeat(v2f32 *uv, const struct sampler_state *state) {
    (*uv)[0] = (*uv)[0] - floorf((*uv)[0]);
    (*uv)[1] = (*uv)[1] - floorf((*uv)[1]);
}

static inline void wrap_uv_clamp(v2f32 *uv, const struct sampler_state *state) {
    (*uv)[0] = MIN(MAX((*uv)[0], 0.f), 1.f);
    (*uv)[1] = MIN(MAX((*uv)[1], 0.f), 1.f);
}

/* layout-aware memory fetchers */

static inline void fetch_texel_4B_linear(uint32_t texel_buf[], v2u32 texel, const struct sampler_state *state) {
    texel_buf[0] = *(uint32_t *)(state->texture_vram_addr + (texel[0] + texel[1] * state->texture_extent[0]) * sizeof(uint32_t));
}

static inline void fetch_quad_4B_linear(uint32_t texel_buf[], v2u32 texel, const struct sampler_state *state) {
    uint32_t texture_width = state->texture_extent[0];
    uint32_t *quad_base = (uint32_t *)(state->texture_vram_addr + (texel[0] + texel[1] * texture_width) * sizeof(uint32_t));

    texel_buf[0] = *(quad_base);
    texel_buf[1] = *(quad_base + 1);
    texel_buf[2] = *(quad_base + texture_width);
    texel_buf[3] = *(quad_base + texture_width + 1);
}

static inline void fetch_texel_4B_tiled_8x8(uint32_t texel_buf[], v2u32 texel, const struct sampler_state *state) {
    const uint32_t tile_stride = 8 * 8;

    v2u32 tile = texel >> 3;
    v2u32 local_texel = texel & 7;

    texel_buf[0] = *(uint32_t *)(state->texture_vram_addr + (tile[0] + tile[1] * state->texture_extent[0] / 8) * tile_stride * sizeof(uint32_t) + (local_texel[0] + local_texel[1] * 8) * sizeof(uint32_t));
}

/* public test fetch routines */

static inline uint32_t native_fetch_rgba8_nearest(v2f32 uv) {
    struct sampler_state state = {
        .texture_vram_addr = cbuf + 64,
        .texture_extent = (v2u32){64, 64},
    };

    wrap_uv_clamp(&uv, &state);
    v2u32 texel_coords = (v2u32){uv[0] * 64.f, uv[1] * 64.f};

    v4u8 texel;
    fetch_texel_4B_linear((uint32_t *)&texel, texel_coords, &state);

    return (uint32_t)texel;
}

static inline uint32_t native_fetch_rgba8_bilinear(v2f32 uv) {
    struct sampler_state state = {
        .texture_vram_addr = cbuf + 64,
        .texture_extent = (v2u32){64, 64},
    };

    wrap_uv_clamp(&uv, &state);
    uv *= 64.f; // tex size normalization

    v2u32 texel_coords = (v2u32){uv[0], uv[1]};
    v2f32 interp_params = (v2f32){(uv[0] - texel_coords[0]), (uv[1] - texel_coords[1])};

    // fetch
    v4u8 texel_buf[4];
    fetch_quad_4B_linear((uint32_t *)texel_buf, texel_coords, &state);

    // filter on interp unit
    return (uint32_t)sample_bilinear_rgba8(interp_params, texel_buf[0], texel_buf[1], texel_buf[2], texel_buf[3]);
}
