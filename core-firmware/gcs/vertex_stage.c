#include "unit.h"

#include <chip.h>
#include <common/ex_simd.h>
#include <common/si_proto.h>

struct in_attribs {
    float pos[3];
};

static inline void dispatch_vertex(v4f32 *v_pos, struct clip_point *out_clip, uint32_t vertex_index) {
    /* user vertex shader */

    // indexed draw
    // const uint16_t idx = ((const uint16_t *)vaddr(gs.ibuf))[vertex_index];
    // const struct in_attribs *attribs = (const struct in_attribs *)(vaddr(gs.vbuf)) + idx;

    // array draw
    const struct in_attribs *attribs = (const struct in_attribs *)(vaddr(gs.vbuf)) + vertex_index;

    v4f32 p = {
        attribs->pos[0],
        attribs->pos[1],
        attribs->pos[2],
        1.f,
    };

    const m4f32 *mvp = (const m4f32 *)cbuf;
    p = ex_mul4(*mvp, p);

    /* vertex early post-process */

    // prespective divide -> to NDCs
    float w_inv = 1.f / p[3];
    p[0] *= w_inv;
    p[1] *= w_inv;
    p[2] *= w_inv;

    *v_pos = p;

    // TODO: shader attrib divides...
}

/* vertex stage entry */

// shared buffer for vertex-to-fragment state, vertex outputs will be written there
struct gcs_v2f_state v2f;

void dispatch_vertex_batch(struct scs_vertex_batch *batch) {
    static const uint8_t v_count = 3;

    uint32_t local_vertex_index = batch->index_base;
    uint32_t output_vertex_index = 0;
    v2f.prim_count = 0;

    // accumulated shading_area between all prims
    int32_t shading_area[4] = {gs.fb_extent[0], gs.fb_extent[1], 0, 0};

    for (uint32_t prim_i = 0; prim_i < batch->primitive_count; prim_i++) {
        /* vertex stage */

        v4f32 v_positions[v_count];

        // Cohen–Sutherland algo out-codes (only using 6 bits)
        uint8_t v_out_codes = 0x3F;  // AND'ed viewport
        uint8_t gb_out_codes = 0x00; // OR'ed guard-band

        for (uint8_t pvi = 0; pvi < v_count; local_vertex_index++, pvi++) {
            // vertex shader
            dispatch_vertex(&v_positions[pvi], &v2f.clip_buf[output_vertex_index + pvi], local_vertex_index);

            // post-shader

            const float w = v_positions[pvi][3], nw = -w;

            // SPEC NOTE: based on OpenGL, but D3D and Vulkan must use 0 < z < w
            v_out_codes &= (v_positions[pvi][0] < nw) << 0 | (v_positions[pvi][0] > w) << 1 |
                           (v_positions[pvi][1] < nw) << 2 | (v_positions[pvi][1] > w) << 3 |
                           (v_positions[pvi][2] < nw) << 4 | (v_positions[pvi][2] > w) << 5;

            gb_out_codes |= (v_positions[pvi][0] < GUARD_BAND_MIN) << 0 | (v_positions[pvi][0] > GUARD_BAND_MAX) << 1 |
                            (v_positions[pvi][1] < GUARD_BAND_MIN) << 2 | (v_positions[pvi][1] > GUARD_BAND_MAX) << 3 |
                            (v_positions[pvi][2] < GUARD_BAND_MIN) << 4 | (v_positions[pvi][2] > GUARD_BAND_MAX) << 5;
        }

        /* vertex post-processing */

        if (v_out_codes) {
            // prim entirely outside the viewport, cull it
            continue;
        }

        if (gb_out_codes) {
            // prim both outside the guard-bands and inside viewport, very rare, perform slow clip
            // FIXME: implement guard-band clipping, for now cull instead

            continue;
        }

        for (uint8_t pvi = 0; pvi < v_count; pvi++) {
            // viewport transform

            // at this point it's safe to convert the float [v_positions] to
            // [clip_point]s fixed-point int32s without over/under flow

            struct clip_point *clip = &v2f.clip_buf[output_vertex_index + pvi];

            clip->x = v_positions[pvi][0] * gs.viewport_transform_params[0][0] + gs.viewport_transform_params[0][1];
            clip->y = v_positions[pvi][1] * gs.viewport_transform_params[1][0] + gs.viewport_transform_params[1][1];
            clip->z = v_positions[pvi][2] * gs.viewport_transform_params[2][0] + gs.viewport_transform_params[2][1];
        }

        /* late primitive processing */

        // computes (double) the signed area of the trig
        int32_t signed_area = (v2f.clip_buf[output_vertex_index + 1].x - v2f.clip_buf[output_vertex_index + 0].x) * (v2f.clip_buf[output_vertex_index + 2].y - v2f.clip_buf[output_vertex_index + 0].y) -
                              (v2f.clip_buf[output_vertex_index + 2].x - v2f.clip_buf[output_vertex_index + 0].x) * (v2f.clip_buf[output_vertex_index + 1].y - v2f.clip_buf[output_vertex_index + 0].y);

        if (signed_area < 0) {
            // back-face, cull
            // TODO: add gstate ctl for face culling

            continue;
        }

        if (signed_area == 0) {
            // degenerate trig, cull
            continue;
        }

        v2f.prim_count++;
        output_vertex_index += v_count;
    }
}
