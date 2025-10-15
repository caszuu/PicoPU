#include "internal.h"

#include <chip.h>
#include <common/ex_simd.h>
#include <common/mc.h>
#include <common/si_proto.h>

#include <hardware/sync.h>

#include <string.h>

struct in_attribs {
    float pos[3];
    float uv[2];
};

static inline void dispatch_vertex(v4f32 *v_pos, struct clip_point *out_clip, uint32_t vertex_index) {
    /* user vertex shader */

    // indexed draw
    const uint16_t idx = ((const uint16_t *)vaddr(gs.ibuf))[vertex_index];
    const struct in_attribs *attribs = (const struct in_attribs *)(vaddr(gs.vbuf)) + idx;

    // array draw
    // const struct in_attribs *attribs = (const struct in_attribs *)(vaddr(gs.vbuf)) + vertex_index;

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
    out_clip->u = attribs->uv[0]; // * w_inv;
    out_clip->v = attribs->uv[1]; // * w_inv;
    out_clip->w = p[3];
}

/* vertex stage entry */

// shared buffer for vertex-to-fragment state, vertex outputs will be written there
struct gcs_v2f_state v2f;
spin_lock_t *locks[2];

void gcs_init() {
    int l = spin_lock_claim_unused(true);
    locks[0] = spin_lock_init(l);

    l = spin_lock_claim_unused(true);
    locks[1] = spin_lock_init(l);
}

static void proc_trigs(struct scs_vertex_batch *batch) {
    // FIXME: implement a multicore loop that doesn't break api order

    for (uint32_t prim_i = 0, vert_i = batch->index_base; prim_i < batch->primitive_count; prim_i += 1, vert_i += 3) {
        /* vertex stage */

        struct clip_point v_clips[3];
        v4f32 v_positions[3];

        // Cohen–Sutherland algo out-codes (only using 6 bits)
        uint8_t v_out_codes = 0x3F;  // AND'ed viewport
        uint8_t gb_out_codes = 0x00; // OR'ed guard-band

        for (uint8_t pvi = 0; pvi < 3; pvi++) {
            // vertex shader
            dispatch_vertex(&v_positions[pvi], &v_clips[pvi], vert_i + pvi);

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

        for (uint8_t pvi = 0; pvi < 3; pvi++) {
            // viewport transform

            // at this point it's safe to convert the float [v_positions] to
            // [clip_point]s fixed-point int32s without over/under flow

            struct clip_point *clip = &v_clips[pvi];

            clip->x = v_positions[pvi][0] * gs.viewport_transform_params[0][0] + gs.viewport_transform_params[0][1];
            clip->y = v_positions[pvi][1] * gs.viewport_transform_params[1][0] + gs.viewport_transform_params[1][1];
            clip->z = v_positions[pvi][2] * gs.viewport_transform_params[2][0] + gs.viewport_transform_params[2][1];
        }

        /* late primitive processing */

        // computes (double) the signed area of the trig
        int32_t signed_area = (v_clips[1].x - v_clips[0].x) * (v_clips[2].y - v_clips[0].y) -
                              (v_clips[2].x - v_clips[0].x) * (v_clips[1].y - v_clips[0].y);

        // struct clip_point temp = v_clips[1];
        // v_clips[1] = v_clips[2];
        // v_clips[2] = temp;
        // signed_area = -signed_area;

        if (signed_area < 0) {
            // back-face, cull
            // TODO: add gstate ctl for face culling

            continue;
        }

        if (signed_area == 0) {
            // degenerate trig, cull
            continue;
        }

        // prim passed, safely add to v2f clip_buf
        uint32_t irq = spin_lock_blocking(locks[0]);
        memcpy(&v2f.clip_buf[v2f.prim_count * 3], v_clips, sizeof(v_clips));
        v2f.prim_count++;
        spin_unlock(locks[0], irq);
    }

    // wait for all cores
    // mc_barrier();
}

static inline void dispatch_point_vert(v4f32 *v_pos, struct clip_point *out_clip, uint32_t vertex_index) {
    /* user vertex shader */

    // indexed draw
    const uint16_t idx = ((const uint16_t *)vaddr(gs.ibuf))[vertex_index];
    const struct in_attribs *attribs = (const struct in_attribs *)(vaddr(gs.vbuf)) + idx;

    // array draw
    // const struct in_attribs *attribs = (const struct in_attribs *)(vaddr(gs.vbuf)) + vertex_index;

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

    // set point_size
    out_clip->u = 15 * w_inv;
    out_clip->w = p[3];
}

static void proc_points(struct scs_vertex_batch *batch) {
    // FIXME: implement a multicore loop that doesn't break api order

    for (uint32_t prim_i = 0, vert_i = batch->index_base; prim_i < batch->primitive_count; prim_i += 1, vert_i += 1) {
        /* vertex stage */

        struct clip_point v_clip;
        v4f32 v_pos;

        dispatch_point_vert(&v_pos, &v_clip, vert_i);

        /* vertex post-processing */

        const float w = v_pos[3], nw = -w;

        if ((v_pos[0] < nw) << 0 | (v_pos[0] > w) << 1 |
            (v_pos[1] < nw) << 2 | (v_pos[1] > w) << 3 |
            (v_pos[2] < nw) << 4 | (v_pos[2] > w) << 5) {
            // point out of screen area, cull
            // TODO: perhaps account for point size (out of spec for OpenGL but nvidia does this)
            continue;
        }

        v_clip.x = v_pos[0] * gs.viewport_transform_params[0][0] + gs.viewport_transform_params[0][1];
        v_clip.y = v_pos[1] * gs.viewport_transform_params[1][0] + gs.viewport_transform_params[1][1];
        v_clip.z = v_pos[2] * gs.viewport_transform_params[2][0] + gs.viewport_transform_params[2][1];

        // prim passed, safely add to v2f clip_buf
        uint32_t irq = spin_lock_blocking(locks[0]);
        v2f.clip_buf[v2f.prim_count] = v_clip;
        v2f.prim_count++;
        spin_unlock(locks[0], irq);
    }

    // wait for all cores
    // mc_barrier();
}

void gcs_vertex_batch(struct scs_vertex_batch *batch) {
    v2f.prim_count = 0;

    switch (gs.rasterizer_mode) {
    case e_prim_trig:
        mc_dispatch((void(*))&proc_trigs, batch);
        proc_trigs(batch);
        break;

    case e_prim_point:
        mc_dispatch((void(*))&proc_points, batch);
        proc_points(batch);
        break;

    default:
        // FIXME: fault
        break;
    };
}
