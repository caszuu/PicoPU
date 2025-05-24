#include "common.h"

#include <chip_state.h>
#include <common/si_proto.h>
#include <usbd/hostbus_driver.h>

static inline void dispatch_vertex(v4f32 *v_pos, struct clip_point *out_clip, const void *attrib_buf) {
    /* user vertex shader */

    const float *in_pos = (float *)(attrib_buf);

    v4f32 p = {
        in_pos[0],
        in_pos[1],
        in_pos[2],
        1.f,
    };

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

void dispatch_vertex_stage(struct scs_vertex_batch *batch) {
    static const uint8_t v_count = 3;

    // local: vertex index in this vertex stream; global: vertex index in the entire draw command
    uint32_t local_vertex_index = 0, global_vertex_index = batch->vertex_base;
    uint32_t output_primitive_count = 0, output_vertex_index = 0;

    const uint8_t *attrib_buf = (uint8_t *)(batch + 1);

    // accumulated shading_area between all prims
    int32_t shading_area[4] = {((struct gcs_cbuf_state *)chip_state.cbuf)->fb_extent[0], ((struct gcs_cbuf_state *)chip_state.cbuf)->fb_extent[1], 0, 0};

    for (uint32_t prim_i = 0; prim_i < batch->primitive_count; prim_i++, local_vertex_index += v_count) {
        /* vertex stage */

        v4f32 v_positions[v_count];

        // Cohen–Sutherland algo out-codes (only using 6 bits)
        uint8_t v_out_codes = 0x3F;  // AND'ed viewport
        uint8_t gb_out_codes = 0x00; // OR'ed guard-band

        for (uint8_t pvi = 0; pvi < v_count; global_vertex_index++, pvi++) {
            // vertex shader
            static const uint32_t attrib_count = 3; // + 2;
            dispatch_vertex(&v_positions[pvi], &v2f.clip_buf[output_vertex_index + pvi], ((float *)attrib_buf) + (local_vertex_index + pvi) * attrib_count);

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

            // format_dbg("FIXME: unimplemented guard-band clipping reached, culling instead!");
            continue;
        }

        for (uint8_t pvi = 0; pvi < v_count; pvi++) {
            // viewport transform

            // at this point it's safe to convert the float [v_positions] to
            // [clip_point]s fixed-point int32s without over/under flow

            struct clip_point *clip = &v2f.clip_buf[output_vertex_index + pvi];

            *clip = (struct clip_point){
                .x = v_positions[pvi][0] * ((struct gcs_cbuf_state *)chip_state.cbuf)->viewport_transform_params[0][0] + ((struct gcs_cbuf_state *)chip_state.cbuf)->viewport_transform_params[0][1],
                .y = v_positions[pvi][1] * ((struct gcs_cbuf_state *)chip_state.cbuf)->viewport_transform_params[1][0] + ((struct gcs_cbuf_state *)chip_state.cbuf)->viewport_transform_params[1][1],
                .z = v_positions[pvi][2] * ((struct gcs_cbuf_state *)chip_state.cbuf)->viewport_transform_params[2][0] + ((struct gcs_cbuf_state *)chip_state.cbuf)->viewport_transform_params[2][1],
            };

            // min/max the shading areas

            shading_area[0] = shading_area[0] < clip->x ? shading_area[0] : clip->x;
            shading_area[1] = shading_area[1] < clip->y ? shading_area[1] : clip->y;
            shading_area[2] = shading_area[2] > clip->x ? shading_area[2] : clip->x;
            shading_area[3] = shading_area[3] > clip->y ? shading_area[3] : clip->y;
        }

        /* late primitive processing */

        // computes (double) the signed area of the trig
        int32_t signed_area = (v2f.clip_buf[output_vertex_index + 1].x - v2f.clip_buf[output_vertex_index + 0].x) * (v2f.clip_buf[output_vertex_index + 2].y - v2f.clip_buf[output_vertex_index + 0].y) -
                              (v2f.clip_buf[output_vertex_index + 2].x - v2f.clip_buf[output_vertex_index + 0].x) * (v2f.clip_buf[output_vertex_index + 1].y - v2f.clip_buf[output_vertex_index + 0].y);

        // check winding order using area (negative area == clockwise) and force counter-clockwise winding for raster stage
        if (signed_area < 0) {
            struct clip_point temp = v2f.clip_buf[output_vertex_index + 0];
            v2f.clip_buf[output_vertex_index + 0] = v2f.clip_buf[output_vertex_index + 1];
            v2f.clip_buf[output_vertex_index + 1] = temp;

            signed_area = -signed_area;
        }

        if (signed_area == 0) {
            // degenerate trig, cull
            continue;
        }

        // if (signed_area < 0) {
        //     // back-face, cull
        //     continue;
        // }

        output_primitive_count++;
        output_vertex_index += v_count;
    }

    // FIXME: shading_range limiting

    memcpy(v2f.shading_range, shading_area, sizeof(v2f.shading_range));
    v2f.prim_count = output_primitive_count;

    struct si_dbg_packet p = {
        .type = si_type_dbg,
    };

    // snprintf(p.dbg_message, MAX_SCS_DBG_SIZE, "vertex dbg, prim_count: %d %d %d", output_primitive_count, ((struct gcs_cbuf_state *)chip_state.cbuf)->fb_extent[0], ((struct gcs_cbuf_state *)chip_state.cbuf)->fb_extent[1]);
    // hostbus_xfer_out(&p, sizeof(p));

    // struct gcs_batch_feedback p = {gcs_type_feedback, output_primitive_count};
    // p.c = output_primitive_count;
    // memcpy(p.shading_range, shading_area, sizeof(v2f.shading_range));

    // hostbus_xfer_out(&p, sizeof(p));
}
