#include "common.h"

#include <chip_state.h>
#include <common/gcs_proto.h>
#include <usbd/hostbus_driver.h>

static inline void dispatch_vertex(float v_pos[4], const void *attrib_buf) {
    /* user vertex shader */

    const float *in_pos = (float *)(attrib_buf);

    v_pos[0] = in_pos[0];
    v_pos[1] = in_pos[1];
    v_pos[2] = in_pos[2];
    v_pos[3] = 1.f;

    /* vertex early post-process */

    // prespective divide -> to NDCs
    float w_inv = 1.f / v_pos[3];
    v_pos[0] *= w_inv;
    v_pos[1] *= w_inv;
    v_pos[2] *= w_inv;

    // TODO: shader attrib divides...
}

/* vertex stage entry */

struct clip_point clip_buf[sizeof(struct clip_point) * MAX_VERTICES_PER_VSTREAM];

void process_vertex_stream(struct gcs_vs_header *stream) {
    static const uint8_t v_count = 3;

    // local: vertex index in this vertex stream; global: vertex index in the entire draw command
    uint32_t local_vertex_index = 0, global_vertex_index = stream->base_vertex;
    uint32_t output_primitive_count = 0;

    const uint8_t *attrib_buf = (uint8_t *)(stream + 1);

    // accumulated shading_area between all prims
    int32_t shading_area[4] = {((struct gcs_state *)chip_state.cbuf)->fb_extent[0], ((struct gcs_state *)chip_state.cbuf)->fb_extent[1], 0, 0};

    for (uint32_t prim_i = 0; prim_i < stream->prim_count; prim_i++) {
        /* vertex stage */

        float v_positions[v_count][4];

        // Cohen–Sutherland algo out-codes (only using 6 bits)
        uint8_t v_out_codes = 0x3F;  // AND'ed viewport
        uint8_t gb_out_codes = 0x00; // OR'ed guard-band

        for (uint8_t pvi = 0; pvi < v_count; global_vertex_index++, pvi++) {
            // vertex shader
            dispatch_vertex(v_positions[pvi], ((float *)attrib_buf) + (local_vertex_index + pvi)*v_count);

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

        /* if (v_out_codes) {
            // prim entirely outside the viewport, cull it

            while (true) { watchdog_update(); }
            continue;
        } */

        /* if (gb_out_codes) {
            // prim both outside the guard-bands and inside viewport, very rare, perform slow clip

            format_dbg("FIXME: unimplemented guard-band clipping reached, culling instead!");
            continue;
        } */

        for (uint8_t pvi = 0; pvi < v_count; pvi++) {
            // viewport transform

            // at this point it's safe to convert the float [v_positions] to
            // [clip_point]s fixed-point int32s without over/under flow

            struct clip_point *clip = &clip_buf[local_vertex_index + pvi];

            *clip = (struct clip_point){
                .x = v_positions[pvi][0] * ((struct gcs_state *)chip_state.cbuf)->viewport_transform_params[0][0] + ((struct gcs_state *)chip_state.cbuf)->viewport_transform_params[0][1],
                .y = v_positions[pvi][1] * ((struct gcs_state *)chip_state.cbuf)->viewport_transform_params[1][0] + ((struct gcs_state *)chip_state.cbuf)->viewport_transform_params[1][1],
                .z = v_positions[pvi][2] * ((struct gcs_state *)chip_state.cbuf)->viewport_transform_params[2][0] + ((struct gcs_state *)chip_state.cbuf)->viewport_transform_params[2][1],
            };

            // min/max the shading areas

            shading_area[0] = shading_area[0] < clip->x ? shading_area[0] : clip->x;
            shading_area[1] = shading_area[1] < clip->y ? shading_area[1] : clip->y;
            shading_area[2] = shading_area[2] > clip->x ? shading_area[2] : clip->x;
            shading_area[3] = shading_area[3] > clip->y ? shading_area[3] : clip->y;
        }

        /* late primitive processing */

        // TODO: implement face culling with NDCs
        // FIXME: reorder vertices to force ccw prims

        local_vertex_index += v_count;
        output_primitive_count++;
    }

    /* primitive output */

    struct gcs_po_header p = {gcs_type_po, output_primitive_count};
    memcpy(p.shading_area, shading_area, sizeof(p.shading_area));

    hostbus_xfer_out(&p, sizeof(p));
    hostbus_xfer_out(clip_buf, sizeof(struct clip_point) * output_primitive_count * 3);
}