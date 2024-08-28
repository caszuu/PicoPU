#include "../common/ex_simd.h"
#include "common/cluster_bus.h"
#include "common/picopu_types.h"
#include "graphics_state.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// note: this file contains test definitions which would not be included with
// the firmware outside of debugging normally these funcs are part of the
// pipeline mcode

/* fragment stage test def */

#define MAX_INLINE_TILES 32
#define fo_buf_size(tile_count)                                   \
    sizeof(struct gcs_fo_header) +                                \
        sizeof(uint8_t) * ((tile_count / 2) + (tile_count % 2)) + \
        sizeof(struct color_tile) * tile_count +                  \
        sizeof(struct depth_tile) * tile_count

struct gcs_fo_header fo_header = {gcs_type_fo, 0, 0};
uint8_t mask_buf[((MAX_INLINE_TILES / 2) + (MAX_INLINE_TILES % 2))];
struct color_tile ct_buf[MAX_INLINE_TILES];
struct depth_tile dt_buf[MAX_INLINE_TILES];

void reset_fragment_output() {
    fo_header = (struct gcs_fo_header){gcs_type_fo, 0, 0};

    // reset masks which are ORed into the buffer
    memset(mask_buf, 0, ((MAX_INLINE_TILES / 2) + (MAX_INLINE_TILES % 2)));
}

void stream_fragment_output() {
    if (fo_header.tile_count != 0) {
        uint16_t p_size = fo_buf_size(fo_header.tile_count);

        put_buffer(&p_size, sizeof(uint16_t));
        put_buffer(&fo_header, sizeof(struct gcs_fo_header));

        put_buffer(mask_buf, sizeof(uint8_t) * ((fo_header.tile_count / 2) + (fo_header.tile_count % 2)));
        put_buffer(ct_buf, sizeof(struct color_tile) * fo_header.tile_count);
        put_buffer(dt_buf, sizeof(struct depth_tile) * fo_header.tile_count);

        stdio_flush();

        reset_fragment_output();
    }
}

void exec_fragment_stage(u16_x2_simd p, int32_t ws[12], int16_t area,
                         uint8_t cv_mask) {
    // select depth

    // patch_ds_tile(&dt); // only if no gl_FragDepth writes

    // interp vertex attributes

    for (uint8_t i = 0; i < RENDER_QUAD_SIZE * 2; i++) {
        if (!(cv_mask & (1 << i)))
            continue;

        // jit c shader main
    }

    /* struct color_tile ct = {
        (struct rgba_color){p.v[0], p.v[1], 0, 255},
        (struct rgba_color){p.v[0] + 1, p.v[1], 0, 255},
        (struct rgba_color){p.v[0], p.v[1] + 1, 0, 255},
        (struct rgba_color){p.v[0] + 1, p.v[1] + 1, 0, 255},
    }; */

    struct color_tile ct = {
        (struct rgba_color){ws[0 + 0] / (area / 256), ws[0 + 1] / (area / 256), ws[0 + 2] / (area / 256), 255},
        (struct rgba_color){ws[3 + 0] / (area / 256), ws[3 + 1] / (area / 256), ws[3 + 2] / (area / 256), 255},
        (struct rgba_color){ws[6 + 0] / (area / 256), ws[6 + 1] / (area / 256), ws[6 + 2] / (area / 256), 255},
        (struct rgba_color){ws[9 + 0] / (area / 256), ws[9 + 1] / (area / 256), ws[9 + 2] / (area / 256), 255},
    };

    /* struct color_tile ct = {
        (struct rgba_color){255, 0, 255, 255},
        (struct rgba_color){255, 0, 255, 255},
        (struct rgba_color){255, 0, 255, 255},
        (struct rgba_color){255, 0, 255, 255},
    }; */

    struct depth_tile dt = {100, 100, 100, 100};
    // hw interp blend and format conv

    if (fo_header.tile_count == 0) {
        // calc the pixel index of the first tile

        fo_header.fb_index_base = (p.v[0] / 2) * 4 + (p.v[1] / 2 * 2) * fb_extent[0] /* tile index */;
        //                        (p.v[0] % 2) + (p.v[1] % 2) * 2 /* tile offset */;
    }

    mask_buf[fo_header.tile_count / 2] |= cv_mask << (4 * (fo_header.tile_count % 2));
    ct_buf[fo_header.tile_count] = ct;
    dt_buf[fo_header.tile_count] = dt;

    fo_header.tile_count++;

    if (fo_header.tile_count == MAX_INLINE_TILES) {
        stream_fragment_output();
    }
}

/* vertex stage test def */

#include <hardware/interp.h>

// gcs_po buffers
#define po_buf_size(vert_count)                  \
    sizeof(struct gcs_po_header) +               \
        sizeof(struct clip_point) * vert_count + \
        output_vertex_stride *vert_count

struct clip_point clip_buf[sizeof(struct clip_point) * MAX_VERTICES_PER_STREAM];
uint8_t vertex_out_buf[MAX_VERTEX_OUTPUT_STRIDE * MAX_VERTICES_PER_STREAM];

bool exec_vertex_stage(struct gcs_vs_header *stream, void *in_buf) {
    static const uint8_t v_count = 3; // compile-time const

    // local: vertex index in this vertex stream; global: vertex index in the entire draw command
    uint8_t local_vertex_index = 0;
    uint32_t global_vertex_index = stream->base_vertex;

    uint8_t output_primitive_count = stream->primitive_count;

    // accumulated shading_area between all prims
    rast_int_t shading_area[4] = {0, 0, fb_extent[0], fb_extent[1]};

    for (uint8_t prim_i = 0; prim_i < stream->primitive_count; (local_vertex_index += v_count, prim_i++)) {
        /* vertex stage */

        float v_positions[v_count][4];

        // Cohen–Sutherland algo out-codes (only using 6 bits)
        uint8_t v_out_codes = 0x3F;  // AND'ed viewport
        uint8_t gb_out_codes = 0x00; // OR'ed guard-band

        for (uint8_t pvi = 0; pvi < v_count; (global_vertex_index++, pvi++)) {
            // vertex shader

            const float *in_pos = (float *)(in_buf) + local_vertex_index + pvi;

            v_positions[pvi][0] = in_pos[0];
            v_positions[pvi][1] = in_pos[1];
            v_positions[pvi][2] = in_pos[2];
            v_positions[pvi][3] = 1.f;

            format_dbg("shader vertex %f, %f, %f", v_positions[pvi][0], v_positions[pvi][1], v_positions[pvi][2]);

            vertex_out_buf[(local_vertex_index + pvi) * output_vertex_stride] = in_pos[0];

            // post-shader

            const float w = v_positions[pvi][3], nw = -w;

            // SPEC NOTE: based on OpenGL, D3D and Vulkan must use 0 < z < w
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

            output_primitive_count--;
            local_vertex_index -= v_count;

            continue;
        }

        if (gb_out_codes) {
            // prim both outside the guard-bands and inside viewport, very rare, slow clip

            format_dbg("FIXME: unimplemented guard-band clipping reached, culling instead!");

            output_primitive_count--;
            local_vertex_index -= v_count;

            continue;
        }

        for (uint8_t pvi = 0; pvi < v_count; pvi++) {
            // prespective divide -> to NDCs

            v_positions[pvi][0] /= v_positions[pvi][3];
            v_positions[pvi][1] /= v_positions[pvi][3];
            v_positions[pvi][2] /= v_positions[pvi][3];

            // viewport transform

            // at this point it's safe to convert the float [v_positions] to
            // [clip_point]s fixed-point rast_ints without over/under flow

            // PERF TODO: MAC dsp instructions?

            struct clip_point *clip = &clip_buf[local_vertex_index + pvi];

            *clip = (struct clip_point){
                .x = v_positions[pvi][0] * view_transform_params[0][0] + view_transform_params[0][1],
                .y = v_positions[pvi][1] * view_transform_params[1][0] + view_transform_params[1][1],
                .d = v_positions[pvi][2] * view_transform_params[2][0] + view_transform_params[2][1],
            };

            // min/max the shading area

            shading_area[0] = shading_area[0] < clip->x ? shading_area[0] : clip->x;
            shading_area[1] = shading_area[1] < clip->y ? shading_area[1] : clip->y;
            shading_area[2] = shading_area[2] > clip->x ? shading_area[2] : clip->x;
            shading_area[3] = shading_area[3] > clip->y ? shading_area[3] : clip->y;

            format_dbg("clip vertex %d, %d, %f", clip->x, clip->y, clip->d);
        }

        /* late primitive processing */

        // TODO: implement face culling with NDCs
        // FIXME: reorder vertices to force ccw prims
    }

    /* primitive output */

    const uint16_t output_vertex_count = local_vertex_index;

    struct gcs_po_header p = {gcs_type_po, output_primitive_count};
    uint16_t p_size = po_buf_size(output_vertex_count);

    memcpy(&p.shading_area, shading_area, sizeof(p.shading_area));

    put_buffer(&p_size, sizeof(uint16_t));
    put_buffer(&p, sizeof(p));

    put_buffer(clip_buf, sizeof(struct clip_point) * output_vertex_count);
    put_buffer(vertex_out_buf, output_vertex_stride * output_vertex_count);

    stdio_flush();
}
