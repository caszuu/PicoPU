#include "graphics_state.h"

#include "../common/cluster_bus.h"
#include "../common/ex_simd.h"

#include <hardware/interp.h>

#include "pico/stdlib.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* rasterization stage */

void exec_fragment_stage(u16_x2_simd p, int32_t w_tile[3 * 4], int16_t area,
                         uint8_t cv_mask);
void stream_fragment_output();
void reset_fragment_output();

// rasterizer and fragment stage implementation originaly from the amazing
// Optimizing Software Occlusion Culling series
// https://fgiesen.wordpress.com/2013/02/17/optimizing-sw-occlusion-culling-index/

/* trig rasterizer */

bool is_top_left(const struct clip_point *p0, const struct clip_point *p1, bool is_cw) {
    return (p0->y == p1->y && (p0->x > p1->x != is_cw)) || // is_top - the is_cw is techically not correct when
                                                           // p0[0] == p1[0], but close enough
           (p0->y > p1->y != is_cw);                       // is_left - again, also not correct for is_cw trigs
}

struct trig_edge {
    int16_t a, b;
    int16_t x_step, y_step;

    int32_t w;
};

struct trig_edge init_trig_edge(const struct clip_point *p0,
                                const struct clip_point *p1,
                                const u16_x2_simd orig) {
    struct trig_edge e;

    // edge setup

    // FIXME: check for possible overflows in barycentric calcs.
    // FIXME: re-enable (faulty?) top-left fill rule

    e.a = p0->y - p1->y;
    e.b = p1->x - p0->x;

    int32_t c = p0->x * p1->y - p0->y * p1->x; // - (is_top_left(p0, p1, false)) /* fill rule bias */;

    // step deltas
    e.x_step = e.a * RENDER_QUAD_SIZE;
    e.y_step = e.b * RENDER_QUAD_SIZE;

    // edge function value at origin

    e.w = e.a * orig.v[0] + e.b * orig.v[1] + c;

    return e;
}

// perform early depth test and execute fragment shaders
// an aligned 2x2 quad of pixels

void render_frag_quad(u16_x2_simd p, int32_t ws[3 * 4], int16_t area,
                      uint8_t cv_mask /*note: top 4-bit will be 1*/) {
    /* request_ds_tile(p); // PERF TODO: request p+1 tile for parallel wait

    // eraly depth-test

    struct depth_tile prev_dt = pull_ds_tile();
    struct depth_tile dt = interp_ds_tile(ws);

    // hw interp max depth

    // PERF TODO: hw interp depth max and xor simd compare

    cv_mask &=  (prev_dt.d[0] < dt.d[0]) << 0 |
                (prev_dt.d[1] < dt.d[1]) << 1 |
                (prev_dt.d[2] < dt.d[2]) << 2 |
                (prev_dt.d[3] < dt.d[3]) << 3;

    // early tile discard
    if (!cv_mask) {
        return;
    }

    // hw interp select on coverage

    request_col_tile(p);
    patch_ds_tile(&dt); */

    // execute fragment

    exec_fragment_stage(p, ws, area, cv_mask);
}

// there might be better way to rasterize along edges based on
// interp hw to skip most outside pixels but this will do for now

// TODO: subpixel_precision, perf_counters

// currently implements: non-corrected top-left fill rule, 2x2 tile steps,
// expects cw trigs (corrected in vertex stage)

extern uint8_t packet_buffer[MAX_GCS_PACKET_SIZE];

void raster_trig(const struct gcs_fs_header *stream) {
    // confusing note: due to an unknown cause, when [stream] is used in this
    //                 ptr lookup instead of the direct [packet_buffer] (which
    //                 should be the same ptr) the pico for some reason crashes
    //                 on reads of [clip_points], maybe a miscompile or some
    //                 other obscure reason, but i sure didn't find it

    struct clip_point clip_points[3] = {
        {0, 0, 100},
        {127, 128, 100},
        {0, 127, 100},
    };

    format_dbg("p: %u", clip_points[1].y);

    // interp. clip bounding box (trig bound box, screen box, scissors and frag_stream box)

    // min/max bounds by "inverse" clamping all different bboxes
    interp_config cfg = interp_default_config();
    interp_config_set_clamp(&cfg, true);
    interp_config_set_signed(&cfg, true);
    interp_set_config(interp1, 0, &cfg);

    // min_x

    interp1->base[0] = (int32_t)0;       // initial max value (screen clipping)
    interp1->base[1] = clip_points[0].x; // initial min value

    interp1->accum[0] = clip_points[1].x;
    interp1->base[1] = interp1->peek[0];

    interp1->accum[0] = clip_points[2].x;

    screen_axis_t min_x = interp1->peek[0];

    // min_y

    interp1->base[0] = (int32_t)stream->line_offset; // initial max value (screen clipping)
    interp1->base[1] = clip_points[0].y;             // initial min value

    interp1->accum[0] = clip_points[1].y;
    interp1->base[1] = interp1->peek[0];

    interp1->accum[0] = clip_points[2].y;

    screen_axis_t min_y = interp1->peek[0];

    // max_x

    interp1->base[1] = (int32_t)fb_extent[0] - 1; // initial min value (screen clipping)
    interp1->base[0] = clip_points[0].x;          // initial max value

    interp1->accum[0] = clip_points[1].x;
    interp1->base[0] = interp1->peek[0];

    interp1->accum[0] = clip_points[2].x;

    screen_axis_t max_x = interp1->peek[0];

    // max_y

    interp1->base[1] = (int32_t)stream->line_offset + (int32_t)stream->line_count; // initial min value (screen clipping)
    interp1->base[0] = clip_points[0].y;                                           // initial max value

    interp1->accum[0] = (int32_t)fb_extent[1] - 1;
    interp1->base[1] = interp1->peek[0];

    interp1->accum[0] = clip_points[1].y;
    interp1->base[0] = interp1->peek[0];

    interp1->accum[0] = clip_points[2].y;

    screen_axis_t max_y = interp1->peek[0];

    // TODO: split core work on x axis

    /* rasterize */

    u16_x2_simd p = {min_y, min_x};

    format_dbg("raster prepared: %d %d %d %d", min_x, min_y, max_x, max_y);
    sleep_ms(100);

    // precalc. Barycentric coordicates and offset them on every step
    struct trig_edge e0 = init_trig_edge(&clip_points[1], &clip_points[2], p);
    struct trig_edge e1 = init_trig_edge(&clip_points[2], &clip_points[0], p);
    struct trig_edge e2 = init_trig_edge(&clip_points[0], &clip_points[1], p);

    // precalc. double trig area (for barycentric norm)
    int16_t trig_area = e0.a * clip_points[0].x + e0.b * clip_points[0].y +
                        (clip_points[1].x * clip_points[2].y - clip_points[1].y * clip_points[2].x);

    reset_fragment_output();

    for (; p.v[1] <= max_y; p.v[1] += RENDER_QUAD_SIZE) {
        int16_t ws[] = {e0.w, e1.w, e2.w};

        for (p.v[0] = min_x; p.v[0] <= max_x; p.v[0] += RENDER_QUAD_SIZE) {
            // PERF TODO: early-out by w test

            // from what i can tell, for a non-simd system, this is close to the
            // most optimal [ws] and [cv_mask] calculation as i can get on godbold with gcc (-O2)
            //
            // giving the compiler as much room as possible to optimize yields
            // the best results, the m33's register cound and alu is the
            // bottlenect here (i think)
            //
            // if you find a more optimal way to test and iterate the w params
            // feel free to PR!

            /* clang-format off */

            int32_t w_tile[3 /*edge count*/ * 4 /*tile pixel count*/] = {
                ws[0], ws[1], ws[2],
                ws[0] + e0.a, ws[1] + e1.a, ws[2] + e2.a,
                ws[0] + e0.b, ws[1] + e1.b, ws[2] + e2.b,
                ws[0] + e0.a + e0.b, ws[1] + e1.a + e1.b, ws[2] + e2.a + e2.b,
            };

            uint8_t cv_mask =
                ((w_tile[0 + 0] | w_tile[0 + 1] | w_tile[0 + 2]) >= 0) << 0 |
                ((w_tile[3 + 0] | w_tile[3 + 1] | w_tile[3 + 2]) >= 0) << 1 |
                ((w_tile[6 + 0] | w_tile[6 + 1] | w_tile[6 + 2]) >= 0) << 2 |
                ((w_tile[9 + 0] | w_tile[9 + 1] | w_tile[9 + 2]) >= 0) << 3;

            /* clang-format on */

            // condition equivalent to: any(w + tile_offset >= 0)
            if (cv_mask) {
                render_frag_quad(p, w_tile, trig_area, cv_mask);
            } else {
                // tile gap, stream buffered tiles
                stream_fragment_output();
            }

            sleep_us(200);

            // offset step
            ws[0] += e0.x_step;
            ws[1] += e1.x_step;
            ws[2] += e2.x_step;
        }

        // row step, stream buffered tiles
        stream_fragment_output();

        format_dbg("row: %u", p.v[1]);
        // sleep_ms(1000);

        // offset row
        e0.w += e0.y_step;
        e1.w += e1.y_step;
        e2.w += e2.y_step;
    }
}

void process_fragment_stream(struct gcs_fs_header *stream) {
    raster_trig(stream);

    struct gcs_ready p = {gcs_type_ready};
    uint16_t p_size = sizeof(p);

    put_buffer(&p_size, sizeof(uint16_t));
    put_buffer(&p, sizeof(p));
    stdio_flush();
}
