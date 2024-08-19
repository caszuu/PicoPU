#include "graphics_state.h"

#include "../common/cluster_bus.h"
#include "../common/ex_simd.h"

#include <hardware/interp.h>

#include "pico/stdlib.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* rasterization stage */

bool is_top_left(const struct clip_point* p0, const struct clip_point* p1, bool is_cw) {
    return (p0->y == p1->y && (p0->x > p1->x != is_cw)) || // is_top - the is_cw is techically not correct when p0[0] == p1[0], but close enough
           (p0->y > p1->y != is_cw);                       // is_left - again, also not correct for is_cw trigs
}

// "inverse" two's-compliment: due to bit-packed ex_simd types not being well defined when crossing the signed 0 <-> -1 barrier (causing an overflow),
//                             we invert the sign bit turning INT32_MIN into 0x0000, INT32_MAX into 0xFFFF and most notably 0 into 0x8000 and -1 into 0x7FFF
//                             closing the overflow gap between the positive and negative halfs and allowing us to do unsigned arithmetic with signed values
//                             note: this only porperly works for addition and subtraction, multiplies and divides will *not* be correct in this form

struct trig_edge {
    uint16_t x_step;
    uint16_t y_step;

    int16_t a, b;
    int16_t w;
};

// filters all two's-compliment sign bits in a [u16_x4_simd]
#define SIGN_MASK ((0x8000) | (0x8000 << 16) | (0x8000ULL << 32) | (0x8000ULL << 48))

#define inv_tc(signed_val) (signed_val ^ 0x8000) /*only flip sign bit*/
#define inv_tc_i16_x4_simd(...) (__VA_ARGS__.o ^ SIGN_MASK)
#define inv_tc_i16_x2_simd(...) (__VA_ARGS__.o ^ (SIGN_MASK & UINT32_MAX))

struct trig_edge init_trig_edge(const struct clip_point* p0, const struct clip_point* p1, const u16_x2_simd orig) {
    struct trig_edge e;
    
    // edge setup

    /*
    int16_t a = inv_tc(p0->y - p1->y);
    int16_t b = inv_tc(p1->x - p0->x);

    uint16_t c = inv_tc(p0->x * p1->y - p0->y * p1->x + (is_top_left(p0, p1, false) * -1));

    // step deltas
    uint16_t t = inv_tc(a * RENDER_QUAD_SIZE);
    e.x_step_offset.o = (iu16_x4_simd){ t, t, t, t }.o;

    t = inv_tc(b * RENDER_QUAD_SIZE);
    e.y_step_offset.o = (iu16_x4_simd){ t, t, t, t }.o;

    // initial pixel block

    iu16_x4_simd x = { inv_tc_i16_x4_simd((u16_x4_simd){ orig.o + (u16_x4_simd){ 0, 1, 0, 1 }.o }) };
    iu16_x4_simd y = { inv_tc_i16_x4_simd((u16_x4_simd){ orig.o + (u16_x4_simd){ 0, 0, 1, 1 }.o }) };

    // edge function values at origin

    e.w.o = (iu16_x4_simd){ a, a, a, a }.o * x.o + (iu16_x4_simd){ b, b, b, b }.o * y.o + (iu16_x4_simd){ c, c, c, c }.o;
    */

    /* this is the quivalent of the above (bacause we can't multiply with _simd) */

    // FIXME: check for possible overflows in barycentric calcs.

    int16_t a = p0->y - p1->y;
    int16_t b = p1->x - p0->x;

    int16_t c = p0->x * p1->y - p0->y * p1->x; // - (is_top_left(p0, p1, false)) /* fill rule bias */;

    // step deltas
    e.x_step = a * RENDER_QUAD_SIZE;
    e.y_step = b * RENDER_QUAD_SIZE;

    // initial pixel block

    uint16_t o_x1 = orig.v[0] + 1;
    uint16_t o_y1 = orig.v[1] + 1;

    // edge function values at origin

    // e.w.o = inv_tc_i16_x4_simd((u16_x4_simd){ a * orig.v[0], a * o_x1, a * orig.v[0], a * o_x1 }) + 
    //         inv_tc_i16_x4_simd((u16_x4_simd){ b * orig.v[1], b * orig.v[1], b * o_y1, b * o_y1 }) + 
    //         (iu16_x4_simd){ c, c, c, c }.o;

    e.w = a * orig.v[0] + b * orig.v[1] + c;
    e.a = a;
    e.b = b;

    return e;
}

// perform early depth test and execute fragment shaders
// an aligned 2x2 quad of pixels

void exec_fragment_stage(u16_x2_simd p, iu16_x4_simd* ws, uint8_t cv_mask);
void stream_fragment_output();
void reset_fragment_output();

void render_frag_quad(u16_x2_simd p, iu16_x4_simd* ws, uint8_t cv_mask /*note: top 4-bit will be 1*/) {
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

    exec_fragment_stage(p, ws, cv_mask);
}

// there might be better way to rasterize along edges based on
// interp hw to skip most outside pixels but this will do for now

// rasterizer and fragment stage implementation originaly from the amazing Optimizing Software Occlusion Culling series
// https://fgiesen.wordpress.com/2013/02/17/optimizing-sw-occlusion-culling-index/

// TODO: subpixel_precision, perf_counters
// currently implements: non-corrected top-left fill rule, 2x2 tile steps, hardcoded ccw trigs

extern uint8_t packet_buffer[MAX_GCS_PACKET_SIZE];

void raster_trig(const struct gcs_fs_header* stream) {
    // confusing note: due to an unknown cause, when [stream] is used in this ptr lookup instead of the direct [packet_buffer] (which should be the same ptr)
    //                 the pico for some reason crashes on reads of [clip_points], maybe a miscompile or some other obscure reason, but i sure didn't find it

    struct clip_point clip_points[3] = { { 0, 0, 100 }, { 0, 127, 100 }, { 127, 127, 100 } };// (struct clip_point*)(packet_buffer + sizeof(struct gcs_fs_header) + sizeof(struct clip_point) * 3 * SHADER_CHIP_ID);
    format_dbg("p: %u", clip_points[1].y);
    
    // interp. clip bounding box (trig bound box, screen box, scissors and frag_stream box)

    // min/max bounds by "inverse" clamping all different bboxes
    interp_config cfg = interp_default_config();
    interp_config_set_clamp(&cfg, true);
    interp_config_set_signed(&cfg, true);
    interp_set_config(interp1, 0, &cfg);

    interp1->base[0] = (int32_t)0; // initial max value (screen clipping)
    interp1->base[1] = clip_points[0].x; // initial min value

    interp1->accum[0] = clip_points[1].x;
    interp1->base[1] = interp1->peek[0];

    interp1->accum[0] = clip_points[2].x;

    // interp1->accum[0] = scissors_state;
    // interp1->base[0] = interp1->peek[0];

    screen_axis_t min_x = interp1->peek[0];

    interp1->base[0] = (int32_t)stream->line_offsets[SHADER_CHIP_ID]; // initial max value (screen clipping)
    interp1->base[1] = clip_points[0].y; // initial min value

    interp1->accum[0] = clip_points[1].y;
    interp1->base[1] = interp1->peek[0];

    interp1->accum[0] = clip_points[2].y;

    // interp1->accum[0] = scissors_state;
    // interp1->base[1] = interp1->peek[0];

    screen_axis_t min_y = interp1->peek[0];

    interp1->base[1] = (int32_t)fb_extent[0] - 1; // initial min value (screen clipping)
    interp1->base[0] = clip_points[0].x; // initial max value

    interp1->accum[0] = clip_points[1].x;
    interp1->base[0] = interp1->peek[0];

    interp1->accum[0] = clip_points[2].x;

    // interp1->accum[0] = scissors_state;
    // interp1->base[1] = interp1->peek[0];

    screen_axis_t max_x = interp1->peek[0];

    interp1->base[1] = (int32_t)stream->line_offsets[SHADER_CHIP_ID] + (int32_t)stream->line_counts[SHADER_CHIP_ID]; // initial min value (screen clipping)
    interp1->base[0] = clip_points[0].x; // initial max value

    interp1->accum[0] = (int32_t)fb_extent[1] - 1;
    interp1->base[1] = interp1->peek[0];

    interp1->accum[0] = clip_points[1].x;
    interp1->base[0] = interp1->peek[0];

    interp1->accum[0] = clip_points[2].x;

    // interp1->accum[0] = scissors_state;
    // interp1->base[1] = interp1->peek[0];

    screen_axis_t max_y = interp1->peek[0];

    // TODO: split core work on x axis

    /* rasterize */

    u16_x2_simd p = { min_y, min_x };

    format_dbg("raster prepared: %d %d %d %d", min_x, min_y, max_x, max_y);
    sleep_ms(100);

    // precalc. Barycentric coordicates and offset them on every step
    struct trig_edge e0 = init_trig_edge(&clip_points[1], &clip_points[2], p);
    struct trig_edge e1 = init_trig_edge(&clip_points[2], &clip_points[0], p);
    struct trig_edge e2 = init_trig_edge(&clip_points[0], &clip_points[1], p);

    reset_fragment_output();

    for (; p.v[1] <= max_y; p.v[1] += RENDER_QUAD_SIZE) {
        // iu16_x4_simd ws[] = { e0.w, e1.w, e2.w };
        uint16_t ws[] = { e0.w, e1.w, e2.w };
        
        for (p.v[0] = min_x; p.v[0] <= max_x; p.v[0] += RENDER_QUAD_SIZE) {
            // iu16_x4_simd mask = { .o = ws[0].o & ws[1].o & ws[2].o & SIGN_MASK };           
            // uint16_t mask = ws[0] & ws[1] & ws[2] & 0x8000;
            u16_x4_simd mask = { 
                ws[0] & ws[1] & ws[2] & 0x8000,
                ws[0] + e0.a & ws[1] + e1.a & ws[2] + e2.a & 0x8000,
                ws[0] + e0.b & ws[1] + e1.b & ws[2] + e2.b & 0x8000,
                ws[0] + e0.a + e0.b & ws[1] + e1.a + e1.b & ws[2] + e2.a + e2.b & 0x8000,
            };
            
            // condition equivalent to: any(mask.v >= 0) note: remember, we're in inverse two's-compliment mode here, therefore checking if any sign bits are 1
            if (mask.o) {
                uint8_t pack_mask = (mask.v[0] >> 15) | (mask.v[1] >> 14) | (mask.v[2] >> 13) | (mask.v[3] >> 12);
                // format_dbg(" cv: 0x%hhx %hhu %hhu %hhu %hhu", pack_mask, (mask.v[0] >> 15), (mask.v[1] >> 14), (mask.v[2] >> 13), (mask.v[3] >> 12));
                // format_dbg(" cv: 0x%llx %llx %llx %llx %llx", SIGN_MASK, ws[0].o, ws[1].o, ws[0].o & ws[1].o & ws[2].o & SIGN_MASK, (iu16_x4_simd){ .o = ws[0].o & ws[1].o & ws[2].o & SIGN_MASK }.o);

                render_frag_quad(p, (iu16_x4_simd*)ws, pack_mask);
            } else {
                // tile gap, stream buffered tiles
                stream_fragment_output();
            }

            sleep_us(500);

            // offset step
            ws[0] += e0.x_step;
            ws[1] += e1.x_step;
            ws[2] += e2.x_step;
        }

        // row step, stream buffered tiles
        stream_fragment_output();

        format_dbg("row: %u", p.v[0]);
        // sleep_ms(1000);

        // offset row
        e0.w += e0.y_step;
        e1.w += e1.y_step;
        e2.w += e2.y_step;
    }
}

void process_fragment_stream(struct gcs_fs_header* stream) {
    raster_trig(stream);

    struct gcs_ready p = { gcs_type_ready };
    uint16_t p_size = sizeof(p);

    put_buffer(&p_size, sizeof(uint16_t));
    put_buffer(&p, sizeof(p));
    stdio_flush();
}
