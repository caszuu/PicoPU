#include "../common/cluster_bus.h"
#include "../common/ex_simd.h"

#include <stdint.h>
#include <stdbool.h>

/* rasterization stage */

bool is_top_left(const struct clip_point* p0, const struct clip_point* p1, bool is_cw) {
    return (p0->y == p1->y && (p0->x > p1->x != is_cw)) || // is_top - the is_cw is techically not correct when p0[0] == p1[0], but close enough
           (p0->y > p1->y != is_cw);                       // is_left - again, also not correct for is_cw trigs
}

struct trig_edge {
    i16_x4_simd x_step_offset;
    i16_x4_simd y_step_offset;  

    i16_x4_simd w;
};

// filters all two's-compliment sign bits in a [i16_x4_simd]
#define SIGN_MASK (0x8000) | (0x8000 << 16) | (0x8000ULL << 32) | (0x8000ULL << 48)

struct trig_edge init_trig_edge(const struct clip_point* p0, const struct clip_point* p1, const i16_x2_simd orig) {
    struct trig_edge e;
    
    // edge setup

    // FIXME: check for possible overflows in barycentric calcs.

    int16_t a = p0->y - p1->y;
    int16_t b = p1->x - p0->x;

    int16_t c = p0->x * p1->y - p0->y * p1->x + (is_top_left(p0, p1, false) * -1) /* fill rule bias */;

    // step deltas
    int16_t t = a * RENDER_QUAD_SIZE;
    e.x_step_offset.o = (i16_x4_simd){ t, t, t, t }.o;

    t = b * RENDER_QUAD_SIZE;
    e.y_step_offset.o = (i16_x4_simd){ t, t, t, t }.o;

    // initial pixel block

    i16_x4_simd x = { orig.o + (i16_x4_simd){ 0, 1, 0, 1 }.o };
    i16_x4_simd y = { orig.o + (i16_x4_simd){ 0, 0, 1, 1 }.o };

    // edge function values at origin

    e.w.o = (i16_x4_simd){ a, a, a, a }.o * x.o + (i16_x4_simd){ b, b, b, b }.o * y.o + (i16_x4_simd){ c, c, c, c }.o;

    return e;
}

// perform early depth test and execute fragment shaders
// an aligned 2x2 quad of pixels

void render_frag_quad(i16_x2_simd p, i16_x4_simd* ws, uint8_t cv_mask /*note: top 4-bit will be 1*/) {
    request_ds_tile(p); // PERF TODO: request p+1 tile for parallel wait

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
    patch_ds_tile(&dt);

    // execute fragment

    exec_fragment_stage(p, ws, cv_mask);

    {
        // select depth

        patch_ds_tile(&dt); // only if no gl_FragDepth writes

        // interp vertex attributes

        for (uint8_t i = 0; i < RENDER_QUAD_SIZE * 2; i++) {
            if (!(cv_mask & (1 << i))) continue;
            
            // jit c shader main
        }

        // late depth test: if gl_FragDepth writes
        // patch_ds_tile(&dt);

        struct color_tile ct = pull_col_tile();

        // hw interp blend and format conv

        fragment_output(&ct, &dt);
    }
}

// there might be better way to rasterize along edges based on
// interp hw to skip most outside pixels but this will do for now

// TODO: subpixel_precision, perf_counters
// currently implements: non-corrected top-left fill rule, 2x2 tile steps, hardcoded ccw trigs

void raster_trig(struct clip_point clip_points[3], const struct gcs_fs_header* stream) {
    // interp. min/max (trig bound box, screen box, scissors and frag_stream box)
    
    uint16_t min_x = ;
    uint16_t min_y = stream->min_y;
    uint16_t max_x = ;
    uint16_t max_y = stream->max_y;

    // TODO: split core work on x axis

    // rasterize
    
    i16_x2_simd p = { min_y, min_x };

    // precalc. Barycentric coordicates and offset them on every step
    struct trig_edge e0 = init_trig_edge(&clip_points[1], &clip_points[2], p);
    struct trig_edge e1 = init_trig_edge(&clip_points[2], &clip_points[0], p);
    struct trig_edge e2 = init_trig_edge(&clip_points[0], &clip_points[1], p);

    for (; p.v[0] <= max_y; p.v[0] += RENDER_QUAD_SIZE) {
        i16_x4_simd ws[] = { e0.w, e1.w, e2.w };
        
        for (; p.v[1] <= max_x; p.v[1] += RENDER_QUAD_SIZE) {
            i16_x4_simd mask = { (ws[0].o | ws[1].o | ws[2].o) & SIGN_MASK };

            // condition equivalent to: any(mask.v >= 0)
            if (!(~mask.o)) {
                uint8_t pack_mask = ~((mask.o & 1) | ((mask.o >> 16) & 1) | ((mask.o >> 32) & 1) | ((mask.o >> 48) & 1));
                
                render_frag_quad(p, ws, pack_mask);
            }

            // offset step
            ws[0].o += e0.x_step_offset.o;
            ws[1].o += e1.x_step_offset.o;
            ws[2].o += e2.x_step_offset.o;
        }

        // offset row
        e0.w.o += e0.y_step_offset.o;
        e1.w.o += e1.y_step_offset.o;
        e2.w.o += e2.y_step_offset.o;
    }
}
