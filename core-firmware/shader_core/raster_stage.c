#include "common/picopu_types.h"
#include "graphics_state.h"

#include <usbd/hostbus_driver.h>
#include "chip_state.h"

#include "../common/cluster_bus.h"
#include "../common/ex_simd.h"

#include <hardware/interp.h>

#include "pico/stdlib.h"
#include <pico/platform/compiler.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// rasterization stage //

void exec_fragment_stage(v2i32 p, v4i32 w_tile[3], int32_t area,
                         uint8_t cv_mask);
void stream_fragment_output();
void reset_fragment_output();

// rasterizer and fragment stage implementations originaly from the amazing
// Optimizing Software Occlusion Culling series
// https://fgiesen.wordpress.com/2013/02/17/optimizing-sw-occlusion-culling-index/

bool is_top_left(const struct clip_point *p0, const struct clip_point *p1, bool is_cw) {
    return (p0->y == p1->y && (p0->x > p1->x != is_cw)) || // is_top - the is_cw is techically not correct when
                                                           // p0[0] == p1[0], but close enough
           (p0->y > p1->y != is_cw);                       // is_left - again, also not correct for is_cw trigs
}

// multi-trig rasterizer //

struct trig_edge {
    int32_t at, bt; // the tile size times the a and b edge function params (used for x and y steps)
    int32_t w;
};

static inline struct trig_edge init_trig_edge(const struct clip_point *p0,
                                              const struct clip_point *p1) {
    struct trig_edge e;

    // edge setup

    // FIXME: check for possible overflows in barycentric calcs.
    // FIXME: re-enable (faulty?) top-left fill rule

    int32_t a = p0->y - p1->y;
    int32_t b = p1->x - p0->x;

    int32_t c = p0->x * p1->y - p0->y * p1->x; // - (is_top_left(p0, p1, false)) /* fill rule bias */;

    // step deltas
    e.at = a * RENDER_QUAD_SIZE;
    e.bt = b * RENDER_QUAD_SIZE;

    // edge function value at {0, 0} (later offset by offset_trig_edge)
    e.w = c;

    return e;
}

static inline int32_t offset_trig_edge(struct trig_edge e, v2i32 offset) {
    return e.at / RENDER_QUAD_SIZE * offset[0] + e.bt / RENDER_QUAD_SIZE * offset[1] + e.w;
}

/* rasterizer state and setup impl */

struct rasterizer_state {
    // active shading range
    screen_axis_t min_x, max_x, min_y, max_y;

    // active trig states
    struct trig_edge trigs_e0[MAX_PRIMS_PER_FSTREAM];
    struct trig_edge trigs_e1[MAX_PRIMS_PER_FSTREAM];
    struct trig_edge trigs_e2[MAX_PRIMS_PER_FSTREAM];

    rast_int_t trig_areas[MAX_PRIMS_PER_FSTREAM]; // note: stores the *double* of the trig window-space area
    v4u16 trig_ranges[MAX_PRIMS_PER_FSTREAM];
};

static struct rasterizer_state rast_state;

// setup initial trig edge values and areas
static inline void rast_trig_setup(const struct gcs_fs_header *stream) {
    const struct clip_point *clip_buf = (struct clip_point *)((uint8_t *)(stream) + sizeof(struct gcs_fs_header));

    for (uint8_t i = 0; i < stream->prim_count; i++) {
        const struct clip_point *p0 = &clip_buf[i * 3 + 0];
        const struct clip_point *p1 = &clip_buf[i * 3 + 1];
        const struct clip_point *p2 = &clip_buf[i * 3 + 2];

        rast_state.trigs_e0[i] = init_trig_edge(p2, p1);
        rast_state.trigs_e1[i] = init_trig_edge(p0, p2);
        rast_state.trigs_e2[i] = init_trig_edge(p1, p0);

        rast_state.trig_areas[i] = offset_trig_edge(init_trig_edge(p2, p1), (v2i32){p0->x, p0->y});

        rast_state.trig_ranges[i] = (v4u16){
            MIN(MIN(p0->x, p1->x), p2->x), // min_x
            MIN(MIN(p0->y, p1->y), p2->y), // min_y
            MAX(MAX(p0->x, p1->x), p2->x), // max_x
            MAX(MAX(p0->y, p1->y), p2->y), // max_y
        };
    }
}

/* rasterizer loop */

// setup w_tile buf and call the fragment entry point
static inline void rast_trig_dispatch_frag(/*register*/ v4i32 w_tiles[], v2i32 p, int32_t area, uint8_t cv_mask) {
    // copy out the 'register' [w_tiles] to a unrelated location as to not force the compiler to flush out the original [w_tiles]
    v4i32 w_buf[] = {w_tiles[0], w_tiles[1], w_tiles[2]};

    exec_fragment_stage(p, w_buf, area, cv_mask);
}

// rasterize the currently setup trig state and dispatch fragment invocations
static inline void rast_trigs(const struct gcs_fs_header *stream) {
    format_dbg("raster prepared: %d %d %d %d", rast_state.min_x, rast_state.min_y, rast_state.max_x, rast_state.max_y);
    // sleep_ms(100);

    reset_fragment_output();

    for (uint8_t trig_i = 0; trig_i < stream->prim_count; trig_i++) {
        // current texel position and trimmed shading range

        uint16_t min_x = MAX(rast_state.min_x, rast_state.trig_ranges[trig_i][0]);
        uint16_t min_y = MAX(rast_state.min_y, rast_state.trig_ranges[trig_i][1]);
        uint16_t max_x = MIN(rast_state.max_x, rast_state.trig_ranges[trig_i][2]);
        uint16_t max_y = MIN(rast_state.max_y, rast_state.trig_ranges[trig_i][3]);

        v2i32 p = {min_x, min_y};

        format_dbg("raster prepared: %d %d %d %d", min_x, min_y, max_x, max_y);

        // FIXME: trim shading range to the current trig

        // edge function tile setup (calc. in-tile offsets + initial edge values)

        struct trig_edge e0 = rast_state.trigs_e0[trig_i];
        struct trig_edge e1 = rast_state.trigs_e1[trig_i];
        struct trig_edge e2 = rast_state.trigs_e2[trig_i];

        rast_int_t trig_area = rast_state.trig_areas[trig_i];

        // edge function hot-storage, even tho not explicitly it's an alias for the 6 64-bit simd registers holding it
        v4i32 w_tiles[] = {
            (v4i32){0, e0.at, e0.bt, e0.at + e0.bt} / RENDER_QUAD_SIZE + offset_trig_edge(e0, (v2i32)p), // edge 0
            (v4i32){0, e1.at, e1.bt, e1.at + e1.bt} / RENDER_QUAD_SIZE + offset_trig_edge(e1, (v2i32)p), // edge 1
            (v4i32){0, e2.at, e2.bt, e2.at + e2.bt} / RENDER_QUAD_SIZE + offset_trig_edge(e2, (v2i32)p), // edge 2
        };

        // raster (tight) loop-over

        // PERF NOTE:
        //   on the Cortex-M33 (RP2350), the entirity of w_tiles is stored in the VFP/SIMD register file
        //   for the duration of the for loop to avoid stalling for 48 bytes of edge data on every loop
        //   (the w_tiles will most likely get evicted when dispatching fragment invocations but not otherwise)

        //   please make sure after every change to check the disaasembly that the vectors are still being preserved
        //   and not being evicted on every loop step

        for (; p[1] <= max_y; p[1] += RENDER_QUAD_SIZE) {
            int i = 0;

            for (p[0] = min_x; p[0] <= max_x; (p[0] += RENDER_QUAD_SIZE, i++)) {
                // check for any fragments which are under *all* edge functions
                v4i32 simd_cv_mask = (w_tiles[0] | w_tiles[1] | w_tiles[2]) >= 0;

                // pack an usable coverage mask from the 128-bit simd variant above
                uint8_t cv_mask = (simd_cv_mask[0] & 1) | (simd_cv_mask[1] & (1 << 1)) | (simd_cv_mask[2] & (1 << 2)) | (simd_cv_mask[3] & (1 << 3));

                // dispatch fragment stage for this tile if any fragments are covered
                if (cv_mask) {
                    rast_trig_dispatch_frag(w_tiles, p, trig_area, cv_mask);
                } else {
                    // tile gap, stream buffered tiles
                    stream_fragment_output();
                }

                // offset step
                w_tiles[0] += e0.at;
                w_tiles[1] += e1.at;
                w_tiles[2] += e2.at;
            }

            // row step, stream buffered tiles
            stream_fragment_output();

            // reset step
            w_tiles[0] -= e0.at * i;
            w_tiles[1] -= e1.at * i;
            w_tiles[2] -= e2.at * i;

            // offset row
            w_tiles[0] += e0.bt;
            w_tiles[1] += e1.bt;
            w_tiles[2] += e2.bt;
        }
    }
}

/* raster and fragment stage entry point */

static void process_trigs(struct gcs_fs_header *stream) {
    // setup initial state

    screen_axis_t range_tile_count_x = (stream->shading_range[2] - stream->shading_range[0]) / RENDER_QUAD_SIZE;
    screen_axis_t range_tile_count_y = (stream->shading_range[3] - stream->shading_range[1]) / RENDER_QUAD_SIZE;

    if (range_tile_count_x * range_tile_count_y > /*MAX_INLINE_TILES*/ 32) {
        // FIXME: figure out how to split into multiple ranges
    }

    rast_state.min_x = stream->shading_range[0];
    rast_state.min_y = stream->shading_range[1];
    rast_state.max_x = stream->shading_range[2];
    rast_state.max_y = stream->shading_range[3];

    rast_trig_setup(stream);

    // start rasterizing shading range(s)

    rast_trigs(stream);
}

void process_fragment_stream(struct gcs_fs_header *stream) {
    switch (((struct gcs_state *)(chip_state.cbuf))->rasterizer_mode) {
    case e_prim_trig:
        process_trigs(stream);
        break;

        // case e_prim_line:
        //     process_lines(stream);
        //     break;

        // case e_prim_point:
        //     process_points(stream);
        //     break;

    default:
        // FIXME: fault
        break;
    }

    struct gcs_ready p = {gcs_type_ready};
    hostbus_xfer_out(&p, sizeof(p));
}
