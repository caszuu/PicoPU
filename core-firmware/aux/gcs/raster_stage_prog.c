#include "common.h"
#include "sampler.h"
#include "shader_env.h"

#include <chip.h>
#include <common/ex_simd.h>
#include <common/si_proto.h>

#include <common/dma_mem.h>
#include <common/instru.h>

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* multi-trig rasterizer */

struct trig_params {
    // edge function params
    int32_t a[3], b[3], c[3];

    float z[3];
    float area_inv; // note: inverse of *double* the trigs screen-space area

    int32_t culld[3];
};

struct tile_slot {
    uint16_t c_tile[RASTER_TILE_SIZE * RASTER_TILE_SIZE];
    uint16_t z_tile[RASTER_TILE_SIZE * RASTER_TILE_SIZE];

    v2i32 p;

    uint32_t cv_mask; // only 16 bits are used for TILE_SIZE == 4
    uint32_t trig_mask;
};

struct vis_edge {
    uint16_t edge; // the screen-space coord of this edge
    uint8_t trig;  // what trig this edge applies to
    bool pol;      // edge polarity (true == trig is now visible; false == trig is no longer visible)
};

// pull vertex-to-fragment state buffer from vertex_stage.c
extern struct gcs_v2f_state v2f;

static struct trig_params trigs[MAX_TRIGS_PER_BATCH];
static struct tile_slot slots[4];

static struct vis_edge x_edges[MAX_TRIGS_PER_BATCH * 2];
static struct vis_edge y_edges[MAX_TRIGS_PER_BATCH * 2];

// trig setup //

static inline int32_t offset_trig_edge(int32_t a, int32_t b, int32_t c, v2i32 offset) {
    return a * offset[0] + b * offset[1] + c;
}

static inline int32_t init_trig_edge(struct trig_params *trig, uint32_t ei, v2i32 p) {
    return offset_trig_edge(trig->a[ei], trig->b[ei], trig->c[ei], p);
}

static inline int32_t setup_edge_params(struct trig_params *trig, uint32_t ei, const struct clip_point *p0, const struct clip_point *p1) {
    // edge setup
    // TODO: top-left fill rule
    // TODO: subpixel calc

    int32_t a = p0->y - p1->y;
    int32_t b = p1->x - p0->x;

    int32_t c = p0->x * p1->y - p0->y * p1->x;

    // step deltas
    trig->a[ei] = a;
    trig->b[ei] = b;

    // offset initial edge function value at origin
    trig->c[ei] = c;

    return c; // for trig area calc
}

static inline void insert_edge(struct vis_edge *lut, uint32_t len, struct vis_edge elem) {
    int32_t i = len - 1;

    while (i >= 0 && elem.edge < lut[i].edge) {
        lut[i + 1] = lut[i];
        i--;
    }

    lut[i + 1] = elem;
}

static void insert_vis_edges(const struct clip_point *clips, uint32_t trig) {
    // find bounding box edges (and align to TILE_SIZE)

    uint32_t x_begin = MIN(MIN(clips[0].x, clips[1].x), clips[2].x) & ~(RASTER_TILE_SIZE - 1);
    uint32_t x_end = (MAX(MAX(clips[0].x, clips[1].x), clips[2].x) + (RASTER_TILE_SIZE - 1)) & ~(RASTER_TILE_SIZE - 1);

    uint32_t y_begin = MIN(MIN(clips[0].y, clips[1].y), clips[2].y) & ~(RASTER_TILE_SIZE - 1);
    uint32_t y_end = (MAX(MAX(clips[0].y, clips[1].y), clips[2].y) + (RASTER_TILE_SIZE - 1)) & ~(RASTER_TILE_SIZE - 1);

    // insert them into sorted _edges tables

    uint32_t len = trig * 2;
    assert(len + 1 < MAX_TRIGS_PER_BATCH * 2);

    insert_edge(x_edges, len, (struct vis_edge){x_begin, trig, true});
    insert_edge(x_edges, len + 1, (struct vis_edge){x_end, trig, false});

    insert_edge(y_edges, len, (struct vis_edge){y_begin, trig, true});
    insert_edge(y_edges, len + 1, (struct vis_edge){y_end, trig, false});
}

static void rast_setup_trigs() {
    for (uint32_t i = 0; i < v2f.prim_count; i++) {
        struct trig_params *trig = &trigs[i];
        const struct clip_point *p0 = &v2f.clip_buf[i * 3 + 0];
        const struct clip_point *p1 = &v2f.clip_buf[i * 3 + 1];
        const struct clip_point *p2 = &v2f.clip_buf[i * 3 + 2];

        // setup edge functions
        int32_t v1v2_c = setup_edge_params(trig, 0, p1, p2);
        setup_edge_params(trig, 1, p2, p0);
        setup_edge_params(trig, 2, p0, p1);

        int32_t trig_area = offset_trig_edge(trig->a[0], trig->b[0], v1v2_c, (v2i32){p0->x, p0->y});

        float one_over_trig_area = 1.f / trig_area;
        trig->area_inv = one_over_trig_area;

        // setup visibility edges
        insert_vis_edges(p0, i);

        // setup z
        trig->z[0] = p0->z * one_over_trig_area;
        trig->z[1] = p1->z * one_over_trig_area;
        trig->z[2] = p2->z * one_over_trig_area;

        // setup edge culling distance
        trig->culld[0] = MAX(MAX(MAX(0, trig->a[0]), trig->b[0]), trig->a[0] + trig->b[0]) * -RASTER_TILE_SIZE;
        trig->culld[1] = MAX(MAX(MAX(1, trig->a[1]), trig->b[1]), trig->a[1] + trig->b[1]) * -RASTER_TILE_SIZE;
        trig->culld[2] = MAX(MAX(MAX(2, trig->a[2]), trig->b[2]), trig->a[2] + trig->b[2]) * -RASTER_TILE_SIZE;

        // clips
        // trig->clips = p0;
    }
}

// trig-level loop //

#include <../sc/shader_env.h>

static void rast_tile_trig(struct tile_slot *slot, uint32_t trig_i) {
    // PERF TODO: edge functions should probably be cached between steps

    struct trig_params *trig = &trigs[trig_i];
    int32_t w[3] = {
        init_trig_edge(trig, 0, slot->p),
        init_trig_edge(trig, 1, slot->p),
        init_trig_edge(trig, 2, slot->p),
    };

    // early-out if tile is fully outside the trig
    if (w[0] < trig->culld[0] || w[1] < trig->culld[1] || w[2] < trig->culld[2]) {
        return;
    }

    struct frag_input in = {
        .slot = slot->c_tile,
        .attrs = &v2f.clip_buf[trig_i * 3],
    };

    for (int32_t tile_y = 0; tile_y < RASTER_TILE_SIZE; tile_y++) {
        in.w[0] = w[0];
        in.w[1] = w[1];
        in.w[2] = w[2];

        for (int32_t tile_x = 0; tile_x < RASTER_TILE_SIZE; tile_x++) {
            // test if fragment is covered by the current trig using barycentric coords
            bool frag_cv = (in.w[0] | in.w[1] | in.w[2]) >= 0;

            if (frag_cv) {
                // finally dispatch fragment
                in.p = (v2i32){tile_x, tile_y};
                in.z = (trig->z[0] * in.w[0] + trig->z[1] * in.w[1] + trig->z[2] * in.w[2]); // ?? awful one time float int conv, need to figure out fixed point arith

                frag_cv = __frag(in);
            }

            // write coverage
            slot->cv_mask |= frag_cv << (tile_x + tile_y * RASTER_TILE_SIZE);

            // step collum
            in.w[0] += trig->a[0];
            in.w[1] += trig->a[1];
            in.w[2] += trig->a[2];
        }

        // step row
        w[0] += trig->b[0];
        w[1] += trig->b[1];
        w[2] += trig->b[2];
    }
}

// tile-level loop //

static void read_in_tile(struct tile_slot *slot) {
    for (uint32_t x = 0; x < RASTER_TILE_SIZE; x++) {
        for (uint32_t y = 0; y < RASTER_TILE_SIZE; y++) {
            slot->z_tile[x + y * RASTER_TILE_SIZE] = ((uint16_t *)vaddr(gs.fb_zs))[slot->p[0] + x + (slot->p[1] + y) * gs.fb_extent[0]];
        }
    }

    for (uint32_t x = 0; x < RASTER_TILE_SIZE; x++) {
        for (uint32_t y = 0; y < RASTER_TILE_SIZE; y++) {
            slot->c_tile[x + y * RASTER_TILE_SIZE] = ((uint16_t *)vaddr(gs.fb_c0))[slot->p[0] + x + (slot->p[1] + y) * gs.fb_extent[0]];
        }
    }
}

static void write_out_tile(struct tile_slot *slot) {
    // temp. dvi_buf format conv and output
    for (uint32_t x = 0; x < RASTER_TILE_SIZE; x++) {
        for (uint32_t y = 0; y < RASTER_TILE_SIZE; y++) {
            ((uint16_t *)vaddr(gs.fb_c0))[slot->p[0] + x + (slot->p[1] + y) * gs.fb_extent[0]] = slot->c_tile[x + y * RASTER_TILE_SIZE];
        }
    }

    for (uint32_t x = 0; x < RASTER_TILE_SIZE; x++) {
        for (uint32_t y = 0; y < RASTER_TILE_SIZE; y++) {
            ((uint16_t *)vaddr(gs.fb_zs))[slot->p[0] + x + (slot->p[1] + y) * gs.fb_extent[0]] = slot->z_tile[x + y * RASTER_TILE_SIZE];
        }
    }
}

static void rast_tile(struct tile_slot *slot, v2i32 p, uint32_t trig_mask) {
    *slot = (struct tile_slot){
        .p = p,
        .cv_mask = 0,
        .trig_mask = trig_mask,
    };

    // read-in tile from fb (if required)
    read_in_tile(slot);

    // dispatch visible trigs (in api order)

    uint32_t vis = slot->trig_mask;
    for (uint32_t i = __builtin_ctz(vis); vis; vis &= ~(1u << i), i = __builtin_ctz(vis)) {
        rast_tile_trig(slot, i);
    }

    // for (uint32_t i = 0; i < RASTER_TILE_SIZE * RASTER_TILE_SIZE; i++) {
    //     slot->c_tile[i] = colour_rgb565(slot->trig_mask);
    // }

    // write-out tile to fb
    write_out_tile(slot);
}

// batch-level loop //

static void rast_line(v2i32 p, uint32_t y_mask) {
    uint32_t x_mask = 0;

    for (uint32_t xi = 0; xi < v2f.prim_count * 2; xi++) {
        const uint32_t e = x_edges[xi].edge;

        if (!(y_mask & (1u << x_edges[xi].trig))) {
            // edge not relevant for this row, skip edge
            continue;
        }

        if (!x_mask) {
            // no trigs visible, skip to edge
            p[0] = e;

        } else {
            // iterate tiles until edge
            for (; p[0] < e; p[0] += RASTER_TILE_SIZE) {
                struct tile_slot *slot = &slots[0]; // defer_for_free_slot();
                rast_tile(slot, p, x_mask);
            }
        }

        // update visibility mask
        x_mask = x_edges[xi].pol ? x_mask | (1u << x_edges[xi].trig) : x_mask & ~(1u << x_edges[xi].trig);
    }
}

static void rast_rect() {
    v2i32 p = {};
    uint32_t y_mask = 0;

    assert(v2f.prim_count <= 32);

    for (uint32_t yi = 0; yi < v2f.prim_count * 2; yi++) {
        const uint32_t e = y_edges[yi].edge;

        if (!y_mask) {
            // no trigs visible, skip to edge
            p[1] = e;

        } else {
            // iterate rows until edge
            for (; p[1] < e; p[1] += RASTER_TILE_SIZE) {
                rast_line(p, y_mask);
            }
        }

        // update visibility mask
        y_mask = y_edges[yi].pol ? y_mask | (1u << y_edges[yi].trig) : y_mask & ~(1u << y_edges[yi].trig);
    }
}

void rast_trigs() {
    // setup rasterizer state
    rast_setup_trigs();

    // enter raster loop
    rast_rect();
}

/* raster / fragment stage entry */

void dispatch_raster_batch(struct scs_raster_batch *b) {
    switch (gs.rasterizer_mode) {
    case e_prim_trig:
        rast_trigs();
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
}
