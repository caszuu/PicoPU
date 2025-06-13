#include "common.h"
#include "sampler.h"

#include <chip_state.h>
#include <common/si_proto.h>

#include <common/dma_mem.h>
#include <common/instru.h>

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// huge shoutout to Fabian Giesen and theirs blogs by which this implementation is heavily inspired
//  - https://fgiesen.wordpress.com/2013/02/17/optimizing-sw-occlusion-culling-index/
//  - https://fgiesen.wordpress.com/2011/07/09/a-trip-through-the-graphics-pipeline-2011-index/

/* multi-trig rasterizer */

#define TILE_SLOT_COUNT 4

#define RAST_DMACH0 10
#define RAST_DMACH1 11

// pull vertex-to-fragment state buffer from vertex_stage.c
extern struct gcs_v2f_state v2f;

struct trig_state {
    int32_t a[3], b[3];

    int32_t so[6]; /* tile sample offsets */
    float z[3];

    int32_t area; // note: stores the *double* of the trig window-space area
    float area_inv;
};

struct tile_slot {
    uint32_t c_tile[RASTER_TILE_SIZE * RASTER_TILE_SIZE];
    uint16_t z_tile[RASTER_TILE_SIZE * RASTER_TILE_SIZE];

    v2i32 p;
    uint32_t cv_tile; /* first 16 bits are used for TILE_SIZE == 4 */

    /* bitsets of trigs that are contained by this tile */
    uint32_t uniform_trigs;
    uint32_t edge_trigs;
};

static struct trig_state trig_states[MAX_TRIGS_PER_BATCH];

static int32_t trig_bw_buf[MAX_TRIGS_PER_BATCH][3];
static int32_t trig_w_buf[MAX_TRIGS_PER_BATCH][3];

static struct tile_slot tile_slots[TILE_SLOT_COUNT];

// bitset of tile indices which received their tile data and are ready to be dispatched
static uint32_t pending_tiles;

/* trig setup */

static inline int32_t offset_trig_edge(int32_t a, int32_t b, int32_t c, v2i32 offset) {
    return a * offset[0] + b * offset[1] + c;
}

static inline int32_t init_trig_edge(uint32_t ti, uint32_t ei, const struct clip_point *p0, const struct clip_point *p1, v2i32 origin) {
    // edge setup
    // TODO: top-left fill rule
    // TODO: subpixel calc

    int32_t a = p0->y - p1->y;
    int32_t b = p1->x - p0->x;

    int32_t c = p0->x * p1->y - p0->y * p1->x;

    // step deltas
    trig_states[ti].a[ei] = a;
    trig_states[ti].b[ei] = b;

    // offset initial edge function value at origin
    trig_bw_buf[ti][ei] = c;

    return c; // for trig area calc
}

static inline void rast_setup_trigs() {
    v2i32 origin = {v2f.shading_range[0], v2f.shading_range[1]};

    for (uint32_t i = 0; i < v2f.prim_count; i++) {
        struct trig_state *trig = &trig_states[i];
        const struct clip_point *p0 = &v2f.clip_buf[i * 3 + 0];
        const struct clip_point *p1 = &v2f.clip_buf[i * 3 + 1];
        const struct clip_point *p2 = &v2f.clip_buf[i * 3 + 2];

        // setup w plane equations
        int32_t v1v2_c = init_trig_edge(i, 0, p1, p2, origin);
        init_trig_edge(i, 1, p2, p0, origin);
        init_trig_edge(i, 2, p0, p1, origin);

        int32_t trig_area = offset_trig_edge(trig->a[0], trig->b[0], v1v2_c, (v2i32){p0->x, p0->y});
        float one_over_trig_area = 1.f / trig_area;

        trig->area = trig_area;
        trig->area_inv = one_over_trig_area;

        // setup tile sample offsets
        for (uint32_t e = 0; e < 3; e++) {
            if ((trig->a[e] ^ trig->b[e]) < 0) {
                trig->so[e * 2 + 0] = trig->a[e] * (RASTER_TILE_SIZE - 1);
                trig->so[e * 2 + 1] = trig->b[e] * (RASTER_TILE_SIZE - 1);
            } else {
                trig->so[e * 2 + 0] = (trig->a[e] + trig->b[e]) * (RASTER_TILE_SIZE - 1);
                trig->so[e * 2 + 1] = 0;
            }
        }

        // setup z
        trig->z[0] = p0->z * one_over_trig_area;
        trig->z[1] = p1->z * one_over_trig_area;
        trig->z[2] = p2->z * one_over_trig_area;
    }
}

/* trig rasterizer */

#define PACK_RGBA8(r, g, b, a) ((r) | ((g) << 8) | ((b) << 16) | ((a) << 24))
static bool is_odd_trig;

// dispatches the per-fragment logic and fragment shader, returns if fragment was discarded
static inline bool dispatch_frag(struct tile_slot *tile, const struct trig_state* trig, int32_t w[], int32_t tile_x, int32_t tile_y) {
    // depth test and write (assume no rc with early-z writes)
    float fw[] = {w[0], w[1], w[2]};
    
    float z = (trig->z[0] * fw[0] + trig->z[1] * fw[1] + trig->z[2] * fw[2]);
    uint16_t iz = (z * .5f + .5f) * UINT16_MAX;

    if (tile->z_tile[tile_x + tile_y * RASTER_TILE_SIZE] <= iz)
        return false;
    tile->z_tile[tile_x + tile_y * RASTER_TILE_SIZE] = iz;

    // stencil test and write

    // frag shader
    // tile->c_tile[tile_x + tile_y * RASTER_TILE_SIZE] = PACK_RGBA8(tile_x * 64, 0, tile_y * 64, 255);
    // tile->c_tile[tile_x + tile_y * RASTER_TILE_SIZE] = PACK_RGBA8(w[0] / (area / 255), w[1] / (area / 255), w[2] / (area / 255), 255);
    // float uv[2] = { trig_attrib[0][0] * wf[0] + trig_attrib[0][1] * wf[1] + trig_attrib[0][2] * wf[2], trig_attrib[1][0] * wf[0] + trig_attrib[1][1] * wf[1] + trig_attrib[1][2] * wf[2] };
    // float uv[2] = {(tile->p[0] + tile_x) * (1.f / 640.f), (tile->p[1] + tile_y) * (1.f / 480.f)};

    v2f32 uv;
    if (is_odd_trig) {
        uv = (v2f32){0.f * fw[0] * trig->area_inv + 1.f * fw[1] * trig->area_inv + 0.f * fw[2] * trig->area_inv, 0.f * fw[0] * trig->area_inv + 1.f * fw[1] * trig->area_inv + 1.f * fw[2] * trig->area_inv};
    } else {
        uv = (v2f32){0.f * fw[0] * trig->area_inv + 1.f * fw[1] * trig->area_inv + 1.f * fw[2] * trig->area_inv, 0.f * fw[0] * trig->area_inv + 0.f * fw[1] * trig->area_inv + 1.f * fw[2] * trig->area_inv};
    }

    // v2f32 uv = v2f.clip_buf[trig_i].uv;

    // INSTRU_RESET_SCOPE
    tile->c_tile[tile_x + tile_y * RASTER_TILE_SIZE] = PACK_RGBA8((uint32_t)(uv[0] * 255.f), (uint32_t)(uv[1] * 255.f), 0, 255);
    // tile->c_tile[tile_x + tile_y * RASTER_TILE_SIZE] = native_fetch_rgba8_nearest(uv);
    // tile->c_tile[tile_x + tile_y * RASTER_TILE_SIZE] = native_fetch_rgba8_bilinear(uv); // & 0xff0000ff;
    // INSTRU_SUBMIT_SCOPE_ID(0)

    return true;
}

static inline void rast_edge_tile(uint32_t trig_i, const struct trig_state *trig, struct tile_slot *slot) {
    const int32_t *bw = trig_bw_buf[trig_i];
    int32_t w[3] = {
        offset_trig_edge(trig->a[0], trig->b[0], bw[0], slot->p),
        offset_trig_edge(trig->a[1], trig->b[1], bw[1], slot->p),
        offset_trig_edge(trig->a[2], trig->b[2], bw[2], slot->p),
    };

    // thread local tile coverage mask, will be orred with other cores mask
    uint32_t local_cv_mask = 0;

    for (int32_t tile_y = 0; tile_y < RASTER_TILE_SIZE; tile_y++) {
        int32_t row_w[] = {w[0], w[1], w[2]};

        for (int32_t tile_x = 0; tile_x < RASTER_TILE_SIZE; tile_x++) {
            // test if fragment is covered by the current trig using barycentric coords
            bool frag_cv = (row_w[0] | row_w[1] | row_w[2]) >= 0;

            if (frag_cv) {
                // finally dispatch fragment
                frag_cv = dispatch_frag(slot, trig, row_w, tile_x, tile_y);
            }

            // write coverage
            local_cv_mask |= frag_cv << (tile_x + tile_y * RASTER_TILE_SIZE);

            // step collum
            row_w[0] += trig->a[0];
            row_w[1] += trig->a[1];
            row_w[2] += trig->a[2];
        }

        // step row
        w[0] += trig->b[0];
        w[1] += trig->b[1];
        w[2] += trig->b[2];
    }

    slot->cv_tile |= local_cv_mask;
}

// known: rasterize and shade a uniformly covered tile
static inline void rast_full_tile(uint32_t trig_i, const struct trig_state *trig, struct tile_slot *slot) {
    const int32_t *bw = trig_bw_buf[trig_i];
    int32_t w[3] = {
        offset_trig_edge(trig->a[0], trig->b[0], bw[0], slot->p),
        offset_trig_edge(trig->a[1], trig->b[1], bw[1], slot->p),
        offset_trig_edge(trig->a[2], trig->b[2], bw[2], slot->p),
    };

    // thread local tile coverage mask, will be orred with other cores mask
    uint32_t local_cv_mask = 0;

    for (int32_t tile_y = 0; tile_y < RASTER_TILE_SIZE; tile_y++) {
        int32_t row_w[] = {w[0], w[1], w[2]};

        for (int32_t tile_x = 0; tile_x < RASTER_TILE_SIZE; tile_x++) {
            // dispatch fragment
            bool frag_cv = dispatch_frag(slot, trig, row_w, tile_x, tile_y);
            local_cv_mask |= frag_cv << (tile_x + tile_y * RASTER_TILE_SIZE);

            // step collum
            row_w[0] += trig->a[0];
            row_w[1] += trig->a[1];
            row_w[2] += trig->a[2];
        }

        // step row
        w[0] += trig->b[0];
        w[1] += trig->b[1];
        w[2] += trig->b[2];
    }

    slot->cv_tile |= local_cv_mask;
}

extern uint16_t *dvi_fb;
extern uint16_t zs_fb[320 * 240];

static __force_inline uint8_t colour_rgb332(uint32_t rgba) {
    return ((rgba >> 16) & 0xc0) >> 6 | ((rgba >> 8) & 0xe0) >> 3 | ((rgba) & 0xe0) >> 0;
}

static __force_inline uint16_t colour_rgb565(uint32_t rgbx) {
    return ((rgbx >> 16) & 0xf8) >> 3 | ((rgbx >> 8) & 0xfc) << 3 | ((rgbx) & 0xf8) << 8;
}

static __force_inline uint32_t from_rgb565(uint16_t rgb) {
    return ((rgb << 3) & 0xf8) << 16 | ((rgb >> 3) & 0xfc) << 8 | ((rgb >> 8) & 0xf8);
}

static void rast_dispatch_tile(struct tile_slot *slot) {
    for (uint32_t trig_i = 0; trig_i < v2f.prim_count; trig_i++) {
        struct trig_state *trig = &trig_states[trig_i];
        is_odd_trig = trig_i & 1; // temp.

        if (slot->uniform_trigs & (1u << trig_i))
            rast_full_tile(trig_i, trig, slot);

        if (slot->edge_trigs & (1u << trig_i))
            rast_edge_tile(trig_i, trig, slot);
    }

    if (!slot->cv_tile)
        return;

    // fb output

    // temp. dvi_buf format conv and output
    for (uint32_t x = 0; x < 4; x++) {
        for (uint32_t y = 0; y < 4; y++) {
            dvi_fb[slot->p[0] + x + (slot->p[1] + y) * 320] = colour_rgb565(slot->c_tile[x + y * 4]);
        }
    }

    // dma_memcpy32(RAST_DMACH1, &zs_fb[(slot->p[0] * 4 + slot->p[1] * 320)], slot->z_tile, sizeof(slot->z_tile) / sizeof(uint32_t));
    for (uint32_t x = 0; x < 4; x++) {
        for (uint32_t y = 0; y < 4; y++) {
            zs_fb[slot->p[0] + x + (slot->p[1] + y) * 320] = slot->z_tile[x + y * 4];
        }
    }
}

static void rast_dispatch_tiles() {
    // FIXME: this needs to be done better
    //        eats too many cycles when not many tiles are pending

    // if (!pending_tiles)
    //     return;

    for (uint32_t i = 0; i < TILE_SLOT_COUNT; i++) {
        if (pending_tiles & (1u << i)) {
            rast_dispatch_tile(&tile_slots[i]);
            pending_tiles &= ~(1u << i);
        }
    }

    // while (pending_tiles) {
    //     uint32_t i = __builtin_clz(pending_tiles);
    //     rast_dispatch_tile(&tile_slots[i]);

    //     pending_tiles &= ~(1u << i);
    // }
}

static void rast_request_tile(struct tile_slot *slot) {
    // attachment tile clears
    // FIXME: stub

    // dma_memset32(RAST_DMACH0, slot->z_tile, UINT32_MAX, sizeof(slot->z_tile) / sizeof(uint32_t));
    // dma_memset32(RAST_DMACH1, slot->c_tile, 0, sizeof(slot->c_tile) / sizeof(uint32_t));

    // dma_memcpy32(RAST_DMACH0, slot->z_tile, &zs_fb[(slot->p[0] * 4 + slot->p[1] * 320)], sizeof(slot->z_tile) / sizeof(uint32_t));
    for (uint32_t x = 0; x < 4; x++) {
        for (uint32_t y = 0; y < 4; y++) {
            slot->z_tile[x + y * 4] = zs_fb[slot->p[0] + x + (slot->p[1] + y) * 320];
        }
    }

    // dma_memcpy32(RAST_DMACH1, slot->c_tile, &dvi_fb[(slot->p[0] + slot->p[1] * 320)], sizeof(slot->c_tile) / sizeof(uint32_t));
    // expand tile from *linear* dvi buffer for rendering
    for (uint32_t x = 0; x < 4; x++) {
        for (uint32_t y = 0; y < 4; y++) {
            slot->c_tile[x + y * 4] = from_rgb565(dvi_fb[slot->p[0] + x + (slot->p[1] + y) * 320]);
        }
    }

    pending_tiles |= 1u;
}

static struct tile_slot *defer_for_free_slot() {
    // FIXME: stub
    return &tile_slots[0];
}

void rast_range() {
    // setup bilinear sampler interp

    interp_config cfg = interp_default_config();
    interp_config_set_blend(&cfg, true);
    interp_set_config(interp0, 0, &cfg);

    cfg = interp_default_config();
    interp_config_set_signed(&cfg, false);
    interp_set_config(interp0, 1, &cfg);

    // intra-tile rasterizer loop

    uint32_t min_x = v2f.shading_range[0], min_y = v2f.shading_range[1], max_x = v2f.shading_range[2], max_y = v2f.shading_range[3];
    v2i32 p = {min_x, min_y};

    for (; p[1] <= max_y; p[1] += RASTER_TILE_SIZE) {
        p[0] = min_x;

        // step row on all trigs (TODO: inline this into the trig_i loop somehow)
        for (uint32_t trig_i = 0; trig_i < v2f.prim_count; trig_i++) {
            struct trig_state *trig = &trig_states[trig_i];
            int32_t *bw = trig_bw_buf[trig_i], *w = trig_w_buf[trig_i];

            w[0] = offset_trig_edge(trig->a[0], trig->b[0], bw[0], p);
            w[1] = offset_trig_edge(trig->a[1], trig->b[1], bw[1], p);
            w[2] = offset_trig_edge(trig->a[2], trig->b[2], bw[2], p);
        }

        for (; p[0] <= max_x; p[0] += RASTER_TILE_SIZE) {
            // assume whole tile is visible in screen (aka screen size must be RASTER_TILE_SIZE aligned)
            // TODO: don't :p

            // shade awaiting slots with fetched tile data
            rast_dispatch_tiles();

            // pop a free tile slot, block for dispatches if none are free
            // FIXME: claim the slot
            struct tile_slot *slot = defer_for_free_slot();

            int32_t *w;
            int32_t *a;

            uint32_t edge_trigs = 0, uniform_trigs = 0; // bitset of all trigs that are contained by the current tile
            bool tile_fetched = false;                  // contains if current tile has already been requested from the broker

            // rasterize trigs in-order and step collum
            for (uint32_t trig_i = 0; trig_i < v2f.prim_count;
                 w[0] += a[0] * RASTER_TILE_SIZE,
                          w[1] += a[1] * RASTER_TILE_SIZE,
                          w[2] += a[2] * RASTER_TILE_SIZE,
                          trig_i++) {
                struct trig_state *trig = &trig_states[trig_i];
                w = trig_w_buf[trig_i];
                a = trig->a;

                // check tile-wide coverage status using barycentric coords
                int32_t w_samples[6] = {
                    w[0] + trig->so[0],
                    w[0] + trig->so[1],
                    w[1] + trig->so[2],
                    w[1] + trig->so[3],
                    w[2] + trig->so[4],
                    w[2] + trig->so[5],
                };

                // PERF FIXME: tiles outside the trig along the edges are tagged as edge tiles wasting cycles
                bool uniform_tile = ((w_samples[0] ^ w_samples[1]) | (w_samples[2] ^ w_samples[3]) | (w_samples[4] ^ w_samples[5])) >= 0; // no edges go through this tile
                bool tile_not_cv = (w[0] | w[1] | w[2]) < 0;                                                                              // top-left tile fragment is outside the trig half-spaces

                // early-out if whole tile is outside of trig
                if (uniform_tile && tile_not_cv)
                    continue;

                if (!tile_fetched) {
                    slot->p = p;

                    rast_request_tile(slot);
                    tile_fetched = true;
                }

                if (uniform_tile)
                    uniform_trigs |= 1u << trig_i;

                if (!uniform_tile)
                    edge_trigs |= 1u << trig_i;
            }

            if (!tile_fetched)
                continue;

            slot->cv_tile = 0;

            slot->edge_trigs = edge_trigs;
            slot->uniform_trigs = uniform_trigs;
        }
    }

    // shade last un-processed slots
    rast_dispatch_tiles();
}

void rast_trigs() {
    // setup rasterizer state
    rast_setup_trigs();

    // enter raster loop
    rast_range();
}

/* raster / fragment stage entry */

void dispatch_raster_stage() {
    switch (((struct gcs_cbuf_state *)(chip_state.cbuf))->rasterizer_mode) {
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
