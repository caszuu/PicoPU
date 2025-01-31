#include "common.h"

#include <usbd/hostbus_driver.h>
#include <common/gcs_proto.h>
#include <chip_state.h>

#include <common/instru.h>

#include <stdbool.h>
#include <string.h>
#include <math.h>

// huge shoutout to Fabian Giesen and theirs blogs by which this implementation is heavily inspired
//  - https://fgiesen.wordpress.com/2013/02/17/optimizing-sw-occlusion-culling-index/
//  - https://fgiesen.wordpress.com/2011/07/09/a-trip-through-the-graphics-pipeline-2011-index/

/* multi-trig rasterizer */

#define MAX_TRIGS_PER_STREAM 32

struct trig_state {
    int32_t a[3], b[3];

    int32_t so[6]; /* tile sample offsets */
    int32_t w[3];
    int32_t bw[3];

    float z[3];

    int32_t area; // note: stores the *double* of the trig window-space area
    float area_inv;
};

struct trig_rast_state {
    // active trig states
    struct trig_state trig_states[MAX_TRIGS_PER_STREAM];
};

static struct trig_rast_state trig_rast_state;

// temp. single-tile storage buffers TODO: replace with multi-tile buffers and enable multi-tile rasterizer loops
static uint32_t /* RGBA_UINT8 */ c_tile[RASTER_TILE_SIZE * RASTER_TILE_SIZE];
static float /* D_F32 */ z_tile[RASTER_TILE_SIZE * RASTER_TILE_SIZE];
static uint32_t cv_tile; // only first 16-bits are used

// trig setup

static inline int32_t offset_trig_edge(int32_t a, int32_t b, int32_t c, v2i32 offset) {
    return a * offset[0] + b * offset[1] + c;
}

static inline int32_t init_trig_edge(struct trig_state *trig, uint32_t e_i, const struct clip_point *p0, const struct clip_point *p1, v2i32 origin) {
    // edge setup
    // TODO: top-left fill rule
    // TODO: subpixel calc

    int32_t a = p0->y - p1->y;
    int32_t b = p1->x - p0->x;

    int32_t c = p0->x * p1->y - p0->y * p1->x;

    // step deltas
    trig->a[e_i] = a;
    trig->b[e_i] = b;

    // offset initial edge function value at origin
    trig->bw[e_i] = trig->w[e_i] = offset_trig_edge(a, b, c, origin);

    return c; // for trig area calc
}

static inline void rast_setup_trigs(struct gcs_fs_header *stream, v2i32 origin) {
    const struct clip_point *clip_buf = (struct clip_point *)((uint8_t *)(stream) + sizeof(struct gcs_fs_header));

    for (uint32_t i = 0; i < stream->prim_count; i++) {
        struct trig_state *trig = &trig_rast_state.trig_states[i];
        const struct clip_point *p0 = &clip_buf[i * 3 + 0];
        const struct clip_point *p1 = &clip_buf[i * 3 + 1];
        const struct clip_point *p2 = &clip_buf[i * 3 + 2];

        // setup w plane equations
        int32_t v2v1_c = init_trig_edge(trig, 0, p2, p1, origin);
        init_trig_edge(trig, 1, p0, p2, origin);
        init_trig_edge(trig, 2, p1, p0, origin);

        int32_t trig_area = offset_trig_edge(trig->a[0], trig->b[0], v2v1_c, (v2i32){p0->x, p0->y});
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

// temp. and WIP testing texture samplers

#define PACK_RGBA8(r, g, b, a) (r) | ((g) << 8) | ((b) << 16) | ((a) << 24)

static inline uint32_t native_fetch_rgba8_nearest(v2u32 texel_coords) {
    uint32_t* texel_addr = ((uint32_t*)(chip_state.cbuf + 64) /* temp. hardcoded tex base */) + texel_coords[0] + texel_coords[1] * 64 /* temp. hardcoded tex width */;
    return *texel_addr;
}

typedef int8_t v4i8 __attribute__((vector_size(4)));
typedef uint8_t v4u8 __attribute__((vector_size(4)));

#include <hardware/interp.h>

static inline uint32_t native_fetch_rgba8_bilinear(v2f32 uv) {
    uv *= 64.f; // tex size normalization
    
    v2u32 texel_quad_base = (v2u32){ uv[0], uv[1] };
    v2f32 interp_params = (v2f32){ (uv[0] - texel_quad_base[0]), (uv[1] - texel_quad_base[1]) };

    // fetch
    uint32_t* texel_addr = ((uint32_t*)(chip_state.cbuf + 64) /* temp. hardcoded tex base */) + texel_quad_base[0] + texel_quad_base[1] * 64 /* temp. hardcoded tex width */;

    v4u8 texel0 = (v4u8)*(texel_addr);
    v4u8 texel1 = (v4u8)*(texel_addr + 1);
    v4u8 texel2 = (v4u8)*(texel_addr + 1 * 64);
    v4u8 texel3 = (v4u8)*(texel_addr + 1 * 64 + 1);

    // filter on interp unit
    
    interp0->accum[1] = interp_params[0] * 255;
    v4u32 horiz_samples;
    
    for (uint32_t chan = 0; chan < 4; chan++) {
        interp0->base01 = (uint32_t)(texel0[chan]) | (uint32_t)(texel1[chan]) << 16;
        horiz_samples[chan] = interp0->peek[1];
    }

    for (uint32_t chan = 0; chan < 4; chan++) {
        interp0->base01 = (uint32_t)(texel2[chan]) | (uint32_t)(texel3[chan]) << 16;
        horiz_samples[chan] |= interp0->peek[1] << 16;    
    }

    interp0->accum[1] = interp_params[1] * 255;

    v4u8 sample;
    for (uint32_t chan = 0; chan < 4; chan++) {
        interp0->base01 = horiz_samples[chan];
        sample[chan] = interp0->peek[1];
    }

    return (uint32_t)sample;
}

static bool is_odd_trig;

// trig rasterizer

// dispatches the per-fragment logic and fragment shader, returns if fragment was discarded
static inline bool dispatch_frag(int32_t w[], float wf[], float z, int32_t area, float area_inv, v2i32 p, int32_t tile_x, int32_t tile_y) {
    // depth test and write (assume no rc with early-z writes)
    if (z_tile[tile_x + tile_y * RASTER_TILE_SIZE] >= z) return false;
    z_tile[tile_x + tile_y * RASTER_TILE_SIZE] = z;

    // stencil test and write

    // frag shader
    // c_tile[tile_x + tile_y * RASTER_TILE_SIZE] = PACK_RGBA8(tile_x * 64, 0, tile_y * 64, 255); 
    // c_tile[tile_x + tile_y * RASTER_TILE_SIZE] = PACK_RGBA8(w[0] / (area / 255), w[1] / (area / 255), w[2] / (area / 255), 255);
    // float uv[2] = { trig_attrib[0][0] * wf[0] + trig_attrib[0][1] * wf[1] + trig_attrib[0][2] * wf[2], trig_attrib[1][0] * wf[0] + trig_attrib[1][1] * wf[1] + trig_attrib[1][2] * wf[2] };
    // float uv[2] = { (p[0] + tile_x) * (1.f / 640.f), (p[1] + tile_y) * (1.f / 480.f) };
    
    v2f32 uv;
    if (is_odd_trig) {
        uv = (v2f32){ 0.f * wf[0] * area_inv + 1.f * wf[1] * area_inv + 0.f * wf[2] * area_inv, 0.f * wf[0] * area_inv + 1.f * wf[1] * area_inv + 1.f * wf[2] * area_inv };
    } else {
        uv = (v2f32){ 0.f * wf[0] * area_inv + 1.f * wf[1] * area_inv + 1.f * wf[2] * area_inv, 0.f * wf[0] * area_inv + 0.f * wf[1] * area_inv + 1.f * wf[2] * area_inv };
    }
    // uv = (v2f32){ 0.f, 0.f };
    
    // assume uv is within texture and texture size normalized
    v2u32 texel_coords = { uv[0] * 64, uv[1] * 64 }; // nearest (towards 0) texel conversion; TODO: set fpu to nearest rounding
    
    // INSTRU_RESET_SCOPE
    // c_tile[tile_x + tile_y * RASTER_TILE_SIZE] = PACK_RGBA8(texel_coords[0], texel_coords[1], 0, 255);
    c_tile[tile_x + tile_y * RASTER_TILE_SIZE] = native_fetch_rgba8_nearest(texel_coords);
    // c_tile[tile_x + tile_y * RASTER_TILE_SIZE] = native_fetch_rgba8_bilinear(uv); // & 0xff0000ff;
    // INSTRU_SUBMIT_SCOPE_ID(0)

    return true;
}

static inline void rast_edge_tile(int32_t bw[3], v2i32 p, struct trig_state* trig) {
    int32_t w[] = { bw[0], bw[1], bw[2] };
    float z[] = { trig->z[0], trig->z[1], trig->z[2] };

    const int32_t a[] = { trig->a[0], trig->a[1], trig->a[2] };
    // const int32_t b[] = { trig->b[0], trig->b[1], trig->b[2] };

    const int32_t area = trig->area;
    const float area_inv = trig->area_inv;

    // thread local tile coverage mask, will be orred with other cores mask
    uint32_t local_cv_mask = 0;

    for (int32_t tile_y = 0; tile_y < RASTER_TILE_SIZE /* multi-core split */; tile_y++) {
        int32_t row_w[] = { w[0], w[1], w[2] };
        
        for (int32_t tile_x = 0; tile_x < RASTER_TILE_SIZE; tile_x++) {
            // test if fragment is covered by the current trig using barycentric coords
            bool frag_cv = (row_w[0] | row_w[1] | row_w[2]) >= 0;

            if (frag_cv) {
                float row_wf[] = { row_w[0], row_w[1], row_w[2] };
                float frag_z = (z[0] * row_wf[0] + z[1] * row_wf[1] + z[2] * row_wf[2]);

                // finally dispatch fragment
                frag_cv = dispatch_frag(row_w, row_wf, frag_z, area, area_inv, p, tile_x, tile_y);
            }

            // write coverage
            local_cv_mask |= frag_cv << (tile_x + tile_y * RASTER_TILE_SIZE);

            // step collum
            row_w[0] += a[0];
            row_w[1] += a[1];
            row_w[2] += a[2];
        }

        // step row
        w[0] += trig->b[0];
        w[1] += trig->b[1];
        w[2] += trig->b[2];
    }

    cv_tile |= local_cv_mask; // FIXME: unsafe, race cond
}

// known: rasterize and shade a full tile (by def full coverage); [bw] are current barycentric vars at the top-left fragment of the tile 
static inline void rast_full_tile(int32_t bw[3], v2i32 p, struct trig_state* trig) {
    int32_t w[] = { bw[0], bw[1], bw[2] };
    float z[] = { trig->z[0], trig->z[1], trig->z[2] };

    const int32_t a[] = { trig->a[0], trig->a[1], trig->a[2] };
    // const int32_t b[] = { trig->b[0], trig->b[1], trig->b[2] };

    const int32_t area = trig->area;
    const float area_inv = trig->area_inv;

    // thread local tile coverage mask, will be orred with other cores mask
    uint32_t local_cv_mask = 0;

    for (int32_t tile_y = 0; tile_y < RASTER_TILE_SIZE /* multi-core split */; tile_y++) {
        int32_t row_w[] = { w[0], w[1], w[2] };
        
        for (int32_t tile_x = 0; tile_x < RASTER_TILE_SIZE; tile_x++) {
            float row_wf[] = { row_w[0], row_w[1], row_w[2] };
            float frag_z = (z[0] * row_wf[0] + z[1] * row_wf[1] + z[2] * row_wf[2]);

            // dispatch fragment
            bool frag_cv = dispatch_frag(row_w, row_wf, frag_z, area, area_inv, p, tile_x, tile_y);
            local_cv_mask |= frag_cv << (tile_x + tile_y * RASTER_TILE_SIZE);

            // step collum
            row_w[0] += a[0];
            row_w[1] += a[1];
            row_w[2] += a[2];
        }

        // step row
        w[0] += trig->b[0];
        w[1] += trig->b[1];
        w[2] += trig->b[2];
    }

    cv_tile |= local_cv_mask; // FIXME: unsafe, race cond
}

void rast_range(uint32_t min_x, uint32_t min_y, uint32_t max_x, uint32_t max_y, uint32_t trig_count) {
    // setup bilinear sampler interp

    interp_config cfg = interp_default_config();
    interp_config_set_blend(&cfg, true);
    interp_set_config(interp0, 0, &cfg);

    cfg = interp_default_config();
    interp_config_set_signed(&cfg, false);
    interp_set_config(interp0, 1, &cfg);
    
    // intra-tile rasterizer loop

    v2i32 p = {min_x, min_y};

    for (; p[1] <= max_y; p[1] += RASTER_TILE_SIZE) {
        for (p[0] = min_x; p[0] <= max_x; p[0] += RASTER_TILE_SIZE) {
            // assume whole tile is visible in screen (aka screen size must be RASTER_TILE_SIZE aligned)
            // TODO: don't :p

            // TODO: requesting, awaiting and selecting tiles from fb

            int32_t *w;
            int32_t *a;

            bool tile_fetched = false;

            // rasterize trigs in-order and step collum
            for (uint32_t trig_i = 0; trig_i < trig_count; 
                w[0] += a[0] * RASTER_TILE_SIZE, 
                w[1] += a[1] * RASTER_TILE_SIZE,
                w[2] += a[2] * RASTER_TILE_SIZE,
                trig_i++) {
                struct trig_state *trig = &trig_rast_state.trig_states[trig_i];
                w = trig->w;
                a = trig->a;
                is_odd_trig = trig_i & 1; // temp.
            
                // check tile-wide coverage status using barycentric coords
                int32_t w_samples[6] = {
                    w[0] + trig->so[0], w[0] + trig->so[1],
                    w[1] + trig->so[2], w[1] + trig->so[3],
                    w[2] + trig->so[4], w[2] + trig->so[5],
                };

                bool uniform_tile = ((w_samples[0] ^ w_samples[1]) | (w_samples[2] ^ w_samples[3]) | (w_samples[4] ^ w_samples[5])) >= 0; // no edges go through this tile
                bool tile_not_cv = (w[0] | w[1] | w[2]) < 0; // top-left tile fragment is outside the trig half-spaces

                // bool s0_cv = (w_samples[0] | w_samples[2] | w_samples[4]) < 0;
                // bool s1_cv = (w_samples[1] | w_samples[3] | w_samples[5]) < 0;

                // early-out if whole tile is outside of trig
                if (uniform_tile && tile_not_cv) continue;

                if (!tile_fetched) {
                    // tile data required, fetch from fb if not already
                    // TODO: fetch tile from fb

                    memset(c_tile, 0, sizeof(c_tile));
                    for (uint32_t i = 0; i < RASTER_TILE_SIZE * RASTER_TILE_SIZE; i++) {
                        z_tile[i] = -1.f;
                    }

                    cv_tile = 0;

                    tile_fetched = true;
                }

                // dispatch fast interp and fragment stage for uniformly covered tiles
                if (uniform_tile) rast_full_tile(w, p, trig);

                // dispatch fine rasterizer for (literal) edge case tiles
                if (!uniform_tile) rast_edge_tile(w, p, trig);
            }

            // xfer fragment outputs if tile was modified
            if (tile_fetched && cv_tile) {
                struct gcs_fo_header h = (struct gcs_fo_header){
                    gcs_type_fo,
                    1,
                    { p[0], p[1] },
                    // instru_sample_buf[0],
                };

                hostbus_xfer_out(&h, sizeof(h));
                hostbus_xfer_out(&cv_tile, 2);
                hostbus_xfer_out(c_tile, sizeof(c_tile));
                hostbus_xfer_out(z_tile, sizeof(z_tile));
            }
        }

        // step row on all trigs (TODO: inline this into the trig_i loop somehow)
        for (uint32_t trig_i = 0; trig_i < trig_count; trig_i++) {
            struct trig_state *trig = &trig_rast_state.trig_states[trig_i];
            int32_t *bw = trig->bw, *w = trig->w;

            bw[0] += trig->b[0] * RASTER_TILE_SIZE;
            bw[1] += trig->b[1] * RASTER_TILE_SIZE;
            bw[2] += trig->b[2] * RASTER_TILE_SIZE;

            memcpy(w, bw, sizeof(int32_t) * 3);
        }
    }
}

void rast_trigs(struct gcs_fs_header *stream) {
    // setup rasterizer state
    rast_setup_trigs(stream, (v2i32){stream->shading_range[0], stream->shading_range[1]});

    // enter raster loop
    rast_range(stream->shading_range[0], stream->shading_range[1], stream->shading_range[2], stream->shading_range[3], stream->prim_count);
}

/* raster / fragment stage entry */

void process_fragment_stream(struct gcs_fs_header *stream) {
    switch (((struct gcs_state *)(chip_state.cbuf))->rasterizer_mode) {
    case e_prim_trig:
        rast_trigs(stream);
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
