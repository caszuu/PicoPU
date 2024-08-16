#include "graphics_state.h"
#include "../common/ex_simd.h"

#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define MAX_INLINE_TILES 32
#define fo_buf_size(tile_count) sizeof(struct gcs_fo_header) + sizeof(uint8_t) * ((tile_count / 2) + (tile_count % 2)) + sizeof(struct color_tile) * tile_count + sizeof(struct depth_tile) * tile_count

// uint8_t fo_buf[sizeof(uint16_t) + fo_buf_size(MAX_INLINE_TILES)] = { 0, 0, gcs_type_fo };

// uint8_t* mask_buf = (uint8_t*)(fo_buf + sizeof(uint16_t) + sizeof(struct gcs_fo_header));
// struct color_tile* ct_buf = (struct color_tile*)(fo_buf + sizeof(uint16_t) + sizeof(struct gcs_fo_header) + sizeof(uint8_t) * ((MAX_INLINE_TILES / 2) + (MAX_INLINE_TILES % 2)));
// struct depth_tile* dt_buf = (struct depth_tile*)(fo_buf + sizeof(uint16_t) + sizeof(struct gcs_fo_header) + sizeof(uint8_t) * ((MAX_INLINE_TILES / 2) + (MAX_INLINE_TILES % 2)) + sizeof(struct color_tile) * MAX_INLINE_TILES);

struct gcs_fo_header fo_header = { gcs_type_fo };
uint8_t mask_buf[sizeof(uint8_t) * ((MAX_INLINE_TILES / 2) + (MAX_INLINE_TILES % 2))];
struct color_tile ct_buf[sizeof(struct color_tile) * MAX_INLINE_TILES];
struct depth_tile dt_buf[sizeof(struct depth_tile) * MAX_INLINE_TILES];

void exec_fragment_stage(i16_x2_simd p, i16_x4_simd* ws, uint8_t cv_mask) {
    // select depth

    // patch_ds_tile(&dt); // only if no gl_FragDepth writes
    
    // interp vertex attributes

    for (uint8_t i = 0; i < RENDER_QUAD_SIZE * 2; i++) {
        if (!(cv_mask & (1 << i))) continue;
        

        // jit c shader main
    }

    struct color_tile ct = { p.v[0], p.v[1], 0, 255 }; // pull_col_tile();
    struct depth_tile dt = { INT32_MAX, INT32_MAX, INT32_MAX, INT32_MAX };
    // hw interp blend and format conv

    if (fo_header.tile_count == 0) {
        // calc the pixel index of the first tile

        fo_header.fb_index_base =  (p.v[0] / 2) * 4 + (p.v[1] & (~1) /* / 2 * 2 */) * fb_extent[0] /* tile index */ + 
                            (p.v[0] % 2) + (p.v[1] % 2) * 2 /* tile offset */;
    }

    mask_buf[fo_header.tile_count] |= cv_mask << (4 * (fo_header.tile_count % 2));
    ct_buf[fo_header.tile_count] = ct;
    dt_buf[fo_header.tile_count] = dt;
    fo_header.tile_count++;

    if (fo_header.tile_count == MAX_INLINE_TILES) {
        uint16_t p_size = fo_buf_size(MAX_INLINE_TILES);
        fo_header.tile_count = MAX_INLINE_TILES;

        fwrite(&p_size, sizeof(uint16_t), 1, stdout);
        fwrite(&fo_header, sizeof(fo_header), 1, stdout);

        fwrite(&mask_buf, sizeof(uint8_t) * ((MAX_INLINE_TILES / 2) + (MAX_INLINE_TILES % 2)), 1, stdout);
        fwrite(&ct_buf, sizeof(struct color_tile) * MAX_INLINE_TILES, 1, stdout);
        fwrite(&dt_buf, sizeof(struct depth_tile) * MAX_INLINE_TILES, 1, stdout);
        
        fflush(stdout);

        // reset masks which are ORed into the buffer
        memset(mask_buf, 0, ((MAX_INLINE_TILES / 2) + (MAX_INLINE_TILES % 2)));
        
        fo_header.tile_count = 0;
    }
}

void early_fragment_output() {
    if (fo_header.tile_count != 0) {
        uint16_t p_size = fo_buf_size(fo_header.tile_count);
        fo_header.tile_count = fo_header.tile_count;

        fwrite(&p_size, sizeof(uint16_t), 1, stdout);
        fwrite(&fo_header, sizeof(fo_header), 1, stdout);

        fwrite(mask_buf, sizeof(uint8_t) * ((MAX_INLINE_TILES / 2) + (MAX_INLINE_TILES % 2)), 1, stdout);
        fwrite(ct_buf, sizeof(struct color_tile) * MAX_INLINE_TILES, 1, stdout);
        fwrite(dt_buf, sizeof(struct depth_tile) * MAX_INLINE_TILES, 1, stdout);
        
        fflush(stdout);

        // reset masks which are ORed into the buffer
        memset(mask_buf, 0, ((MAX_INLINE_TILES / 2) + (MAX_INLINE_TILES % 2)));

        fo_header.tile_count = 0;
    }
}

bool exec_vertex_stage(uint32_t vertex_index, void* in_buf, struct clip_point* clip_buf, void* out_buf) {
    const float* in_pos = (float*)in_buf;
    
    clip_buf->x = in_pos[0] * fb_extent[0];
    clip_buf->y = in_pos[1] * fb_extent[1];
    clip_buf->d = in_pos[2] * INT32_MAX;

    format_dbg("vertex %d, %d, %u", clip_buf->x, clip_buf->y, clip_buf->d);

    return true;
}