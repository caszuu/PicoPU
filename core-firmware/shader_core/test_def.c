#include "graphics_state.h"
#include "../common/ex_simd.h"

#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

// note: this file contains test definitions which would not be included with the firmware outside of debugging

#define MAX_INLINE_TILES 32
#define fo_buf_size(tile_count) sizeof(struct gcs_fo_header) + sizeof(uint8_t) * ((tile_count / 2) + (tile_count % 2)) + sizeof(struct color_tile) * tile_count + sizeof(struct depth_tile) * tile_count

struct gcs_fo_header fo_header = { gcs_type_fo, 0, 0 };
uint8_t mask_buf[((MAX_INLINE_TILES / 2) + (MAX_INLINE_TILES % 2))];
struct color_tile ct_buf[MAX_INLINE_TILES];
struct depth_tile dt_buf[MAX_INLINE_TILES];

void reset_fragment_output() {
    fo_header = (struct gcs_fo_header){ gcs_type_fo, 0, 0 };

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

void exec_fragment_stage(u16_x2_simd p, int16_t* ws, uint8_t cv_mask) {
    // select depth

    // patch_ds_tile(&dt); // only if no gl_FragDepth writes
    
    // interp vertex attributes

    for (uint8_t i = 0; i < RENDER_QUAD_SIZE * 2; i++) {
        if (!(cv_mask & (1 << i))) continue;
        

        // jit c shader main
    }

    struct color_tile ct = { (struct rgba_color){ p.v[0], p.v[1], 0, 255}, (struct rgba_color){ p.v[0] + 1, p.v[1], 0, 255}, (struct rgba_color){ p.v[0], p.v[1] + 1, 0, 255}, (struct rgba_color){ p.v[0] + 1, p.v[1] + 1, 0, 255} }; // pull_col_tile();
    // struct color_tile ct = { (struct rgba_color){ ws[0].v[0], ws[1].v[0], ws[2].v[0], 255}, (struct rgba_color){ ws[0].v[1], ws[1].v[1], ws[2].v[1], 255}, (struct rgba_color){ ws[0].v[2], ws[1].v[2], ws[2].v[2], 255}, (struct rgba_color){ ws[0].v[3], ws[1].v[3], ws[2].v[3], 255} }; // pull_col_tile();
    // struct color_tile ct = { (struct rgba_color){ 255, 0, 255, 255 }, (struct rgba_color){ 255, 0, 255, 255 }, (struct rgba_color){ 255, 0, 255, 255 }, (struct rgba_color){ 255, 0, 255, 255 } };
    struct depth_tile dt = { 100, 100, 100, 100 };
    // hw interp blend and format conv

    if (fo_header.tile_count == 0) {
        // calc the pixel index of the first tile

        format_dbg("x: %d %d y: %d %d  %d", p.v[0], p.v[1], (p.v[0] / 2) * 4, (p.v[1] / 2 * 2) * fb_extent[0], fb_extent[0]);

        fo_header.fb_index_base = (p.v[0] / 2) * 4 + (p.v[1] / 2 * 2) * fb_extent[0] /* tile index */;
                                  // (p.v[0] % 2) + (p.v[1] % 2) * 2 /* tile offset */;
    }

    mask_buf[fo_header.tile_count / 2] |= cv_mask << (4 * (fo_header.tile_count % 2));
    ct_buf[fo_header.tile_count] = ct;
    dt_buf[fo_header.tile_count] = dt;
    
    // format_dbg("fp: %hhx %hhx %hhx %hhx", ct.c[0].r, ct.c[0].g, ct.c[0].b, ct.c[0].a);
    
    fo_header.tile_count++;

    if (fo_header.tile_count == MAX_INLINE_TILES) {
        stream_fragment_output();
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
