#include "common.h"
#include <chip.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MIN(a, b) (a < b ? a : b)
#define MAX(a, b) (a < b ? b : a)

uint8_t cbuf[CONSTANT_BUFFER_SIZE];
uint8_t vram_blk[VRAM_RAMBLK_SIZE];

struct gcs_gstate gs;
struct gcs_v2f_state v2f;

void rast_trigs();

int main() {
    // setup vm state

    gs.fb_extent[0] = 1024;
    gs.fb_extent[1] = 1024;
    gs.fb_c0 = 0;
    gs.fb_zs = (uint8_t *)(2 * 1024 * 1024);

    // for (uint32_t i = 0; i < 1024 * 1024; i++) {
    //     ((uint16_t *)vaddr(gs.fb_zs))[i] = UINT16_MAX;
    // }
    memset(vaddr(gs.fb_zs), 255, 2 * 1024 * 1024);

    v2f.prim_count = 32;

    for (uint32_t i = 0; i < v2f.prim_count; i++) {
        v2f.clip_buf[i * 3 + 0] = (struct clip_point){
            .x = rand() % 1024,
            .y = rand() % 1024,
            .z = 0.f,
        };
        v2f.shading_range[0] = MIN(v2f.shading_range[0], v2f.clip_buf[i * 3 + 0].x);
        v2f.shading_range[1] = MIN(v2f.shading_range[1], v2f.clip_buf[i * 3 + 0].y);
        v2f.shading_range[2] = MAX(v2f.shading_range[2], v2f.clip_buf[i * 3 + 0].x);
        v2f.shading_range[3] = MAX(v2f.shading_range[3], v2f.clip_buf[i * 3 + 0].y);

        v2f.clip_buf[i * 3 + 1] = (struct clip_point){
            .x = rand() % 1024,
            .y = rand() % 1024,
            .z = 0.f,
        };
        v2f.shading_range[0] = MIN(v2f.shading_range[0], v2f.clip_buf[i * 3 + 1].x);
        v2f.shading_range[1] = MIN(v2f.shading_range[1], v2f.clip_buf[i * 3 + 1].y);
        v2f.shading_range[2] = MAX(v2f.shading_range[2], v2f.clip_buf[i * 3 + 1].x);
        v2f.shading_range[3] = MAX(v2f.shading_range[3], v2f.clip_buf[i * 3 + 1].y);

        v2f.clip_buf[i * 3 + 2] = (struct clip_point){
            .x = rand() % 1024,
            .y = rand() % 1024,
            .z = 0.f,
        };
        v2f.shading_range[0] = MIN(v2f.shading_range[0], v2f.clip_buf[i * 3 + 2].x);
        v2f.shading_range[1] = MIN(v2f.shading_range[1], v2f.clip_buf[i * 3 + 2].y);
        v2f.shading_range[2] = MAX(v2f.shading_range[2], v2f.clip_buf[i * 3 + 2].x);
        v2f.shading_range[3] = MAX(v2f.shading_range[3], v2f.clip_buf[i * 3 + 2].y);
    }

    printf("rast stage setup done, running...\n");

    for (int i = 0; i < 50; i++) {
        rast_trigs();
    }

    FILE *f = fopen("fb.bin", "wb");
    fwrite(vram_blk, 2 * 1024 * 1024, 1, f);

    fclose(f);
}
