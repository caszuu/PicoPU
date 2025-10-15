#pragma once

#include <gcs/unit.h>
#include <stdint.h>

#define CONSTANT_BUFFER_SIZE 1024 * 4
#define VRAM_RAMBLK_SIZE 1024 * 480

/* global mockbird device buffers / regions */

extern uint8_t cbuf[CONSTANT_BUFFER_SIZE];
extern uint8_t vram_blk[VRAM_RAMBLK_SIZE];

#define vaddr(addr) (vram_blk + (uint32_t)addr)

extern struct gcs_gstate gs;

void init_cp_layer();
void cp_loop();
