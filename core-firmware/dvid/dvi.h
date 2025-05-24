#pragma once

#include <stdbool.h>
#include <stdint.h>

enum dvi_mode {
    e_mode_640x480_60Hz,
    e_mode_640x480_30Hz,
};

enum dvi_format {
    e_fmt_rgb332,
    e_fmt_rgb565,
    e_fmt_rgbx8888,
};

/* dvi hstx init/deinit */

// (re)init hstx, configure an output resolution and pixel format and starts dma interupts (on the local core)
void dvi_modeset(enum dvi_mode modeset, enum dvi_format fmt, uint8_t *initial_fb);

// disable hstx and dma interupts
void dvi_unset();

// sets up hstx clocks based on current dvi_mode and pll_sys
void dvi_reclock();

/* dvi real-time api */

// flip on-screen framebuffer mid scan-out, may cause screen tearing
void dvi_flip_immediate(uint8_t *fb);

// flip on-screen framebuffer on next vsync
//   allow_rewrite - true if overriding a already queued fb is valid (mailbox)
void dvi_flip_vsync(uint8_t *fb, bool allow_rewrite);
