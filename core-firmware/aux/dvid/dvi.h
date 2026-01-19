#pragma once

#include <stdbool.h>
#include <stdint.h>

struct dvi_mode {
    uint32_t h_front_porch;
    uint32_t h_sync_width;
    uint32_t h_back_porch;
    uint32_t h_active_pixels;

    uint32_t v_front_porch;
    uint32_t v_sync_width;
    uint32_t v_back_porch;
    uint32_t v_active_lines;

    uint32_t pixel_clock_hz;

    // TODO: support sync polarity, currently both are assumed to be negative
    bool h_sync_polarity, v_sync_polarity;

    // basic support for on-the-fly integer scaling
    // when scaled, the dvid will assume the fb is _active_pixels|lines divided by the scale value
    // (eg. 640x480 with 2x scale == 320x240 fb)
    // 
    // set to 0 or 1 to disable scaling (more efficient), max scale is 16
    uint32_t output_scale;
};

enum dvi_format {
    e_fmt_rgb332,
    e_fmt_rgb565,
    e_fmt_rgbx8888,
};

/* dvi hstx init/deinit */

// (re)init hstx, configure an output resolution and pixel format and starts dma interupts (on the local core)
void dvi_modeset(struct dvi_mode *modeset, enum dvi_format fmt, uint8_t *initial_fb);

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
