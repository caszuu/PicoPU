#include "dvi.h"

#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/structs/dma.h>
#include <hardware/structs/bus_ctrl.h>
#include <hardware/structs/hstx_ctrl.h>
#include <hardware/structs/hstx_fifo.h>

#include <assert.h>
#include <string.h>

/*
 * a lil hstx dvi driver modified from pico-examples
 * https://github.com/raspberrypi/pico-examples/blob/master/hstx/dvi_out_hstx_encoder/dvi_out_hstx_encoder.c
 */

// ----------------------------------------------------------------------------
// DVI constants

#define TMDS_CTRL_00 0x354u
#define TMDS_CTRL_01 0x0abu
#define TMDS_CTRL_10 0x154u
#define TMDS_CTRL_11 0x2abu

#define SYNC_V0_H0 (TMDS_CTRL_00 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V0_H1 (TMDS_CTRL_01 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V1_H0 (TMDS_CTRL_10 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V1_H1 (TMDS_CTRL_11 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))

#define MODE_H_TOTAL_PIXELS (                \
    MODE_H_FRONT_PORCH + MODE_H_SYNC_WIDTH + \
    MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS)
#define MODE_V_TOTAL_LINES (                 \
    MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH + \
    MODE_V_BACK_PORCH + MODE_V_ACTIVE_LINES)

#define HSTX_CMD_RAW (0x0u << 12)
#define HSTX_CMD_RAW_REPEAT (0x1u << 12)
#define HSTX_CMD_TMDS (0x2u << 12)
#define HSTX_CMD_TMDS_REPEAT (0x3u << 12)
#define HSTX_CMD_NOP (0xfu << 12)

// ----------------------------------------------------------------------------
// DVI driver state

static bool dvi_active;

static struct dvi_mode set_mode;
static enum dvi_format set_format;

// The framebuffers are owned and flipped by external code asynchronously, only
// vsync flips are performed by the hstx/dma logic.
static uint8_t *on_screen_fb;
static uint8_t *on_flip_fb;

// Lists are padded with NOPs to be >= HSTX FIFO size, to avoid DMA rapidly
// pingponging and tripping up the IRQs.

static uint32_t vblank_line_vsync_off[7];
static uint32_t vblank_line_vsync_on[7];
static uint32_t vactive_line[9];

// set to set_mode.v_front_porch + set_mode.v_sync_width + set_mode.v_back_porch
static uint32_t vblank_line_count;

// set to set_mode.h_active_pixels divided by set_mode.output_scale
static uint32_t h_active_fb_pixels;

#define DMACH_COUNT 4

static dma_channel_config blank_configs[DMACH_COUNT];
static dma_channel_config active_configs[DMACH_COUNT];

// ----------------------------------------------------------------------------
// DMA logic

// As the channels chain to each other in a ring, we reconf them as they finish
static uint32_t ch_num = 0;

// A channel chains are cued up initially, so the first time we enter this
// handler it is to cue up the channel-count-n scanline after the first line has completed.
static uint v_scanline = DMACH_COUNT;

// During the vertical active period, we take two IRQs per scanline: one to
// post the command list, and another to post the pixels.
static bool vactive_cmdlist_posted = false;

static void __time_critical_func(dma_irq_handler)() {
    dma_channel_hw_t *ch = &dma_hw->ch[ch_num];
    dma_hw->intr = 1u << ch_num;

    if (v_scanline >= set_mode.v_front_porch && v_scanline < (set_mode.v_front_porch + set_mode.v_sync_width)) {
        dma_channel_set_config(ch_num, &blank_configs[ch_num], false);
        ch->read_addr = (uintptr_t)vblank_line_vsync_on;
        ch->transfer_count = count_of(vblank_line_vsync_on);
    } else if (v_scanline < vblank_line_count) {
        dma_channel_set_config(ch_num, &blank_configs[ch_num], false);
        ch->read_addr = (uintptr_t)vblank_line_vsync_off;
        ch->transfer_count = count_of(vblank_line_vsync_off);
    } else if (!vactive_cmdlist_posted) {
        dma_channel_set_config(ch_num, &blank_configs[ch_num], false);
        ch->read_addr = (uintptr_t)vactive_line;
        ch->transfer_count = count_of(vactive_line);
        vactive_cmdlist_posted = true;
    } else {
        dma_channel_set_config(ch_num, &active_configs[ch_num], false);
        ch->read_addr = (uintptr_t)&on_screen_fb[(v_scanline - vblank_line_count) / set_mode.output_scale * (h_active_fb_pixels << set_format)];
        ch->transfer_count = set_mode.output_scale == 1 ? /*packed*/ (h_active_fb_pixels << set_format) / sizeof(uint32_t) : /*non-packed*/ h_active_fb_pixels;
        vactive_cmdlist_posted = false;
    }

    ch_num = (ch_num + 1) % DMACH_COUNT;

    if (!vactive_cmdlist_posted) {
        v_scanline = (v_scanline + 1) % (vblank_line_count + set_mode.v_active_lines);

        if (v_scanline == 0) {
            if (on_flip_fb) {
                // perform vsync flip if queued

                on_screen_fb = on_flip_fb;
                on_flip_fb = NULL;
            }
        }
    }
}

// ----------------------------------------------------------------------------
// Main driver

static void setup_hstx() {
    uint32_t scale = set_mode.output_scale;
    bool is_packed = scale == 1;

    switch (set_format) {
    case e_fmt_rgb332:
        // Configure HSTX's TMDS encoder for RGB332
        hstx_ctrl_hw->expand_tmds =
            2 << HSTX_CTRL_EXPAND_TMDS_L2_NBITS_LSB |
            0 << HSTX_CTRL_EXPAND_TMDS_L2_ROT_LSB |
            2 << HSTX_CTRL_EXPAND_TMDS_L1_NBITS_LSB |
            29 << HSTX_CTRL_EXPAND_TMDS_L1_ROT_LSB |
            1 << HSTX_CTRL_EXPAND_TMDS_L0_NBITS_LSB |
            26 << HSTX_CTRL_EXPAND_TMDS_L0_ROT_LSB;

        // Pixels (TMDS) come in 4 8-bit chunks. Control symbols (RAW) are an
        // entire 32-bit word.
        hstx_ctrl_hw->expand_shift =
            (is_packed ? 4 : scale) << HSTX_CTRL_EXPAND_SHIFT_ENC_N_SHIFTS_LSB |
            (is_packed ? 8 : 0) << HSTX_CTRL_EXPAND_SHIFT_ENC_SHIFT_LSB |
            1 << HSTX_CTRL_EXPAND_SHIFT_RAW_N_SHIFTS_LSB |
            0 << HSTX_CTRL_EXPAND_SHIFT_RAW_SHIFT_LSB;

        break;

    case e_fmt_rgb565:
        // Configure HSTX's TMDS encoder for RGB565
        hstx_ctrl_hw->expand_tmds =
            4 << HSTX_CTRL_EXPAND_TMDS_L2_NBITS_LSB |
            8 << HSTX_CTRL_EXPAND_TMDS_L2_ROT_LSB |
            5 << HSTX_CTRL_EXPAND_TMDS_L1_NBITS_LSB |
            3 << HSTX_CTRL_EXPAND_TMDS_L1_ROT_LSB |
            4 << HSTX_CTRL_EXPAND_TMDS_L0_NBITS_LSB |
            29 << HSTX_CTRL_EXPAND_TMDS_L0_ROT_LSB;

        // Pixels (TMDS) come in 2 16-bit chunks. Control symbols (RAW) are an
        // entire 32-bit word.
        hstx_ctrl_hw->expand_shift =
            (is_packed ? 2 : scale) << HSTX_CTRL_EXPAND_SHIFT_ENC_N_SHIFTS_LSB |
            (is_packed ? 16 : 0) << HSTX_CTRL_EXPAND_SHIFT_ENC_SHIFT_LSB |
            1 << HSTX_CTRL_EXPAND_SHIFT_RAW_N_SHIFTS_LSB |
            0 << HSTX_CTRL_EXPAND_SHIFT_RAW_SHIFT_LSB;

        break;

    case e_fmt_rgbx8888:
        // Configure HSTX's TMDS encoder for RGBX8888
        hstx_ctrl_hw->expand_tmds =
            7 << HSTX_CTRL_EXPAND_TMDS_L2_NBITS_LSB |
            0 << HSTX_CTRL_EXPAND_TMDS_L2_ROT_LSB |
            7 << HSTX_CTRL_EXPAND_TMDS_L1_NBITS_LSB |
            8 << HSTX_CTRL_EXPAND_TMDS_L1_ROT_LSB |
            7 << HSTX_CTRL_EXPAND_TMDS_L0_NBITS_LSB |
            16 << HSTX_CTRL_EXPAND_TMDS_L0_ROT_LSB;

        // Both pixels (TMDS) and control symbols (RAW) come in as
        // entire 32-bit words.
        hstx_ctrl_hw->expand_shift =
            (is_packed ? 1 : scale) << HSTX_CTRL_EXPAND_SHIFT_ENC_N_SHIFTS_LSB |
            0 << HSTX_CTRL_EXPAND_SHIFT_ENC_SHIFT_LSB |
            1 << HSTX_CTRL_EXPAND_SHIFT_RAW_N_SHIFTS_LSB |
            0 << HSTX_CTRL_EXPAND_SHIFT_RAW_SHIFT_LSB;

        break;
    }

    // Serial output config: clock period of 5 cycles, pop from command
    // expander every 5 cycles, shift the output shiftreg by 2 every cycle.
    hstx_ctrl_hw->csr = 0;
    hstx_ctrl_hw->csr =
        HSTX_CTRL_CSR_EXPAND_EN_BITS |
        5u << HSTX_CTRL_CSR_CLKDIV_LSB |
        5u << HSTX_CTRL_CSR_N_SHIFTS_LSB |
        2u << HSTX_CTRL_CSR_SHIFT_LSB |
        HSTX_CTRL_CSR_EN_BITS;

    // HSTX outputs 0 through 7 appear on GPIO 12 through 19.
    // Pinout on Pico DVI sock:
    //
    //   GP12 D0+  GP13 D0-
    //   GP14 CK+  GP15 CK-
    //   GP16 D2+  GP17 D2-
    //   GP18 D1+  GP19 D1-

    // Assign clock pair to two neighbouring pins:
    hstx_ctrl_hw->bit[2] = HSTX_CTRL_BIT0_CLK_BITS;
    hstx_ctrl_hw->bit[3] = HSTX_CTRL_BIT0_CLK_BITS | HSTX_CTRL_BIT0_INV_BITS;
    for (uint lane = 0; lane < 3; ++lane) {
        // For each TMDS lane, assign it to the correct GPIO pair based on the
        // desired pinout:
        static const int lane_to_output_bit[3] = {0, 6, 4};
        int bit = lane_to_output_bit[lane];
        // Output even bits during first half of each HSTX cycle, and odd bits
        // during second half. The shifter advances by two bits each cycle.
        uint32_t lane_data_sel_bits =
            (lane * 10) << HSTX_CTRL_BIT0_SEL_P_LSB |
            (lane * 10 + 1) << HSTX_CTRL_BIT0_SEL_N_LSB;
        // The two halves of each pair get identical data, but one pin is inverted.
        hstx_ctrl_hw->bit[bit] = lane_data_sel_bits;
        hstx_ctrl_hw->bit[bit + 1] = lane_data_sel_bits | HSTX_CTRL_BIT0_INV_BITS;
    }

    for (int i = 12; i <= 19; ++i) {
        gpio_set_function(i, 0); // HSTX
    }

    // setup hstx commands from modeset

    vblank_line_count = set_mode.v_front_porch + set_mode.v_sync_width + set_mode.v_back_porch;
    h_active_fb_pixels = set_mode.h_active_pixels / set_mode.output_scale;

    uint32_t vsync_off[] = {
        HSTX_CMD_RAW_REPEAT | set_mode.h_front_porch,
        SYNC_V1_H1,
        HSTX_CMD_RAW_REPEAT | set_mode.h_sync_width,
        SYNC_V1_H0,
        HSTX_CMD_RAW_REPEAT | (set_mode.h_back_porch + set_mode.h_active_pixels),
        SYNC_V1_H1,
        HSTX_CMD_NOP,
    };

    uint32_t vsync_on[] = {
        HSTX_CMD_RAW_REPEAT | set_mode.h_front_porch,
        SYNC_V0_H1,
        HSTX_CMD_RAW_REPEAT | set_mode.h_sync_width,
        SYNC_V0_H0,
        HSTX_CMD_RAW_REPEAT | (set_mode.h_back_porch + set_mode.h_active_pixels),
        SYNC_V0_H1,
        HSTX_CMD_NOP,
    };

    uint32_t vactive[] = {
        HSTX_CMD_RAW_REPEAT | set_mode.h_front_porch,
        SYNC_V1_H1,
        HSTX_CMD_NOP,
        HSTX_CMD_RAW_REPEAT | set_mode.h_sync_width,
        SYNC_V1_H0,
        HSTX_CMD_NOP,
        HSTX_CMD_RAW_REPEAT | set_mode.h_back_porch,
        SYNC_V1_H1,
        HSTX_CMD_TMDS | set_mode.h_active_pixels,
    };

    memcpy(vblank_line_vsync_off, vsync_off, sizeof(vblank_line_vsync_off));
    memcpy(vblank_line_vsync_on, vsync_on, sizeof(vblank_line_vsync_on));
    memcpy(vactive_line, vactive, sizeof(vactive_line));
}

static void setup_dma() {
    // All channels are set up identically, to transfer a whole scanline and
    // then chain to the next channel. Each time a channel finishes, we
    // reconfigure the one that just finished, meanwhile the next channel
    // is already making progress.

    dma_channel_config c;
    for (uint32_t i = 0; i < DMACH_COUNT; i++) {
        dma_channel_claim(i);

        c = dma_channel_get_default_config(i);
        channel_config_set_chain_to(&c, (i + 1) % DMACH_COUNT);
        channel_config_set_dreq(&c, DREQ_HSTX);
        dma_channel_configure(
            i,
            &c,
            &hstx_fifo_hw->fifo,
            vblank_line_vsync_off,
            count_of(vblank_line_vsync_off),
            false);

        blank_configs[i] = c;

        // check if scaling is enabled and adjust xfer size
        // to only transfer one pixel per transfer (required for hstx scaling)
        if (set_mode.output_scale != 1) {
            dma_channel_transfer_size_t s;
            switch (set_format) {
            case e_fmt_rgb332:
                s = DMA_SIZE_8;
                break;

            case e_fmt_rgb565:
                s = DMA_SIZE_16;
                break;

            case e_fmt_rgbx8888:
                s = DMA_SIZE_32;
                break;
            }

            channel_config_set_transfer_data_size(&c, s);
        }
        active_configs[i] = c;

        dma_hw->ints0 |= (1u << i);
        dma_hw->inte0 |= (1u << i);
    }

    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    dma_channel_start(0);
}

void dvi_modeset(struct dvi_mode *modeset, enum dvi_format fmt, uint8_t *initial_fb) {
    // spindown dvi hw if active
    if (dvi_active)
        dvi_unset();

    set_mode = *modeset;
    set_format = fmt;

    if (set_mode.output_scale == 0)
        set_mode.output_scale = 1;

    on_screen_fb = initial_fb;
    on_flip_fb = NULL;

    setup_hstx();
    dvi_reclock();

    setup_dma();

    dvi_active = true;
}

void dvi_unset() {
    // FIXME: check if dma interupts are running on the local core

    if (!dvi_active)
        return;

    irq_set_enabled(DMA_IRQ_0, false);
    irq_remove_handler(DMA_IRQ_0, dma_irq_handler);

    // block until all buffered dma channels finish
    for (uint32_t i = 0; i < DMACH_COUNT; i++) {
        dma_channel_wait_for_finish_blocking(i);
        dma_channel_unclaim(i);
    }

    dvi_active = false;
}

void dvi_reclock() {
    // get current system clock
    uint32_t sys_hz = clock_get_hz(clk_sys);

    uint32_t bit_hz = set_mode.pixel_clock_hz * 10; // 10 bits per px
    uint32_t hstx_hz = bit_hz / 2;                  // ddr

    clock_configure(
        clk_hstx,
        0,
        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
        sys_hz,
        hstx_hz);
}

void dvi_flip_immediate(uint8_t *fb) {
    on_screen_fb = fb;
}

void dvi_flip_vsync(uint8_t *fb, bool allow_rewrite) {
    assert(!on_flip_fb || allow_rewrite);
    on_flip_fb = fb;
}
