#include "chip_state.h"

#include <common/si_proto.h>
#include <dvid/dvi.h>
#include <hardware/regs/addressmap.h>
#include <usbd/hostbus_driver.h>

#include <hardware/clocks.h>
#include <hardware/watchdog.h>
#include <pico/bootrom.h>
#include <pico/stdlib.h>

#include <common/instru.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

struct chip_state chip_state;

void shader_stall() {
    // drive debug led to fault color
    // send gcs_fault if valid

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    watchdog_disable();

    for (uint32_t i = 0; i < 3; i++) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(500);
    }

    rom_reboot(0x102 /*REBOOT_TYPE_BOOTSEL | NO_RETURN_ON_SUCCESS*/, 1, 0, 0); // note: delay_ms must be non-zero to work
}

void dispatch_vertex_stage(struct scs_vertex_batch *batch);
void dispatch_raster_stage(struct scs_raster_batch *batch);

// very temp. globals and logic, will be replaced by a new vram / xfer impl

static uint8_t scs_packet_buf[16][MAX_SCS_PACKET_SIZE];
static uint16_t scs_read_head, scs_write_head;

uint16_t *dvi_fb;
uint16_t dvi0_fb[320 * 240];
// uint16_t dvi1_fb[320 * 240];
#define dvi1_fb dvi0_fb
uint16_t zs_fb[320 * 240];

static uint32_t fb_index = 0;

static void dispatch_scs(uint8_t *cmd) {
    // start listening on scs commands

    switch (*(enum si_packet_type *)(cmd)) {
        /* scs cmds */

    case si_type_ld_cbuf:
        struct scs_ld_cbuf *ld = (struct scs_ld_cbuf *)(cmd);
        // memcpy(chip_state.cbuf + ld->range_offset, (void *)(ld + 1), ld->range_size);
        assert(false);
        break;

    case si_type_ld_cbuf_inline:
        struct scs_ld_cbuf_inline *ldi = (struct scs_ld_cbuf_inline *)(cmd);
        memcpy(chip_state.cbuf + ldi->range_offset, (void *)(ldi + 1), ldi->range_size);

        break;

        /* gcs cmds */

    case si_type_vbatch:
        dispatch_vertex_stage((struct scs_vertex_batch *)cmd);

        struct si_dbg_packet dp = {
            .type = si_type_dbg,
        };

        // snprintf(dp.dbg_message, MAX_SCS_DBG_SIZE, "vertex end dbg");
        // hostbus_xfer_out(&dp, sizeof(dp));

        struct scs_batch_finished vp = {si_type_finished};
        hostbus_xfer_out(&vp, sizeof(vp));

        break;

    case si_type_rbatch:
        // struct si_dbg_packet dp = {
        //     .type = si_type_dbg,
        // };

        // snprintf(dp.dbg_message, MAX_SCS_DBG_SIZE, "raster dbg %d %d", );
        // hostbus_xfer_out(&dp, sizeof(dp));

        dispatch_raster_stage((struct scs_raster_batch *)cmd);

        struct scs_batch_finished rp = {si_type_finished};
        hostbus_xfer_out(&rp, sizeof(rp));

        break;

    case si_type_flip:
        dvi_flip_immediate((uint8_t *)dvi_fb);

        if (fb_index) {
            dvi_fb = dvi0_fb;
        } else {
            dvi_fb = dvi1_fb;
        }

        fb_index = (fb_index + 1) % 2;
        
        memset(dvi_fb, 0, sizeof(dvi0_fb));
        memset(zs_fb, 255, sizeof(zs_fb));
        break;

    default:
        __builtin_unreachable();
    }
}

// temp.
#define SI_RX_RING_SIZE 1024

uint8_t si_rx_buf[SI_RX_RING_SIZE];
static uint32_t si_rx_head;

void __time_critical_func(si_irq_handler)(uint32_t size) {
    static bool t = false;
    t ^= true;

    watchdog_update();
    gpio_put(PICO_DEFAULT_LED_PIN, t);

    // read header

    // uint16_t transfer_count = *(uint16_t *)&si_rx_buf[si_rx_head];
    uint8_t *p = &si_rx_buf[0]; // [si_rx_head + 2];
    // si_rx_head = (si_rx_head + 2 + transfer_count) % SI_RX_RING_SIZE;

    // branch to packet handler

    enum si_packet_type ptype = *(enum si_packet_type *)p;
    switch (ptype & ~15u) {
    case si_class_scs:
        memcpy(&scs_packet_buf[scs_write_head], p, size);
        scs_write_head = (scs_write_head + 1) % 16;

        assert(scs_read_head != scs_write_head); // ring buf overflow check
        break;

    case si_class_unordered:
        if (ptype == si_type_flash)
            rom_reboot(0x102 /*REBOOT_TYPE_BOOTSEL | NO_RETURN_ON_SUCCESS*/, 1, 0, 0); // note: delay_ms must be non-zero to work

        // si_xfer_cb(ptype, p);
        break;
    }
}

void si_loop() {
    while (true) {
        tud_task();
        watchdog_update();

        if (scs_read_head != scs_write_head) {
            dispatch_scs(scs_packet_buf[scs_read_head]);
            scs_read_head = (scs_read_head + 1) % 16;

            continue;
        }

        // __wfi();
    }
}

/* firmware entry point */

#include <hardware/vreg.h>

extern void start_single_chip_scs();

int main() {
    stdio_init_all();

    if (watchdog_enable_caused_reboot()) {
        shader_stall();
    }

    // init shader unit hw

    watchdog_enable(2000, 1);
    // watchdog_enable(10000, 1);
    instru_init();

    // test oc
    // vreg_set_voltage(VREG_VOLTAGE_1_15);
    // set_sys_clock_khz(300000, true);

    // chip_state.cbuf = 0x15000000;
    // sfe_setup_psram(47); // for pga2350

    // anti-quirk sleep:
    //   for some reason without at least ~100us of sleep any dvi / hstx ops on the pimoroni pga2350 would
    //   cause the entire chip to die (and watchdog reboot), idk why, idk how, but for your own sanity leave it as is
    //   (this is not the case for the pico 2, so there're most likely some hw shenanigans at play)
    sleep_ms(5);

    dvi_fb = dvi0_fb;
    dvi_modeset(e_mode_640x480_60Hz, e_fmt_rgb565, (uint8_t *)dvi1_fb);

    // start listening for scs cmds
    start_single_chip_scs();
}
