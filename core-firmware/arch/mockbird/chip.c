#include <chip.h>

#include <common/instru.h>
#include <dvid/dvi.h>
#include <hardware/clocks.h>
#include <hardware/regs/addressmap.h>

#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/watchdog.h>

#include <assert.h>
#include <pico/bootrom.h>
#include <pico/stdlib.h>

/* global buffers and fw top-level code */

uint8_t cbuf[CONSTANT_BUFFER_SIZE];
uint8_t vram_blk[VRAM_RAMBLK_SIZE];

struct gcs_gstate gs;

static void shader_stall() {
    // blink three times to signalize a stall occured
    // and reboot to pico usb flash

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    watchdog_disable();

    for (uint32_t i = 0; i < 3; i++) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(500);
    }

    reset_usb_boot(0, 0);
}

int main() {
    // check for shader stall / shader crash

    if (watchdog_enable_caused_reboot()) {
        shader_stall();
    }

    // init shader unit hw

    watchdog_enable(2000, 1);
    instru_init();

    // test oc
    // vreg_set_voltage(VREG_VOLTAGE_1_15);
    // set_sys_clock_khz(300000, true);

    // anti-quirk sleep:
    //   for some reason without at least ~100us of sleep any dvi / hstx ops on the pimoroni pga2350 would
    //   cause the entire chip to die (and watchdog reboot), idk why, idk how, but for your own sanity leave it as is
    //   (this is not the case for the pico 2, so there're most likely some hw shenanigans at play)
    sleep_ms(5);

    // cea-816 - 640x480@60Hz
    struct dvi_mode m = {
        .h_front_porch = 16,
        .h_sync_width = 96,
        .h_back_porch = 48,
        .h_active_pixels = 640,

        .v_front_porch = 10,
        .v_sync_width = 2,
        .v_back_porch = 33,
        .v_active_lines = 480,

        .pixel_clock_hz = 25175 * KHZ,
    };

    dvi_modeset(&m, e_fmt_rgb565, (uint8_t *)SRAM_BASE);

    // start listening for host commands
    init_cp_layer();
    cp_loop();
}
