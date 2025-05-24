#include "chip_state.h"

#include <hardware/gpio.h>
#include <hardware/watchdog.h>

#include <bsp/board_api.h>
#include <tusb.h>

#include <usbd/hostbus_driver.h>
#include <common/gcs_proto.h>
#include <common/picopu_types.h>

#include <pico.h>
// #include <pico/stdio_usb.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// a single chip mock implementation for testing
//
// while operating the shader core firmware in single chip mode
// the chip listens for scs commands on the usb vendor endpoint which it then executes in-place

extern void si_loop();

void start_single_chip_scs() {
    // wait for host listener

    board_init();
    tusb_init();

    if (board_init_after_tusb) {
        board_init_after_tusb();
    }

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    gpio_put(PICO_DEFAULT_LED_PIN, true);

    while (!tud_ready()) {
        tud_task();
        watchdog_update();
    }

    sleep_ms(1000);
    gpio_put(PICO_DEFAULT_LED_PIN, false);

    // start SU top-level loop

    bool t = true;

    si_loop();
    // while (true) {
    //     tud_task();
    //     watchdog_update();
    // }
}
