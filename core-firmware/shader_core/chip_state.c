#include "../common/cluster_bus.h"
#include "graphics_state.h"

#include "chip_state.h"

#include "hardware/watchdog.h"
#include "pico/stdlib.h"

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

    while (true) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(500);
    }
}

static enum scs_cmd_type *await_scs();

static void enter_scs() {
    // start listening on scs commands

    while (true) {
        enum scs_cmd_type *cmd = 0; // await_scs();

        switch (*cmd) {
        case scs_type_ld_cbuf:
            struct scs_ld_cbuf *ld = (struct scs_ld_cbuf *)(cmd);
            memcpy(chip_state.cbuf + ld->range_offset, (void *)(ld + 1), ld->range_size);
            break;

        case scs_type_ld_bin:
            struct scs_ld_bin *ldb = (struct scs_ld_bin *)(cmd);
            memcpy(chip_state.prog_buf + ldb->bin_buf_offset, (void *)(ld + 1), ldb->bin_size);
            break;

        case scs_type_disp_bin:
            struct scs_disp_bin *disp = (struct scs_disp_bin *)(cmd);
            void (*prog_entry)() = (void (*)())(chip_state.prog_buf + disp->entry_buf_offset);

            (*prog_entry)();
            break;
        }
    }
}

int main() {
    stdio_init_all();

    watchdog_enable(2000, 1);

    if (watchdog_caused_reboot()) {
        shader_stall();
    }

    // enable serial device on host by sending traffic
    // printf("Hello, world!\n");
    // sleep_ms(100);

    // struct gcs_begin b = {
    //     .type = gcs_type_begin,
    //     .fb_extent = {128, 128},
    //     .view_transform = {{128.f / 2, 0 + 128.f / 2}, {128.f / 2, 0 + 128.f / 2}, {1.f, 0.f}},
    // };

    // enter_graphics_state(&b);

    // enter test mode
    // start_mock_broker();

    start_single_chip_scs();
}
