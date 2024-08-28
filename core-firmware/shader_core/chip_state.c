#include "../common/cluster_bus.h"
#include "graphics_state.h"

#include "chip_state.h"

#include "hardware/watchdog.h"
#include "pico/stdlib.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

shader_chip_state_t chip_state;

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

int main() {
    stdio_init_all();

    // watchdog_enable(2000, 1);

    // if (watchdog_caused_reboot()) {
    //     shader_stall();
    // }

    // enable serial device on host by sending traffic
    printf("Hello, world!\n");
    sleep_ms(100);

    stdio_set_translate_crlf(&stdio_usb, false);

    struct gcs_begin b = {
        .type = gcs_type_begin,
        .fb_extent = {128, 128},
        .view_transform = {{128.f / 2, 0 + 128.f / 2}, {128.f / 2, 0 + 128.f / 2}, {1.f, 0.f}},
    };

    enter_graphics_state(&b);
    return 0;

    struct gcs_ready p = {
        gcs_type_ready,
    };

    while (true) {
        uint8_t buf[MAX_GCS_PACKET_SIZE];
        *(uint16_t *)&buf[0] = sizeof(struct gcs_ready);
        memcpy(buf + sizeof(uint16_t), &p, sizeof(struct gcs_ready));

        fwrite(&buf, sizeof(uint16_t) + sizeof(struct gcs_ready), 1, stdout);
        fflush(stdout);
        sleep_ms(500);
    }

    chip_state = cs_idle;
}
