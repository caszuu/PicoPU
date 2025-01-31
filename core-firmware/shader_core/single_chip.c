#include "graphics_state.h"
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
// while operationg the shader core firmware on a single core
// the other core is in a simplified broker-like mode that communicates with the host usb driver

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

    uint8_t scs_buf[16 * 1024];
    while (true) {
        hostbus_xfer_in_blocking(scs_buf, 16 * 1024);

        gpio_put(PICO_DEFAULT_LED_PIN, t);
        t = !t;

        enter_scs(scs_buf);

        tud_task();
        watchdog_update();
    }
}

/*
void start_mock_graphics() {
    // wait for host listener

    board_init();
    tusb_init();

    if (board_init_after_tusb) {
        board_init_after_tusb();
    }

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    gpio_put(PICO_DEFAULT_LED_PIN, true);

    while (!tud_ready() || true) {
        tud_task();
        watchdog_update();
    }

    sleep_ms(1000);
    gpio_put(PICO_DEFAULT_LED_PIN, false);

    // setup mock gcs state

    {
        struct gcs_begin b = {
            .type = gcs_type_begin,
            .fb_extent = {512, 512},
            .view_transform = {{512.f / 2, 0 + 512.f / 2}, {512.f / 2, 0 + 512.f / 2}, {1.f, 0.f}},
        };

        memcpy(fb_extent, b.fb_extent, sizeof(fb_extent));
        memcpy(view_transform_params, b.view_transform, sizeof(view_transform_params));

        struct gcs_gp_conf p = {
            .prim_mode = e_prim_trig,
        };

        configure_pipeline(&p);
    }

    // setup mock gcs pipeline

    {
        struct gcs_gp_bind_header p = {
            .type = gcs_type_gp_bind,
            .pipeline_size = 0,
            // FIXME: mock pipeline
        };

        bind_pipeline(&p);
    }

    watchdog_update();

    // start "receiving" gcs streams

    while (true) {
        // "received draw call" - dispatch vertex streams

        {
            static const uint32_t prim_count = 1;
            uint8_t vs_buf[sizeof(struct gcs_vs_header) + (sizeof(float) * 3 * prim_count)];

            struct gcs_vs_header *vs = (struct gcs_vs_header *)vs_buf;
            *vs = (struct gcs_vs_header){
                .type = gcs_type_vs,
                .base_vertex = 0,
                .primitive_count = 1,
            };

            // FIXME: mock vertex data

            process_vertex_stream(vs);
        }

        watchdog_update();
        tud_task();

        // "received a shading range" - dispatch fragment stream for the specified shading range

        {
            static const uint32_t prim_count = 100;
            uint8_t fs_buf[sizeof(struct gcs_fs_header) + (sizeof(struct clip_point) * 3 * prim_count)];

            struct gcs_fs_header *fs = (struct gcs_fs_header *)fs_buf;
            *fs = (struct gcs_fs_header){
                .type = gcs_type_fs,
                .prim_count = prim_count,
                .shading_range = {0, 0, 511, 511}, // while fb for now
            };

            struct clip_point *clip_buf = (struct clip_point *)(fs + 1);

            for (uint32_t i = 0; i < 10; i++) {
                for (uint32_t j = 0; j < 10; j++) {
                    clip_buf[(i + j * 10) * 3 + 0] = (struct clip_point){0 + (i * 40), 0 + (j * 40), 0};
                    clip_buf[(i + j * 10) * 3 + 1] = (struct clip_point){0 + (i * 40), 127 + (j * 40), 0};
                    clip_buf[(i + j * 10) * 3 + 2] = (struct clip_point){127 + (i * 40), 127 + (j * 40), UINT32_MAX};
                }
            }

            // FIXME: mock vertex assemblies

            process_fragment_stream(fs);
        }

        watchdog_update();
        tud_task();
    }
}
*/