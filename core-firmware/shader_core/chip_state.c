#include "graphics_state.h"

#include "chip_state.h"
#include <usbd/hostbus_driver.h>

#include <hardware/watchdog.h>
#include <hardware/clocks.h>
#include <pico/stdlib.h>
#include <pico/bootrom.h>

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

void dispatch_vertex_stage(struct gcs_assign_batch *batch);
void dispatch_raster_stage();

void enter_gcs(void *cmd_buf) {
    enum gcs_types *p = (enum gcs_types *)cmd_buf;

    if (*p == gcs_type_assign) {
        dispatch_vertex_stage(p);
        dispatch_raster_stage();

        struct gcs_batch_finished p = {gcs_type_finished};
        hostbus_xfer_out(&p, sizeof(p));
    } else {
        assert(false);
    }
}

void enter_scs(void *cmd) {
    // start listening on scs commands

    switch (*(enum scs_cmd_type *)(cmd)) {
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

    case scs_type_disp_gcs:
        enter_gcs(cmd + 4);
        break;

    case scs_type_flash:
        rom_reboot(0x102 /*REBOOT_TYPE_BOOTSEL | NO_RETURN_ON_SUCCESS*/, 1, 0, 0); // note: delay_ms must be non-zero to work
        break;
    }
}

int main() {
    stdio_init_all();

    if (watchdog_enable_caused_reboot()) {
        shader_stall();
    }

    // watchdog_enable(2000, 1);
    watchdog_enable(10000, 1);

    instru_init();

    // test oc
    // set_sys_clock_khz(315000, true);

    start_single_chip_scs();
}
