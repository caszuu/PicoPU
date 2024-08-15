#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/watchdog.h"

#include "chip_state.h"
#include "../common/cluster_bus.h"

#include <string.h>

shader_chip_state_t chip_state;

void shader_stall() {
    // drive debug led to fault color
    // send gcs_fault if valid
}

int main() {
    stdio_init_all();

    if (watchdog_caused_reboot()) {
        shader_stall();
    }

    struct gcs_ready p = {
        gcs_type_ready,
    }; 

    while (true) {
        uint8_t buf[MAX_GCS_PACKET_SIZE];
        *(uint16_t*)&buf[0] = sizeof(struct gcs_ready);
        memcpy(buf + sizeof(uint16_t), &p, sizeof(struct gcs_ready));

        fwrite(&buf, sizeof(uint16_t) + sizeof(struct gcs_ready), 1, stdout);
        fflush(stdout);
        // printf();
        sleep_ms(500);
    }

    chip_state = cs_idle;
}