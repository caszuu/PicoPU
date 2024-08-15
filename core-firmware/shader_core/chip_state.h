#pragma once

enum shader_chip_state_e {
    cs_idle = 0,
    cs_graphics,
    // cs_compute,
};
typedef enum shader_chip_state_e shader_chip_state_t;

extern shader_chip_state_t chip_state;