#pragma once
#include <stdint.h>

// hwq command protocol //

typedef uint8_t hwq_cmd_t;
enum hwq_cmd_types {
    HWQ_CMD_INVALID = 0,
    HWQ_CMD_EXECUTE,
};

// execute a series of commands from a vram buffer
struct hwq_execute_cmd {
    hwq_cmd_t type;

    uint32_t cmds; // addr
    uint32_t wait_sem;
};

union hwq_cmd {
    hwq_cmd_t type;
    struct hwq_execute_cmd execute;
};
