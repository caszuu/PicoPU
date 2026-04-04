#pragma once

#include <stdbool.h>
#include <stdint.h>

enum cp_cmd_type {
    CP_CMD_SENTINEL = 0, // end of buffer

    CP_CMD_DUMMY, // dummy cmd - for testing
};

struct cp_cmd_base {
    enum cp_cmd_type type;

    // bitset of queue-local semaphores, which need to be signaled before dispatching the cmd
    uint32_t wait_bits;

    // bitset of queue-local semaphores, which to setup (increment) on cmd dispatch and signal (decrement) on cmd finalization
    uint32_t signal_bits;
};

// command layouts //

struct cp_cmd_dummy {
    struct cp_cmd_base base;
    char msg[16];
};

union cp_cmd_data {
    struct cp_cmd_base base;

    struct cp_cmd_dummy dummy;
};
