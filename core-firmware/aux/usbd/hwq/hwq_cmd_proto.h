#pragma once
#include <stdint.h>

// hwq command protocol //

#define HWQ_API_VERSION 1

typedef uint8_t hwq_control_result_t;
enum hwq_control_result {
    HWQ_CTL_RESULT_SUCCESS = 0,

    HWQ_CTL_RESULT_API_VERSION_MISMATCH,
    HWQ_CTL_RESULT_ALREADY_INITIALIZED,
};

typedef uint8_t hwq_cmd_t;
enum hwq_cmd_types {
    HWQ_CMD_INVALID = 0,

    HWQ_CMD_ATTACH_QUEUE, // attach queue - dispatch a new cmdbuf on the specified queue index
    HWQ_CMD_ABORT_QUEUE,  // abort queue - abort and stop any pending commands on the specified queue index

    HWQ_CMD_RESET, // reset scheduler - stop all pending tasks and reset the command processor
                   //                   to the initial state
};

/*
 * execute a series of commands from a vram buffer, only valid when queue is not attached
 */

struct hwq_attach_cmd {
    hwq_cmd_t type;

    uint16_t queue_idx;
    uint32_t cmdbuf; // addr
};

/*
 * stops queue execution and pending commands and sets into the not attached state
 */

struct hwq_abort_cmd {
    hwq_cmd_t type;
    uint16_t queue_idx;
};

union hwq_cmd_data {
    hwq_cmd_t type;

    struct hwq_attach_cmd attach;
    struct hwq_abort_cmd abort;
};
