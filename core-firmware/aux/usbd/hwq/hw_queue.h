#pragma once

#include "hwq_cmd_proto.h"
#include <stdbool.h>

// hardware command queue driver //

enum hwq_result {
    HWQ_RESULT_QUEUE_NOT_ATTACHED = -3,
    HWQ_RESULT_QUEUE_REATTACHED,
    HWQ_RESULT_NO_CMDS_AVAILABLE,

    HWQ_RESULT_SUCCESS = 0,
};

// checks if the hw queue is currently attached to a host.
bool hwq_is_attached();

// try to pop the next device cmd from the hw queue.
enum hwq_result hwq_next_cmd(union hwq_cmd *cmd);
