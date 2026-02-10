#pragma once
#include <stdint.h>

// hwq submit endpoint protocol //

// NOTE: this protocol header describes the usb protocol only.
//       fot the cmd descriptions look into hwq_cmd_proto.h

typedef uint8_t hwq_ep_cmd_t;
enum hwq_ep_cmd_types {
    HWQ_EP_CMD_INVALID = 0,
    HWQ_EP_CMD_SUBMIT,
};

// submit a cmdbuf execution to the queue
struct hwq_ep_submit_cmd {
    hwq_ep_cmd_t type;

    uint32_t cmds;
    uint32_t wait_sem;
};
