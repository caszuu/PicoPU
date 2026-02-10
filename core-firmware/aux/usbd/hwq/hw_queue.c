#include "hw_queue.h"
#include "hwq_cmd_proto.h"
#include "hwq_ep_proto.h"

#include <util/u_fifo.h>

#include <device/usbd.h>
#include <device/usbd_pvt.h>
#include <tusb.h>

#include <stdbool.h>
#include <stdint.h>

#define HWQ_CMD_MAX_SIZE 16
#define HWQ_CMD_BUFFER_SIZE 512

// hw queue driver //

static uint8_t hw_cmd_xfer_buf[HWQ_CMD_MAX_SIZE];
static uint8_t hw_cmd_queue_buf[HWQ_CMD_BUFFER_SIZE];

// TODO: probably change to an array of hwq_cmds, no need for byte fifo (u_element_fifo?)
static struct u_fifo hw_cmd_queue;

static bool hwq_is_host_attached = 0;
static bool hwq_was_reset = 0;

static void hwq_init() {
    hw_cmd_queue = (struct u_fifo){
        .buf = hw_cmd_queue_buf,
        .buf_size = HWQ_CMD_BUFFER_SIZE,
    };

    hwq_is_host_attached = false;
}

bool hwq_is_attached() {
    return hwq_is_host_attached;
}

enum hwq_result hwq_next_cmd(union hwq_cmd *cmd) {
    if (!hwq_is_host_attached)
        return HWQ_RESULT_QUEUE_NOT_ATTACHED;

    if (hwq_was_reset) {
        hwq_was_reset = false;
        return HWQ_RESULT_QUEUE_REATTACHED;
    }

    if (!fifo_get_free(&hw_cmd_queue))
        return HWQ_RESULT_NO_CMDS_AVAILABLE;

    hwq_cmd_t type;
    fifo_peek(&hw_cmd_queue, &type, 1, 0);

    switch (type) {
    case HWQ_CMD_EXECUTE:
        fifo_pop(&hw_cmd_queue, cmd, sizeof(struct hwq_execute_cmd));
        break;

    default:
        assert(false && "unknown cmd");
    }

    return HWQ_RESULT_SUCCESS;
}

// usb <-> hw queue interface //

static uint32_t hwq_rhport;
static uint32_t hwq_submit_ep;

static void hwq_usb_init() {
    hwq_init();
}

static void hwq_usb_reset(uint32_t rhport) {
    // clean-up usb state
    usbd_edpt_close(rhport, hwq_submit_ep);

    hwq_was_reset = true;
    hwq_is_host_attached = false;

    // clean-up and reinit queue state
    hwq_init();
}

#define USBD_HWQ_SUBCLASS 0x03

static uint16_t hwq_usb_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len) {
    // is this the wanted usb driver?
    TU_VERIFY(TUSB_CLASS_VENDOR_SPECIFIC == itf_desc->bInterfaceClass && USBD_HWQ_SUBCLASS == itf_desc->bInterfaceSubClass && 0x00 == itf_desc->bInterfaceProtocol, 0);

    const uint16_t drv_len = sizeof(tusb_desc_interface_t) + sizeof(tusb_desc_endpoint_t);
    TU_VERIFY(max_len >= drv_len, 0);

    // open submit endpoint
    const uint8_t *ep_desc = tu_desc_next(itf_desc);
    bool ok = usbd_edpt_open(rhport, (tusb_desc_endpoint_t *)ep_desc);

    if (!ok)
        return 0;

    hwq_rhport = rhport;
    hwq_submit_ep = ((tusb_desc_endpoint_t *)ep_desc)->bEndpointAddress;

    hwq_is_host_attached = true;

    // setup initial ep read
    usbd_edpt_xfer(rhport, hwq_submit_ep, hw_cmd_xfer_buf, HWQ_CMD_MAX_SIZE);

    return drv_len;
}

static bool hwq_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req) {
    return false;
}

static bool hwq_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
    if (ep_addr != hwq_submit_ep)
        return false;

    // parse and process cmd
    struct hwq_ep_submit_cmd *cmd = (struct hwq_ep_submit_cmd *)hw_cmd_xfer_buf;

    if (cmd->type != HWQ_EP_CMD_SUBMIT) {
        // prof_log("incorrect hwq cmd, ignoring.");
        goto read_next;
    }

    fifo_push(&hw_cmd_queue, hw_cmd_xfer_buf, sizeof(*cmd));

read_next:
    usbd_edpt_xfer(rhport, hwq_submit_ep, hw_cmd_xfer_buf, HWQ_CMD_MAX_SIZE);
    return true;
}
