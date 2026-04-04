#include "hw_queue.h"
#include "hwq_cmd_proto.h"

#include <cp/cp.h>

#include <device/usbd.h>
#include <device/usbd_pvt.h>
#include <tusb.h>

#include <stdbool.h>
#include <stdint.h>

#define HWQ_CMD_MAX_SIZE 16
#define HWQ_CMD_BUFFER_SIZE 512

#define USBD_HWQ_SUBCLASS 0x03

// hw queue driver //

static uint8_t hw_cmd_xfer_buf[HWQ_CMD_MAX_SIZE];

static bool is_host_connected;
static bool is_host_initialized;

static void hwq_reset() {
    is_host_connected = false;
    is_host_initialized = false;
}

bool hwq_is_host_present() {
    return is_host_initialized;
}

// usb <-> scheduler interface //

static uint32_t hwq_rhport;
static uint32_t hwq_submit_ep;

static void hwq_usb_init() {
    hwq_reset();
}

static void hwq_usb_reset(uint8_t rhport) {
    // clean-up usb state
    usbd_edpt_close(rhport, hwq_submit_ep);

    // clean-up and reinit queue state
    hwq_reset();
}

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

    is_host_connected = true;

    // setup initial ep read
    usbd_edpt_xfer(rhport, hwq_submit_ep, hw_cmd_xfer_buf, HWQ_CMD_MAX_SIZE);

    return drv_len;
}

static bool hwq_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req) {
    if (req->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR ||
        req->bmRequestType_bit.recipient != TUSB_REQ_RCPT_INTERFACE)
        return false;

    bool is_recv = req->bmRequestType_bit.direction == TUSB_DIR_IN;

    if (req->bRequest != USBD_HWQ_SUBCLASS)
        return false;

    if (stage != CONTROL_STAGE_SETUP)
        return true;

    switch (req->wIndex) {
    case 0: /* initialize */
        if (is_recv)
            return false;

        if (req->wValue != HWQ_API_VERSION) {
            hwq_control_result_t res = HWQ_CTL_RESULT_API_VERSION_MISMATCH;

            tud_control_xfer(rhport, req, &res, sizeof(res));
            return true;
        }

        if (is_host_initialized) {
            hwq_control_result_t res = HWQ_CTL_RESULT_ALREADY_INITIALIZED;

            tud_control_xfer(rhport, req, &res, sizeof(res));
            return true;
        }

        hwq_control_result_t res = HWQ_CTL_RESULT_SUCCESS;

        tud_control_xfer(rhport, req, &res, sizeof(res));
        return true;

    default:
        return false;
    }
}

static bool hwq_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
    // validate xfer

    if (ep_addr != hwq_submit_ep)
        return false;

    if (result != XFER_RESULT_SUCCESS) {
        hwq_usb_reset(rhport);
        return true;
    }

    // parse and process cmd

    union hwq_cmd_data *cmd = (union hwq_cmd_data *)hw_cmd_xfer_buf;

    switch (cmd->type) {
    case HWQ_CMD_ATTACH_QUEUE:
        cp_attach_queue(cmd->attach.queue_idx, (void *)cmd->attach.cmdbuf);
        break;

    case HWQ_CMD_ABORT_QUEUE:
        assert("abort not yet implemented");
        break;

    case HWQ_CMD_RESET:
        // FIXME: proper scheduler reset
        hwq_usb_reset(rhport);
        break;

    default:
        break;
    }

    usbd_edpt_xfer(rhport, hwq_submit_ep, hw_cmd_xfer_buf, HWQ_CMD_MAX_SIZE);
    return true;
}

// usb driver descriptor //

/* clang-format off */

static usbd_class_driver_t const app_driver =
{
#if CFG_TUSB_DEBUG >= 2
    .name             = "hwq",
#endif
    .init             = hwq_usb_init,
    .reset            = hwq_usb_reset,
    .open             = hwq_usb_open,
    .control_xfer_cb  = hwq_control_xfer_cb,
    .xfer_cb          = hwq_xfer_cb,
    .sof              = NULL
};

/* clang-format on */

void hwq_get_driver_desc(usbd_class_driver_t *out_desc) {
    *out_desc = app_driver;
}
