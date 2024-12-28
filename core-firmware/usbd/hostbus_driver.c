#include "hostbus_driver.h"

#include <hardware/watchdog.h>

#include <common/picopu_types.h>
#include <shader_core/graphics_state.h>

#include <common/tusb_types.h>
#include <common/tusb_verify.h>
#include <device/usbd_pvt.h>
#include <tusb.h>

/* hostbus state */

#define TRANSFER_BUF_SIZE (16 * 1024)

static uint8_t hb_rhport;
static uint8_t transfer_up;
static uint8_t transfer_down;

static uint8_t transfer_up_buf[TRANSFER_BUF_SIZE];
static uint8_t transfer_down_buf[TRANSFER_BUF_SIZE];

// down xfer fifo buf
struct hb_xfer {
    uint16_t xfer_offset;
    uint16_t xfer_size;
};

static struct hb_xfer down_xfer_buf[128];
static uint16_t down_xfer_buffered;
static uint16_t down_xfer_sent;

struct hostbus_device_state hostbus_device;

/* usbd driver impl */

void hostbus_init() {
}

void hostbus_reset(uint8_t rhport) {
    hostbus_device = (struct hostbus_device_state){
        .state = e_hb_idle,
    };
}

uint16_t hostbus_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len) {
    TU_VERIFY(itf_desc->bInterfaceClass == TUSB_CLASS_VENDOR_SPECIFIC, 0);

    const uint16_t len = sizeof(tusb_desc_interface_t) + itf_desc->bNumEndpoints * sizeof(tusb_desc_endpoint_t);
    TU_VERIFY(len <= max_len, 0);

    // cleanup old endpoints
    if (transfer_up)
        usbd_edpt_close(rhport, transfer_up);
    if (transfer_down)
        usbd_edpt_close(rhport, transfer_down);

    // reset device state
    hb_rhport = rhport;
    down_xfer_buffered = 0;
    down_xfer_sent = 0;

    // open hostbus endpoints
    const uint8_t *ep_desc = tu_desc_next(itf_desc);

    ep_desc = tu_desc_next(ep_desc);
    TU_ASSERT(usbd_open_edpt_pair(rhport, ep_desc, 2, TUSB_XFER_BULK, &transfer_up, &transfer_down));

    // setup up xfer for the initial hostbus cmd
    TU_ASSERT(usbd_edpt_xfer(rhport, transfer_up, transfer_up_buf, TRANSFER_BUF_SIZE));

    return len;
}

bool hostbus_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req) {
    if (stage != CONTROL_STAGE_SETUP)
        return true;

    return false; // leave handling at tusb core?
}

static bool hostbus_xfer_in(uint8_t rhport, uint16_t xfer_size, uint16_t offset) {
    return usbd_edpt_xfer(rhport, transfer_up, &transfer_up_buf[offset], xfer_size);
}

static bool hostbus_xfer_in_cb(uint8_t rhport, uint32_t xferred_bytes) {
    if (hostbus_device.state == e_hb_idle) {
        enum hb_type *type = (enum hb_type *)(transfer_up_buf);

        switch (*type) {
        case hb_type_transfer:
            TU_ASSERT(hostbus_xfer_in(rhport, sizeof(struct hb_transfer_header) - 1, 1));
            hostbus_device.state = e_hb_mid_xfer_up_header;

            return true;

        case hb_type_scs_proc:
            hostbus_device.enter_scs = true;
            break;

        case hb_type_flash:
            break;
        }
    } else if (hostbus_device.state == e_hb_mid_xfer_up_header) {
        struct hb_transfer_header *xfer = (struct hb_transfer_header *)(transfer_up_buf);
        hostbus_xfer_in(rhport, xfer->xfer_size, 0);

        hostbus_device.state = e_hb_mid_xfer_up_data;

        return true;
    } else if (hostbus_device.state == e_hb_mid_xfer_up_data) {
        // memcpy(xfer->device_addr, (void *)(xfer + 1), xferred_bytes);
        memcpy(hostbus_device.scs_cmd_buf, transfer_up_buf, xferred_bytes);

        hostbus_device.state = e_hb_idle;
    }

    // read next cmd
    usbd_edpt_xfer(rhport, transfer_up, transfer_up_buf, sizeof(struct hb_base_header));

    return true;
}

bool hostbus_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
    TU_VERIFY(result == XFER_RESULT_SUCCESS);

    if (ep_addr == transfer_up) {
        return hostbus_xfer_in_cb(rhport, xferred_bytes);
    } else if (ep_addr == transfer_down) {
        down_xfer_sent++;

        if (down_xfer_buffered != down_xfer_sent) {
            // start next down xfer
            struct hb_xfer *xfer = &down_xfer_buf[down_xfer_sent];

            if (!usbd_edpt_xfer(hb_rhport, transfer_down, transfer_down_buf + xfer->xfer_offset, xfer->xfer_size)) {
                watchdog_reboot(0, 0, 0);
            }
        } else {
            // reset down xfer buf
            down_xfer_buffered = 0;
            down_xfer_sent = 0;
        }

        return true;
    } else {
        return false;
    }
}

/* public api */

void hostbus_xfer_out(void *buf, uint16_t size) {
    while (down_xfer_buffered == 128 || (down_xfer_buffered ? down_xfer_buf[down_xfer_buffered - 1].xfer_offset + down_xfer_buf[down_xfer_buffered - 1].xfer_size > TRANSFER_BUF_SIZE : false)) {
        tud_task();
    } // block for xfer fin

    struct hb_xfer *xfer = &down_xfer_buf[down_xfer_buffered];
    xfer->xfer_offset = down_xfer_buffered ? down_xfer_buf[down_xfer_buffered - 1].xfer_offset + down_xfer_buf[down_xfer_buffered - 1].xfer_size : 0;
    xfer->xfer_size = size;

    memcpy(transfer_down_buf + xfer->xfer_offset, buf, size);
    if (!down_xfer_buffered) {
        if (!usbd_edpt_xfer(hb_rhport, transfer_down, transfer_down_buf + xfer->xfer_offset, size)) {
            watchdog_reboot(0, 0, 0);
        }
    }

    down_xfer_buffered++;
}

/* clang-format off */

static usbd_class_driver_t const _app_drivers[] =
{
    {
  #if CFG_TUSB_DEBUG >= 2
        .name             = "hostbus",
  #endif
        .init             = hostbus_init,
        .reset            = hostbus_reset,
        .open             = hostbus_open,
        .control_xfer_cb  = hostbus_control_xfer_cb,
        .xfer_cb          = hostbus_xfer_cb,
        .sof              = NULL
    },
};

/* clang-format on */

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
    *driver_count += TU_ARRAY_SIZE(_app_drivers);

    return _app_drivers;
}
