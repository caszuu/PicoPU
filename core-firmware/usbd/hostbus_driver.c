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
static uint8_t transfer_in;
static uint8_t transfer_out;

static uint8_t transfer_out_buf[TRANSFER_BUF_SIZE];

struct hostbus_device_state hb_dev;

/* usbd driver impl */

void hostbus_init() {
}

void hostbus_reset(uint8_t rhport) {
    hb_dev = (struct hostbus_device_state){
        .in_xfer_bytes = 0,
        .out_xfer_buffered = 0,
        .out_xfer_sent = 0,
    };
}

uint16_t hostbus_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len) {
    TU_VERIFY(itf_desc->bInterfaceClass == TUSB_CLASS_VENDOR_SPECIFIC, 0);

    const uint16_t len = sizeof(tusb_desc_interface_t) + itf_desc->bNumEndpoints * sizeof(tusb_desc_endpoint_t);
    TU_VERIFY(len <= max_len, 0);

    // cleanup old endpoints
    if (transfer_in)
        usbd_edpt_close(rhport, transfer_in);
    if (transfer_out)
        usbd_edpt_close(rhport, transfer_out);

    // reset device state
    hb_rhport = rhport;
    hb_dev = (struct hostbus_device_state){
        .in_xfer_bytes = 0,
        .out_xfer_buffered = 0,
        .out_xfer_sent = 0,
    };

    // open hostbus endpoints
    const uint8_t *ep_desc = tu_desc_next(itf_desc);

    ep_desc = tu_desc_next(ep_desc);
    TU_ASSERT(usbd_open_edpt_pair(rhport, ep_desc, 2, TUSB_XFER_BULK, &transfer_in, &transfer_out));

    return len;
}

bool hostbus_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req) {
    if (stage != CONTROL_STAGE_SETUP)
        return true;

    return false; // leave handling at tusb core?
}

bool hostbus_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
    TU_VERIFY(result == XFER_RESULT_SUCCESS);

    if (ep_addr == transfer_in) {
        // signal that in xfer is complete
        hb_dev.in_xfer_bytes = xferred_bytes;
        return true;
    } else if (ep_addr == transfer_out) {
        hb_dev.out_xfer_sent++;

        if (hb_dev.out_xfer_buffered != hb_dev.out_xfer_sent) {
            // start next out xfer
            struct hb_xfer *xfer = &hb_dev.out_xfer_buf[hb_dev.out_xfer_sent];

            if (!usbd_edpt_xfer(hb_rhport, transfer_out, transfer_out_buf + xfer->xfer_offset, xfer->xfer_size)) {
                watchdog_reboot(0, 0, 0);
            }
        } else {
            // reset out xfer buf
            hb_dev.out_xfer_buffered = 0;
            hb_dev.out_xfer_sent = 0;
        }

        return true;
    } else {
        return false;
    }
}

/* public api */

uint16_t hostbus_xfer_in_blocking(void *buf, uint16_t max_bytes) {
    // no read xfer must be active
    assert(!hb_dev.in_xfer_bytes);
    if (!usbd_edpt_xfer(hb_rhport, transfer_in, buf, max_bytes)) {
        watchdog_reboot(0, 0, 0);
    }

    // block until xfer is complete
    while (!hb_dev.in_xfer_bytes) {
        tud_task();
        watchdog_update();
    }

    uint16_t bytes_read = hb_dev.in_xfer_bytes;
    hb_dev.in_xfer_bytes = 0;

    return bytes_read;
}

void hostbus_xfer_out(void *buf, uint16_t size) {
    while (hb_dev.out_xfer_buffered == 128 || (hb_dev.out_xfer_buffered ? hb_dev.out_xfer_buf[hb_dev.out_xfer_buffered - 1].xfer_offset + hb_dev.out_xfer_buf[hb_dev.out_xfer_buffered - 1].xfer_size + size > TRANSFER_BUF_SIZE : false)) {
        tud_task();
    } // block for xfer fin

    struct hb_xfer *xfer = &hb_dev.out_xfer_buf[hb_dev.out_xfer_buffered];
    xfer->xfer_offset = hb_dev.out_xfer_buffered ? hb_dev.out_xfer_buf[hb_dev.out_xfer_buffered - 1].xfer_offset + hb_dev.out_xfer_buf[hb_dev.out_xfer_buffered - 1].xfer_size : 0;
    xfer->xfer_size = size;

    if (xfer->xfer_offset + xfer->xfer_size > TRANSFER_BUF_SIZE) {
        watchdog_reboot(0, 0, 0);
    }

    assert(xfer->xfer_offset + xfer->xfer_size <= TRANSFER_BUF_SIZE);
    memcpy(transfer_out_buf + xfer->xfer_offset, buf, size);

    if (!hb_dev.out_xfer_buffered) {
        if (!usbd_edpt_xfer(hb_rhport, transfer_out, transfer_out_buf + xfer->xfer_offset, size)) {
            watchdog_reboot(0, 0, 0);
        }
    }

    hb_dev.out_xfer_buffered++;
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
