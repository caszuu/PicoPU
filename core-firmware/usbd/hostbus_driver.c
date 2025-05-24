#include "hostbus_driver.h"

#include <hardware/watchdog.h>

#include <common/picopu_types.h>

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
        .in_xferred_bytes = 0,
        .out_xfer_buffered = 0,
        .out_xfer_sent = 0,
    };
}

extern uint8_t si_rx_buf[1024];

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
        .in_xferred_bytes = 0,
        .out_xfer_buffered = 0,
        .out_xfer_sent = 0,
    };

    // open hostbus endpoints
    const uint8_t *ep_desc = tu_desc_next(itf_desc);

    ep_desc = tu_desc_next(ep_desc);
    TU_ASSERT(usbd_open_edpt_pair(rhport, ep_desc, 2, TUSB_XFER_BULK, &transfer_in, &transfer_out));

    usbd_edpt_xfer(hb_rhport, transfer_in, si_rx_buf, 1022);

    return len;
}

bool hostbus_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req) {
    if (stage != CONTROL_STAGE_SETUP)
        return true;

    return false; // leave handling at tusb core?
}

extern void si_irq_handler(uint32_t bytes);

bool hostbus_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
    TU_VERIFY(result == XFER_RESULT_SUCCESS);

    if (ep_addr == transfer_in) {
        // *(uint16_t *)(si_rx_buf + hb_dev.in_xferred_bytes) = (xferred_bytes + 1) / 2;

        // uint32_t head = hb_dev.in_xferred_bytes + 2;
        // hb_dev.in_xferred_bytes += xferred_bytes + 2;

        // if (hb_dev.in_xferred_bytes >= 1024) {
        //     memcpy(si_rx_buf + head, in_buf, 1024 - head);
        //     memcpy(si_rx_buf, in_buf + head, xferred_bytes - (1024 - head));

        //     hb_dev.in_xferred_bytes %= 1024;
        // } else {
        //     memcpy(si_rx_buf + head, in_buf, xferred_bytes);
        // }

        // usbd_edpt_xfer(hb_rhport, transfer_in, in_buf, 1024);
        // si_irq_handler();

        // *(uint16_t *)(si_rx_buf) = xferred_bytes;

        si_irq_handler(xferred_bytes);
        usbd_edpt_xfer(hb_rhport, transfer_in, si_rx_buf, 1022);

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
