#include "usb.h"
#include "device/usbd.h"

#include <hardware/gpio.h>
#include <hardware/watchdog.h>
#include <pico/bootrom.h>

#include <common/tusb_types.h>
#include <common/tusb_verify.h>
#include <device/usbd_pvt.h>
#include <tusb.h>

#include <assert.h>
#include <stdint.h>
#include <string.h>

/* usb state buffers */

static uint8_t usb_rhport;
static uint8_t ep_xfer_in;
static uint8_t ep_xfer_out;
static bool ep_xfer_out_idle;

static uint8_t in_xfer_buf[2][USB_XFER_MAX_PAK_SIZE];
static bool in_xfer_current_buf;

static uint8_t out_xfer_buf[USB_XFER_RING_COUNT][USB_XFER_MAX_PAK_SIZE];
static uint16_t out_xfer_sizes[USB_XFER_RING_COUNT];
static uint32_t out_xfer_buf_head, out_xfer_buf_tail;

struct usb_stream {
    uint32_t bytes_remaining;
    void *next_addr;
};

static struct usb_stream in_stream;
static struct usb_stream out_stream;

/* usb usbd impl */

static void usb_init() {
}

static void usb_reset(uint8_t rhport) {
    if (ep_xfer_in)
        usbd_edpt_close(rhport, ep_xfer_in);
    if (ep_xfer_out)
        usbd_edpt_close(rhport, ep_xfer_out);

    out_xfer_buf_head = 0;
    out_xfer_buf_tail = 0;

    memset(&in_stream, 0, sizeof(in_stream));
    memset(&out_stream, 0, sizeof(out_stream));
}

static uint16_t usb_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len) {
    TU_VERIFY(TUSB_CLASS_VENDOR_SPECIFIC == itf_desc->bInterfaceClass);

    const uint16_t drv_len = sizeof(tusb_desc_interface_t) + itf_desc->bNumEndpoints * sizeof(tusb_desc_endpoint_t);
    TU_VERIFY(drv_len <= max_len);

    // setup endpoints
    usb_rhport = rhport;

    const uint8_t *xfer_ep_pair = tu_desc_next(itf_desc);
    TU_VERIFY(usbd_open_edpt_pair(usb_rhport, xfer_ep_pair, 2, TUSB_XFER_BULK, &ep_xfer_in, &ep_xfer_out));

    // setup initial read-in
    usbd_edpt_xfer(usb_rhport, ep_xfer_in, in_xfer_buf[in_xfer_current_buf], USB_XFER_MAX_PAK_SIZE);
    ep_xfer_out_idle = true;

    return drv_len;
}

// an ugly hack used by usb_ctl_data() while a usb_ctl_pak_cb() call is ongoing
static tusb_control_request_t const *current_ctl_req = NULL;

static bool usb_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req) {
    if (stage != CONTROL_STAGE_SETUP && stage != CONTROL_STAGE_DATA)
        return true; // is this right?

    current_ctl_req = req;

    bool handled = usb_ctl_pak_cb(req->bRequest, req->wValue, req->wIndex, stage == CONTROL_STAGE_DATA);

    current_ctl_req = NULL;
    return handled;
}

static bool usb_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
    TU_VERIFY(result == XFER_RESULT_SUCCESS);

    if (ep_addr == ep_xfer_in) {
        if (in_stream.bytes_remaining) {
            // stream in-progress, handle xfer to dst
            in_stream.bytes_remaining -= xferred_bytes;
            in_stream.next_addr += xferred_bytes;

            if (in_stream.bytes_remaining) {
                uint32_t seg_size = MIN(in_stream.bytes_remaining, USB_XFER_MAX_STREAM_SEG_SIZE);
                usbd_edpt_xfer(rhport, ep_xfer_in, in_stream.next_addr, seg_size);

                return true;
            }

            // stream finished, pull in next pak
            usb_setup_in_pak();
            return true;
        }

        // cmd xfer, callback to handler

        in_xfer_current_buf ^= true;
        usb_pak_in_cb(in_xfer_buf[!in_xfer_current_buf], xferred_bytes);

        return true;
    } else if (ep_addr == ep_xfer_out) {
        if (out_stream.bytes_remaining) {
            // stream in-progress, handle xfer from src
            out_stream.bytes_remaining -= xferred_bytes;
            out_stream.next_addr += xferred_bytes;

            if (out_stream.bytes_remaining) {
                uint32_t seg_size = MIN(out_stream.bytes_remaining, USB_XFER_MAX_STREAM_SEG_SIZE);
                usbd_edpt_xfer(rhport, ep_xfer_out, out_stream.next_addr, seg_size);

                return true;
            }
        }

        // out xfer finished, setup next xfer

        if (out_xfer_buf_tail == out_xfer_buf_head) {
            ep_xfer_out_idle = true;
            return true; // no xfers queued
        }

        uint32_t next_tail = (out_xfer_buf_tail + 1) % USB_XFER_RING_COUNT;

        if (out_xfer_sizes[out_xfer_buf_tail]) {
            // setup pak
            usbd_edpt_xfer(rhport, ep_xfer_out, out_xfer_buf[out_xfer_buf_tail], out_xfer_sizes[out_xfer_buf_tail]);
        } else {
            // setup stream
            memcpy(&out_stream, out_xfer_buf[out_xfer_buf_tail], sizeof(struct usb_stream));

            uint32_t seg_size = MIN(out_stream.bytes_remaining, USB_XFER_MAX_STREAM_SEG_SIZE);
            usbd_edpt_xfer(usb_rhport, ep_xfer_out, out_stream.next_addr, seg_size);
        }

        out_xfer_buf_tail = next_tail;
        return true;
    }

    return false;
}

/* usb public api */

void usb_setup_in_pak() {
    assert(!usbd_edpt_busy(usb_rhport, ep_xfer_in));

    bool ok = usbd_edpt_xfer(usb_rhport, ep_xfer_in, in_xfer_buf[in_xfer_current_buf], USB_XFER_MAX_PAK_SIZE);
}

void usb_setup_in_stream(void *dst, uint32_t size) {
    assert(!usbd_edpt_busy(usb_rhport, ep_xfer_in));

    in_stream.bytes_remaining = size;
    in_stream.next_addr = dst;

    uint32_t seg_size = MIN(size, USB_XFER_MAX_STREAM_SEG_SIZE);
    bool ok = usbd_edpt_xfer(usb_rhport, ep_xfer_in, dst, seg_size);
}

static uint32_t await_next_out_slot() {
    uint32_t next_head = (out_xfer_buf_head + 1) % USB_XFER_RING_COUNT;

    // check if queue is full, block if it is
    while (next_head == out_xfer_buf_tail) {
        tud_task();
        watchdog_update();
    }

    return next_head;
}

void usb_out_pak(const void *p, uint16_t size) {
    uint32_t next_head = await_next_out_slot();

    // copy xfer data into queue
    assert(size <= USB_XFER_MAX_PAK_SIZE);
    assert(size != 0); // size 0 paks are internally interpreted as streams

    memcpy(out_xfer_buf[out_xfer_buf_head], p, size);
    out_xfer_sizes[out_xfer_buf_head] = size;

    // submit xfer
    if (ep_xfer_out_idle) {
        ep_xfer_out_idle = false;
        usbd_edpt_xfer(usb_rhport, ep_xfer_out, out_xfer_buf[out_xfer_buf_head], size);

        out_xfer_buf_tail = next_head;
    }

    out_xfer_buf_head = next_head;
}

void usb_out_stream(void *src, uint32_t size) {
    uint32_t next_head = await_next_out_slot();

    // copy stream data into queue (with pak size == 0 to mark as a stream)
    struct usb_stream *stream = (struct usb_stream *)out_xfer_buf[out_xfer_buf_head];
    out_xfer_sizes[out_xfer_buf_head] = 0;

    stream->bytes_remaining = size;
    stream->next_addr = src;

    // submit xfer
    if (ep_xfer_out_idle) {
        ep_xfer_out_idle = false;

        memcpy(&out_stream, stream, sizeof(struct usb_stream));

        uint32_t seg_size = MIN(size, USB_XFER_MAX_STREAM_SEG_SIZE);
        usbd_edpt_xfer(usb_rhport, ep_xfer_out, src, seg_size);

        // FIXME: is a irq race cond possible here between buf_tail and buf_head?
        out_xfer_buf_tail = next_head;
    }

    out_xfer_buf_head = next_head;
}

void usb_ctl_data(void *buf, uint16_t len) {
    assert(current_ctl_req);
    tud_control_xfer(usb_rhport, current_ctl_req, buf, len);
}

void usb_ctl_status() {
    assert(current_ctl_req);
    tud_control_status(usb_rhport, current_ctl_req);
}

/* clang-format off */

static usbd_class_driver_t const _app_drivers[] =
{
    {
  #if CFG_TUSB_DEBUG >= 2
        .name             = "pdrv-proto",
  #endif
        .init             = usb_init,
        .reset            = usb_reset,
        .open             = usb_open,
        .control_xfer_cb  = usb_control_xfer_cb,
        .xfer_cb          = usb_xfer_cb,
        .sof              = NULL
    },
};

/* clang-format on */

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
    *driver_count += TU_ARRAY_SIZE(_app_drivers);

    return _app_drivers;
}
