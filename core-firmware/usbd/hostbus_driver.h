#pragma once

#include <common/picopu_types.h>
#include <common/tusb_types.h>
#include <tusb.h>

/* a usbd driver designed for the PicoPU and it's driver */
// the implementation is based of the https://github.com/notro/pico-usbtest and https://github.com/raspberrypi/pico-sdk/pull/197 driver impl.

struct hostbus_config_descriptor /* TUSB_ATTR_PACKED */ {
    tusb_desc_configuration_t config;
    tusb_desc_interface_t interface;

    tusb_desc_endpoint_t sync_endpoint;
    tusb_desc_endpoint_t trans_up_endpoint;
    tusb_desc_endpoint_t trans_down_endpoint;
};

#define HOSTBUS_ENDPOINT_DESC(_addr, _attr, _size, _interval) \
    {                                                         \
        .bLength = sizeof(tusb_desc_endpoint_t),              \
        .bDescriptorType = TUSB_DESC_ENDPOINT,                \
        .bEndpointAddress = _addr,                            \
        .bmAttributes = _attr,                                \
        .wMaxPacketSize = _size,                              \
        .bInterval = _interval,                               \
    }

// out xfer fifo buf
struct hb_xfer {
    uint16_t xfer_offset;
    uint16_t xfer_size;
};

struct hostbus_device_state {
    // used for blocking in xfers, 0 until in xfer is complete
    uint16_t in_xfer_bytes;

    struct hb_xfer out_xfer_buf[128];
    uint16_t out_xfer_buffered;
    uint16_t out_xfer_sent;
};

extern struct hostbus_device_state hb_dev;

// blocks for a in hb packet, returns true number of bytes read
uint16_t hostbus_xfer_in_blocking(void* buf, uint16_t max_bytes);

// puts a hb packet on the out endpoint, may block, buffer can be deleted after return
void hostbus_xfer_out(void* buf, uint16_t size);

/* hostbus protocol impl */

enum hb_type {
    hb_null = 0,

    hb_type_transfer,
    // hb_type_fetch, // used for fence / sync ops

    hb_type_scs_proc,
    hb_type_flash,

    hb_type_dbg,
};

struct __attribute__((packed)) hb_base_header {
    enum hb_type type;
};

struct __attribute__((packed)) hb_transfer_header {
    enum hb_type type;

    uint32_t xfer_size;
    res_addr_t device_addr;
};

struct __attribute__((packed)) hb_dbg_header {
    enum hb_type type;

    char dbg_msg[512];
};