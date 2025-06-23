#pragma once

#include <stdint.h>
#include <tusb.h>

/*
 * this implementation is based on the https://github.com/notro/pico-usbtest and https://github.com/raspberrypi/pico-sdk/pull/197 driver impl.
 */

#define USB_XFER_MAX_PAK_SIZE 64
#define USB_XFER_MAX_STREAM_SEG_SIZE 1024

#define USB_XFER_RING_COUNT 4

struct usb_config_descriptor {
    tusb_desc_configuration_t config;
    tusb_desc_interface_t interface;

    tusb_desc_endpoint_t trans_up_endpoint;
    tusb_desc_endpoint_t trans_down_endpoint;
};

#define USB_ENDPOINT_DESC(_addr, _attr, _size, _interval) \
    {                                                     \
        .bLength = sizeof(tusb_desc_endpoint_t),          \
        .bDescriptorType = TUSB_DESC_ENDPOINT,            \
        .bEndpointAddress = _addr,                        \
        .bmAttributes = _attr,                            \
        .wMaxPacketSize = _size,                          \
        .bInterval = _interval,                           \
    }

// public api //

// called when a packet is received from host
void usb_pak_in_cb(const void *p, uint16_t size);

// pull-in next usb xfer, can only be called from usb_pak_in_cb
void usb_setup_in_pak();
void usb_setup_in_stream(void *dst, uint32_t size);

// queue data to host, blocking if out queue is full
void usb_out_pak(const void *p, uint16_t size);
void usb_out_stream(void *src, uint32_t size);

// called when a control packet is received from host
bool usb_ctl_pak_cb(uint8_t req, uint16_t val, uint16_t idx, bool is_data_stage);

// handle control xfer data and status stages, can only be called from usb_ctl_pak_cb
//   note: usb_ctl_data handles both data and status stages, do not call both _data and _status at once
void usb_ctl_data(void *buf, uint16_t len);
void usb_ctl_status();
