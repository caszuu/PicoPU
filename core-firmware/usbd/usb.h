#pragma once

#include <stdint.h>
#include <tusb.h>

/*
 * this implementation is based of the https://github.com/notro/pico-usbtest and https://github.com/raspberrypi/pico-sdk/pull/197 driver impl.
 */

#define USB_XFER_MAX_SIZE 64
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

// usb support todo:
// - recv
//   - stream-style xfer passthrough
// - api
//   - setup out stream-style xfer

// public api //

void usb_pak_in_cb(const void *p, uint16_t size);
void usb_setup_in_pak();
void usb_setup_in_stream(void *dst, uint32_t size);

void usb_out_pak(const void *p, uint16_t size);
// void usb_out_stream(void *buf, uint32_t size, sync_sem_t *fin_sem);

bool usb_ctl_pak_cb(uint8_t req, uint16_t val, uint16_t idx, uint8_t *buf, uint16_t *len);
