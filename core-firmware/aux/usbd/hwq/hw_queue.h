#pragma once

#include <device/usbd_pvt.h>
#include <stdbool.h>

// hardware command queue driver //

/*
 * checks if the hw queue interface is currently attached to a host.
 */

bool hwq_is_host_present();

/*
 * insert the hwq tinyusb driver to a usbd decriptor list. after initializing
 * the driver will become active and start awaiting a host.
 */

void hwq_get_driver_desc(usbd_class_driver_t *out_desc);
