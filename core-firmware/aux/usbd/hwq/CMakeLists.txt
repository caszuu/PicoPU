add_library(drv_usb_hw_queue
    hw_queue.c
)

target_link_libraries(drv_usb_hw_queue
    aux_includes
    drv_usb_common

    aux_cp
)
