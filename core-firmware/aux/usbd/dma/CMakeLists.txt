add_library(drv_usb_dma
    usb_dma.c
)

target_link_libraries(drv_usb_dma
    drv_usb_common
)
