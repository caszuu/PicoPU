add_library(drv_usb_dma
    dma.c
)

target_link_libraries(drv_usb_dma
    hardware_dma
    hardware_irq

    aux_includes
    drv_usb_common
)
