add_library(drv_dvi
    dvi.c
)

target_link_libraries(drv_dvi
    hardware_dma
    hardware_irq
    hardware_hstx
)
