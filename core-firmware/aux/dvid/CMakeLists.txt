add_library(drv_dvi
    dvi.c
)

target_include_directories(drv_dvi PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(drv_dvi
    hardware_dma
    hardware_irq
    hardware_hstx
)
