# generic usb support library

add_library(drv_usb_common INTERFACE)

target_include_directories(drv_usb_common INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(drv_usb_common INTERFACE
    hardware_dma
    hardware_irq

    pico_stdlib
    pico_unique_id

    tinyusb_device
    tinyusb_board
)

# usb interface implementations

add_subdirectory(dma)
