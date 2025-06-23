add_executable(mockbird_firmware
    chip.c
    cp_layer.c

    # gcs
    ../../gcs/vertex_stage.c
    ../../gcs/raster_stage.c

    # usbd
    ../../usbd/usb_descriptors.c
    ../../usbd/usb.c

    # dvid
    ../../dvid/dvi.c
)

# Setup pico sdk
pico_set_program_name(mockbird_firmware "mockbird_firmware")
pico_set_program_version(mockbird_firmware "0.1")

pico_enable_stdio_uart(mockbird_firmware 0)
pico_enable_stdio_usb(mockbird_firmware 0)

target_link_libraries(mockbird_firmware
        pico_stdlib)

target_include_directories(mockbird_firmware PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts or any other standard includes, if required
    ${CMAKE_CURRENT_LIST_DIR}/../../usbd # specifically for 'tusb_config.h'
)

# Add any user requested libraries
target_link_libraries(mockbird_firmware 
    hardware_dma
    hardware_pio
    hardware_interp
    hardware_watchdog
    hardware_clocks

    pico_unique_id
    tinyusb_device
    tinyusb_board

    pico_bootrom
)

pico_add_extra_outputs(mockbird_firmware)

