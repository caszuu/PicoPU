# Add executable. Default name is the project name, version 0.1

add_executable(shader_firmware
    chip_state.c
    
    graphics_state.c
    raster_stage.c
    vertex_stage.c

    test_def.c
)

pico_set_program_name(shader_firmware "shader_firmware")
pico_set_program_version(shader_firmware "0.1")

# Generate PIO header
# pico_generate_pio_header(shader_firmware ${CMAKE_CURRENT_LIST_DIR}/blink.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(shader_firmware 0)
pico_enable_stdio_usb(shader_firmware 1)

# Add the standard library to the build
target_link_libraries(shader_firmware
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(shader_firmware PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# Add any user requested libraries
target_link_libraries(shader_firmware 
        hardware_dma
        hardware_pio
        hardware_interp
        hardware_watchdog
        hardware_clocks
        )

pico_add_extra_outputs(shader_firmware)

