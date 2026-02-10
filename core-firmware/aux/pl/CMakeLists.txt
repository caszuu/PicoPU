# pico-link driver #

add_library(drv_pl
    pl.c
)

# compile pio phy program
pico_generate_pio_header(drv_pl ${CMAKE_CURRENT_LIST_DIR}/pl_phy.pio)

target_link_libraries(drv_pl
    hardware_pio
    hardware_dma
    hardware_irq

    aux_includes
)

# pico-link tests #

add_executable(pl_tests
    pl_test.c

    tests/test_tx_minimal.c
)

pico_set_program_name(pl_tests "pl_test")
pico_set_program_version(pl_tests "none")

pico_enable_stdio_uart(pl_tests 0)
pico_enable_stdio_usb(pl_tests 1)

# Wait for host serial console to connect
target_compile_definitions(pl_tests PRIVATE PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=2500)

target_include_directories(pl_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../.. # match the main project include paths for consistentcy
)

target_link_libraries(pl_tests
    pico_stdlib
    pico_stdio

    drv_pl
)

pico_add_extra_outputs(pl_tests)
