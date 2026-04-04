add_library(aux_cp
    cp.c
)

target_link_libraries(aux_cp
    hardware_sync
    pico_runtime_init_headers # required (?) for hardware_sync

    aux_includes
)
