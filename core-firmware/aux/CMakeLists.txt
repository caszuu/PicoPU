# add the top-level aux directory to the include path
add_library(aux_includes INTERFACE)
target_include_directories(aux_includes INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

# add the auxiliary projects
add_subdirectory(dvid)
add_subdirectory(usbd)
# add_subdirectory(pl)
