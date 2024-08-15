#include "graphics_state.h"
#include "../common/cluster_bus.h"

#include <stdlib.h>
#include <stdbool.h>

#define SHOULD_VERTEX_ALIAS vertex_stride >= output_vertex_stride

/* vertex pipeline stages */

// points to the start of the vertex assemblies in one of the packet_buffer
// the memory range is gonna be reused of the
uint8_t* vertex_in_pbuf;

// points to the start of the output vertex assemblies buffer, this might
// point to the same range as [vertex_in_pbuf] if buffers can be aliased safely
uint8_t* vertex_out_pbuf;

void assign_vertex_stream(struct gcs_vs_header* stream) {
    if (!stream->vertex_counts[SHADER_CHIP_ID]) goto finish; // no verticies for this chip

    vertex_in_pbuf = (uint8_t*)(stream + 1);
    vertex_out_pbuf = SHOULD_VERTEX_ALIAS ? vertex_in_pbuf : malloc((uint16_t)(stream->vertex_counts[SHADER_CHIP_ID]) * (uint16_t)(output_vertex_stride));
    
    uint8_t* vertex_clip_buf;

    // assign core1 vertices
    
    // exec vertex shader and primitive culling
    // FIXME: hard-coded for trigs
    // FIXME: sanity size checks

    const uint8_t per_prim_vertex_count = 3;

    bool is_visible = false;
    uint8_t prim_vertex_index = 0;

    uint16_t output_vertex_index = 0;

    for (uint32_t vertex_index = 0; vertex_index < stream->vertex_counts[SHADER_CHIP_ID]; vertex_index++) {
        is_visible |= exec_vertex_shader(
            vertex_index, 
            vertex_in_pbuf + vertex_index * vertex_stride,
            vertex_clip_buf + output_vertex_index * sizeof(screen_axis_t) * 2, 
            vertex_out_pbuf + output_vertex_index * output_vertex_stride
        );

        output_vertex_index++;

        if (++prim_vertex_index == per_prim_vertex_count) {
            // primitive finish - cull or store

            if (cull) {
                output_vertex_index -= per_prim_vertex_count;
            }

            // reset primitive state

            is_visible = false;
            prim_vertex_index = 0;

            continue;
        }
    }
    
finish:
    uint16_t prim_count = output_vertex_index / per_prim_vertex_count;

    transmit_primitive_outputs();

    if (!(SHOULD_VERTEX_ALIAS)) free(vertex_out_pbuf);
}
