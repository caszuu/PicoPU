#include "graphics_state.h"
#include "../common/cluster_bus.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

/* vertex pipeline stages */

// points to the start of the vertex assemblies in one of the packet_buffer
// the memory range is gonna be reused of the
uint8_t* vertex_in_pbuf;

// gcs_po buffers
#define po_buf_size(vert_count) sizeof(struct gcs_po_header) + sizeof(screen_axis_t) * (vert_count / 3) + sizeof(struct clip_point) * vert_count + output_vertex_stride * vert_count

screen_axis_t frag_size_buf[sizeof(screen_axis_t) * (MAX_VERTICES_PER_STREAM / 3)];
struct clip_point clip_buf[sizeof(struct clip_point) * MAX_VERTICES_PER_STREAM];
uint8_t vertex_out_buf[MAX_VERTEX_OUTPUT_STRIDE * MAX_VERTICES_PER_STREAM];

bool exec_vertex_stage(uint32_t vertex_index, void* in_buf, struct clip_point* clip_buf, void* out_buf);

void assign_vertex_stream(struct gcs_vs_header* stream) {
    if (!stream->vertex_counts[SHADER_CHIP_ID]) { // no verticies for this chip
        struct gcs_po_header p = { gcs_type_po, 0 };
        uint16_t p_size = sizeof(p);

        format_dbg("empty vertex stream");

        fwrite(&p_size, sizeof(uint16_t), 1, stdout);
        fwrite(&p, sizeof(p), 1, stdout);
        fflush(stdout);

        return;
    }

    uint8_t vertex_count = stream->vertex_counts[SHADER_CHIP_ID];

    vertex_in_pbuf = (uint8_t*)(stream) + sizeof(struct gcs_vs_header);

    // assign core1 vertices
    
    // exec vertex shader and primitive culling
    // FIXME: hard-coded for trigs
    // FIXME: sanity size checks

    const uint8_t per_prim_vertex_count = 3;

    bool is_visible = false;
    uint8_t prim_vertex_index = 0;
    uint8_t prim_index = 0;

    uint16_t output_vertex_index = 0;

    format_dbg("starting vertex stream");

    for (uint32_t vertex_index = 0; vertex_index < vertex_count; vertex_index++) {
        is_visible |= exec_vertex_stage(
            vertex_index, 
            vertex_in_pbuf + vertex_index * vertex_stride,
            clip_buf + output_vertex_index, 
            vertex_out_buf + output_vertex_index * output_vertex_stride
        );

        output_vertex_index++;

        if (++prim_vertex_index == per_prim_vertex_count) {
            // primitive finish - cull or store

            // FIXME: ccw/cw backface/frontface culling
            if (is_visible) {
                output_vertex_index -= per_prim_vertex_count;
            }
            frag_size_buf[prim_index] = fb_extent[0]; // FIXME: temp.

            // reset primitive state

            is_visible = false;
            prim_vertex_index = 0;

            prim_index++;
            continue;
        }
    }
    
    format_dbg("finished vertex stream");

    struct gcs_po_header p = { gcs_type_po, vertex_count / 3 };
    uint16_t p_size = po_buf_size(vertex_count);

    fwrite(&p_size, sizeof(uint16_t), 1, stdout);
    fwrite(&p, sizeof(p), 1, stdout);

    fwrite(frag_size_buf, sizeof(screen_axis_t) * p.primitive_count, 1, stdout);
    fwrite(clip_buf, sizeof(struct clip_point) * vertex_count, 1, stdout);
    fwrite(vertex_out_buf, output_vertex_stride * vertex_count, 1, stdout);

    fflush(stdout);
}
