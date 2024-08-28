#include "../common/cluster_bus.h"
#include "graphics_state.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

/* vertex pipeline stages */

bool exec_vertex_stage(struct gcs_vs_header *h, void *in_buf);

void process_vertex_stream(struct gcs_vs_header *stream) {
    if (!stream->primitive_count) { // no verticies for this chip
        struct gcs_po_header p = {gcs_type_po, 0};
        uint16_t p_size = sizeof(p);

        format_dbg("empty vertex stream");

        put_buffer(&p_size, sizeof(uint16_t));
        put_buffer(&p, sizeof(p));
        stdio_flush();

        return;
    }

    uint8_t *in_buf = (uint8_t *)(stream) + sizeof(struct gcs_vs_header);

    // TODO: assign core1 vertices

    exec_vertex_stage(stream, in_buf);
}
