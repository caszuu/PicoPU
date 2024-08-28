#include "graphics_state.h"

#include "hardware/dma.h"
#include "hardware/watchdog.h"

#include <common/cluster_bus.h>

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* gcs state */

uint8_t packet_buffer[MAX_GCS_PACKET_SIZE];

screen_axis_t fb_extent[2];
float view_transform_params[3][2];

/* gp state */

uint8_t *vertex_mcode;
uint8_t *fragment_mcode;

uint8_t const_buffer[MAX_CONSTANT_BUFFER_SIZE];

uint8_t vertex_stride;
uint8_t output_vertex_stride;

// vertices per primitive (first_count, next_count)
uint8_t gp_verts_per_prim[2];

// a preconfigured memcpy config without a channel
dma_channel_config dma_cpy;

volatile bool exit_graphics_state;

void send_ready() {
    struct gcs_ready p = {gcs_type_ready};
    uint16_t p_size = sizeof(p);

    put_buffer(&p_size, sizeof(uint16_t));
    put_buffer(&p, sizeof(p));
    stdio_flush();
}

void send_dbg(struct gcs_dbg *p) {
    uint16_t p_size = sizeof(*p);

    put_buffer(&p_size, sizeof(uint16_t));
    put_buffer(p, sizeof(*p));
    stdio_flush();
}

void enter_graphics_state(struct gcs_begin *conf) {
    // setup the gcs pio block and iqrs

    vertex_mcode = 0;
    fragment_mcode = 0;

    memset(const_buffer, 0, sizeof(const_buffer));

    vertex_stride = 0;

    dma_cpy = dma_channel_get_default_config(0);
    channel_config_set_transfer_data_size(&dma_cpy, DMA_SIZE_8);
    channel_config_set_read_increment(&dma_cpy, true);
    channel_config_set_write_increment(&dma_cpy, true);

    memcpy(fb_extent, conf->fb_extent, sizeof(fb_extent));
    memcpy(view_transform_params, conf->view_transform, sizeof(view_transform_params));

    // FIXME: bootup gcs pio

    send_ready();

    while (!exit_graphics_state) {
        uint16_t p_size;
        fread(&p_size, sizeof(uint16_t), 1, stdin);

        uint16_t size_read = 0;
        while (p_size > size_read) {
            uint16_t n =
                fread(packet_buffer + size_read, 1, p_size - size_read, stdin);

            // if (!n) watchdog_reboot(0, 0, 0);
            size_read += n;
        }

        uint8_t type = packet_buffer[0];

        format_dbg("Packet received; s: %hu t: %hhx", p_size, type);

        switch (type) {
        case gcs_type_begin:
            // FIXME: state change mocking

            send_ready();
            break;

        case gcs_type_gp_conf:
            configure_pipeline((struct gcs_gp_conf_header *)packet_buffer);
            break;

        case gcs_type_vs:
            process_vertex_stream((struct gcs_vs_header *)packet_buffer);
            break;

        case gcs_type_fs:
            process_fragment_stream((struct gcs_fs_header *)packet_buffer);
            break;

        default:
            watchdog_reboot(0, 0, 0);
        }

        // wait for gcs command interupts, if an interupt takes
        // abnormaly long, the watchdog will trigger a shader_stall
        watchdog_update();
    }
}

void cleanup_graphics_state() {
    if (vertex_mcode) {
        free(vertex_mcode);
        free(fragment_mcode);
    }

    send_ready();
}

void configure_pipeline(struct gcs_gp_conf_header *conf) {
    if (vertex_mcode) {
        free(vertex_mcode);
        free(fragment_mcode);
    }

    uint8_t *seek = (uint8_t *)(conf + 1);

    // vertex mcode

    vertex_mcode = malloc(conf->vertex_mcode_size);
    int vert_chan = dma_claim_unused_channel(false);

    if (vert_chan != -1) {
        channel_config_set_chain_to(&dma_cpy, vert_chan);
        dma_channel_configure(vert_chan, &dma_cpy, vertex_mcode, seek,
                              conf->vertex_mcode_size, true);
    } else {
        // fallback sw memcpy
        memcpy(vertex_mcode, seek, conf->vertex_mcode_size);
    }

    seek += conf->vertex_mcode_size;

    // fragment mcode

    fragment_mcode = malloc(conf->fragment_mcode_size);
    int frag_chan = dma_claim_unused_channel(false);

    if (frag_chan != -1) {
        channel_config_set_chain_to(&dma_cpy, frag_chan);
        dma_channel_configure(frag_chan, &dma_cpy, fragment_mcode, seek,
                              conf->fragment_mcode_size, true);
    } else {
        // fallback sw memcpy
        memcpy(fragment_mcode, seek, conf->fragment_mcode_size);
    }

    seek += conf->fragment_mcode_size;

    // pipeline state

    vertex_stride = conf->vertex_stride;
    output_vertex_stride = conf->output_vertex_stride;
    memcpy(gp_verts_per_prim, (uint8_t[]){3, 3}, sizeof(gp_verts_per_prim));

    dma_channel_wait_for_finish_blocking(vert_chan);
    dma_channel_wait_for_finish_blocking(frag_chan);

    send_ready();
}

void update_c_buffer(struct gcs_cb_header *info) {
    uint8_t *seek = (uint8_t *)(info) + sizeof(struct gcs_cb_header);

    memcpy(const_buffer + info->range_offset, seek, info->range_size);
    send_ready();
}
