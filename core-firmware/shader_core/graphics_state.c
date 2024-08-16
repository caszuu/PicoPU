#include "graphics_state.h"

#include "hardware/watchdog.h"
#include "hardware/dma.h"

#include <common/cluster_bus.h>

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>

/* gcs state buffer */

uint8_t packet_buffer[MAX_GCS_PACKET_SIZE];

/* graphics state */

screen_axis_t fb_extent[2];

uint8_t* vertex_mcode;
uint8_t* fragment_mcode;

// uint8_t* const_buffers[MAX_CONSTANT_BUFFERS];
uint8_t* const_buffer;

uint8_t vertex_stride;
uint8_t output_vertex_stride;
primitive_mode_t prim_mode;

// a preconfigured memcpy config without a channel
dma_channel_config dma_cpy;

volatile bool exit_graphics_state;

/* graphics commands */

void send_ready() {
    struct gcs_ready p = { gcs_type_ready };
    uint16_t p_size = sizeof(p);

    fwrite(&p_size, sizeof(uint16_t), 1, stdout);
    fwrite(&p, sizeof(p), 1, stdout);
    fflush(stdout);
}

void send_dbg(struct gcs_dbg* p) {
    uint16_t p_size = sizeof(*p);
    
    fwrite(&p_size, sizeof(uint16_t), 1, stdout);
    fwrite(p, sizeof(*p), 1, stdout);
    fflush(stdout);
}

void enter_graphics_state() {
    // setup the gcs pio block and iqrs

    // memcpy(fb_extent, info->fb_extent, sizeof(fb_extent));

    vertex_mcode = 0;
    fragment_mcode = 0;

    // memset(const_buffers, 0, sizeof(const_buffers[0]) * MAX_CONSTANT_BUFFERS);
    const_buffer = 0;

    vertex_stride = 0;
    prim_mode = 0;

    dma_cpy = dma_channel_get_default_config(0);
    channel_config_set_transfer_data_size(&dma_cpy, DMA_SIZE_8);
    channel_config_set_read_increment(&dma_cpy, true);
    channel_config_set_write_increment(&dma_cpy, true);

    // FIXME: bootup gcs pio

    send_ready();

    while (!exit_graphics_state) {
        uint16_t p_size;
        fread(&p_size, sizeof(uint16_t), 1, stdin);

        uint16_t size_read = 0;
        while (p_size > size_read) {
            uint16_t n = fread(packet_buffer + size_read, 1, p_size - size_read, stdin);

            // if (!n) watchdog_reboot(0, 0, 0);
            size_read += n;
        }

        uint8_t type = packet_buffer[0];

        format_dbg("Packet received; s: %hu t: %hhx", p_size, type);

        switch (type) {
        case gcs_type_begin:
            struct gcs_begin* h = (struct gcs_begin*)packet_buffer;
            memcpy(fb_extent, h->fb_extent, sizeof(fb_extent));

            send_ready();
            break;
        
        case gcs_type_gp_conf:
            configure_pipeline((struct gcs_gp_conf_header*)packet_buffer);
            break;

        case gcs_type_vs:
            assign_vertex_stream((struct gcs_vs_header*)packet_buffer);
            break;

        case gcs_type_fs:
            assign_fragment_stream((struct gcs_fs_header*)packet_buffer);
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

    if (const_buffer) {
        free(const_buffer);
    }

    send_ready();
}

void configure_pipeline(struct gcs_gp_conf_header* conf) {
    if (vertex_mcode) {
        free(vertex_mcode);
        free(fragment_mcode);
    }

    uint8_t* seek = (uint8_t*)(conf + 1);

    // vertex mcode

    vertex_mcode = malloc(conf->vertex_mcode_size);
    int vert_chan = dma_claim_unused_channel(false);

    if (vert_chan != -1) {
        channel_config_set_chain_to(&dma_cpy, vert_chan);
        dma_channel_configure(vert_chan, &dma_cpy, vertex_mcode, seek, conf->vertex_mcode_size, true);
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
        dma_channel_configure(frag_chan, &dma_cpy, fragment_mcode, seek, conf->fragment_mcode_size, true);
    } else {
        // fallback sw memcpy
        memcpy(fragment_mcode, seek, conf->fragment_mcode_size);
    }

    seek += conf->fragment_mcode_size;

    // pipeline state

    vertex_stride = conf->vertex_stride;
    output_vertex_stride = conf->output_vertex_stride;
    prim_mode = conf->prim_mode;

    dma_channel_wait_for_finish_blocking(vert_chan);
    dma_channel_wait_for_finish_blocking(frag_chan);

    send_ready();
}

void update_c_buffer(struct gcs_cb_header* info) {
    if (const_buffer) free(const_buffer);

    assert(!info->buffer_index);
    uint8_t* seek = (uint8_t*)(info + 1);

    const_buffer = malloc(info->buffer_size);
    memcpy(const_buffer, seek, info->buffer_size);

    send_ready();
}

/* vertex pipeline stages */

void assign_vertex_stream(struct gcs_vs_header* stream);

/* fragment pipeline stages */

void assign_fragment_stream(struct gcs_fs_header* stream);
