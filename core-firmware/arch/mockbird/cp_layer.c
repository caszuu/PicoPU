#include "chip.h"

#include <common/instru.h>
#include <common/pdrv_proto.h>
#include <common/si_proto.h>
#include <dvid/dvi.h>
#include <gcs/unit.h>
#include <usbd/usb.h>

#include <bsp/board_api.h>
#include <tusb.h>

#include <hardware/gpio.h>
#include <hardware/watchdog.h>

#include <assert.h>
#include <pico/bootrom.h>
#include <pico/stdlib.h>
#include <stdint.h>
#include <string.h>

/* command processor fw layer */

#define CMD_QUEUE_MAX_ENTRY_SIZE 64
#define CMD_QUEUE_ENTRY_COUNT 8

static uint8_t queue_fifo[CMD_QUEUE_ENTRY_COUNT][CMD_QUEUE_MAX_ENTRY_SIZE];
static uint32_t queue_head, queue_tail;

static void cp_yield() {
    tud_task();
    watchdog_update();
}

#define align_up(addr, alignment) ((addr + (alignment - 1)) & ~(alignment - 1))
#define next_cmd(p) (const uint32_t *)((const uint8_t *)p + align_up(sizeof(*p), 4u))

/* device command execution */

static const uint32_t *cp_exec_fb_clear(const uint32_t *cmd) {
    const struct cmd_fb_clear *p = (struct cmd_fb_clear *)cmd;

    memset(vaddr(gs.fb_c0), 0, sizeof(uint16_t) * gs.fb_extent[0] * gs.fb_extent[1]);
    memset(vaddr(gs.fb_zs), 255, sizeof(uint16_t) * gs.fb_extent[0] * gs.fb_extent[1]);

    return next_cmd(p);
}

static const uint32_t *cp_exec_fb_present(const uint32_t *cmd) {
    const struct cmd_fb_present *p = (struct cmd_fb_present *)cmd;

    dvi_flip_immediate(vaddr(gs.fb_c0));
    return next_cmd(p);
}

static const uint32_t *cp_exec_xfer(const uint32_t *cmd) {
    const struct cmd_xfer *p = (struct cmd_xfer *)cmd;

    if (p->ptype == cmd_type_xfer_to_device) {
        // request host to peform a to-device xfer
        struct ucmd_xfer_from_device req = {
            .ptype = ucmd_type_xfer_to_device,
            .xfer_idx = p->xfer_idx,
        };

        usb_out_pak(&req, sizeof(req));
    } else {
        // perform a to-host xfer
        struct ucmd_xfer_from_device header = {
            .ptype = ucmd_type_xfer_to_host,
            .xfer_idx = p->xfer_idx,
        };

        usb_out_pak(&header, sizeof(header));
        usb_out_stream(vaddr(p->vram_addr), p->xfer_size);
    }

    return next_cmd(p);
}

static const uint32_t *cp_exec_gcs_write(const uint32_t *cmd) {
    const struct cmd_write_cbuf *p = (const struct cmd_write_cbuf *)cmd;
    memcpy(cbuf + p->offset, vaddr(p->src_addr), p->size);

    return next_cmd(p);
}

static const uint32_t *cp_exec_gcs_push(const uint32_t *cmd) {
    enum cmd_type *ptype = (enum cmd_type *)cmd;

    void *dst;
    const void *src;
    uint32_t size;

    if (*ptype == cmd_type_push_gstate) {
        const struct cmd_push_gstate *p = (struct cmd_push_gstate *)cmd;

        src = cmd + 1; // keep 4-byte aligned
        dst = &gs + p->offset;
        size = p->size;
    } else /*cmd_type_push_cbuf*/ {
        const struct cmd_push_cbuf *p = (struct cmd_push_cbuf *)cmd;

        src = cmd + 2;
        dst = &cbuf + p->offset;
        size = p->size;
    }

    memcpy(dst, src, size);
    return (const uint32_t *)(align_up((uint32_t)src + size, 4u));
}

static const uint32_t *cp_exec_gcs_draw(const uint32_t *cmd) {
    const struct cmd_gcs_draw *p = (struct cmd_gcs_draw *)cmd;

    // batch dispatch
    for (uint32_t pi = 0, vi = 0; pi < p->prim_count; pi += MAX_TRIGS_PER_BATCH, vi += MAX_VERTICES_PER_BATCH) {
        const uint32_t batch_size = MIN(MAX_TRIGS_PER_BATCH, p->prim_count - pi);

        // vertex stage
        struct scs_vertex_batch vb = {
            .type = si_type_vbatch,
            .v2f_idx = 0,

            .primitive_count = batch_size,
            .index_base = vi + p->index_base,
        };

        dispatch_vertex_batch(&vb);

        // raster stage
        struct scs_raster_batch rb = {
            .type = si_type_rbatch,
            .v2f_idx = 0,
        };

        dispatch_raster_batch(&rb);
    }

    // note: since all cmds on mockbird are sync only, the draw_latch flag can be ignored
    // TODO: implement indexed draws

    return next_cmd(p);
}

static void cp_exec(const uint32_t *cmdbuf) {
    while (true) {
        cp_yield();

        enum cmd_type *ptype = (enum cmd_type *)cmdbuf;

        switch (*ptype) {
        case cmd_type_end:
            return; // end of cmdbuf

        case cmd_type_clear:
            cmdbuf = cp_exec_fb_clear(cmdbuf);
            break;

        case cmd_type_present:
            cmdbuf = cp_exec_fb_present(cmdbuf);
            break;

        case cmd_type_xfer_to_device:
        case cmd_type_xfer_to_host:
            cmdbuf = cp_exec_xfer(cmdbuf);
            break;

        case cmd_type_write_cbuf:
            cmdbuf = cp_exec_gcs_write(cmdbuf);
            break;

        case cmd_type_push_cbuf:
        case cmd_type_push_gstate:
            cmdbuf = cp_exec_gcs_push(cmdbuf);
            break;

        case cmd_type_draw:
            cmdbuf = cp_exec_gcs_draw(cmdbuf);
            break;

        default:
            assert(false);
            break;
        }
    }
}

/* cp entry */

void init_cp_layer() {
    // init usb sys

    board_init();
    tusb_init();

    if (board_init_after_tusb) {
        board_init_after_tusb();
    }

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    gpio_put(PICO_DEFAULT_LED_PIN, true);

    while (!tud_ready()) {
        tud_task();
        watchdog_update();
    }

    sleep_ms(1000);
    gpio_put(PICO_DEFAULT_LED_PIN, false);

    // reset cp state

    queue_head = 0;
    queue_tail = 0;
}

void cp_loop() {
    while (true) {
        cp_yield();

        if (queue_head == queue_tail)
            continue; // no queued cmds

        uint32_t next_tail = (queue_tail + 1) % CMD_QUEUE_ENTRY_COUNT;
        struct ucmd_enqueue *qp = (struct ucmd_enqueue *)queue_fifo[queue_tail];

        cp_exec((const uint32_t *)(vaddr(qp->cmdbuf)));
        queue_tail = next_tail;
    }
}

void usb_pak_in_cb(const void *p, uint16_t size) {
    enum ucmd_type *ptype = (enum ucmd_type *)p;

    switch (*ptype) {
    case ucmd_type_xfer_to_device:
        const struct ucmd_xfer_from_host *xfc = (struct ucmd_xfer_from_host *)p;
        usb_setup_in_stream(vaddr(xfc->vram_addr), xfc->xfer_size);
        return;

    case ucmd_type_xfer_to_host:
        assert(false); // to-host xfer requests are not supported yet
        break;

    case ucmd_type_enqueue:
        uint32_t next_head = (queue_head + 1) % CMD_QUEUE_ENTRY_COUNT;
        assert(next_head != queue_tail); // queue overflow

        memcpy(queue_fifo[queue_head], p, size);
        queue_head = next_head;
        break;

    default:
        assert(false);
        break;
    }

    usb_setup_in_pak();
}

/* pdrv usb interface */

enum usb_dev_ctl_type {
    usb_ctl_type_flash = 0,
    usb_ctl_type_hwinfo,

    // usb_ctl_type_modeset,
    usb_ctl_type_led_update,
};

struct usb_hwinfo {
    char hwarch[8];

    char hwid[16];
    char fwsha[16];
};

bool usb_ctl_pak_cb(uint8_t req, uint16_t val, uint16_t idx, bool is_data_stage) {
    switch (req) {
    case usb_ctl_type_flash:
        reset_usb_boot(val, 0); // note: does not return
        return true;

    case usb_ctl_type_hwinfo:
        struct usb_hwinfo hwi;
        memcpy(hwi.hwarch, "mock\0\0\0\0", 8);

        usb_ctl_data(&hwi, sizeof(hwi));
        return true;

    default:
        return false; // unsupported devctl request
    }
}
