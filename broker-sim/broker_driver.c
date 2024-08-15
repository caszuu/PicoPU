#include <assert.h>
#include <bits/types/wint_t.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "../core-firmware/common/cluster_bus.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

static const screen_axis_t fb_extent[] = { 100, 100 };

struct rgba_color* fb_color_buffer;
depth_t* fb_depth_buffer;

void save_fb() {
  // linearize (untile) fb images

  struct rgba_color* color_buf;
  uint8_t* depth_buf;

  color_buf = malloc(sizeof(struct rgba_color) * fb_extent[0] * fb_extent[1]);
  depth_buf = malloc(sizeof(uint8_t) * fb_extent[0] * fb_extent[1]);

  for (uint16_t y = 0; y < fb_extent[1] / RENDER_QUAD_SIZE; y++) {
    for (uint16_t x = 0; x < fb_extent[0] / RENDER_QUAD_SIZE; x++) {
      for (uint8_t tile_i = 0; tile_i < RENDER_QUAD_SIZE * 2; tile_i++) {
        uint32_t lin_i = (x * 2) + (tile_i % 2) /* x axis */ + ((y * 2 + tile_i / 2) * fb_extent[0]) /* y axis */;
        uint32_t fb_i = (x * 4) /* x axis */ + (y * 2 * fb_extent[0]) /* y axis*/ + tile_i /* tile index */;

        // printf("tx: %d ty: %d li: %d fbi: %d ti: %d\n", x, y, lin_i, fb_i, tile_i);
      
        color_buf[lin_i] = fb_color_buffer[fb_i];
        depth_buf[lin_i] = fb_depth_buffer[fb_i] / (32 / 8); // map to 8-bit channel range
      }
    }
  }

  // save linear image buffers

  stbi_write_png_compression_level = 2;
  stbi_write_png("broker_sim_color0.png", fb_extent[0], fb_extent[1], 4, fb_color_buffer, sizeof(struct rgba_color) * fb_extent[0]);
  stbi_write_png("broker_sim_depth.png", fb_extent[0], fb_extent[1], 1, depth_buf, sizeof(uint8_t) * fb_extent[0]);

  free(color_buf);
  free(depth_buf);
}

void clear_fb(struct rgba_color clear_color, depth_t clear_depth) {
  for (uint32_t i = 0; i < fb_extent[0] * fb_extent[1]; i++) {
    fb_color_buffer[i] = clear_color;
    fb_depth_buffer[i] = clear_depth;
  }
}

/* Serial cluster bus emu */

#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>

const char* picopu_device = "/dev/ttyACM0";
int picopu_fd = 0;

// termios setup from https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
void setup_serial(int speed, int parity) {
  picopu_fd = open(picopu_device, O_RDWR, O_RDWR | O_NOCTTY | O_SYNC);
  if (picopu_fd == -1) {
    printf("can't access picopu device on %s: %s", picopu_device, strerror(errno));
    exit(-1);
  }
  
  struct termios tty;
  if (tcgetattr (picopu_fd, &tty))
  {
    printf("failed to tcgetattr shader chip: %s", strerror(errno));
    exit(-1);
  }

  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;         // disable break processing
  tty.c_lflag = 0;                // no signaling chars, no echo,
                                  // no canonical processing
  tty.c_oflag = 0;                // no remapping, no delays
  tty.c_cc[VMIN]  = 1;            // read does block
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr (picopu_fd, TCSANOW, &tty) != 0)
  {
    printf("failed to tcsetattr shader chip: %s", strerror(errno));
    exit(-1);
  }
}

void stream_shader(void* data, uint16_t size) {
  uint8_t* tbuf = malloc(sizeof(uint16_t) + size);
  
  *(uint16_t*)tbuf = size;
  memcpy(tbuf + 2, data, size);

  // write(picopu_fd, tbuf, sizeof(uint16_t) + size);

  free(tbuf);
}

bool recv_shader(void** res, gcs_type_t expected_type, gcs_type_t secondary_expected_type) {
  uint16_t size;
  int n = read(picopu_fd, &size, sizeof(uint16_t));

  assert(n && "shader chip disconected while busy!");

  uint16_t size_read = 0;
  uint8_t* buf = malloc(size);
  while (size_read < size) {
    n = read(picopu_fd, buf + size_read, size - size_read);

    assert(n && "shader chip disconected while busy!");
    size_read += n;
  }

  assert(*(gcs_type_t*)(buf) == expected_type || *(gcs_type_t*)(buf) == secondary_expected_type && "unexpected gcs packet");

  if (res) {
    *res = buf;
  }
  
  return true;
}

/* broker sim */

int main() {
  float vertex_buffer[3][3][1] = { { 1.f, 1.f, 1.f }, { 1.f, 1.f, 1.f }, { 1.f, 1.f, 1.f }, };
  static const size_t vertex_output_stride = sizeof(float) * 3;

  static const screen_axis_t fb_extent[] = { 128, 128 };

  fb_color_buffer = malloc(sizeof(struct rgba_color) * fb_extent[0] * fb_extent[1]);
  fb_depth_buffer = malloc(sizeof(depth_t) * fb_extent[0] * fb_extent[1]);

  setup_serial(B115200, 0);
  clear_fb((struct rgba_color){ 64, 64, 64, 255 }, UINT32_MAX);

  // begin

  printf("seting up gcs state...\n");

  struct gcs_begin begin_msg = {
    gcs_type_begin,
    { 100, 100 },
  };

  stream_shader(&begin_msg, sizeof(struct gcs_begin));
  recv_shader(NULL, gcs_type_ready, gcs_type_ready); // wait for gcs_ready

  // pipeline

  struct gcs_gp_conf_header pipeline = {
    gcs_type_gp_conf,
    0, 0,
    sizeof(vertex_buffer[0]),
    sizeof(float[3]),
    2, // prim mode
  };

  stream_shader(&pipeline, sizeof(struct gcs_gp_conf_header));
  recv_shader(NULL, gcs_type_ready, gcs_type_ready);

  // vertex stream

  printf("streming vertex commands...\n");

  struct gcs_vs_header* v_stream = malloc(sizeof(struct gcs_vs_header) + sizeof(vertex_buffer));
  *v_stream = (struct gcs_vs_header){
    gcs_type_vs,
    1, 0
  };

  memcpy((void*)v_stream + sizeof(struct gcs_vs_header), vertex_buffer, sizeof(vertex_buffer));

  stream_shader(&v_stream, sizeof(struct gcs_vs_header) + sizeof(vertex_buffer));
  free(v_stream);
  
  struct gcs_po_header* prim_output;
  recv_shader((void**)&prim_output, gcs_type_po, gcs_type_po);

  printf("received primitive outputs...\n");

  uint8_t* po_seek = (uint8_t*)prim_output + sizeof(struct gcs_po_header);

  const screen_axis_t* size_buf = (screen_axis_t*)po_seek;
  po_seek += sizeof(screen_axis_t) * prim_output->primitive_count;
   
  const struct clip_point* prim_buf = (struct clip_point*)po_seek;
  po_seek += sizeof(struct clip_point) * 3 * prim_output->primitive_count;

  const uint8_t* vertex_output_buf = po_seek;
  po_seek += vertex_output_stride * 3 * prim_output->primitive_count;

  // fragment stream

  for (uint8_t i = 0; i < prim_output->primitive_count; i++) {
    printf("streaming fragment commands for primitive_index: %d\n", i);
        
    struct gcs_fs_header* f_stream = malloc(sizeof(struct gcs_fs_header) + sizeof(struct clip_point) * 3 + vertex_output_stride * 3);
    f_stream->type = gcs_type_fs;

    memcpy(&f_stream->line_offsets, (screen_axis_t[]) {0, 0, 0, 0}, sizeof(f_stream->line_offsets));
    memcpy(&f_stream->line_counts, (uint8_t[]) {size_buf[i], 0, 0, 0}, sizeof(f_stream->line_counts));

    uint8_t* fs_seek = (uint8_t*)(f_stream) + sizeof(struct gcs_fs_header);

    memcpy(fs_seek, &prim_buf[i * 3], sizeof(struct clip_point) * 3);
    fs_seek += sizeof(struct clip_point) * 3 * CHIPS_PER_CLUSTER;

    memcpy(fs_seek, &vertex_output_buf[i * vertex_output_stride * 3], vertex_output_stride * 3);
    fs_seek += vertex_output_stride * 3 * CHIPS_PER_CLUSTER;

    stream_shader(&f_stream, sizeof(struct gcs_fs_header) + sizeof(struct clip_point) * 3 + vertex_output_stride * 3);
    free(f_stream);

    // fragment output

    while (true) {
      struct gcs_fo_header* f_output;
      recv_shader((void**)&f_output, gcs_type_fo, gcs_type_ready);

      // if shader chip finished its fragment stream
      if (f_output->type == gcs_type_ready) break;

      printf("receiving fragment outputs, patching framebuffer...\n");

      uint8_t* fo_seek = (uint8_t*)f_output + sizeof(struct gcs_fo_header);

      uint8_t* cv_mask_buf = fo_seek;
      fo_seek += f_output->tile_count / 2 + (f_output->tile_count % 2);

      const struct color_tile* ct_buf = (struct color_tile*)fo_seek;
      fo_seek += f_output->tile_count * sizeof(struct color_tile);

      const struct depth_tile* dt_buf = (struct depth_tile*)fo_seek;
      fo_seek += f_output->tile_count * sizeof(struct depth_tile);

      // PERF TODO: dma request color tile range before depth-test

      depth_t* fb_dt = &fb_depth_buffer[f_output->fb_index_base];
      for (uint8_t i = 0; i < f_output->tile_count; i++) {
        // late depth-test

        const struct depth_tile* dt = &dt_buf[i];

        uint8_t depth_mask = (fb_dt[0] < dt->d[0]) << 0 |
                   (fb_dt[1] < dt->d[1]) << 1 |
                   (fb_dt[2] < dt->d[2]) << 2 |
                   (fb_dt[3] < dt->d[3]) << 3;

        memcpy(&fb_dt[0], (depth_mask & (1 << 0)) ? &dt->d[0] : &fb_dt[0], sizeof(depth_t));
        memcpy(&fb_dt[1], (depth_mask & (1 << 1)) ? &dt->d[1] : &fb_dt[1], sizeof(depth_t));
        memcpy(&fb_dt[2], (depth_mask & (1 << 2)) ? &dt->d[2] : &fb_dt[2], sizeof(depth_t));
        memcpy(&fb_dt[3], (depth_mask & (1 << 3)) ? &dt->d[3] : &fb_dt[3], sizeof(depth_t));

        cv_mask_buf[i / 2] &= depth_mask << (4 * (i & 2));
        fb_dt += sizeof(depth_t) * RENDER_QUAD_SIZE * 2;
      }

      struct rgba_color* fb_ct = &fb_color_buffer[f_output->fb_index_base];
      for (uint8_t i = 0; i < f_output->tile_count; i++) {
        // framebuffer patch

        const struct color_tile* ct = &ct_buf[i];
        const uint8_t cv_mask = cv_mask_buf[i / 2] << 4 * (i % 2);

        // PERF TODO: hw interp selector (mask out) and single dma write after

        memcpy(&fb_ct[0], (cv_mask & (1 << 0)) ? &ct->c[0] : &fb_ct[0], sizeof(struct rgba_color));
        memcpy(&fb_ct[1], (cv_mask & (1 << 1)) ? &ct->c[1] : &fb_ct[1], sizeof(struct rgba_color));
        memcpy(&fb_ct[2], (cv_mask & (1 << 2)) ? &ct->c[2] : &fb_ct[2], sizeof(struct rgba_color));
        memcpy(&fb_ct[3], (cv_mask & (1 << 3)) ? &ct->c[3] : &fb_ct[3], sizeof(struct rgba_color));
        
        fb_ct += sizeof(struct rgba_color) * RENDER_QUAD_SIZE * 2;
      }      
    };

    // display

    printf("fragment stream finished, saving framebuffer...\n");
    save_fb();
  }

  return 0;
}
