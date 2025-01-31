from hostd import GCSHeaders, SCSHeaders, scs_read, scs_write

import os
import time
import math
import struct

# == test fb preview ==

import pygame as pg

try:
    from pyglm import glm
except ImportError:
    import glm

DEBUG_WINDOW = True
fb_res = (640, 480)

pg.init()

if DEBUG_WINDOW:
    fb = pg.display.set_mode(fb_res, flags=pg.RESIZABLE, vsync=False)
    pg.display.set_caption("gtest fb view")
else:
    fb = pg.Surface(fb_res)

fb_patch_fifo = []

def patch_fb(tile_count: int, fb_base: tuple[int, int], cv_buf: bytes, col_data: bytes, d_data: bytes) -> None:
    for i in range(tile_count):
        col_tile = col_data[i * GCSHeaders.COL_TILE_SIZE:(i + 1) * GCSHeaders.COL_TILE_SIZE]
        # d_tile = d_data[i * GCSHeaders.D_TILE_SIZE:(i + 1) * GCSHeaders.D_TILE_SIZE]

        cv = int.from_bytes(cv_buf[i * 2:i * 2 + 2], 'little')

        for tx in range(4):
            for ty in range(4):
                ti = tx + ty * 4
        
                if cv & (1 << ti):
                    fb.set_at((fb_base[0] + (i * 4) + tx, fb_base[1] + ty), col_tile[ti * 4:ti * 4 + 3])

os.makedirs("fbs/", exist_ok=True)

fb_save_index = 0
def save_fb() -> None:
    global fb_save_index
    pg.image.save(fb, f"fbs/fb_col{fb_save_index}.png")
    fb_save_index += 1

# == test driver ==

vert_fmt = '<fff'
clip_fmt = '<iif'

bytes_written = 0
bytes_read = 0

ctime = 0

def gen_cube_verts() -> list[bytes]:
    vert_array = [
        # front face
        glm.vec3(-1, -1, -1),
        glm.vec3(-1, 1, -1),
        glm.vec3(1, 1, -1),
        glm.vec3(-1, -1, -1),
        glm.vec3(1, 1, -1),
        glm.vec3(1, -1, -1),

        # back face
        glm.vec3(-1, -1, 1),
        glm.vec3(-1, 1, 1),
        glm.vec3(1, 1, 1),
        glm.vec3(-1, -1, 1),
        glm.vec3(1, 1, 1),
        glm.vec3(1, -1, 1),

        # left face
        glm.vec3(-1, -1, -1),
        glm.vec3(-1, -1, 1),
        glm.vec3(-1, 1, 1),
        glm.vec3(-1, -1, -1),
        glm.vec3(-1, 1, 1),
        glm.vec3(-1, 1, -1),

        # right face
        glm.vec3(1, -1, -1),
        glm.vec3(1, -1, 1),
        glm.vec3(1, 1, 1),
        glm.vec3(1, -1, -1),
        glm.vec3(1, 1, 1),
        glm.vec3(1, 1, -1),
    ]

    # rotate vertices on cpu side (for now) to animate the cube
    clip_space_verts = []
    for v in vert_array:
        clip_space_verts.append(glm.rotateY(glm.rotateX(v * glm.vec3(.5), glm.radians(45)), time.perf_counter() / 5))

    return clip_space_verts

def dispatch_gcs_frame():
    global bytes_written, bytes_read, ctime

    # setup gcs cbuf

    cmd_data = SCSHeaders.pack_ld_cbuf(0, GCSHeaders.pack_gs(fb_res))
    scs_write(cmd_data)
    bytes_written += len(cmd_data)

    # vertex stage
    
    # vert_buf = []
    # 
    # for _ in range(1):
    #     # test_prims = [
    #     #     struct.pack(clip_fmt, random.randint(0, FB_RES - 1), random.randint(0, FB_RES // 2), 0),
    #     #     struct.pack(clip_fmt, random.randint(0, FB_RES // 2), random.randint(FB_RES // 2, FB_RES - 1), 0),
    #     #     struct.pack(clip_fmt, random.randint(FB_RES // 2, FB_RES * 2 - 1), random.randint(FB_RES // 2, FB_RES * 2 - 1), float('inf')),
    #     # ]
    # 
    #     test_prim = [
    #         struct.pack(vert_fmt, -1, -1, 0),
    #         struct.pack(vert_fmt, -1, 1, 0),
    #         struct.pack(vert_fmt, -.5, 1, 0),
    #     ]
    #     vert_buf.extend(test_prim)

    vert_buf = gen_cube_verts()
    ctime += 1 / 15
    ctime = 0
    
    cmd_data = SCSHeaders.pack_disp_gcs() + GCSHeaders.pack_vs(0, len(vert_buf) // 3, b''.join(vert_buf))
    scs_write(cmd_data)
    bytes_written += len(cmd_data)

    start_time = time.perf_counter()

    prim_output = scs_read(65535)
    bytes_read += len(prim_output)

    prim_count, shading_range = GCSHeaders.unpack_po(prim_output)
    clip_buf = scs_read(65535)
    bytes_read += len(clip_buf)

    fin_time = time.perf_counter()
    print(f"prim output received! vertex time: {fin_time - start_time}s")

    # fragment stage
    cmd_data = SCSHeaders.pack_disp_gcs() + GCSHeaders.pack_fs(shading_range, prim_count, clip_buf)
    scs_write(cmd_data)
    bytes_written += len(cmd_data)

    start_time = time.perf_counter()

    # await fragment output streams and ready gcs cmds
    while True:
        p_head = scs_read(65535)

        if GCSHeaders.is_ready(p_head):
            break

        # read frag output stream
        tile_count, fb_base_x, fb_base_y, to_read = GCSHeaders.unpack_fo(p_head)
        bytes_read += 6 + to_read

        cv_buf = scs_read(tile_count * 2)
        col_buf = scs_read(tile_count * GCSHeaders.COL_TILE_SIZE)
        d_buf = scs_read(tile_count * GCSHeaders.D_TILE_SIZE)

        patch_fb(tile_count, (fb_base_x, fb_base_y), cv_buf, col_buf, d_buf)

    fin_time = time.perf_counter()
    print(f"gcs ready received! fragment time: {fin_time - start_time}s")
    print(fb_res)

    save_fb()

# upload test texture

img_data = pg.image.load("test.png")
img_tex = pg.transform.scale(img_data, (64, 64))

tex_data = pg.image.tobytes(img_tex, 'RGBA')
tex_xfer = SCSHeaders.pack_ld_cbuf(64, tex_data)
scs_write(tex_xfer)

# mini frame loop

while True:
    fb.fill((0, 0, 0))
    dispatch_gcs_frame()

    # frame bandwidth monitor
    print(f" up: {round(bytes_written / 1024, 2)}kB/frame down: {round(bytes_read / 1024, 2)}kB/frame")
    bytes_written = 0
    bytes_read = 0

    if DEBUG_WINDOW:
        pg.display.update()

        for e in pg.event.get():
            if e.type == pg.QUIT:
                exit()
            elif e.type == pg.WINDOWRESIZED:
                fb.fill((0, 0, 0))
                fb_res = (e.dict['x'] - (e.dict['x'] % 2), e.dict['y']  - (e.dict['y'] % 2))
    
    print("---")