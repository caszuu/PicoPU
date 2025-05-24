from hostd import GCSHeaders, SCSHeaders, _dev_read, _dev_write

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

DEBUG_WINDOW = False
DEBUG_CPU_WIREFRAME = False
fb_res = (640, 480)
# fb_res = (120, 100)

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
    pg.image.save(fb, f"fbs/fb_col{fb_save_index:0=3}.png")
    fb_save_index += 1

# == test driver ==

vert_fmt = '<fff'
clip_fmt = '<iif'

bytes_written = 0
bytes_read = 0

ctime = 1

m_vbuf = []
m_fbuf = []

def load_model(path = "bunny.obj"):
    with open(path, 'r') as f:
        for l in f.readlines():
            if l.startswith("v "):
                # parse vertex
                l = l[2:].strip()
                axis = l.split(" ")

                m_vbuf.append(glm.vec3(float(axis[0]), float(axis[1]), float(axis[2])))

            elif l.startswith("f "):
                # parse face
                l = l[2:].strip()
                indices = l.split(" ")

                m_fbuf.append((int(indices[0].split("/")[0]) - 1, int(indices[1].split("/")[0]) - 1, int(indices[2].split("/")[0]) - 1))
    
    print(f"loaded model verts: {len(m_vbuf)} faces: {len(m_fbuf)}")

load_model("teapot.obj")

def cpu_vert_stage(vert_array: list[glm.vec3]) -> list[bytes]:
    # rotate vertices on cpu side (for now) to animate the verts
    clip_space_verts = []
    for v in vert_array:
        clip_space_verts.append(glm.rotateY(glm.rotateX(v * glm.vec3(.2), glm.radians(45)), ctime / 5))

    return clip_space_verts

def gen_model_vert_array() -> list[bytes]:
    vert_array = []

    for f in m_fbuf:
        for i in f:
            vert_array.append(m_vbuf[i] * glm.vec3(1, -1, 1))

    return vert_array

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

    return cpu_vert_stage(vert_array)

def dispatch_gcs_frame():
    global bytes_written, bytes_read, ctime

    # setup gcs cbuf

    cmd_data = SCSHeaders.pack_ld_cbuf(0, GCSHeaders.pack_gs(fb_res))
    _dev_write(cmd_data)
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

    # vert_buf = gen_cube_verts()
    vert_buf = cpu_vert_stage(gen_model_vert_array())
    ctime += 1 / 15

    # split up vertex array into 6-vert vertex streams
    vert_streams = [vert_buf[i:i + 6] for i in range(0, len(vert_buf), 6)]
    start_time = time.perf_counter()

    for vs in vert_streams:

        # assign batch
        cmd_data = SCSHeaders.pack_disp_gcs() + GCSHeaders.pack_assign(0, len(vs) // 3, b''.join(vs))
        _dev_write(cmd_data)
        bytes_written += len(cmd_data)

        # await fragment output streams and ready gcs cmds
        while True:
            p_bytes = _dev_read(65535)
            p = GCSHeaders.unpack(p_bytes)

            if p[0] == 16:
                print(p)

            if p[0] == 17:
                break

            if p[0] == 18:
                continue

                # read frag output stream
                tile_count, fb_base_x, fb_base_y, to_read = GCSHeaders.unpack_fo(p_head)
                bytes_read += 6 + to_read

                cv_buf = _dev_read(tile_count * 2)
                col_buf = _dev_read(tile_count * GCSHeaders.COL_TILE_SIZE)
                d_buf = _dev_read(tile_count * GCSHeaders.D_TILE_SIZE)

                patch_fb(tile_count, (fb_base_x, fb_base_y), cv_buf, col_buf, d_buf)

    fin_time = time.perf_counter()
    print(f"gcs fin received! fragment time: {fin_time - start_time}s")

    if DEBUG_CPU_WIREFRAME:
        points = []
        for i, v in enumerate(vert_buf):
            points.append(((v[0] * .5 + .5) * fb_res[0], (v[1] * .5 + .5) * fb_res[1]))

        for i in range(len(vert_buf) // 3):
            pg.draw.lines(fb, (255, 200, 200), True, points[i * 3:i * 3 + 3])

    # save_fb()

# upload test texture

img_data = pg.image.load("test.png")
img_tex = pg.transform.scale(img_data, (64, 64))

tex_data = pg.image.tobytes(img_tex, 'RGBA')
tex_xfer = SCSHeaders.pack_ld_cbuf(64, tex_data)
_dev_write(tex_xfer)

# mini frame loop

while True: # for i in range(math.ceil(math.pi * 2 * 5 / (1 / 15))):
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
