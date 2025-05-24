from hostd2 import SIProto, DeviceState

import os
import sys
import time
import math
import struct

import pygame as pg
# import pygame.freetype as freetype
# import pygame.gfxdraw as gfx

try:
    from pyglm import glm
except ImportError:
    import glm

# == debug window / visualiz ==

DEBUG_WINDOW = False
DEBUG_CPU_WIREFRAME = False

# FB_RES = (640, 380)
FB_RES = (320, 240)

if DEBUG_WINDOW:
    pg.init()

    fb = pg.display.set_mode(FB_RES, flags=pg.RESIZABLE, vsync=False)
    pg.display.set_caption("pdisp fb view")
else:
    fb = pg.Surface(FB_RES)

dev: DeviceState

# == pdisp device bufs ==

named_cmdbufs: dict[str, list] = {"def": []}
active_cmdbuf: str = "def"

named_vert_arrays: dict[str, list] = {}

# == pdisp utils ==

def print_cmds(cmdbuf: list) -> None:
    print(f"{active_cmdbuf} buf commands:")

    for cmd in cmdbuf:
        print(f"  {cmdbuf}")

def load_wavefront_model(varray: str, path: str) -> None:
    # parse obj file

    vbuf = []
    tbuf = []
    fbuf = []
    
    with open(path, 'r') as f:
        for l in f.readlines():
            if l.startswith("v "):
                # parse vertex
                l = l[2:].strip()
                axis = l.split(" ")

                vbuf.append(glm.vec3(float(axis[0]), float(axis[1]), float(axis[2])))

            elif l.startswith("vt "):
                l = l[3:].strip()
                coords = l.split(" ")

                tbuf.append(glm.vec2(float(coords[0]), float(coords[1])))

            elif l.startswith("f "):
                # parse face
                l = l[2:].strip()
                indices = l.split(" ")

                if indices[0].split("/")[1] == "":
                    print("error: obj does not contain texture coordinates")
                    return

                fbuf.append((int(indices[0].split("/")[0]) - 1, int(indices[1].split("/")[0]) - 1, int(indices[2].split("/")[0]) - 1,
                             int(indices[0].split("/")[1]) - 1, int(indices[1].split("/")[1]) - 1, int(indices[2].split("/")[1]) - 1))

    # gen a vert array

    vert_array = []

    for f in fbuf:
        vert_array.append((vbuf[f[0]], tbuf[f[3]]))
        vert_array.append((vbuf[f[1]], tbuf[f[4]]))
        vert_array.append((vbuf[f[2]], tbuf[f[5]]))

    named_vert_arrays[varray] = vert_array
    print(f"loaded \"{path}\"; verts: {len(vbuf)} faces: {len(fbuf)}")

def cpu_vert_stage(vert_array: list[glm.vec3]) -> list[bytes]:
    # rotate vertices on cpu side (for now) to animate the verts
    clip_space_verts = []
    for v in vert_array:
        clip_space_verts.append(glm.rotateY(glm.rotateX(v[0] * glm.vec3(.2), glm.radians(145)), time.perf_counter() / 5))
        # clip_space_verts.append(v[1])

    return clip_space_verts

def submit_raw(cmd: tuple) -> None:
    dev.xfer_write(cmd[0])

def submit_gcs_batch(cmd: tuple) -> None:
    v_data = cpu_vert_stage(cmd[0])
    v_streams = [v_data[i:i + 30] for i in range(0, len(v_data), 30)]

    for i, vs in enumerate(v_streams):
        cmd_data = SIProto.pack_vertex_batch(0, 0, len(vs) // 3, b''.join(vs))

        # assign vbatch
        dev.xfer_write(cmd_data)

        # await batch finish packet
        while True:
            p_bytes = dev.xfer_read(65535)
            p = SIProto.unpack(p_bytes)

            if p[0] == 2: # finished
                break

            if p[0] == 18:
                print(p)

            else:
                print("unknown si", p)

        cmd_data = SIProto.pack_raster_batch(0)

        # assign rbatch
        dev.xfer_write(cmd_data)

        # await batch fragments and finished packets
        while True:
            p_bytes = dev.xfer_read(65535)
            p = SIProto.unpack(p_bytes)

            if p[0] == 2: # finished
                break

            elif p[0] == 18:
                print(p)

            else:
                print("unknown si", p)

    # if DEBUG_CPU_WIREFRAME:
    #     points = []
    #     for i, v in enumerate(vert_buf):
    #         points.append(((v[0] * .5 + .5) * fb_res[0], (v[1] * .5 + .5) * fb_res[1]))

    #     for i in range(len(vert_buf) // 3):
    #         pg.draw.lines(fb, (255, 200, 200), True, points[i * 3:i * 3 + 3])   

# == pdisp script cmds ==

def sel_cmdbuf(args: list[str]) -> None:
    global active_cmdbuf
    assert(len(args) == 1)

    active_cmdbuf = args[0]
    named_cmdbufs.setdefault(args[0], [])

def print_cmdbuf(args: list[str]) -> None:
    assert(len(args) <= 1)

    cmdbuf = args[0] if args else active_cmdbuf
    print_cmds(named_cmdbufs[cmdbuf])

def clear_cmdbuf(args: list[str]) -> None:
    assert(len(args) <= 1)

    cmdbuf = args[0] if args else active_cmdbuf
    named_cmdbufs[cmdbuf].clear()

def exec_cmdbuf(args: list[str]) -> None:
    assert(len(args) <= 1)
    cmdbuf = named_cmdbufs[args[0] if args else active_cmdbuf]

    if not cmdbuf:
        print("error: no cmds to execute")
        return

    start_time = time.perf_counter()
    for cmd in cmdbuf:
        cb = cmd[0]
        cmd_args = cmd[1:]

        cb(cmd_args)
    
    fin_time = time.perf_counter()
    print(f"cmdbuf finished, exec time: {fin_time - start_time}s")

def cmd_ld_img(args: list[str]) -> None:
    path = args[0]
    x = int(args[1])
    y = int(args[2])

    img = pg.image.load(args[0])
    img = pg.transform.scale(img, (x, y))

    img_data = pg.image.tobytes(img, 'RGBA')
    packets = [img_data[i:i + 512] for i in range(0, len(img_data), 512)]

    for i, d in enumerate(packets):
        cmd_data = SIProto.pack_ld_cbuf_inline(64 + i * 512, d)
        named_cmdbufs[active_cmdbuf].append((submit_raw, cmd_data))

def cmd_ld_gstate(args: list[str]) -> None:
    if args:
        offset = int(args[0]), int(args[1])
    else:
        offset = (0, 0)

    if len(args) > 2:
        extent = int(args[2]), int(args[3])
    else:
        extent = FB_RES

    rast_mode = 3 # rast-only for now int(args[4])
    cmd_data = SIProto.pack_ld_cbuf_inline(0, SIProto.pack_gs(offset, extent))

    named_cmdbufs[active_cmdbuf].append((submit_raw, cmd_data))

    # temp. perspective load
    m = glm.perspective(45., 4/3, .1, 10.)

    cmd_data = SIProto.pack_ld_cbuf_inline(16 * 1024, bytes(m))
    named_cmdbufs[active_cmdbuf].append((submit_raw, cmd_data))

def cmd_gcs_assign(args: list[str]) -> None:
    varray = named_vert_arrays[args[0]]
    named_cmdbufs[active_cmdbuf].append((submit_gcs_batch, varray))

def ld_obj_file(args: list[str]) -> None:
    load_wavefront_model(args[0], args[1])

    # v_data = b''.join(cpu_vert_stage(named_vert_arrays[args[0]]))
    # packets = [v_data[i:i + 512] for i in range(0, len(v_data), 512)]

    # for i, d in enumerate(packets):
    #     cmd_data = SIProto.pack_ld_cbuf_inline((16 * 1024) + i * 512, d)
    #     named_cmdbufs[active_cmdbuf].append((submit_raw, cmd_data))

def cmd_flip(args: list[str]) -> None:
    cmd_data = SIProto.pack_flip()
    named_cmdbufs[active_cmdbuf].append((submit_raw, cmd_data))

# == pdisp top level driver ==

def exec_pdisp_cmd(usr_cmd: str) -> None:
    cmd = usr_cmd.split(" ")
    for i in range(len(cmd)):
        cmd[i] = cmd[i].strip()

    args = cmd[1:]

    if cmd[0] == "" or cmd[0][0] == "#":
        return
        
    elif cmd[0] == "exit":
        sys.exit()

    elif cmd[0].startswith("."):
        path = cmd[0][1:]

        with open(path, 'r') as f:
            exec_pdisp_script(f.readlines())

    # cmdbuf commands

    elif cmd[0] == "sel":
        sel_cmdbuf(args)

    elif cmd[0] == "print":
        print_cmdbuf(args)

    elif cmd[0] == "clear":
        clear_cmdbuf(args)

    elif cmd[0] == "exec":
        exec_cmdbuf(args)

    elif cmd[0] == "exec_loop":
        while True:
            exec_cmdbuf(args)

    # device commands

    elif cmd[0] == "ld_img":
        cmd_ld_img(args)

    elif cmd[0] == "ld_gstate":
        cmd_ld_gstate(args)
        
    elif cmd[0] == "ld_obj":
        ld_obj_file(args)
        
    elif cmd[0] == "vbatch":
        cmd_gcs_assign(args)

    elif cmd[0] == "flip":
        cmd_flip(args)

    else:
        print("error: unknown command")

def pdisp_shell() -> None:
    while True:
        cmd = input(f"{active_cmdbuf} > ")
        exec_pdisp_cmd(cmd)

def exec_pdisp_script(cmds: list[str]) -> None:
    for cmd in cmds:
        exec_pdisp_cmd(cmd)

if __name__ == "__main__":
    dev = DeviceState()

    exec_pdisp_cmd(".gtest.ds")
    pdisp_shell()
