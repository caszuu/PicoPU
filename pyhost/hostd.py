import usb.core
import usb.util

import struct
import math
import time
import random

print("initializing pyhost driver...")

# == init device ==

dev = usb.core.find(idVendor=0xcafe, idProduct=0x4000)

if dev is None:
    raise ValueError("no PicoPU device found!")

dev.set_configuration()
cfg = dev.get_active_configuration()
itf = cfg[(0,0)]

sync_int = itf[0]
trans_up = itf[1]
trans_down = itf[2]

print(f"using device: {"picopu"} at bus: {dev.bus} addr: {dev.address}")
print("driver init done!\n")

# == picopu structs ==

class HostbusHeaders:
    # struct formats for packing hostbus packet headers

    XFER = '<BII'
    @staticmethod
    def pack_xfer(dev_addr: int, data: bytes) -> bytes:
        return struct.pack(HostbusHeaders.XFER, 1, len(data), dev_addr) + data

    SCS_PROC = '<B'
    @staticmethod
    def pack_sps_proc() -> bytes:
        return struct.pack(HostbusHeaders.SCS_PROC, 2)

class SCSHeaders:
    LD_CBUF = '<BHH'
    @staticmethod
    def pack_ld_cbuf(offset: int, cdata: bytes) -> bytes:
        return struct.pack(SCSHeaders.LD_CBUF, 1, offset, len(cdata)) + cdata

    LD_BIN = '<BHH'
    @staticmethod
    def pack_ld_bin(offset: int, prog_bin: bytes) -> bytes:
        return struct.pack(SCSHeaders.LD_BIN, 2, offset, len(prog_bin)) + prog_bin

    DISP_BIN = '<BH'
    @staticmethod
    def pack_disp_bin(entry_offset: int) -> bytes:
        return struct.pack(SCSHeaders.DISP_BIN, 3, entry_offset)

class GCSHeaders:
    FS = '<BHHHHB'
    @staticmethod
    def pack_fs(shade_range: tuple[int, int, int, int], prims: list[bytes]) -> bytes:
        return struct.pack(GCSHeaders.FS, 2, *shade_range, len(prims) // 3) + b''.join(prims)

    COL_TILE_SIZE = 4 * 4
    D_TILE_SIZE = 4 * 4

    FO = '<BBI'
    @staticmethod
    def unpack_fo(data: bytes) -> tuple[int, int, int]:
        _, tile_count, fb_base_index = struct.unpack(GCSHeaders.FO, data)
        to_read = math.ceil(tile_count / 2) + tile_count * GCSHeaders.COL_TILE_SIZE + tile_count * GCSHeaders.D_TILE_SIZE

        return (tile_count, fb_base_index, to_read)

    @staticmethod
    def unpack_fo_bufs(data: bytes) -> tuple[bytes, bytes, bytes]:
        cv_buf = data[:math.ceil(tile_count / 2)]
        col_buf = data[len(cv_buf):tile_count * GCSHeaders.COL_TILE_SIZE]
        d_buf = data[len(cv_buf) + len(col_buf):]

        return (cv_buf, col_buf, d_buf)

    READY = '<B'
    @staticmethod
    def is_ready(data: bytes) -> bool:
        return data[0:1] == int(16).to_bytes(1, 'little', signed=False)

# == test fb preview ==

FB_RES = 128

import pygame as pg

pg.init()
fb = pg.display.set_mode((FB_RES, FB_RES), vsync=False)
# fb = pg.Surface((FB_RES, FB_RES))

def patch_fb(tile_count: int, fb_base_index: int, cv_buf: bytes, col_data: bytes) -> None:
    for i in range(tile_count):
        col_tile = col_data[i * GCSHeaders.COL_TILE_SIZE:(i + 1) * GCSHeaders.COL_TILE_SIZE]
        x, y = (((fb_base_index + i*4) // 4 * 2) % FB_RES, ((fb_base_index + i*4) // 4 * 2) // FB_RES * 2)

        cv = cv_buf[i // 2] >> (i % 2 * 4)

        if cv & 1:
            fb.set_at((x, y), col_tile[0:3])
        if cv & 2:
            fb.set_at((x + 1, y), col_tile[4:7])
        if cv & 4:
            fb.set_at((x, y + 1), col_tile[8:11])
        if cv & 8:
            fb.set_at((x + 1, y + 1), col_tile[12:15])

    pg.display.update()

i = 0
def save_fb() -> None:
    global i
    pg.image.save(fb, f"fbs/fb_col{i}.png")
    i += 1

# == test driver ==

clip_fmt = '<iif'

def dispatch_frame():
    prims_buf = []

    for i in range(16):
        test_prims = [
            struct.pack(clip_fmt, random.randint(0, FB_RES - 1), random.randint(0, FB_RES // 2), 0),
            struct.pack(clip_fmt, random.randint(0, FB_RES // 2), random.randint(FB_RES // 2, FB_RES - 1), 0),
            struct.pack(clip_fmt, random.randint(FB_RES // 2, FB_RES - 1), random.randint(FB_RES // 2, FB_RES - 1), float('inf')),
        ]
        prims_buf.extend(test_prims)
    
    # dispatch a test gcs fragment stream
    d = HostbusHeaders.pack_xfer(0x00, GCSHeaders.pack_fs((0, 0, FB_RES - 1, FB_RES - 1), prims_buf))

    start_time = time.perf_counter()

    # trans_up.write(d[0:1])
    trans_up.write(d[0:1])
    trans_up.write(d[1:9])
    trans_up.write(d[9:])

    trans_up.write(HostbusHeaders.pack_sps_proc())

    print(trans_down.read(13, 5000).tobytes().hex())
    # print(trans_down.read(8, 5000).tobytes().hex())

    # await fragment output streams and ready gcs cmds
    while True:
        p_head = trans_down.read(6, 5000).tobytes()

        if GCSHeaders.is_ready(p_head):
            break

        # read frag output stream
        tile_count, fb_base_index, to_read = GCSHeaders.unpack_fo(p_head)

        cv_buf = trans_down.read(math.ceil(tile_count / 2))
        col_buf = trans_down.read(tile_count * GCSHeaders.COL_TILE_SIZE)
        d_buf = trans_down.read(tile_count * GCSHeaders.D_TILE_SIZE)

        patch_fb(tile_count, fb_base_index, cv_buf, col_buf)

    fin_time = time.perf_counter()
    print(f"gcs ready received! frame time: {fin_time - start_time}s")

    # save_fb()

while True:
    # fb.fill((0, 0, 0))
    dispatch_frame()

    pg.display.update()