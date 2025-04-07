from typing import Callable
import usb.core
import usb.util

import struct
import math
import time
import argparse
import sys

print("initializing pyhost driver...")

parser = argparse.ArgumentParser()
parser.add_argument("--flash", action="store_true", dest="reboot_flash", help="Reboots a PicoPU device into BOOTSEL mode to be flashed with firmware and exit.")

args = parser.parse_args()

# == init device ==

dev = usb.core.find(idVendor=0xcafe, idProduct=0x4000)

if dev is None:
    if args.reboot_flash:
        sys.exit() # device might already be in BOOTSEL mode

    raise ValueError("no PicoPU device found!")

dev.set_configuration()
cfg = dev.get_active_configuration()
itf = cfg[(0,0)]

sync_int = itf[0]
trans_up = itf[1]
trans_down = itf[2]

def _dev_write(data: bytes) -> None:
    trans_up.write(data)

def _dev_read(size: int) -> bytes:
    return trans_down.read(size, 5000).tobytes()

print(f"using device: {"picopu"} at bus: {dev.bus} addr: {dev.address}")
print("driver init done!\n")

# == picopu structs ==

systick_avg = 0.
systick_count = 0
systick_min = 2 ** 24
systick_max = 0

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

    DISP_GCS = '<BBBB' # 3 padding bytes
    @staticmethod
    def pack_disp_gcs() -> bytes:
        return struct.pack(SCSHeaders.DISP_GCS, 4, 0, 0, 0)

    FLASH = '<B'
    @staticmethod
    def pack_flash() -> bytes:
        return struct.pack(SCSHeaders.FLASH, 5)

# class GCSHeaders:
#     VS = '<BBHI' # 1 padding short
#     @staticmethod
#     def pack_vs(base_vertex: int, prim_count: int, vert_data: bytes) -> bytes:
#         return struct.pack(GCSHeaders.VS, 0, prim_count, 0, base_vertex) + vert_data
# 
#     FS = '<BBHHHHH' # 1 padding short
#     @staticmethod
#     def pack_fs(shade_range: tuple[int, int, int, int], prim_count: int, clip_buf: bytes) -> bytes:
#         return struct.pack(GCSHeaders.FS, 1, prim_count, *shade_range, 0) + clip_buf
# 
#     COL_TILE_SIZE = 16 * 4
#     D_TILE_SIZE =  16 * 4
# 
#     PO = '<BBiiii'
#     @staticmethod
#     def unpack_po(data: bytes) -> tuple[int, tuple[int, int, int, int]]:
#         elems = struct.unpack(GCSHeaders.PO, data)
# 
#         return (elems[1], elems[2:6])
# 
#     FO = '<BBHH'
#     FO_INS = '<BBHHI'
#     @staticmethod
#     def unpack_fo(data: bytes) -> tuple[int, int, int, int]:
#         if not False:
#             _, tile_count, fb_base_x, fb_base_y = struct.unpack(GCSHeaders.FO, data)
#         else:
#             pass
#             _, tile_count, fb_base_x, fb_base_y, systick_sample = struct.unpack(GCSHeaders.FO_INS, data)
#             systick_sample = (2 ** 24) - systick_sample # systick is counting down from 0x00ffffff to 0x0
#         
#             global systick_avg, systick_count, systick_min, systick_max
#             systick_avg = (systick_avg * systick_count + systick_sample) / (systick_count + 1)
#             systick_min = min(systick_min, systick_sample)
#             systick_max = max(systick_max, systick_sample)
#             systick_count += 1
#             
#             print(systick_sample, systick_min, systick_max, systick_avg)
# 
#         to_read = math.ceil(tile_count / 2) + tile_count * GCSHeaders.COL_TILE_SIZE + tile_count * GCSHeaders.D_TILE_SIZE
#         return (tile_count, fb_base_x, fb_base_y, to_read)
# 
#     READY = '<B'
#     @staticmethod
#     def is_ready(data: bytes) -> bool:
#         return data[0:1] == int(16).to_bytes(1, 'little', signed=False)
# 
#     GCS_STATE = '<HHffffffB'
#     @staticmethod
#     def pack_gs(extent: tuple[int, int]) -> bytes:
#         offset = (0, 0)
#         viewport_transform = (extent[0] / 2, offset[0] + extent[0] / 2, extent[1] / 2, offset[1] + extent[1] / 2, 1, 0)
# 
#         return struct.pack(GCSHeaders.GCS_STATE, *extent, *viewport_transform, 3)

class GCSHeaders:
    ASSIGN = '<HBBI'
    @staticmethod
    def pack_assign(base_vertex:int, prim_count: int, vert_data: bytes) -> bytes:
        return struct.pack(GCSHeaders.ASSIGN, 0, 0, prim_count, base_vertex) + vert_data

    COL_TILE_SIZE = 16 * 4
    D_TILE_SIZE =  16 * 4

    FEEDBACK = '<HHIIIII'
    FINISHED = '<HHI'

    @staticmethod
    def unpack(data: bytes):
        if data[0:1] == int(16).to_bytes(1, 'little', signed=False):
            return struct.unpack(GCSHeaders.FEEDBACK, data)

        elif data[0:1] == int(17).to_bytes(1, 'little', signed=False):
            p = struct.unpack(GCSHeaders.FINISHED, data)
            p = (p[0], p[2]) # remove padding

            if False:
                systick_sample = (2 ** 24) - p[1] # systick is counting down from 0x00ffffff to 0x0
            
                global systick_avg, systick_count, systick_min, systick_max
                systick_avg = (systick_avg * systick_count + systick_sample) / (systick_count + 1)
                systick_min = min(systick_min, systick_sample)
                systick_max = max(systick_max, systick_sample)
                systick_count += 1
                
                print(systick_sample, systick_min, systick_max, systick_avg)
            
            return p
        
        else:
            raise ValueError("gcs packet of unknown type from su")

    GCS_STATE = '<HHffffffB'
    @staticmethod
    def pack_gs(extent: tuple[int, int]) -> bytes:
        offset = (0, 0)
        viewport_transform = (extent[0] / 2, offset[0] + extent[0] / 2, extent[1] / 2, offset[1] + extent[1] / 2, 1, 0)

        return struct.pack(GCSHeaders.GCS_STATE, *extent, *viewport_transform, 3)

# == pyhost cmd api ==

def submit_cmdbuf_blocking(cmdbuf: list[tuple[bytes, Callable]]) -> None:
    for cmd in cmdbuf:
        # submit cmd to device
        _dev_write(cmd[0])

        # call cmd host handler
        cmd[1]()

# == pyhost modes ==

if args.reboot_flash:
    trans_up.write(SCSHeaders.pack_flash())
    time.sleep(.5) # wait for firmware to reboot

    sys.exit()
