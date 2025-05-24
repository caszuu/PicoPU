import usb.core
import usb.util

import struct
import math
import time
import argparse
import sys

# == init device ==

class DeviceState:
    def __init__(self) -> None:
        self.dev = usb.core.find(idVendor=0xcafe, idProduct=0x4000)

        if self.dev is None:
            if args.reboot_flash:
                sys.exit() # device might already be in BOOTSEL mode

            raise ValueError("no PicoPU device found!")

        self.dev.set_configuration()
        cfg = self.dev.get_active_configuration()
        itf = cfg[(0,0)]

        self.sync_int = itf[0]
        self.trans_up = itf[1]
        self.trans_down = itf[2]

        print(f"using device: {"picopu"} at bus: {self.dev.bus} addr: {self.dev.address}")

    def xfer_write(self, data: bytes) -> None:
        self.trans_up.write(data)

    def xfer_read(self, size: int) -> bytes:
        return self.trans_down.read(size, 5000).tobytes()

# == picopu structs ==

systick_avg = 0.
systick_count = 0
systick_min = 2 ** 24
systick_max = 0

class SIProto:
    # si packets
    
    LD_CBUF_INLINE = '<BxHI'
    @staticmethod
    def pack_ld_cbuf_inline(offset: int, cdata: bytes) -> bytes:
        return struct.pack(SIProto.LD_CBUF_INLINE, 4, len(cdata), offset) + cdata

    FLASH = '<B'
    @staticmethod
    def pack_flash() -> bytes:
        return struct.pack(SIProto.FLASH, 17)

    VBATCH = '<BBBxI'
    @staticmethod
    def pack_vertex_batch(v2f_idx: int, vertex_base: int, prim_count: int, vertex_data: bytes) -> bytes:
        return struct.pack(SIProto.VBATCH, 0, v2f_idx, prim_count, vertex_base) + vertex_data

    RBATCH = '<BB'
    @staticmethod
    def pack_raster_batch(v2f_idx: int) -> bytes:
        return struct.pack(SIProto.RBATCH, 1, v2f_idx)

    FLIP = '<B'
    @staticmethod
    def pack_flip() -> bytes:
        return struct.pack(SIProto.FLIP, 6)

    FINISHED = '<BB'
    DBG = '<B63s'

    @staticmethod
    def unpack(data: bytes):
        # if data[0:1] == int(16).to_bytes(1, 'little', signed=False):
        #     return struct.unpack(GCSHeaders.FEEDBACK, data)

        if data[0:1] == int(18).to_bytes(1, 'little', signed=False):
            return struct.unpack(SIProto.DBG, data[:64])

        elif data[0:1] == int(2).to_bytes(1, 'little', signed=False):
            p = struct.unpack(SIProto.FINISHED, data)

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
            raise ValueError("si packet of unknown type from su")
    
    # utils
    
    GCS_STATE = '<HHffffffB'
    @staticmethod
    def pack_gs(offset: tuple[int, int], extent: tuple[int, int]) -> bytes:
        viewport_transform = (extent[0] / 2, offset[0] + extent[0] / 2, extent[1] / 2, offset[1] + extent[1] / 2, 1, 0)

        return struct.pack(SIProto.GCS_STATE, *extent, *viewport_transform, 3)

# == pyhost modes ==

parser = argparse.ArgumentParser()
parser.add_argument("--flash", action="store_true", dest="reboot_flash", help="Reboots a PicoPU device into BOOTSEL mode to be flashed with firmware and exit.")

args = parser.parse_args()

if args.reboot_flash:
    dev = DeviceState()
    
    dev.xfer_write(SIProto.pack_flash())
    time.sleep(.5) # wait for firmware to reboot

    sys.exit()
