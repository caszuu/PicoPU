package main

import (
	"encoding/binary"
	"flag"
	"log"

	"github.com/caszuu/PicoPu/pdrv"
	"github.com/google/gousb"
)

var (
	res_w = flag.Uint("w", 640, "framebuffer width resolution")
	res_h = flag.Uint("h", 480, "framebuffer height resolution")
)

// a small cli utility to test out and benchmark
// the pdrv and matching pico-pu firmware

type gcsGstate struct {
	fbExtent [2]uint16
	viewport [3][2]float32

	rasterMode byte
}

func packAndQueueCmd(dev *pdrv.Device, p any) int {
	buf := make([]byte, 1024)

	pSize, _ := binary.Encode(buf, binary.LittleEndian, p)
	err := dev.QueueOutXfer(buf[:pSize])
	if err != nil {
		log.Fatalln("failed to queue cmd:", err)
	}

	return pSize
}

func packAndQueueCmdWithTail(dev *pdrv.Device, p any, tail []byte) int {
	buf := make([]byte, 1024)

	pSize, _ := binary.Encode(buf, binary.LittleEndian, p)
	copy(buf[pSize:], tail)

	err := dev.QueueOutXfer(buf[:pSize+len(tail)])
	if err != nil {
		log.Fatalln("failed to queue tcmd:", err)
	}

	return pSize + len(tail)
}

func setupGcsGstate(dev *pdrv.Device) {
	buf := make([]byte, 1024)

	offset := [2]uint{0, 0}
	extent := [2]uint{*res_w, *res_h}

	gstate := gcsGstate{
		fbExtent:   [2]uint16{uint16(extent[0]), uint16(extent[1])},
		rasterMode: 3, // trig_fill

		viewport: [3][2]float32{
			{float32(extent[0]) / 2, float32(offset[0]) + float32(extent[0])/2},
			{float32(extent[1]) / 2, float32(offset[1]) + float32(extent[1])/2},
			{1, 0},
		},
	}

	p := pdrv.SiWriteCbufInline{
		Ptype:       pdrv.SiTypeLoadCbufInline,
		RangeSize:   32,
		RangeOffset: 0,
	}
	siSize, _ := binary.Encode(buf, binary.LittleEndian, p)
	gsSize, _ := binary.Encode(buf[siSize:], binary.LittleEndian, gstate)

	err := dev.QueueOutXfer(buf[:siSize+gsSize])
	if err != nil {
		log.Fatalln("failed to queue gstate:", err)
	}
}

// pdrv list
//  - dev_ctl api - done
//  - vram/xfer subsys
//    - immediate alloc/mgr
//    - cmdbuf deferred ops
//  - cmdbuf subsys

func main() {
	flag.Parse()
	log.SetFlags(log.Ltime | log.Lmicroseconds)

	ctx := gousb.NewContext()
	defer ctx.Close()

	dev, err := pdrv.InitDevice(ctx)
	if err != nil {
		log.Fatalln("failed to init device:", err)
	}
	defer dev.Destroy()

	// init benchmark

	setupGcsGstate(dev)

	// varray := genTrigArray()
	varray, err := loadVertexArray("teapot.obj")
	if err != nil {
		log.Fatalln("failed to load varray:", err)
	}

	log.Printf("varray: %d", varray.vertCount)

	// frame loop

	pBuf := make([]byte, 1024)

	for {
		// vbatch dispatch

		vstreams := varray.assembleVertexStreams()

		for _, vs := range vstreams {
			vb := pdrv.SiVBatch{
				Ptype:      pdrv.SiTypeVBatch,
				V2fIdx:     0,
				PrimCount:  byte(len(vs) / 4 / 3 / 3),
				VertexBase: 0,
			}
			packAndQueueCmdWithTail(dev, vb, vs)

			for {
				_, err := dev.ReadInXfer(pBuf)
				if err != nil {
					log.Fatalln("error at vbatch:", err)
				}

				var pVal uint8
				_, err = binary.Decode(pBuf, binary.LittleEndian, &pVal)
				if err != nil {
					log.Fatalln("dec err:", err)
				}

				pType := pdrv.SiPacketType(pVal)

				if pType == pdrv.SiTypeFin {
					break
				} else if pType == pdrv.SiTypeDbg {
					log.Println("vdbg:", string(pBuf[1:64]))
				} else {
					log.Println("unknown vbatch si pType:", pType)
				}
			}

			// vbatch dispatch

			fb := pdrv.SiRBatch{
				Ptype:  pdrv.SiTypeRBatch,
				V2fIdx: 0,
			}
			packAndQueueCmd(dev, fb)

			for {
				_, err := dev.ReadInXfer(pBuf)
				if err != nil {
					log.Fatalln("error in rbatch:", err)
				}

				var pVal uint8
				_, err = binary.Decode(pBuf, binary.LittleEndian, &pVal)
				if err != nil {
					log.Fatalln("dec err:", err)
				}

				pType := pdrv.SiPacketType(pVal)

				if pType == pdrv.SiTypeFin {
					break
				} else if pType == pdrv.SiTypeDbg {
					log.Println("rdbg:", string(pBuf[1:64]))
				} else {
					log.Println("unknown rbatch si pType:", pType)
				}
			}
		}

		p := pdrv.SiFlip{
			Ptype: pdrv.SiTypeFlip,
		}
		packAndQueueCmd(dev, p)

		log.Println("frame fin")
	}
}
