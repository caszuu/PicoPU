package main

import (
	"encoding/binary"
	"flag"
	"log"
	"time"

	"github.com/caszuu/PicoPu/pdrv"
	"github.com/go-gl/mathgl/mgl32"
	"github.com/google/gousb"
)

var (
	res_w = flag.Uint("w", 640, "framebuffer width resolution")
	res_h = flag.Uint("h", 480, "framebuffer height resolution")

	objPath   = flag.String("m", "cube.obj", "model file path")
	ledMode   = flag.Uint("lm", 1, "led mode (0 - off; 1 - rainbow; 2 - reactive)")
	ledBright = flag.Float64("lb", .2, "led brightness (0.0 to 1.0)")
)

var (
	rotX = float32(0)
	rotY = float32(0)
)

// a tiny test model viewer directly using pdrv

type gcsGstate struct {
	vbuf pdrv.VramAddr
	ibuf pdrv.VramAddr
	fbC0 pdrv.VramAddr
	fbZs pdrv.VramAddr

	fbExtent [2]uint16
	viewport [3][2]float32

	rasterMode byte
}

func allocFramebuffer(valloc *pdrv.VramHeapScope) (*pdrv.VramAlloc, pdrv.VramAddr, pdrv.VramAddr, error) {
	fbCSize := pdrv.VramSize(*res_w * *res_h * 2)
	fbZsSize := pdrv.VramSize(*res_w * *res_h * 2)

	log.Printf("wh: %d %d c0: %d c1: %d zs: %d", *res_w, *res_h, fbCSize, fbCSize, fbZsSize)

	fb, err := valloc.Alloc(fbCSize + fbCSize + fbZsSize)
	if err != nil {
		return nil, 0, 0, err
	}

	return fb, pdrv.VramAddr(fbCSize), pdrv.VramAddr(fbCSize + fbCSize), nil
}

func setupGstate(cb *pdrv.Cmdbuf, vbuf pdrv.VramAddr, ibuf pdrv.VramAddr, fb *pdrv.VramAlloc, zsOffset pdrv.VramAddr, cOffset pdrv.VramAddr) {
	buf := make([]byte, 128)

	offset := [2]uint{0, 0}
	extent := [2]uint{*res_w, *res_h}

	gstate := gcsGstate{
		vbuf: vbuf,
		ibuf: ibuf,
		fbC0: fb.Addr() + cOffset,
		fbZs: fb.Addr() + zsOffset,

		fbExtent:   [2]uint16{uint16(extent[0]), uint16(extent[1])},
		rasterMode: 3, // trig_fill

		viewport: [3][2]float32{
			{float32(extent[0]) / 2, float32(offset[0]) + float32(extent[0])/2},
			{float32(extent[1]) / 2, float32(offset[1]) + float32(extent[1])/2},
			{1, 0},
		},
	}

	gsSize, err := binary.Encode(buf, binary.LittleEndian, gstate)
	if err != nil {
		panic(err)
	}

	err = cb.CmdPushGstate(buf[:(gsSize+3)&^3], 0)
	if err != nil {
		log.Fatalln("failed to push gstate:", err)
	}
}

func updateUniforms(dev *pdrv.Device, unif *pdrv.VramAlloc) {
	// update cbuf data (following the demo_cbuf layout)

	mq := mgl32.AnglesToQuat(rotX, rotY, 0, mgl32.XYZ)

	m := mgl32.Scale3D(.5, .5, .5)
	m = m.Mul4(mq.Mat4())

	nm := mgl32.Ident4()

	buf := make([]byte, 0)
	buf, _ = binary.Append(buf, binary.LittleEndian, m)
	buf, _ = binary.Append(buf, binary.LittleEndian, nm)
	buf, _ = binary.Append(buf, binary.LittleEndian, mgl32.Vec4{-1, -1, -1}.Normalize())
	buf, _ = binary.Append(buf, binary.LittleEndian, mgl32.Vec4{.8, .2, .8})

	// stage cbuf data

	err := dev.SubmitXferToDevice(buf, unif, 0)
	if err != nil {
		log.Fatalln("failed to stage cbuf:", err)
	}
}

func main() {
	flag.Parse()
	log.SetFlags(log.Ltime | log.Lmicroseconds)

	// setup device

	usbCtx := gousb.NewContext()
	defer usbCtx.Close()

	dev, err := pdrv.InitDevice(usbCtx)
	if err != nil {
		log.Fatalln("failed to init device:", err)
	}
	defer dev.Destroy()

	switch *ledMode {
	case 0:
		break
	case 1:
		go patternRainbow(dev, float32(*ledBright))
	case 2:
		go patternReactive(dev, float32(*ledBright))
	default:
		log.Fatalln("invalid led mode")
	}

	valloc, err := dev.CreateHeapScope(0)
	if err != nil {
		log.Fatalln("failed to create a heap scope:", err)
	}

	// setup device bufs

	fb, c1Offset, zsOffset, err := allocFramebuffer(valloc)
	if err != nil {
		log.Fatalln("failed to alloc fb:", err)
	}
	defer fb.Free()

	m, err := loadModelFile(dev, valloc, *objPath)
	if err != nil {
		log.Fatalln("failed to load model:", err)
	}
	defer m.Destroy()

	unifSize := pdrv.VramSize((16*2 + 4*2) * 4)
	unif, err := valloc.Alloc(unifSize)
	if err != nil {
		log.Fatalln("failed to alloc uniform buf:", err)
	}
	defer unif.Free()

	// record frame cmdbuf

	cb := dev.NewCmdbuf()

	setupGstate(cb, m.VbufAddr(), m.IbufAddr(), fb, zsOffset, 0)
	cb.CmdWriteCbuf(unif, 0, pdrv.VramRange{Size: unifSize, Offset: 0})
	cb.CmdClear()

	m.Draw(cb)
	cb.CmdPresent()

	setupGstate(cb, m.VbufAddr(), m.IbufAddr(), fb, zsOffset, c1Offset)
	cb.CmdClear()

	m.Draw(cb)
	cb.CmdPresent()

	if err := cb.Finalize(valloc); err != nil {
		log.Fatalln("failed to finalize cmdbuf:", err)
	}
	defer cb.Destroy()

	// frame loop

	for {
		// update anim

		rotX += .05
		rotY += .05

		// submit frame

		updateUniforms(dev, unif)

		err = dev.SubmitEnqueue(cb)
		if err != nil {
			log.Fatalln("failed to submit frame:", err)
		}

		time.Sleep(time.Millisecond * 30)
	}
}
