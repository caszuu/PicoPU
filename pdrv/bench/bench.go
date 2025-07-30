package main

import (
	"encoding/binary"
	"flag"
	"fmt"
	"log"
	"time"

	"github.com/caszuu/PicoPu/pdrv"
	"github.com/go-gl/mathgl/mgl32"
)

var (
	res_w = flag.Uint("w", 320, "framebuffer width resolution")
	res_h = flag.Uint("h", 240, "framebuffer height resolution")

	objPath   = flag.String("m", "cube.obj", "model file path")
	texPath   = flag.String("t", "cube.png", "image file path")
	ledMode   = flag.Uint("lm", 1, "led mode (0 - off; 1 - rainbow; 2 - audio reactive)")
	ledBright = flag.Float64("lb", .1, "led brightness (0.0 to 1.0)")
)

var (
	rotX = float32(0)
	rotY = float32(0)
)

// a tiny test model viewer directly using pdrv

func allocFramebuffer(dev *pdrv.Device) (*pdrv.VramAlloc, pdrv.VramRes, pdrv.VramRes, pdrv.VramRes, error) {
	fbCSize := pdrv.VramSize(*res_w * *res_h * 2)
	fbZsSize := pdrv.VramSize(*res_w * *res_h * 2)

	log.Printf("wh: %d %d c0: %d c1: %d zs: %d", *res_w, *res_h, fbCSize, fbCSize, fbZsSize)

	fb, err := dev.AllocVram(fbCSize+fbCSize+fbZsSize, pdrv.HeapCapFbOps)
	if err != nil {
		return nil, pdrv.VramRes{}, pdrv.VramRes{}, pdrv.VramRes{}, err
	}

	fbC0, _ := pdrv.ResRange(fb, fbCSize, 0)
	fbC1, _ := pdrv.ResRange(fb, fbCSize, fbCSize)
	fbZs, _ := pdrv.ResRange(fb, fbZsSize, fbCSize*2)

	return fb, fbC0, fbC1, fbZs, nil
}

func updateUniforms(dev *pdrv.Device, unif *pdrv.VramAlloc, tex *Texture) {
	// update cbuf data (following the demo_cbuf layout)

	mq := mgl32.AnglesToQuat(rotX, rotY, 0, mgl32.XYZ)

	m := mgl32.Scale3D(.5, .5, .5)
	m = m.Mul4(mq.Mat4())

	nm := mq.Mat4()

	buf := make([]byte, 0)
	buf, _ = binary.Append(buf, binary.LittleEndian, m)
	buf, _ = binary.Append(buf, binary.LittleEndian, nm)
	buf, _ = binary.Append(buf, binary.LittleEndian, mgl32.Vec4{-1, -1.5, -.5}.Normalize())

	buf, err := binary.Append(buf, binary.LittleEndian, tex.Extent())
	buf, _ = binary.Append(buf, binary.LittleEndian, tex.Addr())

	if err != nil {
		panic(err)
	}

	// stage cbuf data

	err = dev.SubmitXferToDevice(buf, unif, 0)
	if err != nil {
		log.Fatalln("failed to stage cbuf:", err)
	}
}

func main() {
	flag.Parse()
	log.SetFlags(log.Ltime | log.Lmicroseconds)

	// setup device

	dev, err := pdrv.InitDevice()
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

	// setup device bufs

	fb, c0Res, c1Res, zsRes, err := allocFramebuffer(dev)
	if err != nil {
		log.Fatalln("failed to alloc fb:", err)
	}
	defer fb.Free()

	m, err := loadModelFile(dev, *objPath)
	if err != nil {
		log.Fatalln("failed to load model:", err)
	}
	defer m.Destroy()

	tex, err := loadTexture(dev, *texPath)
	if err != nil {
		log.Fatalln("failed to load texture:", err)
	}
	defer tex.Destroy()

	unifSize := pdrv.VramSize((16*2 + 4*2 + 2 + 1) * 4)
	unif, err := dev.AllocVram(unifSize, pdrv.HeapCapShaderOps|pdrv.HeapCapXferOps)
	if err != nil {
		log.Fatalln("failed to alloc uniform buf:", err)
	}
	defer unif.Free()
	unifRes := pdrv.ResAll(unif)

	a, err := dev.AllocVram(8*1024, pdrv.HeapCapShaderOps|pdrv.HeapCapXferOps)
	if err != nil {
		log.Fatalln("nope:", err)
	}
	defer a.Free()

	// record frame cmdbuf

	f, err := dev.NewFence()
	if err != nil {
		log.Fatalln("failed to alloc frame fence:", err)
	}
	defer f.Destroy()

	// frame loop

	frameIndex := 0

	for {
		// update anim

		rotX += .0125 * 2
		rotY += .025 * 2

		updateUniforms(dev, unif, tex)

		// encode frame

		cb := dev.NewCmdbuf()

		{
			enc := cb.Encode()

			var cRes pdrv.VramRes
			if frameIndex%2 == 0 {
				cRes = c0Res
			} else {
				cRes = c1Res
			}

			enc.SetViewport(0, 0, 320, 240)
			enc.BindFramebuffer([2]int{320, 240}, cRes, zsRes)
			enc.BindVertexBuffer(m.Vbuf(), m.Ibuf())

			enc.Clear()
			enc.WriteConstants(unifRes, 0)

			m.Draw(enc)
			enc.Present()

			enc.SignalFence(f)

			if err := enc.Finalize(); err != nil {
				log.Fatalln("failed to finalize cmdbuf:", err)
			}
		}

		// submit frame

		startTime := time.Now().UnixMicro()

		err = dev.SubmitEnqueue(cb)
		if err != nil {
			log.Fatalln("failed to submit frame:", err)
		}

		f.Wait()
		f.Reset()

		cb.Destroy()

		ft := time.Now().UnixMicro() - startTime
		fmt.Printf("  frame time: %d.%dms \r", ft/1000, ft%1000)

		frameIndex += 1
	}
}
