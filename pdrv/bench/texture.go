package main

import (
	"image"
	"image/draw"
	_ "image/png"
	"log"
	"os"

	"github.com/caszuu/PicoPu/pdrv"
)

type Texture struct {
	device *pdrv.Device

	texExtent [2]uint32
	texAlloc  *pdrv.VramAlloc
}

func loadTexture(dev *pdrv.Device, path string) (*Texture, error) {
	// load from disk

	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}

	img, _, err := image.Decode(f)
	if err != nil {
		return nil, err
	}

	htex := image.NewRGBA(img.Bounds())
	draw.Draw(htex, htex.Bounds(), img, image.Point{0, 0}, draw.Src)

	log.Printf("twh: %d %d ts: %d", htex.Rect.Dx(), htex.Rect.Dy(), len(htex.Pix))

	// stage to device

	tex, err := dev.AllocVram(pdrv.VramSize(len(htex.Pix)), pdrv.HeapCapShaderOps|pdrv.HeapCapXferOps)
	if err != nil {
		return nil, err
	}

	err = dev.SubmitXferToDevice(htex.Pix, tex, 0)
	if err != nil {
		tex.Free()
		return nil, err
	}

	return &Texture{
		device: dev,

		texExtent: [2]uint32{uint32(htex.Bounds().Dx()), uint32(htex.Bounds().Dy())},
		texAlloc:  tex,
	}, nil
}

func (t *Texture) Destroy() {
	t.texAlloc.Free()
}

func (t *Texture) Extent() [2]uint32 {
	return t.texExtent
}

func (t *Texture) Addr() pdrv.VramAddr {
	return t.texAlloc.Addr()
}
