package main

import (
	"encoding/binary"
	"errors"
	"log"
	"os"

	"github.com/caszuu/PicoPu/pdrv"
	"github.com/sheenobu/go-obj/obj"
)

type VertexArray struct {
	device *pdrv.Device

	vertexCount int
	indexCount  int

	vertAlloc  *pdrv.VramAlloc
	indexAlloc *pdrv.VramAlloc
}

func loadModelFile(dev *pdrv.Device, valloc *pdrv.VramHeapScope, path string) (*VertexArray, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	reader := obj.NewStandardReader(f)
	obj, err := reader.Read()
	if err != nil {
		return nil, err
	}

	// assemble vertex array

	varray := VertexArray{
		device: dev,

		vertexCount: 0,
		indexCount:  0,
	}

	vbuf := make([]byte, 0)
	// vlookup := make(map[int]int, 0)

	for _, f := range obj.Faces {
		if len(f.Points) != 3 {
			return nil, errors.New("non-triangulated mesh")
		}
		varray.vertexCount += 3

		for _, p := range f.Points {
			vbuf, err = binary.Append(vbuf, binary.LittleEndian, []float32{
				float32(p.Vertex.X), float32(p.Vertex.Y), float32(p.Vertex.Z),
				float32(p.Normal.X), float32(p.Normal.Y), float32(p.Normal.Z),
			})

			if err != nil {
				panic(err)
			}
		}
	}

	// stage to device

	log.Printf("vc: %d ic: %d vbs: %d ibs: %d", varray.vertexCount, varray.indexCount, varray.vertexCount*4*6, varray.indexCount*2)

	varray.vertAlloc, err = valloc.Alloc(4 * 6 * pdrv.VramSize(varray.vertexCount))
	if err != nil {
		return nil, err
	}

	err = dev.SubmitXferToDevice(vbuf, varray.vertAlloc, 0)
	if err != nil {
		varray.vertAlloc.Free()
		return nil, err
	}

	return &varray, nil
}

func (va *VertexArray) Destroy() {
	if va.indexAlloc != nil {
		va.indexAlloc.Free()
	}

	va.vertAlloc.Free()
}

func (va *VertexArray) VbufAddr() pdrv.VramAddr {
	return va.vertAlloc.Addr()
}

func (va *VertexArray) IbufAddr() pdrv.VramAddr {
	if va.indexAlloc == nil {
		return 0
	}

	return va.indexAlloc.Addr()
}

func (va *VertexArray) Draw(cb *pdrv.Cmdbuf) {
	cb.CmdDraw(0, va.vertexCount/3, 0)
}
