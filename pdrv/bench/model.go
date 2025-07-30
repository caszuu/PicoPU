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

	bufAlloc *pdrv.VramAlloc
	vbufRes  pdrv.VramRes
	ibufRes  pdrv.VramRes
}

func loadModelFile(dev *pdrv.Device, path string) (*VertexArray, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	reader := obj.NewStandardReader(f)
	o, err := reader.Read()
	if err != nil {
		return nil, err
	}

	// assemble vertex array

	varray := VertexArray{
		device: dev,

		vertexCount: 0,
		indexCount:  0,
	}

	// err = varray.assembleArray(o, corrupt)
	err = varray.assembleIndexed(o)

	if err != nil {
		return nil, err
	}

	// stage to device

	return &varray, nil
}

func (va *VertexArray) Destroy() {
	va.bufAlloc.Free()
}

func (va *VertexArray) Vbuf() pdrv.VramRes {
	return va.vbufRes
}

func (va *VertexArray) Ibuf() pdrv.VramRes {
	return va.ibufRes
}

func (va *VertexArray) Draw(enc pdrv.CmdEncoder) {
	enc.Draw(0, va.indexCount, pdrv.DrawIndexedBit)
}

func (va *VertexArray) assembleArray(o *obj.Object) error {
	vbuf := make([]byte, 0)

	var err error

	for _, f := range o.Faces {
		if len(f.Points) != 3 {
			return errors.New("non-triangulated mesh")
		}
		va.vertexCount += 3

		for _, p := range f.Points {
			vbuf, err = binary.Append(vbuf, binary.LittleEndian, []float32{
				float32(p.Vertex.X), float32(p.Vertex.Y), float32(p.Vertex.Z),
				float32(p.Normal.X), float32(p.Normal.Y), float32(p.Normal.Z),
				float32(p.Texture.U), float32(p.Texture.V),
			})

			if err != nil {
				panic(err)
			}
		}
	}

	// stage to device

	log.Printf("vc: %d ic: %d vbs: %d ibs: %d", va.vertexCount, va.indexCount, va.vertexCount*4*6, va.indexCount*2)

	va.bufAlloc, err = va.device.AllocVram(4*8*pdrv.VramSize(va.vertexCount), pdrv.HeapCapShaderOps|pdrv.HeapCapXferOps)
	if err != nil {
		return err
	}

	// if corrupt {
	// 	nullbuf := make([]byte, va.bufAlloc.Size())
	// 	va.device.SubmitXferToDevice(nullbuf, va.bufAlloc, 0)
	// }

	err = va.device.SubmitXferToDevice(vbuf, va.bufAlloc, 0)
	if err != nil {
		va.bufAlloc.Free()
		return err
	}

	va.vbufRes = pdrv.ResAll(va.bufAlloc)
	va.ibufRes = pdrv.VramRes{}

	return nil
}

func (va *VertexArray) assembleIndexed(o *obj.Object) error {
	vbuf := make([]byte, 0)
	ibuf := make([]byte, 0)

	vmap := make(map[[3]float32]uint16)

	for _, f := range o.Faces {
		if len(f.Points) != 3 {
			return errors.New("non-triangulated mesh")
		}

		for _, p := range f.Points {
			v := [3]float32{float32(p.Vertex.X), float32(p.Vertex.Y), float32(p.Vertex.Z)}
			i, ok := vmap[v]

			if !ok {
				i = uint16(va.vertexCount)
				va.vertexCount += 1

				vbuf, _ = binary.Append(vbuf, binary.LittleEndian, v)
				vmap[v] = i
			}

			va.indexCount += 1
			ibuf, _ = binary.Append(ibuf, binary.LittleEndian, i)
		}
	}

	// stage to device

	log.Printf("vc: %d ic: %d vbs: %d ibs: %d", va.vertexCount, va.indexCount, va.vertexCount*4*6, va.indexCount*2)

	var err error

	va.bufAlloc, err = va.device.AllocVram(4*3*pdrv.VramSize(va.vertexCount)+2*pdrv.VramSize(va.indexCount), pdrv.HeapCapShaderOps|pdrv.HeapCapXferOps)
	if err != nil {
		return err
	}

	err = va.device.SubmitXferToDevice(append(vbuf, ibuf...), va.bufAlloc, 0)
	if err != nil {
		va.bufAlloc.Free()
		return err
	}

	va.vbufRes, _ = pdrv.ResRange(va.bufAlloc, 4*3*pdrv.VramSize(va.vertexCount), 0)
	va.ibufRes, _ = pdrv.ResRange(va.bufAlloc, 2*pdrv.VramSize(va.indexCount), va.vbufRes.Size)

	return nil
}
