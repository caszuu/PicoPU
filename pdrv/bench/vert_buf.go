package main

import (
	"encoding/binary"
	"errors"
	"os"

	"github.com/go-gl/mathgl/mgl32"
	"github.com/sheenobu/go-obj/obj"
)

type vertexArray struct {
	vertCount int

	attrPos []mgl32.Vec3
	attrTex []mgl32.Vec2
}

func vertToVec3(v obj.Vertex) mgl32.Vec3 {
	return mgl32.Vec3{float32(v.X), float32(v.Y), float32(v.Z)}
}

func texToVec2(t obj.TextureCoord) mgl32.Vec2 {
	return mgl32.Vec2{float32(t.U), float32(t.V)}
}

func genTrigArray() vertexArray {
	pos := [3]mgl32.Vec3{
		{0, 0, .5},
		{0, 1, .5},
		{1, 1, .5},
	}

	va := vertexArray{
		vertCount: 3,
		attrPos:   make([]mgl32.Vec3, 3),
	}

	copy(va.attrPos, pos[:])
	return va
}

func loadVertexArray(path string) (vertexArray, error) {
	var va vertexArray

	f, err := os.Open(path)
	if err != nil {
		return vertexArray{}, err
	}
	defer f.Close()

	reader := obj.NewStandardReader(f)
	obj, err := reader.Read()
	if err != nil {
		return vertexArray{}, err
	}

	va.attrPos = make([]mgl32.Vec3, 0)
	va.attrTex = make([]mgl32.Vec2, 0)

	for _, face := range obj.Faces {
		if len(face.Points) != 3 {
			return vertexArray{}, errors.New("non-triangulated obj file")
		}

		for _, point := range face.Points {
			va.attrPos = append(va.attrPos, vertToVec3(*point.Vertex))
			va.attrTex = append(va.attrTex, texToVec2(*point.Texture))
		}
	}

	va.vertCount = len(obj.Faces) * 3

	return va, nil
}

func (va *vertexArray) assembleVertexStreams() [][]byte {
	const vstride = 4 * 3
	const batch_size = 24

	streams := make([][]byte, 0)

	var j int
	for i := 0; i < va.vertCount; i += batch_size {
		j += batch_size
		if j > va.vertCount {
			j = va.vertCount
		}

		vbuf := make([]byte, vstride*(j-i))

		for k, vert := range va.attrPos[i:j] {
			_, err := binary.Encode(vbuf[k*vstride:k*vstride+vstride], binary.LittleEndian, vert.Mul(.25))
			if err != nil {
				panic(err)
			}
		}

		streams = append(streams, vbuf)
	}

	return streams
}
