package pdrv

/*
 * Go definitions of core-firmware si_proto.h structs
 */

type SiPacketType byte

const (
	SiTypeVBatch SiPacketType = iota
	SiTypeRBatch
	SiTypeFin
	SiTypeLoadCbuf
	SiTypeLoadCbufInline

	SiTypeFlip
)

const (
	SiTypeFlash = iota + 17
	SiTypeDbg
)

type SiVBatch struct {
	Ptype SiPacketType

	V2fIdx    byte
	PrimCount byte

	pad0 byte

	VertexBase uint32
}

type SiRBatch struct {
	Ptype SiPacketType

	V2fIdx byte
}

type SiWriteCbufInline struct {
	Ptype SiPacketType

	pad0 byte

	RangeSize   uint16
	RangeOffset uint32

	/* inline data follow on-wire */
}

type SiFlip struct {
	Ptype SiPacketType
}

type SiFlash struct {
	Ptype SiPacketType
}

type SiDbg struct {
	Ptype SiPacketType
	msg   [63]byte
}
