package pdrv

/* submit on-wire commands */

type scmdType uint8

const (
	scmdSignal scmdType = iota

	scmdXferToDevice
	scmdXferToHost

	scmdEnqueue
)

type signalScmd struct {
	Ctype scmdType

	_       uint8
	SyncIdx uint16
}

type xferFromDeviceScmd struct {
	Ctype scmdType

	_          uint8
	StagingIdx uint16

	XferSize uint32
	VAddr    VramAddr
}

type xferFromHostScmd struct {
	Ctype scmdType

	_          uint8
	StagingIdx uint16

	XferSize uint32
	VAddr    VramAddr
}

type enqueueScmd struct {
	Ctype scmdType

	_ uint8
	_ uint16

	CmdbufAddr VramAddr
}
