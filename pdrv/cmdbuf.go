package pdrv

import (
	"encoding/binary"
	"errors"
)

/* cmdbuf sub-system */

type CmdbufState int

const (
	CmdbufRecording CmdbufState = iota
	CmdbufReady
	CmdbufPending
)

type Cmdbuf struct {
	state  CmdbufState
	device *Device

	// host-side
	encBuf   []byte
	encCount int

	encPendingXfers map[uint8][]byte
	nextXferIdx     uint8

	// device-side
	bufAlloc *VramAlloc
}

func (dev *Device) NewCmdbuf() *Cmdbuf {
	return &Cmdbuf{
		state:  CmdbufRecording,
		device: dev,

		encBuf:   make([]byte, 0),
		encCount: 0,
		bufAlloc: nil,

		encPendingXfers: make(map[uint8][]byte),
		nextXferIdx:     0,
	}
}

func (cb *Cmdbuf) Destroy() error {
	if cb.state == CmdbufRecording {
		return nil // host-only, no need to dealloc
	}

	if cb.state == CmdbufPending {
		return errors.New("can't destroy, cmdbuf is pending")
	}

	cb.bufAlloc.Free()

	return nil
}

func (cb *Cmdbuf) DumpBuf() []byte {
	return cb.encBuf
}

func (cb *Cmdbuf) Finalize(valloc *VramHeapScope) error {
	// validate and finalize

	if cb.state != CmdbufRecording {
		return errors.New("cmdbuf is not in a recording state")
	}

	cb.appendCmd(uint32(0)) // mark end of cmdbuf

	// xfer

	var err error
	cb.bufAlloc, err = valloc.Alloc(VramSize(len(cb.encBuf)))
	if err != nil {
		return err
	}

	err = cb.device.SubmitXferToDevice(cb.encBuf, cb.bufAlloc, 0)
	if err != nil {
		return err
	}

	cb.state = CmdbufReady
	return nil
}

// cmd recording helpers

func (cb *Cmdbuf) ensureRec() error {
	if cb.state != CmdbufRecording {
		return errors.New("cmdbuf is not in a recording state")
	}

	return nil
}

func (cb *Cmdbuf) queueXfer(hbuf []byte) uint8 {
	idx := cb.nextXferIdx
	cb.nextXferIdx += 1

	cb.encPendingXfers[idx] = hbuf
	return idx
}

func (cb *Cmdbuf) appendCmd(cmd any) error {
	var err error
	cb.encBuf, err = binary.Append(cb.encBuf, binary.LittleEndian, cmd)

	// realign to 4 bytes
	cb.encBuf = append(cb.encBuf, []byte("\x00\x00\x00")[:((len(cb.encBuf)+3) & ^3)-len(cb.encBuf)]...)

	return err
}

func (cb *Cmdbuf) appendInlineData(hbuf []byte) error {
	if len(hbuf)%4 != 0 {
		return errors.New("inline data must be 4-byte aligned")
	}

	cb.encBuf = append(cb.encBuf, hbuf...)
	return nil
}
