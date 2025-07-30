package pdrv

import (
	"encoding/binary"
	"errors"
)

// cmdbuf sub-system //

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
	encBuf []byte

	// device-side
	bufAlloc *VramAlloc
}

func (dev *Device) NewCmdbuf() *Cmdbuf {
	return &Cmdbuf{
		state:  CmdbufRecording,
		device: dev,

		encBuf:   make([]byte, 0),
		bufAlloc: nil,
	}
}

func (cb *Cmdbuf) Encode() CmdEncoder {
	return cb.device.arch.newEncoder(cb)
}

func (cb *Cmdbuf) Destroy() {
	if cb.state == CmdbufRecording {
		return // host-only, no need to dealloc
	}

	// TODO: drv warn on in-use destroy
	// if cb.state == CmdbufPending {
	// 	fmt.Println("cmdbuf destroyed while in-use")
	// }

	cb.bufAlloc.Free()
}

// cmd encording helpers

func (cb *Cmdbuf) ensureRec() error {
	if cb.state != CmdbufRecording {
		return errors.New("cmdbuf is not in a recording state")
	}

	return nil
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

func (cb *Cmdbuf) xferFinalized() error {
	// validate

	if cb.state != CmdbufRecording {
		return errors.New("cmdbuf is not in a recording state")
	}

	// xfer

	var err error
	cb.bufAlloc, err = cb.device.AllocVram(VramSize(len(cb.encBuf)), HeapCapXferOps)
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
