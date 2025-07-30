package pdrv

import (
	"errors"
	"fmt"
)

// arch impl interfaces //

type DeviceArch interface {
	newEncoder(cb *Cmdbuf) CmdEncoder
}

type CmdEncoder interface {
	Finalize() error

	// fb ops //

	Clear() error
	Present() error

	// graphics //

	SetViewport(x int, y int, w int, h int) error

	BindFramebuffer(extent [2]int, color VramRes, zs VramRes) error
	BindVertexBuffer(vb VramRes, ib VramRes) error
	// BindTexture(data VramRes, sampler TextureView) error

	PushConstants(hbuf []byte, dstOffset VramSize) error
	WriteConstants(src VramRes, dstOffset VramSize) error

	Draw(first int, count int, draw DrawBits) error

	// xfers //

	XferToDevice(src *Staging, dst *VramAlloc, dstOffset VramSize) error
	XferToHost(dst *Staging, src *VramAlloc, srcOffset VramSize) error

	// sync //

	SignalFence(f *Fence) error
}

func (dev *Device) initArch() error {
	// match device hwinfo

	hwinfo, err := dev.CtlQueryHwInfo()
	if err != nil {
		return err
	}

	archStr := string(hwinfo.HwArch[:])

	switch archStr {
	case "mock\x00\x00\x00\x00":
		err = dev.initMockingArch()

	case "ravn\x00\x00\x00\x00":
		return errors.New("raven arch is not supported yet")

	default:
		return fmt.Errorf("unknown HwArch \"%s\"", archStr)
	}

	return err
}
