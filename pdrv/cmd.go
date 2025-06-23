package pdrv

import (
	"errors"
)

/* device command encodings */

type cmdType uint8

const (
	cmdEnd cmdType = iota

	cmdClear
	cmdPresent

	cmdXferToDevice
	cmdXferToHost

	cmdWriteCbuf
	cmdPushCbuf
	cmdPushGstate
	cmdDraw
)

// fb cmds //

type clearCmd struct {
	ctype     cmdType
	clearBits uint8
}

type presentCmd struct {
	ctype cmdType
}

func (cb *Cmdbuf) CmdClear() error {
	if err := cb.ensureRec(); err != nil {
		return err
	}

	cmd := clearCmd{
		ctype:     cmdClear,
		clearBits: 0,
	}

	return cb.appendCmd(cmd)
}

func (cb *Cmdbuf) CmdPresent() error {
	if err := cb.ensureRec(); err != nil {
		return err
	}

	cmd := presentCmd{
		ctype: cmdPresent,
	}

	return cb.appendCmd(cmd)
}

// vram cmds //

type xferCmd struct {
	ctype   cmdType
	xferIdx uint8

	pad0 uint16

	xferSize VramSize
	vAddr    VramAddr
}

// FIXME: caps validation
func validateXfer(hbuf []byte, dbuf *VramAlloc, offset VramSize) error {
	if dbuf.allocAddr == 0 {
		return errors.New("use after free")
	}

	endAddr := VramSize(len(hbuf)) + offset
	if endAddr > dbuf.allocSize {
		return errors.New("out of bounds")
	}

	return nil
}

func (cb *Cmdbuf) CmdXferToDevice(src []byte, dst *VramAlloc, dstOffset VramSize) error {
	// validate

	if err := cb.ensureRec(); err != nil {
		return err
	}

	if err := validateXfer(src, dst, dstOffset); err != nil {
		return err
	}

	// encode

	idx := cb.queueXfer(src)
	cmd := xferCmd{
		ctype:   cmdXferToDevice,
		xferIdx: idx,

		xferSize: VramSize(len(src)),
		vAddr:    dst.allocAddr + VramAddr(dstOffset),
	}

	return cb.appendCmd(cmd)
}

func (cb *Cmdbuf) CmdXferToHost(dst []byte, src *VramAlloc, srcOffset VramSize) error {
	// validate

	if err := cb.ensureRec(); err != nil {
		return err
	}

	if err := validateXfer(dst, src, srcOffset); err != nil {
		return err
	}

	// encode

	idx := cb.queueXfer(dst)
	cmd := xferCmd{
		ctype:   cmdXferToHost,
		xferIdx: idx,

		xferSize: VramSize(len(dst)),
		vAddr:    src.allocAddr + VramAddr(srcOffset),
	}

	return cb.appendCmd(cmd)
}

// gcs cmds //

type writeCbufCmd struct {
	ctype cmdType
	pad0  uint8

	rangeSize   uint16
	rangeOffset uint16

	pad1    uint16
	srcAddr VramAddr
}

type pushCbufCmd struct {
	ctype cmdType

	rangeSize   uint8
	rangeOffset uint16

	/* inline cbuf data follow */
}

type pushGstateCmd struct {
	ctype cmdType

	rangeSize   uint8
	rangeOffset uint8
}

type DrawBits uint8

const (
	DrawIndexedBit DrawBits = 1 << iota
	DrawLatchBit
)

type drawCmd struct {
	ctype    cmdType
	drawBits DrawBits

	pad0 uint16

	primCount uint32
	indexBase uint32

	// instanceCount uint32
	// instanceBase uint32
}

func (cb *Cmdbuf) CmdWriteCbuf(buf *VramAlloc, bufOffset VramSize, r VramRange) error {
	// validate

	if err := cb.ensureRec(); err != nil {
		return err
	}

	// if r.size+r.offset >= cb.dev.limits.MaxCbufSize {
	// 	return errors.New("out of bounds")
	// }

	// FIXME: vram out of bounds check

	// encode

	cmd := writeCbufCmd{
		ctype: cmdWriteCbuf,

		rangeSize:   uint16(r.Size),
		rangeOffset: uint16(r.Offset),

		srcAddr: buf.allocAddr + VramAddr(bufOffset),
	}

	return cb.appendCmd(cmd)
}

func (cb *Cmdbuf) CmdPushCbuf(hbuf []byte, offset VramSize) error {
	// validate

	if err := cb.ensureRec(); err != nil {
		return err
	}

	// if len(hbuf)+offset > cb.device.limits.MaxCbufSize {
	// 	return errors.New("out of bounds")
	// }

	// if len(hbuf) > cb.device.limits.MaxPushSize {
	// 	return errors.New("push is too big")
	// }

	// encode

	cmd := pushCbufCmd{
		ctype: cmdPushCbuf,

		rangeSize:   uint8(len(hbuf)),
		rangeOffset: uint16(offset),
	}

	err := cb.appendCmd(cmd)
	if err != nil {
		return nil
	}

	return cb.appendInlineData(hbuf)
}

func (cb *Cmdbuf) CmdPushGstate(gs_seg []byte, offset uint8) error {
	// validate

	if err := cb.ensureRec(); err != nil {
		return err
	}

	if len(gs_seg)+int(offset) > 255 {
		return errors.New("out of bounds")
	}

	// encode

	cmd := pushGstateCmd{
		ctype: cmdPushGstate,

		rangeSize:   uint8(len(gs_seg)),
		rangeOffset: offset,
	}

	err := cb.appendCmd(cmd)
	if err != nil {
		return err
	}

	return cb.appendInlineData(gs_seg)
}

func (cb *Cmdbuf) CmdDraw(bits DrawBits, primCount int, indexBase int) error {
	// validation

	if err := cb.ensureRec(); err != nil {
		return err
	}

	// TODO: should we state track gstate and validate?

	// encode

	cmd := drawCmd{
		ctype:    cmdDraw,
		drawBits: bits,

		primCount: uint32(primCount),
		indexBase: uint32(indexBase),
	}

	return cb.appendCmd(cmd)
}

/* submit on-wire commands */

type scmdType uint8

const (
	scmdSync scmdType = iota

	scmdXferToDevice
	scmdXferToHost

	scmdEnqueue
)

type xferFromDeviceScmd struct {
	ctype scmdType

	xferIdx uint8
}

type xferFromHostScmd struct {
	ctype scmdType

	pad0 uint8
	pad1 uint16

	xferSize uint32
	vramAddr VramAddr
}

type enqueueScmd struct {
	ctype scmdType

	pad0 uint8
	pad1 uint16

	cmdbufAddr VramAddr
}
