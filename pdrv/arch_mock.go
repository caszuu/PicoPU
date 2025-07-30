package pdrv

import (
	"encoding/binary"
	"errors"
)

// mocking bird arch impl //

type mockArch struct{}

func (arch mockArch) newEncoder(cb *Cmdbuf) CmdEncoder {
	enc := mockEncoder{
		cb: cb,

		gstateBuf:   gcsGstate{rasterMode: 3},
		gstateDirty: true,
	}

	return &enc
}

func (d *Device) initMockingArch() error {
	d.availableHeaps = make([]*VramHeap, 1)
	d.availableHeaps[0] = newHeap(
		480*1024,
		4,
		HeapCapFbOps|HeapCapShaderOps|HeapCapXferOps,
	)

	d.arch = mockArch{}

	return nil
}

// mocking bird encoder //

type mockEncoder struct {
	cb *Cmdbuf

	gstateBuf   gcsGstate
	gstateDirty bool
}

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

	cmdSignal
)

type gcsGstate struct {
	vbuf VramAddr
	ibuf VramAddr
	fbC0 VramAddr
	fbZs VramAddr

	fbExtent [2]uint16
	viewport [3][2]float32

	rasterMode byte
}

func (enc *mockEncoder) Finalize() error {
	if err := enc.cb.ensureRec(); err != nil {
		return err
	}

	enc.cb.appendCmd(uint32(0)) // terminate cmdbuf
	return enc.cb.xferFinalized()
}

// graphics state //

func (enc *mockEncoder) SetViewport(x int, y int, w int, h int) error {
	if err := enc.cb.ensureRec(); err != nil {
		return err
	}

	enc.gstateDirty = true
	enc.gstateBuf.viewport = [3][2]float32{
		{float32(w) / 2, float32(x) + float32(w)/2},
		{float32(h) / 2, float32(y) + float32(h)/2},
		{1, 0},
	}

	return nil
}

func (enc *mockEncoder) BindFramebuffer(extent [2]int, c0 VramRes, zs VramRes) error {
	if err := enc.cb.ensureRec(); err != nil {
		return err
	}

	// FIXME: attachment size checks

	enc.gstateDirty = true
	enc.gstateBuf.fbExtent = [2]uint16{uint16(extent[0]), uint16(extent[1])}
	enc.gstateBuf.fbC0 = c0.Addr()
	enc.gstateBuf.fbZs = zs.Addr()

	return nil
}

func (enc *mockEncoder) BindVertexBuffer(vb VramRes, ib VramRes) error {
	if err := enc.cb.ensureRec(); err != nil {
		return err
	}

	enc.gstateDirty = true
	enc.gstateBuf.vbuf = vb.Addr()
	enc.gstateBuf.ibuf = ib.Addr()

	return nil
}

func (enc *mockEncoder) flushGstate() error {
	if !enc.gstateDirty {
		return nil
	}
	enc.gstateDirty = false

	buf := make([]byte, 128)
	size, _ := binary.Encode(buf, binary.LittleEndian, enc.gstateBuf)

	// align to 4 bytes (required for inline data)
	size = (size + 3) & ^3

	return enc.pushGstate(buf[:size], 0)
}

// fb cmds //

type clearCmd struct {
	ctype     cmdType
	clearBits uint8
}

type presentCmd struct {
	ctype cmdType
}

func (enc *mockEncoder) Clear() error {
	// validate and flush

	if err := enc.cb.ensureRec(); err != nil {
		return err
	}

	if err := enc.flushGstate(); err != nil {
		return err
	}

	// encode

	cmd := clearCmd{
		ctype:     cmdClear,
		clearBits: 0,
	}

	return enc.cb.appendCmd(cmd)
}

func (enc *mockEncoder) Present() error {
	// validate and flush

	if err := enc.cb.ensureRec(); err != nil {
		return err
	}

	if err := enc.flushGstate(); err != nil {
		return err
	}

	// encode

	cmd := presentCmd{
		ctype: cmdPresent,
	}

	return enc.cb.appendCmd(cmd)
}

// vram cmds //

type xferCmd struct {
	ctype cmdType

	pad0       uint8
	stagingIdx uint16

	xferSize VramSize
	vAddr    VramAddr
}

func validateXfer(staging *Staging, dbuf *VramAlloc, offset VramSize) error {
	if dbuf.Caps()&HeapCapXferOps == 0 {
		return errors.New("xfer cap not supported")
	}

	if dbuf.allocHeap == nil {
		return errors.New("use after free")
	}

	endAddr := VramSize(len(staging.buf)) + offset
	if endAddr > dbuf.allocSize {
		return errors.New("out of bounds")
	}

	return nil
}

func (enc *mockEncoder) XferToDevice(src *Staging, dst *VramAlloc, dstOffset VramSize) error {
	// validate

	if err := enc.cb.ensureRec(); err != nil {
		return err
	}

	if err := validateXfer(src, dst, dstOffset); err != nil {
		return err
	}

	// encode

	cmd := xferCmd{
		ctype:      cmdXferToDevice,
		stagingIdx: src.idx,

		xferSize: VramSize(len(src.buf)),
		vAddr:    dst.allocAddr + VramAddr(dstOffset),
	}

	return enc.cb.appendCmd(cmd)
}

func (enc *mockEncoder) XferToHost(dst *Staging, src *VramAlloc, srcOffset VramSize) error {
	// validate

	if err := enc.cb.ensureRec(); err != nil {
		return err
	}

	if err := validateXfer(dst, src, srcOffset); err != nil {
		return err
	}

	// encode

	cmd := xferCmd{
		ctype:      cmdXferToHost,
		stagingIdx: dst.idx,

		xferSize: VramSize(len(dst.buf)),
		vAddr:    src.allocAddr + VramAddr(srcOffset),
	}

	return enc.cb.appendCmd(cmd)
}

// sync mds //

type signalCmd struct {
	ctype cmdType
	pad0  uint8

	syncIdx uint16
}

func (enc *mockEncoder) SignalFence(f *Fence) error {
	// validate

	if err := enc.cb.ensureRec(); err != nil {
		return err
	}

	// encode

	cmd := signalCmd{
		ctype:   cmdSignal,
		syncIdx: f.idx,
	}

	return enc.cb.appendCmd(cmd)
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

func (enc *mockEncoder) WriteConstants(src VramRes, offset VramSize) error {
	// validate

	if err := enc.cb.ensureRec(); err != nil {
		return err
	}

	// if r.size+r.offset >= cb.dev.limits.MaxCbufSize {
	// 	return errors.New("out of bounds")
	// }

	// FIXME: vram out of bounds check

	// encode

	cmd := writeCbufCmd{
		ctype: cmdWriteCbuf,

		rangeSize:   uint16(src.Size),
		rangeOffset: uint16(offset),

		srcAddr: src.Addr(),
	}

	return enc.cb.appendCmd(cmd)
}

func (enc *mockEncoder) PushConstants(hbuf []byte, offset VramSize) error {
	// validate

	if err := enc.cb.ensureRec(); err != nil {
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

	err := enc.cb.appendCmd(cmd)
	if err != nil {
		return nil
	}

	return enc.cb.appendInlineData(hbuf)
}

func (enc *mockEncoder) pushGstate(gs_seg []byte, offset uint8) error {
	// validate

	if err := enc.cb.ensureRec(); err != nil {
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

	err := enc.cb.appendCmd(cmd)
	if err != nil {
		return err
	}

	return enc.cb.appendInlineData(gs_seg)
}

func (enc *mockEncoder) Draw(first int, count int, bits DrawBits) error {
	// validation and flush

	if err := enc.cb.ensureRec(); err != nil {
		return err
	}

	if err := enc.flushGstate(); err != nil {
		return err
	}

	// encode

	cmd := drawCmd{
		ctype:    cmdDraw,
		drawBits: bits,

		primCount: uint32(count / 3),
		indexBase: uint32(first),
	}

	return enc.cb.appendCmd(cmd)
}
