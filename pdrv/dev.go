package pdrv

import (
	"encoding/binary"
	"errors"
	"fmt"
	"sync"

	"github.com/google/gousb"
)

type Device struct {
	// usb sub-system
	usbDev       *gousb.Device
	usbItfDoneCb func()

	outMu   sync.Mutex
	xferOut *gousb.OutEndpoint

	xferIn *gousb.InEndpoint

	// vram sub-sys
	availableHeaps []*VramHeap

	stagingMu      sync.Mutex
	stagings       map[uint16][]byte
	nextStagingIdx uint16

	// sync sub-sys
	syncMu sync.Mutex

	fences       map[uint16]*Fence
	nextFenceIdx uint16
}

func (dev *Device) initDeviceUsb() error {
	ctx := gousb.NewContext()

	// usb device

	usbDev, err := ctx.OpenDeviceWithVIDPID(0xcafe, 0x4000)
	if err != nil {
		return err
	}

	if usbDev == nil {
		return errors.New("no device found")
	}

	dev.usbDev = usbDev

	itf, done, err := usbDev.DefaultInterface()
	if err != nil {
		return err
	}

	dev.usbItfDoneCb = done

	// endpoints

	inEp, err := itf.InEndpoint(1)
	if err != nil {
		return err
	}

	outEp, err := itf.OutEndpoint(1)
	if err != nil {
		return err
	}

	dev.xferOut = outEp
	dev.xferIn = inEp

	return nil
}

func InitDevice() (*Device, error) {
	dev := &Device{
		stagings:       make(map[uint16][]byte),
		nextStagingIdx: 0,

		fences:       make(map[uint16]*Fence),
		nextFenceIdx: 0,
	}

	err := dev.initDeviceUsb()
	if err != nil {
		return nil, err
	}

	// match device hwinfo

	hwinfo, err := dev.CtlQueryHwInfo()
	if err != nil {
		return nil, err
	}

	archStr := string(hwinfo.HwArch[:])

	switch archStr {
	case "mock\x00\x00\x00\x00":
		err = dev.initMockingArch()

	case "ravn\x00\x00\x00\x00":
		err = dev.initRavenArch()

	default:
		return nil, fmt.Errorf("unknown HwArch \"%s\"", archStr)
	}

	if err != nil {
		return nil, err
	}

	go dev.readbackDaemon()

	return dev, nil
}

func (dev *Device) Destroy() {
	for i := range dev.availableHeaps {
		dev.availableHeaps[i].Destroy()
	}

	dev.usbItfDoneCb()
	dev.usbDev.Close()
}

// vram managment //

func (dev *Device) AllocVram(size VramSize, caps VramHeapCaps) (*VramAlloc, error) {
	for _, heap := range dev.availableHeaps {
		if heap.heapCaps&caps != caps {
			continue
		}

		return heap.Alloc(size)
	}

	return nil, errors.New("no heaps with required caps found")
}

// low-level xfers //

func (dev *Device) rawWriteOut(buf []byte) (int, error) {
	dev.outMu.Lock()
	defer dev.outMu.Unlock()

	return dev.xferOut.Write(buf)
}

func (dev *Device) submitDeviceXfer(hbuf []byte, addr VramAddr) error {
	scmd := xferFromHostScmd{
		Ctype: scmdXferToDevice,

		XferSize: uint32(len(hbuf)),
		VAddr:    addr,
	}

	buf := make([]byte, 64)
	cmdSize, _ := binary.Encode(buf, binary.LittleEndian, scmd)

	dev.outMu.Lock()
	defer dev.outMu.Unlock()

	// TODO: for some reason gousb WriteStreams behave incorectly with usbd streams (often duplicating data for no apparent reason)
	// s, err := dev.setupOutXfer(buf[:cmdSize])
	// if err != nil {
	// 	return err
	// }

	// _, err = s.Write(hbuf)
	// if err != nil {
	// 	return err
	// }

	// return s.Close()

	dev.xferOut.Write(buf[:cmdSize])

	toWrite := len(hbuf)
	for toWrite != 0 {
		segSize := min(toWrite, 512)
		written, err := dev.xferOut.Write(hbuf[:segSize])
		if err != nil {
			return err
		}

		hbuf = hbuf[written:]
		toWrite -= written
	}

	return nil
}

func (dev *Device) readSync(pakBuf []byte) {
	var pak signalScmd
	_, _ = binary.Decode(pakBuf, binary.LittleEndian, &pak)

	f, ok := dev.fences[pak.SyncIdx]
	if !ok {
		panic(fmt.Errorf("device lost: invalid fence %d", pak.SyncIdx))
	}

	f.signal()
}

func (dev *Device) readXfer(pakBuf []byte, rs *gousb.ReadStream) {
	var pak xferFromDeviceScmd
	_, _ = binary.Decode(pakBuf, binary.LittleEndian, &pak)

	dev.stagingMu.Lock()
	staging, ok := dev.stagings[pak.StagingIdx]
	dev.stagingMu.Unlock()

	if !ok {
		panic(fmt.Errorf("device lost: invalid staging %d", 0))
	}

	switch pak.Ctype {
	case scmdXferToDevice:
		err := dev.submitDeviceXfer(staging, pak.VAddr)
		if err != nil {
			panic(fmt.Errorf("device lost: xfer failed: %v", err))
		}

	case scmdXferToHost:
		read := 0

		if len(staging) < int(pak.XferSize) {
			panic(fmt.Errorf("device lost: staging overflow: %d < %d", len(staging), pak.XferSize))
		}

		for read != int(pak.XferSize) {
			c, err := rs.Read(pakBuf)
			if err != nil {
				panic(fmt.Errorf("device lost: xfer failed: %v", err))
			}

			copy(staging[read:read+c], pakBuf[:c])
			read += c
		}

	default:
		panic(fmt.Errorf("device lost: invalid scmd: %d", pak.Ctype))
	}
}

func (dev *Device) readbackDaemon() {
	pakBuf := make([]byte, 512)
	rs, err := dev.xferIn.NewStream(512, 4)

	if err != nil {
		panic(fmt.Errorf("device lost: failed setting up readback stream: %v", err))
	}

	for {
		c, err := rs.Read(pakBuf)
		if err != nil {
			panic(fmt.Errorf("device lost: readback failed: %v", err))
		}

		var pakType scmdType
		_, err = binary.Decode(pakBuf, binary.LittleEndian, &pakType)

		switch pakType {
		case scmdSignal:
			dev.readSync(pakBuf[:c])

		case scmdXferToDevice:
			dev.readXfer(pakBuf[:c], rs)

		case scmdXferToHost:
			dev.readXfer(pakBuf[:c], rs)

		default:
			panic(fmt.Errorf("device lost: invalid scmd %d", pakType))
		}
	}
}

// queue submits //

// TODO: make non-blocking, probably just put UsbStream into a goroutine
func (dev *Device) SubmitXferToDevice(hbuf []byte, dst *VramAlloc, offset VramSize) error {
	// validate

	// FIXME: standard validate
	// if err := validateXfer(hbuf, dst, offset); err != nil {
	// 	return err
	// }

	if dst.Caps()&HeapCapXferOps == 0 {
		return errors.New("missing xfer vram capability")
	}

	// submit
	return dev.submitDeviceXfer(hbuf, dst.Addr()+VramAddr(offset))
}

func (dev *Device) SubmitEnqueue(cb *Cmdbuf) error {
	// validate

	if cb.state != CmdbufReady {
		return errors.New("cmdbuf not ready for enqueue")
	}

	// submit

	scmd := enqueueScmd{
		ctype:      scmdEnqueue,
		cmdbufAddr: cb.bufAlloc.allocAddr,
	}

	buf := make([]byte, 64)
	cmdSize, _ := binary.Encode(buf, binary.LittleEndian, scmd)

	_, err := dev.rawWriteOut(buf[:cmdSize])
	return err
}
