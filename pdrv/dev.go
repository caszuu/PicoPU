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

// low-level xfers

func (dev *Device) rawWriteOut(buf []byte) (int, error) {
	dev.usbMu.Lock()
	defer dev.usbMu.Unlock()

	return dev.xferOut.Write(buf)
}

func (dev *Device) rawReadIn(buf []byte) (int, error) {
	dev.usbMu.Lock()
	defer dev.usbMu.Unlock()

	return dev.xferIn.Read(buf)
}

func (dev *Device) setupOutXfer(setup_cmd []byte) (*gousb.WriteStream, error) {
	// dev.usbMu.Lock()
	// defer dev.usbMu.Unlock()

	_, err := dev.xferOut.Write(setup_cmd)
	if err != nil {
		return nil, err
	}

	return dev.xferOut.NewStream(512, 4)
}

func (dev *Device) setupInXfer() (*gousb.ReadStream, error) {
	// dev.usbMu.Lock()
	// defer dev.usbMu.Unlock()

	return dev.xferIn.NewStream(512, 4)
}

// queue submits

// TODO: make non-blocking, probably just put UsbStream into a goroutine
func (dev *Device) SubmitXferToDevice(hbuf []byte, dst *VramAlloc, offset VramSize) error {
	// validate

	if err := validateXfer(hbuf, dst, offset); err != nil {
		return err
	}

	if dst.Caps()&HeapCapXferOps == 0 {
		return errors.New("missing xfer vram capability")
	}

	// submit

	scmd := xferFromHostScmd{
		ctype: scmdXferToDevice,

		xferSize: uint32(len(hbuf)),
		vramAddr: dst.allocAddr,
	}

	buf := make([]byte, 64)
	cmdSize, _ := binary.Encode(buf, binary.LittleEndian, scmd)

	dev.usbMu.Lock()
	defer dev.usbMu.Unlock()

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
	dev.xferOut.Write(hbuf)
	return nil
}

// TODO: mark cmdbuf as pending and fence sync
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
