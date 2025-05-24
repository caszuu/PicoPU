package pdrv

import (
	"errors"
	"sync"

	"github.com/google/gousb"
)

type Device struct {
	// usb sub-system
	usbMu        sync.Mutex
	usbDev       *gousb.Device
	usbItfDoneCb func()

	xferOut *gousb.OutEndpoint
	xferIn  *gousb.InEndpoint

	xferOutStream gousb.WriteStream
	xferInStream  gousb.ReadStream
}

func InitDevice(ctx *gousb.Context) (*Device, error) {
	var dev Device

	// usb device

	usbDev, err := ctx.OpenDeviceWithVIDPID(0xcafe, 0x4000)
	if err != nil {
		return nil, err
	}

	if usbDev == nil {
		return nil, errors.New("no device found")
	}

	dev.usbDev = usbDev

	itf, done, err := usbDev.DefaultInterface()
	if err != nil {
		return nil, err
	}

	dev.usbItfDoneCb = done

	// endpoints

	inEp, err := itf.InEndpoint(2)
	if err != nil {
		return nil, err
	}

	outEp, err := itf.OutEndpoint(2)
	if err != nil {
		return nil, err
	}

	dev.xferOut = outEp
	dev.xferIn = inEp

	// xfer streams

	outStream, err := outEp.NewStream(4096, 16)
	if err != nil {
		return nil, err
	}

	inStream, err := inEp.NewStream(4096, 16)
	if err != nil {
		return nil, err
	}

	dev.xferOutStream = *outStream
	dev.xferInStream = *inStream

	return &dev, nil
}

func (dev *Device) Destroy() {
	dev.xferInStream.Close()
	dev.xferOutStream.Close()

	dev.usbItfDoneCb()
	dev.usbDev.Close()
}

// low-level xfers

func (dev *Device) QueueOutXfer(buf []byte) error {
	dev.usbMu.Lock()
	defer dev.usbMu.Unlock()

	_, err := dev.xferOut.Write(buf)
	return err
}

func (dev *Device) ReadInXfer(buf []byte) (int, error) {
	dev.usbMu.Lock()
	defer dev.usbMu.Unlock()

	return dev.xferInStream.Read(buf)
}
