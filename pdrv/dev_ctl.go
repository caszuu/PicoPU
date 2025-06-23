package pdrv

import (
	"github.com/google/gousb"
)

type devCtlPacketType byte

const (
	devCtlFlash devCtlPacketType = iota
	devCtlQueryHwInfo
	devCtlLedUpdate

	// devCtlModeset
	// devCtlReclock
	// devCtlInstru
)

type DevHwInfo struct {
	HwArch [8]byte

	Hwid  [16]byte
	Fwsha [16]byte
}

func (dev *Device) CtlFlash() error {
	dev.usbMu.Lock()
	defer dev.usbMu.Unlock()

	_, err := dev.usbDev.Control(0b00100000, uint8(devCtlFlash), 0, 0, nil)

	if err == gousb.ErrorNoDevice || err == gousb.ErrorPipe {
		return nil // "task failed successfully"
	}

	return err
}

func (dev *Device) CtlQueryHwInfo() (DevHwInfo, error) {
	buf := make([]byte, 64)

	dev.usbMu.Lock()
	defer dev.usbMu.Unlock()

	_, err := dev.usbDev.Control(0b10100000, uint8(devCtlQueryHwInfo), 0, 0, buf)
	if err != nil {
		return DevHwInfo{}, err
	}

	var hwInfo DevHwInfo
	copy(hwInfo.HwArch[:], buf[:8])
	copy(hwInfo.Hwid[:], buf[8:24])
	copy(hwInfo.Fwsha[:], buf[24:40])

	return hwInfo, nil
}

func (dev *Device) CtlLedUpdate(ledData []byte, stripIdx uint16) error {
	dev.usbMu.Lock()
	defer dev.usbMu.Unlock()

	_, err := dev.usbDev.Control(0b00100000, uint8(devCtlLedUpdate), stripIdx, 0, ledData)
	return err
}
