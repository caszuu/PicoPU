package pdrv

import (
	"encoding/binary"
)

type devCtlPacketType byte

const (
	devCtlFlash devCtlPacketType = iota
	devCtlModeset
	// devCtlReclock
	// devCtlInstru

	devCtlQueryHwInfo
)

type DevHwInfo struct {
	Fwver [8]byte

	Hwid  [16]byte
	Fwsha [16]byte

	IsSingleChip bool
}

func (dev *Device) CtlFlash() error {
	dev.usbMu.Lock()
	defer dev.usbMu.Unlock()

	_, err := dev.usbDev.Control(0x60, uint8(devCtlFlash), 0, 0, nil)
	return err
}

// func (dev *Device) CtlModeset(modeset *DviModeset) error {
// 	buf := make([]byte, reflect.TypeOf(*modeset).Size())
// 	_, err := binary.Encode(buf, binary.LittleEndian, modeset)

// 	if err != nil {
// 		return err
// 	}

// 	dev.usbMu.Lock()
// 	defer dev.usbMu.Unlock()

// 	_, err = dev.usbDev.Control(0x60, uint8(devCtlModeset), 0, 0, buf)
// 	return err
// }

func (dev *Device) CtlQueryHwInfo() (DevHwInfo, error) {
	buf := make([]byte, 128)

	dev.usbMu.Lock()
	defer dev.usbMu.Unlock()

	_, err := dev.usbDev.Control(0xe0, uint8(devCtlQueryHwInfo), 0, 0, buf)
	if err != nil {
		return DevHwInfo{}, err
	}

	var hwInfo DevHwInfo
	_, _ = binary.Decode(buf, binary.LittleEndian, hwInfo)

	return hwInfo, nil
}
