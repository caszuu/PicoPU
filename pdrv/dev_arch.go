package pdrv

import "errors"

/* arch specific device configs and inits */

// Mockingbird

func (d *Device) initMockingArch() error {
	d.availableHeaps = make([]VramHeap, 1)

	d.availableHeaps[0] = VramHeap{
		heapSize:         480 * 1024,
		heapRowAlignment: 4,
		heapCaps:         HeapCapXferOps | HeapCapShaderOps | HeapCapFbOps,
	}

	return nil
}

// Raven

func (d *Device) initRavenArch() error {
	return errors.New("raven arch is not supported yet")
}
