package pdrv

import (
	"container/list"
	"errors"
	"log/slog"
	"sync"
)

/* vram sub-system */

type VramAddr uint32
type VramSize uint32

type VramHeapCaps uint32

const (
	HeapCapXferOps VramHeapCaps = 1 << iota
	HeapCapShaderOps
	HeapCapFbOps
)

type VramRange struct {
	Size   VramSize
	Offset VramSize
}

type VramRes struct {
	VramRange
	Alloc *VramAlloc
}

func ResAll(alloc *VramAlloc) VramRes {
	return VramRes{VramRange{Size: alloc.Size()}, alloc}
}

func ResRange(alloc *VramAlloc, size VramSize, offset VramSize) (VramRes, error) {
	if size+offset > alloc.Size() {
		return VramRes{}, errors.New("out of bounds")
	}

	return VramRes{VramRange{size, offset}, alloc}, nil
}

func (res VramRes) Addr() VramAddr {
	if !res.Valid() {
		return 0 // FIXME: should be an error or panic
	}

	return res.Alloc.Addr() + VramAddr(res.Offset)
}

func (res VramRes) Valid() bool {
	return res.Alloc != nil
}

// Staging buffers //

type Staging struct {
	device *Device

	idx uint16
	buf []byte
}

func (dev *Device) NewStaging(size VramSize) *Staging {
	dev.stagingMu.Lock()
	defer dev.stagingMu.Unlock()

	idx := dev.nextStagingIdx
	dev.nextStagingIdx += 1

	if idx == ^uint16(0) {
		panic("FIXME: out of staging ids")
	}

	s := make([]byte, size)
	dev.stagings[idx] = s

	return &Staging{
		device: dev,
		idx:    idx,
		buf:    s,
	}
}

func (s *Staging) Destroy() {
	s.device.stagingMu.Lock()
	defer s.device.stagingMu.Unlock()

	delete(s.device.stagings, s.idx)
}

func (s *Staging) View() []byte {
	return s.buf
}

// VramHeap Descriptor //

type VramHeap struct {
	// heap descriptor //

	heapSize         VramSize
	heapRowAlignment VramSize

	heapCaps VramHeapCaps

	// allocator //

	allocMu sync.Mutex

	refCount  int
	freeZones *list.List
}

func newHeap(size VramSize, rowAlignment VramSize, caps VramHeapCaps) *VramHeap {
	fl := list.New()
	fl.PushFront(freeZone{0, VramAddr(size)})

	return &VramHeap{
		heapSize:         size,
		heapRowAlignment: rowAlignment,
		heapCaps:         caps,

		refCount:  0,
		freeZones: fl,
	}
}

func (h *VramHeap) Destroy() {
	if h.refCount != 0 {
		slog.Error("destroyed heap which is still in use! not all heap allocations we're freed")
	}
}

func (h *VramHeap) Size() VramSize {
	return h.heapSize
}

func (h *VramHeap) HeapCaps() VramHeapCaps {
	return h.heapCaps
}

// Heap Allocator //

type freeZone struct {
	base VramAddr
	top  VramAddr
}

func alignUp[T ~uint32](value T, alignment T) T {
	return (value + (alignment - 1)) & ^(alignment - 1)
}

func (h *VramHeap) allocAddr(size VramSize) (VramAddr, error) {
	h.allocMu.Lock()
	defer h.allocMu.Unlock()

	for z := h.freeZones.Front(); z != nil; z = z.Next() {
		zone := z.Value.(freeZone)

		if VramSize(zone.top-zone.base) < size {
			continue // not enough space in zone
		}

		addr := zone.base
		zone.base += VramAddr(size)
		z.Value = zone

		if zone.base == zone.top {
			h.freeZones.Remove(z) // zone fully allocated
		}

		h.refCount += 1
		return addr, nil
	}

	return 0, errors.New("out of heap memory")
}

func (h *VramHeap) Alloc(size VramSize) (*VramAlloc, error) {
	// find suitable free zone

	alignedSize := alignUp(size, h.heapRowAlignment)

	addr, err := h.allocAddr(alignedSize)
	if err != nil {
		return nil, err
	}

	// return alloc struct

	alloc := &VramAlloc{
		allocHeap: h,

		allocSize: alignedSize,
		allocAddr: addr,
	}

	return alloc, nil
}

func (h *VramHeap) freeAddr(addr VramAddr, size VramSize) {
	h.allocMu.Lock()
	defer h.allocMu.Unlock()

	allocBase := addr
	allocTop := addr + VramAddr(size)

	var prev *list.Element
	next := h.freeZones.Front()

	for {
		mergeTop := false
		mergeBase := false

		if next != nil {
			nextZone := next.Value.(freeZone)

			if allocTop > nextZone.base {
				// alloc not in-between prev and next, step to next pair of zones

				prev = next
				next = next.Next()
				continue
			} else if allocTop == nextZone.base {
				mergeTop = true
			}
		}

		if prev != nil {
			prevZone := prev.Value.(freeZone)

			if allocBase == prevZone.top {
				mergeBase = true
			}

			// if allocBase < prevZone.top {
			// 	panic("unreachable")
			// }
		}

		h.refCount -= 1

		if mergeBase && mergeTop {
			// touching free zones on both sides, merge both zones into one
			prev.Value = freeZone{
				base: prev.Value.(freeZone).base,
				top:  next.Value.(freeZone).top,
			}

			h.freeZones.Remove(next)
			return
		}

		if mergeBase {
			// touching a free zone on .base, extend free zone
			prev.Value = freeZone{
				base: prev.Value.(freeZone).base,
				top:  allocTop,
			}

			return
		}

		if mergeTop {
			// touching a free zone on .top, extend free zone
			next.Value = freeZone{
				base: allocBase,
				top:  next.Value.(freeZone).top,
			}

			return
		}

		// no touches found, insert a new free zone

		z := freeZone{
			base: allocBase,
			top:  allocTop,
		}

		if next != nil {
			h.freeZones.InsertBefore(z, next)
		} else {
			h.freeZones.PushBack(z)
		}

		return
	}
}

func (h *VramHeap) Free(alloc *VramAlloc) error {
	if alloc.allocHeap == nil {
		return errors.New("alloc is already freed")
	}

	h.freeAddr(alloc.allocAddr, alloc.allocSize)
	alloc.allocHeap = nil

	return nil
}

// VramAlloc //

type VramAlloc struct {
	allocHeap *VramHeap

	allocAddr VramAddr
	allocSize VramSize
}

func (a *VramAlloc) Size() VramSize {
	return a.allocSize
}

func (a *VramAlloc) Caps() VramHeapCaps {
	return a.allocHeap.heapCaps
}

func (a *VramAlloc) Valid() bool {
	return a.allocHeap != nil
}

func (a *VramAlloc) Addr() VramAddr {
	return a.allocAddr
}

func (a *VramAlloc) Free() error {
	return a.allocHeap.Free(a)
}
