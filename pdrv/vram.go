package pdrv

import (
	"errors"
	"log/slog"
	"sync/atomic"
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

// VramHeap Descriptor //

type VramHeap struct {
	heapSize         VramSize
	heapRowAlignment VramSize

	heapCaps VramHeapCaps

	refC atomic.Int32
}

func (h *VramHeap) destroyHeap() {
	if h.refC.Load() != 0 {
		slog.Error("destroyed heap which is still in use! not all heap allocators we're destroyed")
	}
}

func (h *VramHeap) Size() VramSize {
	return h.heapSize
}

func (h *VramHeap) HeapCaps() VramHeapCaps {
	return h.heapCaps
}

// VramAlloc //

type VramAlloc struct {
	allocScope    *VramHeapScope
	allocScopeIdx int

	allocAddr VramAddr
	allocSize VramSize
}

func (a *VramAlloc) Size() VramSize {
	return a.allocSize
}

func (a *VramAlloc) Caps() VramHeapCaps {
	return a.allocScope.heap.heapCaps
}

func (a *VramAlloc) tagInUse() {
	a.allocScopeIdx = a.allocScope.scopeIdx
}

func (a *VramAlloc) Valid() bool {
	return a.allocAddr != 0
}

func (a *VramAlloc) Addr() VramAddr {
	return a.allocAddr
}

func (a *VramAlloc) Free() error {
	return a.allocScope.Free(a)
}

// VramHeapScope Allocator //

type VramHeapScope struct {
	heap        *VramHeap
	allocBucket []VramAlloc

	scopeIdx      int
	tempStackHead VramAddr
}

func NewHeapScope(heap *VramHeap) *VramHeapScope {
	heap.refC.Add(1)

	return &VramHeapScope{
		heap:        heap,
		allocBucket: make([]VramAlloc, 0, 4096),
		scopeIdx:    0,

		tempStackHead: 4, // start at 4 as addr 0 is considered a nullptr like value
	}
}

func (hs *VramHeapScope) Destroy() {
	if len(hs.allocBucket) != 0 {
		slog.Error("not all allocs were freed, memory leaks occured!")
	}

	hs.heap.refC.Add(-1)
}

func (hs *VramHeapScope) Alloc(size VramSize) (*VramAlloc, error) {
	// TODO: fancy alloc

	// temp. very basic stack alloc
	alignedSize := (size + 3) & ^VramSize(3)

	if alignedSize >= hs.heap.heapSize {
		return nil, errors.New("out of heap memory")
	}

	alloc := &VramAlloc{
		allocScope:    hs,
		allocScopeIdx: hs.scopeIdx,

		allocSize: alignedSize,
		allocAddr: hs.tempStackHead,
	}
	hs.tempStackHead += VramAddr(alignedSize)

	return alloc, nil
}

func (hs *VramHeapScope) Free(alloc *VramAlloc) error {
	if alloc.allocAddr == 0 {
		return errors.New("alloc is already freed")
	}

	if alloc.allocScopeIdx == hs.scopeIdx {
		return errors.New("alloc freed while in-use")
	}

	// TODO: fancy free
	// for now vram is stack allocated, just no-op the free

	return nil
}

func (hs *VramHeapScope) TransitionScope() {
	// FIXME: add some cmdbuf barrier guraantees

	hs.scopeIdx += 1
}
