package pdrv

type Fence struct {
	dev *Device
	idx uint16

	w chan struct{}
	s bool
}

func (dev *Device) NewFence() (*Fence, error) {
	dev.syncMu.Lock()
	defer dev.syncMu.Unlock()

	fenceIdx := dev.nextFenceIdx
	dev.nextFenceIdx += 1

	f := &Fence{
		dev: dev,
		idx: fenceIdx,

		w: make(chan struct{}),
		s: false,
	}

	dev.fences[fenceIdx] = f
	return f, nil
}

func (f *Fence) Destroy() {
	f.dev.syncMu.Lock()
	defer f.dev.syncMu.Unlock()

	delete(f.dev.fences, f.idx)
}

func (f *Fence) Reset() {
	if !f.s {
		return
	}

	f.w = make(chan struct{})
	f.s = false
}

func (f *Fence) Wait() {
	if f.s {
		return
	}

	<-f.w
}

func (f *Fence) Status() bool {
	return f.s
}

func (f *Fence) signal() {
	if f.s {
		return
	}

	f.s = true
	close(f.w)
}
