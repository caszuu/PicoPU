# PicoPU

A mecha project focused on building a very simple open-source General Purpose Graphics Processing Unit (GPGPU) using the amazing `rp2350` MCUs as compute cores. The project it self is made of multiple parts: the software drivers, the GPU firmware and the hardware PCB itself.

The project is currently very WIP, more is to come.

Monorepo subprojects:
- `core-firmware/` - PicoPU firmware
	- `common/` - shared firmware utilities
 	- `usbd/` - a protocol generic `tinyusb` device driver
  	- `dvid/` - a simple hstx-based dvi driver
  	- `gcs/` - an arch agnostic software renderer
	- `arch/` - device architecture specific source files
		- `mockbird/` - RPi Pico 2 (mocking bird) arch firmware
- `pdrv/` - a user-space host-side driver written in go
	- `bench/` - an example application using directly `pdrv` apis
- `boards/` - Board design files
	- `images` - used images, #TODO add images of boards
	- `broker` - EasyEDA Std. file for the Broker/Motherboard
	- `shader` - EasyEDA Std. file for the smaller Shader boards
	- `just-V2` - EasyEDA Std. file for the V2 board
	- `pcie` - KiCad project with the "V3" pcie edition
 