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
- `boards/` - EasyEDA Std design files
	- `broker` - Files for the Broker/Motherboard
	- `shader` - Files for the smaller Shader boards
