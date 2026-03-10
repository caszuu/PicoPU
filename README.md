# PicoPU

A mecha project focused on building a very simple open-source General Purpose Graphics Processing Unit (GPGPU) using the amazing `rp2350` MCUs as compute cores. The project it self is made of multiple parts: the software drivers, the GPU firmware and the hardware PCB itself.

The project is currently very WIP, more is to come.

Monorepo subprojects:
- `core-firmware/` - PicoPU firmware
	- `aux/` - auxiliary drivers and utilities
	 	- `usbd/` - a protocol generic `tinyusb` device driver
  	- `dvid/` - a simple hstx-based dvi driver
		- `pl/` - Pico-Link - a powerful `rp2350` optimized inter-connect protocol
  	- `gcs/` - an arch agnostic software renderer
		- `util/` - general utility headers
	- `arch/` - device architecture specific source files
		- `mock/` - RPi Pico 2 (mocking bird) arch firmware
- `boards/` - EasyEDA Std design files
	- `broker` - Files for the Broker/Motherboard
	- `shader` - Files for the smaller Shader boards
