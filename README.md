# PicoPU

A mecha project focused on building a very simplified open-source version of a General Purpose Graphics Processing Unit. (GPGPU) The project it self is made of multiple parts: the software drivers, the GPU firmware and the hardware PCB itself.

The project is currently very WIP, only the firmware and board are currently being developed.

Monorepo subprojects:
- `core-firmware/` - PicoPU firmware
	- `common/` - shared firmware source files
	- `shader_core/` - shader chip firmware
	- (not yet) `broker_core/` - broker chip firmware
- `broker-sim/` - software broker chip emulator
