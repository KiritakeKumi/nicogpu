## nicoGPU



An open-source GPU project. The current design goal is to implement Vulkan 1.4, achieve CTS (Conformance Test Suite) certification, and provide support for Zink (OpenGL 4.6) on top of that.

Once the project is fully complete, we may consider applying for a `VK_DRIVER_ID` (though this appears to require paying membership fees to the Khronos Group).

FP32 and INT32 operations have already been implemented, and the existing components have passed Vulkan dEQP tests.

The graphics component is still under construction; pull requests from external contributors will not be accepted until the milestone is completed.

There is currently no physical hardware available (we are exploring a tape-out test using the GlobalFoundries 180nm process). For running the design, we recommend using an AMD Virtex UltraScale+ VU19P, VU13P, or VU9P; if you only intend to implement a specific portion of the system, you may choose an FPGA of your preference.

Regardless of the model selected, the FPGA must be equipped with DDR or HBM to serve as video memory (VRAM). There are currently no plans to implement a Unified Memory Architecture (UMA).



For this GPU, I have also designed an instruction set. It is based on the RV32IMF specification (though it may no longer closely resemble it). Initially, I didn't have a name for it, so I referred to it as "xgpu"; later, I decided to officially name it the "nicogpu ISA."

ISA documentation: https://imagination.niconiconi.us/docs/NoitaniGami%20GPU/Xgpu%20ISA/v0.1/

The first version of this GPU is codenamed "Heathcliff."



Project components:

RTL (this directory)

Mesa: https://github.com/KiritakeKumi/nicogpu-mesa

Firmware: https://github.com/KiritakeKumi/nicogpu-firmware

The internal firmware for this GPU is implemented on top of "Ibex" (https://github.com/lowrisc/ibex), an RV32-based RISC-V controller.

Kernel module: https://github.com/KiritakeKumi/nicogpu-module

Due to limited human resources, some parts of the design were completed with the assistance of the Claude model series by Anthropic.

## License

The RTL in this repository is released under the GPL-3.0-only license (see [LICENSE](LICENSE)). Other components: the kernel module and the firmware are released under GPL-2.0 ([nicogpu-module](https://github.com/KiritakeKumi/nicogpu-module), [nicogpu-firmware](https://github.com/KiritakeKumi/nicogpu-firmware)); the [nicogpu-mesa](https://github.com/KiritakeKumi/nicogpu-mesa) branch is released under the MIT license.
