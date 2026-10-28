## nicoGPU



一个开源GPU项目，目前的设计目标是实现 Vulkan 1.4 并通过CTS认证 并在其之上支持 Zink （OpenGL 4.6）

如果可以的话 也可能考虑在彻底完成后申请 VK_DRIVER_ID （不过看上去需要给 Khronos 交会员费）

已经实现 fp32 / int32 的运算，已经实现的部分可以通过  Vulkan dEQP 测试. 

图形部分仍在施工中，在里程碑完成之前不会接收来自外部的Pull requests.

目前并没有真实硬件（正在寻求 GlobalFoundries 180nm的流片测试），推荐使用 AMD Virtex UltraScale+ VU19P / VU13P / VU9P 来运行，如果你只是想启动某一部分那可以自选FPGA.

不过，无论选择何种都需要你的FPGA有DDR或者HBM用来充当显存，暂时并没有设计统一内存（UMA）的想法.



对于这个GPU，我还设计了一个指令集，是在RV32IMF的基础上设计的（虽然可能已经不像是了），最开始并没有想好叫什么名字所以用xgpu代称，后来决定直接使用nicogpu ISA（

ISA文档：https://imagination.niconiconi.us/docs/NoitaniGami%20GPU/Xgpu%20ISA/v0.1/



以及对于这个GPU的第一个版本，目前代号为 Heathcliff



项目相关组件：

RTL （本目录）

mesa ：https://github.com/KiritakeKumi/nicogpu-mesa

firmware：https://github.com/KiritakeKumi/nicogpu-firmware

这颗GPU的内部固件在一个 rv32 的 RISC-V 控制器 ibex（https://github.com/lowrisc/ibex）的基础上实现

内核模块：https://github.com/KiritakeKumi/nicogpu-module



受人手限制，部分设计使用 Anthropic Claude 系列模型完成（


## 许可证

本仓库中的 RTL 以 GPL-3.0-only 发布（见 LICENSE 文件）。其余组件：内核模块与 firmware 以 GPL-2.0 发布（[nicogpu-module](https://github.com/KiritakeKumi/nicogpu-module)、[nicogpu-firmware](https://github.com/KiritakeKumi/nicogpu-firmware)）；[nicogpu-mesa](https://github.com/KiritakeKumi/nicogpu-mesa) 分支以 MIT 发布。
