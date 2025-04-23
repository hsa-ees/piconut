# The PicoNut Project
**Author: Lorenz Sommer 2024**

 The PicoNut project at the Technical University of Applied Sciences Augsburg aims to develop a minimal and at the same time flexibly expandable RISC-V processor that works on common FPGA hardware and provides a complete simulator. The processor is intended to be used in teaching and research, in order to examine different computer architectures, the interaction between hardware and operating systems (Linux, FreeRTOS) and the integration of hardware accelerators, for example for AI applications.

PicoNut ...

-    ... is a minimal, yet extendable RISC-V processor as an open source project.
-    ... runs on inexpensive FPGA boards, e.g. OrangeCrab or ULX03S.
-    ... is expandable with memory protection/MMU (Linux), AI acceleration, various RISC-V extensions.
-    The hardware is modeled in SystemC (C++) in order to be able to build a powerful simulator from the same source code.
-    Good software support is provided by RISC-V compatibility: GNU toolchain (GCC/GDB), newlib, FreeRTOS, Linux.
-    Solid project management includes automated testing, project website and CI/CD techniques.

The project started at the beginning of 2024 and currently involves several enthusiastic students who are working on project work or theses.

## Introduction

The PicoNut hardware models are written in [SystemC](https://systemc.org/). RTL synthesis for FPGA deployment is possible
using the [ICSC (Intel® Compiler for SystemC)](https://github.com/intel/systemc-compiler) and [YOSYS](https://yosyshq.net/yosys/about.html) toolchains.
This allows the entire processor architecture be simulated using the powerful
simulation capabilities of the SystemC C++ library. The modular nature of the PicoNut allows for an easy
exchange of individual submodules for rapid prototyping.

With the initial release of this project the processor supports the RV32I subset of the RISC-V ISA and features
a fully fledged simulation environment capable of running any C programs compiled using the standard gcc RISC-V
toolchain. Future plans include implementing RISC-V extensions such as the A,F,M and V extensions with the aim
of running the Linux operating system as well as enabling AI applications.

The following sections will give a short overview of the system architecture, hardware, software and simulation
aspects of the project in its current iteration.

## System architecture

The PicoNut processor itself is part of a larger system built around it. The nucleus and the "memory unit" (MemU) make up the PicoNut prosessor.
The PicoNut processor, a System bus and any kind of memory make up the required minimal system.

```{figure} figures/piconut-overview.svg
:scale: 120
:alt: Alt text
:align: center

The PicoNut system
```

Memory is to be byte addressable and data is to be interpreted in LSB order by all components.

### The Nucleus

The nucleus is the execution unit of the system. It is executes instructions and accesses memory
and other devices via the memory unit. Communication between the nucleus and the memory unit is
done via the {ref}`sec:idef:ipdp`.

### Memory Unit

The memory unit serves as the processor nuclueus' interface to the wishbone bus. It handles all read and write
requests taken by the nucleus and forwards them to the wishbone bus. It does not equate to a memory management
unit (MMU) at the time of release. Currently its primary function is address space separation to allow various
peripherals connected to the system bus to be addressed by the nucleus. Virtual memory, paging and cache
functionality are future prospects.



### Main System Bus

The current main system bus uses the Wishbone bus protocol. It connects the PicoNut processor, the main memory and all peripherals.

## Software and Simulation

To facilitate testing, debugging and  rapid prototyping, the project includes a comprehensive simulator. It
allows the user to provide the system with a gcc compiled program which is then run. Beyond allowing for
standard output print statements, a `.vcd` trace file containing all nucleus signal states, a core trace
and a memory dump are generated alongside.
To enable rapid integration of new peripherals in software only a simulation only software peripheral interface
class is provided. The peripheral interface class "CSoftPeripheral" allows software models to interface with
hardware models in simulation. This approach unlocks the speed, resources and features of the C++ language for
the verification of hardware modules during the prototyping process, especially when potentially complex
modules are required for the DUT to function.

The simulator can be run by navigating to `~/piconut/systems/refdesign/hw` invoking `make run-tb`.
To enable tracing, invoke `make run-tb-trace`.

