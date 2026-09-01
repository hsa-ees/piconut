(sec:ai-intro)=
# 1. Introduction
**Gundolf Kiefer, 2026-05-08**


## Motivation

The rapid advancement of Artificial Intelligence (AI) has revolutionized various sectors, from healthcare and finance to autonomous vehicles and smart devices. However, the efficient execution of AI algorithms, particularly in mobile and embedded systems, poses significant challenges. These systems require hardware that can deliver high performance while maintaining low power consumption and cost-effectiveness. Traditional architectures often fall short in meeting these demands, highlighting the need for specialized hardware solutions.

RISC-V, an open instruction set architecture (ISA), offers a flexible and scalable platform for developing custom hardware solutions. Its open nature allows for innovation and customization, making it an ideal choice for AI acceleration. As of 2026, several new RISC-V specifications specifically tailored for AI acceleration are under development.

For students and researchers interested in delving deeper into the field of AI hardware, understanding and designing efficient AI processors is a crucial aspect of modern studies. The open-source nature of *PicoNut* and RISC-V provides an opportunity for educational institutions and research labs to experiment, innovate, and contribute to the development of next-generation AI hardware.

Moreover, RISC-V is a strategic asset for technological sovereignty. By adopting RISC-V, Germany and Europe can decrease dependence on proprietary technologies and foster a thriving ecosystem of innovation and collaboration in embedded AI.


## Purpose of the Project

Purpose of the *PicoNut AI* project is to develop a toolkit for customer-specific AI systems targeting low-power applications (Embedded & Edge AI).

A key component is the *PicoNut AI* processor architecture comprising tailored Membrana and Nucleus variants together with an *AI Unit* as a customizable processor extension for AI acceleration. The *AI Unit* implements the integer instructions of the *RISC-V Vector extension (RVV)* and can be extended to towards upcoming matrix extensions. Unlike previously existing open source RVV implementations [2-5], *PicoNut AI* follows the concept of Soft Hardware and is modelled in SystemC, which enables cycle-accurate simulations at very high simulation performance.

Software is a crucial part of the development. By sticking to the RISC-V instruction set, the RISC-V toolchain can be used for developing at the C code level, and libraries and compilers for the efficient inference of neural networks can be leveraged [6-10].

To develop and evaluate AI modules using Python and *PyTorch*, the *AI Playground* has been developed. It provides functionalities to implement, train and evaluate neural networks and export model data as C code without any library dependencies.


## Structure of this Document

Section [AI-Related RISC-V Extensions](#sec:ai-rv_standards) summarizes the activities at *RISC-V International* of standardizing AI-related instruction set extension.

Section [AI Playground](#sec:ai-playground) describes the *Jupyter*-based environment for developing AI models in Python (*PyTorch*) and exporting them as embedded C code.

Section [The PicoNut AI Processor Architecture](#sec:ai-architecture) describes the architecture of the AI-optimized *PicoNut* processor and its current implementation.

Section [Case Study: LeNet-5](#sec:ai-lenet5) presents experimental results based on a variant of the LeNet-5 image recogition model.



(sec:ai-rv_standards)=
# 2. AI-Related RISC-V Extensions 

As of 2026, AI acceleration with RISC-V is addressed by several RISC-V technical working groups (TG) or special interest groups (SIG).



## Application-related Special Interest Groups

* [Vector SIG](https://lists.riscv.org/g/sig-vector)
	- "The Vector SIG will serve as the group responsible for considering all vector extensions to the RISC-V ISA in the future. They will evaluate new proposals, create their own if needed, and work to create task groups (TGs) for new extensions as needed."
	- The group is basically responsible for further developing new extensions to the *RISC-V Vector (RVV)* extension.
* [AI/ML SIG](https://lists.riscv.org/g/sig-ai-ml)
	- "The goal of AI/ML SIG is to promote the research and standardization of global RISC-V AI instruction architecture, basic software and core application technologies."
* [ML/AI Applications SIG](https://lists.riscv.org/g/sig-ml-ai-apps)
	- "The ML/AI Applications SIG coordinates and nurtures the software support for RISC-V in ML compilers, similar to how other software SIGs are tracking RISC-V support for other parts of the software stack."



## RISC-V Vector Extension (RVV)

The RISC-V Vector extension (RVV) has is ratified and documented in the *RISC-V Instruction Set Manual* since version 20240411 [1]. RVV is by the GCC toolchain and the RISC-V reference simulator [Spike/riscv-isa-sim](https://github.com/riscv-software-src/riscv-isa-sim). Hence, software development is well supported and does not require any new tools.

Compared to vector extension found in other standard CPUs, RVV is very flexible both at runtime and at synthesis/implementation time:

* The (maximum) vector length (VLEN) is implementation-dependent allowing for a trade-off between chip area and performance. At runtime, the vector length can be chosen arbitrarily. Also, up to 8 vector registers can be grouped to form very long vectors (LMUL).
* Several data types and widths are supported (8/16/32/64 bits), including a choice of "narrowing" and "widening" operations. For example, an element-wise multiplication of 16 bit vectors can produce a result vector with elements of 32 bits.
* Operations are maskable, i.e. only those vector elements selected by a mask register are processed.
* Flexible memory accesses are supported, including strided load/store (e.g., to read a matrix column) and indexed load/store (for scatter/gather accesses).

RVV comprises several sub-extensions, and an implementation may choose to implement a subset of them. *PicoNut AI* implements *Zve32Zvl\<n\>b*, where \<n\> is the vector length (VLEN) an can be which can be an arbitrary power of two between 64 and 65536. *Zve32* denotes the 32 bit integer subset targeting embedded processors, which most suitable for embedded AI applications.

More information on the RVV can be found in the RISC-V specification (Ch. 30) [1].



## Matrix Extensions

Several technical working groups (TG) work on alternative standards for matrix computations.

* [Vector Matrix Extension (VME)](https://lists.riscv.org/g/tech-vme)
  - The objective of VME is to extend RVV by a relatively simple and straightforward matrix extension based on outer-product matrix multiplication. Unlike AME, VME remains closely coupled to the RVV implementation. Unlike IME, VME introduces a new accumulator register file into which matrix multiplication results are accumulated in order to overcome capacity limitations of the vector register file. A VME matmul operation takes two vector registers, computes the outer product of these operands, and accumulates the results into a large matrix accumulator.

* [Integrated Matrix Extension (IME)](https://lists.riscv.org/g/tech-integrated-matrix-extension)
  - IME builds on top of the RISC-V Vector extension and will define new matrix instructions that operate on the existing vector registers. Several options for storing matrices in vector registers are explored.

* [Attached Matrix Extension (AME)](https://lists.riscv.org/g/tech-attached-matrix-extension)
  - AME is an ISA extension for matrix operations in an attached matrix unit with its own set of matrix registers. The AME instructions are part of the processor instruction stream and must follow program order.

The specifications of IME, AME, and VME are currently work in progress.



(sec:ai-playground)=
# 3. AI Playground

## Overview

Software is a crucial part of the development. By sticking to the RISC-V instruction set, the RISC-V toolchain can be used for developing at the C code level. To develop AI modules using Python and *PyTorch*, the *AI Playground* has been developed. It provides functionalities to implement, train and evaluate neural networks and export model data as C code. The implementation as a Jupyter notebook allows for an interactive use making it well-suited for educational purposes.

The notebook covers the following sections:

1. **Preamble:** Code to initialize the notebook and load all external modules.
2. **Model Definition:** Place to define the neural network model using PyTorch.
3. **Training:** Training and validation of the model, including options to load and save trained parameters from or to disk.
4. **Inference:** Model inference and evaluation.
5. **Inspection:** Visualization of image data or intermediate results.
6. **C Exports:** Exporting model data a C code for RISC-V microcontroller implementations.
7. **Scratch Area** for custom code.

{numref}`fig-lenet5-notebook-1`, {numref}`fig-lenet5-notebook-2`, and {numref}`fig-lenet5-notebook-3` show example uses of the notebook.

```{figure} figures/lenet5-notebook-1.svg
:name: fig-lenet5-notebook-1
*PyTorch* Model
```

```{figure} figures/lenet5-notebook-2.svg
:name: fig-lenet5-notebook-2
Interactive Inference Evaluation
```

```{figure} figures/lenet5-notebook-3.svg
:name: fig-lenet5-notebook-3
Data Inspection
```


## Installation and Usage

To build and stage the playground together with *JupyterLab* and a suitable Python environment (`venv`), enter:
```
$ make MODULES=sw/apps/ai/playground
```
To run the notebook, enter:
```
$ make -C sw/apps/ai/playground run
```

Usually, a browser window opens automatically. In addition, the console output shows instructions and URLs to open the playground manually.

Then follow the instructions in the notebook itself.



(sec:ai-architecture)=
# 4. The *PicoNut AI* Processor Architecture

## Overview

The *PicoNut AI* processor architecture comprises tailored Membrana and Nucleus variants together with an customizable extension for AI acceleration.

{numref}`fig-piconut_ai-structure` depicts the general modular structure. The Nucleus is composed of a base scalar processor (*Nucleus AI*) and the *AI unit*. Both are interconnected by an extension interface which allows each component to be replaced by an alternative implementation. In the future, multiple variants of AI units may be implemented for different applications.

```{figure} figures/piconut_ai-structure.svg
:name: fig-piconut_ai-structure
*PicoNut AI* - Block Diagram
```

The AI unit may have its own dedicated data port (DPort) to allow the implementation of tailored Membranas that allow vector or matrix data accesses to happen in parallel to scalar memory accesses of the base unit. Optionally, these two DPorts may have different bus widths. I.e., for vector and matrix data, a high-performance wide data port may be implemented.

The extension port interface provides signals for the following functionalities:

* passing the current instruction to the AI unit
* passing the two scalar source operands to the AI unit
* receiving a scalar result value from the AI unit
* status signals from the extension to indicate
	- whether the extension unit is ready to accept a new instruction
	- whether previously started instructions have completed
	- whether an instruction can be executed by the AI unit
* further control and status signals for synchronisation
* optional: a DPort interface to let the base unit multiplex memory data accesses for a single data port towards the Membrana.


## *Membrana AI*

### Objectives

An AI-optimized Membrana should be optimized for high throughput at its data ports, in particular:

- support the pipelined IPort/DPort protocol
- optimize sequential reads/writes
- optimize strided read/write accesses, for example, to read ot write matrix columns
- support writes and reads of partial words (only few *bsel* bits set) efficiently; these may, for example, occur during strided accesses with small data types
- offer a dedicated, second DPort for vector and matrix accesses (optional)
- support configurable widths of the DPort(s)


### Soft Hardware Implementation

The soft hardware implementation of Membrana AI (`cpu/membrana_ai`) implements all features mentioned in the previous section. At the time of writing, the only limitation is that DPort widths different from 32 bits have not been tested yet.

The model implements an internal memory that can be pre-initialized by an ELF file and the PicoNut-specific UART module for standard input and output.

The timing behavior reflects the optimal timing possible by the IPort/DPort interfaces. All three ports (IPort, scalar and vector DPort) support the pipeline protocol with a throughput of one word per clock cycle and a latency of 2 cycles. All three ports act in parallel. This reflects a hypothetical cache configuration with a hit rate of 100%.


## *Nucleus AI* (Base Processor)

### Objectives

The *Nucleus AI* base processor should be implemented by a low area processor with reasonable performance, for example, 5-stage pipeline with forwarding and simple branch prediction.

It *must* implement the extension interface for the AI unit. For compatibility with arbitrary Membrana implementations, it *may* offer a DPort to the AI unit and multiplex it at its own DPort towards the Membrana.

It should provide or optimize the following aspects:

- implement the following extensions: Zicsr, M
- implement pipelined memory access at the IPort
- implement pipelined memory access at the DPort(s) for vector operations
- suppport instruction tracing for debugging and profiling


### Soft Hardware Implementation

The soft hardware implementation of *Nucleus AI* (`cpu/nucleus_ai`) implements all features mentioned in the previous section. In particular, it

- implements *RV32IMZicsr*,
- implements the extension interface and instantiates an AI unit as a submodule,
- supports multiplexing of a DPort for Membranas without a second vector DPort,
- supports port pipelining at the IPort and all DPorts (can be switched off for compatibility with Membrana implementations not supporting pipelining).

It is optimized for high simulation performance to allow the evaluation the complex AI applications. This includes an efficient soft modelling of the CSRs. A LeNet-5 inference with a vector implementation can be simulated within approx. 2 seconds on a contemporary PC.

The timing approximates the timing of a single-scalar processor with a classical 5-stage pipeline, ideal branch prediction and full forwarding. This is close to the optimum throughput that can be achieved with the IPort and DPort(s) with port pipelining.

IPort and DPort pipeling are supported, but can be switched off to maintain compatibility the Membrana variants not supporting pipelining.

Instruction tracing is supported for debugging and profiling.


## *AI Unit*

### Objectives

Purpose of the AI unit is to bring AI acceleration to the *PicoNut* system. At the time of writing, it implements the main integer part of the *RISC-V Vector extension (RVV)*, which is sufficient for typical layers of neural networks (i.e. convolutional layers, fully connected layers, pooling, activation).

In the future, this may be extented, for example with aspects of the *RISC-V Attached Matrix Extension*, the *RISC-V Integrated Matrix Extension* or the *RISC-V Vector Matrix Extension*. Also, the AI unit allows to implement custom functionality not covered by any of the official vector or matrix extensions.


### Hardware Architecture

{numref}`fig-piconut_ai-ai_extension` sketches the proposed *AI Unit* and its main components.

The **Vector and Matrix Register File** contains the architectural registers and allows to input and output register contents sequentially. This may be implemented by rotating the register file contents horizontally. In accordance with the RISC-V Vector extension, the number of registers is 32, and the number of elements per register is configurable at synthesis time.

A custom number of **Compute** units contain arithmetic pipelines for all arithmetic operations at a computation width of 32 bits. During the execution of a RISC-V vector operation, vector elements are fed into the compute pipelines and pipeline results are written back into the register file.

Memory accesses are coordinated and performed by the **Vector Load & Store Unit**. It autonomously issues sequences of read or write accesses to vectors of arbitrary types (element widths) using a specific addressing mode (sequential, strided, or scatter/gather). Merging of multiple low-width accesses is also handled by the Vector Load & Store Unit. For example, it can replace up to 4 byte accesses by a single word access, if the data bus is 32 bits wide.

The **Dispatch Unit** communicates with the base Nucleus, expands vector and matrix operations into their element-oriented steps and coordinates the execution of vector/matrix instructions.

```{figure} figures/piconut_ai-ai_extension.svg
:name: fig-piconut_ai-ai_extension
*PicoNut AI* - AI Extension
```



### Soft Hardware Implementation

The present Soft Hardware implementation implements *Zve32x* (with few exceptions - see below), including a configurable number of vector elements (VLEN) and support for all required vector CSRs.

Load and store instructions for element widths of 8, 16 and 32 bits are fully supported with unit-stride, strided and indexed addressing with and without masking.

Deviations from the *Zve32x* specification are:

- Some special load/store operations are not implemented (yet), since they can be replaced by other load/store operations:
     - vector load/store segment instructions
     - unit-stride whole register load/store
     - unit-stride mask load/store, EEW=8
     - unit-stride fault-only-first load

- Fixed-point arithmetic operations are not implemented yet. In our experiments described below, we used standard integer operations for fixed point arithmetics including correct rounding. The fixed-point operations specified by *Zve32x* would add automatic rounding and saturation and may improve performance in some cases at the expense of additional hardware costs.

- The AI unit does not raise exceptions and does not contain any logic related to exceptions.


### Timing of the Soft Hardware Model

The timing of the soft hardware model anticipates the architecture depicted in {numref}`fig-piconut_ai-ai_extension` with a configurable number of compute pipelines (`PN_CFG_PIPE_LANES`), each having a configurable latency (`PN_CFG_PIPE_LATENCY`). The configuration parameters allow to perform accurate timing simulations and to anticipate the effect of their values ahead of the development of real hardware.

Given these two parameters, the latency `L` of a normal arithmetic vector operation with `VL` vector elements of `SEW` bits each can be estimated by
```
L = ceil (VL / (PIPE_LANES * ELEN/SEW)) + PIPE_LATENCY
```
where `ELEN` is the hardware element width and thus the data width of a pipeline (usually 32 bits). The formula assumes that in each clock cycle `PIPE_LANES * ELEN/SEW` new vector elements enter a pipeline, and the single addition of `PIPE_LATENCY` reflects the delay until the last vector elements have been processed completely.

Based on this formula, the effective delay `T` of RVV instructions is calculated as follows:

1. For vector-to-vector operations `T = ceil (VL / (PIPE_LANES * ELEN/SEW))` is assumed (leaving out `PIPE_LATENCY`), since a follow-up operation potentially reading the result vector register would read the first (oldest) vector elements first, and the operation can be started before the last computations leave the pipeline. Assuming that the number of vector elements is usually larger than the pipeline latency, this is a realistic assumption.
2. For instructions with a scalar destination register (`x0`..`x31`), the pipeline latency (`PIPE_LATENCY`) is added to `T` (i.e. `T = L`) in order to reflect an eventual data dependency between this scalar register and other vector or scalar instructions reading the register afterwards.
3. The timing of vector load and store instructions is mainly determined by the protocol of the data port (DPort). The protocol is simulated accurately including its timing. The Vector Load & Store unit performs automatic merging of subsequent operations without reordering and supports port pipelining.

Inside the compute pipelines there are typically no data dependencies, since in a vector processor, different stages usually process different vector elements for the same vector instruction.



(sec:ai-lenet5)=
# 5. Case Study: LeNet-5

## The LeNet-5 Architecture

LeNet-5 is a classic convolutional neural network (CNN) architecture designed by Yann LeCun in the 1990s for handwritten digit recognition [11]. It was one of the first successful applications of CNNs and laid the foundation for modern deep learning in computer vision.

LeNet-5 consists of 7 layers (excluding the input layer), with 2 convolutional layers, 2 subsampling (pooling) layers, and 3 fully connected (dense) layers (see {numref}`fig-lenet5-architecture`). The input is a 32x32 grayscale image (e.g. a hand-written digit), and the output is a probability distribution over 10 classes (e.g. digits 0-9). Each convolutional or fully connected layer besides the output layer is followed by an activation layer. In the implementation used for this work, the *Rectified Linear Unit (ReLU)* function is used as the activation function, and *maximum pooling* is used for the pooling layers.

LeNet-5 is a well-known, yet simple network which on the other hand contains all relevant operations found in modern, powerful models for computer vision tasks. Its simplicity is beneficial both in research on hardware architectures and in education. For example, a complete training with an accuracy of 98% or better can be performed using the AI Playground in a Jupyter Notebook on a normal office PC without a GPU within less than one minute. A complete inference run on a simulated PicoNut system also requires just a few seconds.

```{figure} figures/lenet5-architecture.svg
:name: fig-lenet5-architecture
The LeNet-5 Model Architecture
```



## RVV Implementations of Neural Network Operations

For illustration and evaluation purposes, a number of typical neural network layers have been implemented in RISC-V assembler code using the RISC-V Vector extension (RVV). The code examples show how certain features of the RVV can be used to optimize performance.


### Rectified Linear Unit (ReLU)

{numref}`fig-rvv-relu-implementation` shows how the ReLU function on a large array can be computed efficiently using the RISC-V Vector extension (RVV). The function can operate on an array of arbitrary size and independent from the physical vector register width (`VLEN`).

The `vsetvli` instruction at the beginning of the main loop (after label `.l010:`) combines `LMUL=8` vector registers with 32 bit elements (`e32, m8`) and returns the effective number of elements in `t0`. `a1` contains the remaining number of elements to be processed and is passed to `vsetvli` as the desired number of elements. This way, the total array is divided into chunks of maximum size.

The highlighted code processes a single chunk. The `vle32.v` instruction (vector load) loads the chunk into the register group `v8..v15`. The `vmslt.vx` instruction (vector set-less-than signed with scalar operand) compares all elements with the scalar register `zero` (`x0`) and writes the result (0 or 1) into a bit-organized mask register `v0`. The third instruction `vse32.v` (vector store) makes use of the mask option and writes zeros into memory (only) for all elements which have been less than zero in `v8..v15`.

```{figure} figures/rvv-relu-implementation.svg
:name: fig-rvv-relu-implementation
Computing *ReLU* Activation
```


### Fully Connected Layers

To compute a fully connected layer with `a1` input elements and `a3` output elements, a weight matrix of dimension `a1`x`a3` is required. Each individual input is multiplied with each element of a matrix line. All products referring to one column of the matrix need to be summed up to get one element of the output vector. An optional bias vector may be added to the output as well.

In (scalar) software, the vector-matrix multiplication can  be implemented by two nested loops passing all columns and rows of the matrix. The loop body would read an input element and a matrix element and accumulate the product to an output element, which requires to read *and* write back the new value to the output vector. The performance of such a software would be affected by the memory access behaviour:

- Each input element would be read multiple (`a3`) times.
- Each matrix element would be read exactly once, which is ideal. However, for good cache performance, the matrix should be accessed line-by-line, not column-by-column.
- Each output element would be read and written multiple (`a1`) times. Particularly the write accesses can be challenging for the memory subsystem. If the output elements are computed one after the other (using a processor register for temporary storage and just a single write in the end), the matrix would have to be traversed column-by-column, which is not ideal.

The vector implementation shown in {numref}`fig-rvv-fully_connected-implementation` uses a vector register for accumulating the outputs to overcome eventual performance issues related to writing back intermediate results. Instead of two nested loop, the code has just a single main loop iterating over the lines of the matrix. In each iteration, a complete line is processed and intermediate results are accumulated in the output vector register (`v4..v7`). In this implementation, each input element, each matrix element and each bias element is read exactly once. The output is written exactly once as well. All data memory accesses have consecutive addresses to optimize cache performance. Hence, this implementation exhibits an optimal memory access behaviour, which cannot be improved any further.

Due to a missing second loop in this implementation, the number of output elements and width of the matrix is limited by the vector length *VLEN* and the data element width *VEW*. With 16 bit elements and a vector length of 1024 bits, the maximum number of possible outputs is *LMUL x VLEN/VEW* = 8 * 1024/16 = 512. In LeNet-5, all fully connected layers have less than 128 outputs, and the code in {numref}`fig-rvv-fully_connected-implementation` is optimized for this (i.e. *LMUL=4*, which is sufficient even for a 32 bit data type).

```{figure} figures/rvv-fully_connected-implementation.svg
:name: fig-rvv-fully_connected-implementation
Computing a Fully Connected Layer
```


### Convolutional Layers

In (scalar) software implementations, image convolutions are particularly challenging because of their memory access behaviour. For each image pixel location *(x, y)*, a rectangular window of pixels *p(x-D/2, y-D/2)* ... *p(x+D/2, y+D/2)* of size *D x D* in the input image needs to be traversed and multiplied with weights (see {numref}`fig-lenet5-architecture`). *D* is the kernel dimension. This leads to a large number of repeated memory accesses to the image and weight data. In particular, each image pixel is read *D^2* times, and each weight element is read *X x Y* times, where *X* and *Y* denote the (output) image dimensions.

Dedicated hardware accelerators can greatly accelerate image convolutions by implementing a *sliding window buffer* or a dedicated memory/datapath architecture like a *2D window pipeline* [13]. These are based on special memories implemented by registers and embedded RAM that store *D* lines of an image and can shift their content by one each time a new pixel position *(x, y)* is processed.

As part of this work, a software implementation of the *sliding window buffer* technique could successfully be implemented for both convolutional layers of LeNet-5 with the help of the RVV instruction set. The principle is sketched in {numref}`fig-rvv-convolutional-implementation-1` for a 3x3 kernel and 4 output channels for simplicity. In the figure, the register `v16` is used to store a sliding window buffer. In each pixel iteration, the whole content is shifted, and a new pixel from the input is inserted at the first (upper left) position. This way, the green elements contain a window of inputs, and this window moves over the input. Using a vector gather operation (`vvrgatherei16.vv`), the window is copied into another vector register `v8` with 9 (=3x3) elements. The kernel weights are stored permanently in `v24`, and the multiplication of the window elements with the kernel weights can be performed by a simple vector multiplication (`vmul.vv`).

```{figure} figures/rvv-convolutional-implementation-1.svg
:name: fig-rvv-convolutional-implementation-1
Computing a Convolutional Layer: Sliding Window Buffer (3x3 window, one input channel, 4 output channels)
```

For the accumulation, a tree-like reduction can be applied as shown in {numref}`fig-rvv-convolutional-implementation-2`. All output channels (4 in the example, 6 or 16 in LeNet-5) can be processed in parallel. In addition, groups of product vectors can be joined in physical vector registers and added in parallel as well.

```{figure} figures/rvv-convolutional-implementation-2.svg
:name: fig-rvv-convolutional-implementation-2
Computing a Convolutional Layer: Accumulation (3x3 window, one input channel, 4 output channels)
```

The code (`sw/apps/ai/lenet5/lenet5_asm.S`) contains implementations for both convolutional layers *Conv-1* and *Conv-2*, each implemented for 32 bit (*32f8*) and for 16 bit (*16f4*) fixed point data types. The *16f4* implementations of both layers and the *32f8* implementation of *Conv-1* access the input, weights, bias and output memories exactly once per element and thus achieve the theoretical optimum for the data accesses. The *32f8* implementation of *Conv-2* needs to read all inputs twice to work with the available vector registers with (*VLEN=1024*).


### Max Pooling

By a pooling operation, groups of 4 neighbouring pixels in a 2D array are processed to form an output value - in this case by applying a maximum operation. To exploit parallelism, all upper left / upper right / lower left / lower right pixels of a group are collected in a dedicated vector register, respectively (`v4` / `v5` / `v6` / `v7` in {numref}`fig-rvv-max_pooling-implementation`).

To this end, the vector unit requires vector permutations instructions. Here, the RISC-V `vcompress.vm` instruction is used with precomputed masks.

{numref}`fig-rvv-max_pooling-implementation` shows the relevant code and data flow of a max pooling implementation.

```{figure} figures/rvv-max_pooling-implementation.svg
:name: fig-rvv-max_pooling-implementation
Computing Max Pooling
```



(sec:ai-results)=
# 6. Experimental Results

*The data provided in this chapter is preliminary and subject to change. It may contain errors, inaccuracies, or incomplete information. The experiments are based on the code as of 2026-04-01 in git branch `dev_piconut_ai`.*


## Experimental Setup

For the experiments, LeNet-5 has been trained using the [AI Playground](#sec:ai-playground) and the MNIST dataset [12]. All parameters are stored in a *16f8* fixed point format (16 bits in total, 8 fractional bits). 

The trained model parameters are included in the source code. Its accuracy has been evaluated using 10000 MNIST test images with a reference implementation in C for the data types *32f8* and *16f4*, which are used for all model parameters and internal features. For both data types, the measured accuracy is 99.01% (for validation, run: `make -C sw/apps/ai/lenet5 run`).

All experiments reported in this chapter have been carried out with our LeNet-5 implementation (`sw/apps/ai/lenet5`) and the soft hardware implementation of the *PicoNut AI* CPU (`hw/cpu/nucleus/nucleus_ai` and `hw/cpu/membrana/membrana_ai`). Unless stated otherwise, the configuration equals the defaults as of 2026-04-01 in branch `dev_piconut_ai`. In particular:

* All *IPort* and *DPort* interfaces have a width of 32 Bits.
* The RISC-V M Extension is present and used.
* The Nucleus has a dedicated vector DPort (`PN_CFG_NUCLEUS_AI_USE_VPORT = 1`).
* Both the Nucleus and the Membrana support and use port pipelining (`PN_CFG_NUCLEUS_USE_PORT_PIPELINING = 1`).
* The RVV vector length *RVV* is 1024 (`PN_CFG_VLEN = 1024`).
* The latency of the AI unit's computational pipeline(s) is 10 (`PN_CFG_PIPE_LATENCY = 10`).

In summary, the RISC-V ISA specification is *RV32IMZve32xZvl1024bZicsr*.

{numref}`fig-lenet5-properties` summarizes the properties of the LeNet-5 model. The upper part shows the kernel dimensions of the convolutional layers, the number of input and output channels, and - for the 2D layers - their input and output dimensions. The lower part summarizes the total number of input, weight, bias and output values.

```{figure} figures/lenet5-properties.svg
:name: fig-lenet5-properties
LeNet-5: Model Properties
```

The performance of scalar software (CPU) implementations are frequently limited by two effects:

* The *memory bottleneck*, i.e. the performance is limited by memory accesses, not computations.
* The *instruction decode bottleneck*, i.e. a CPU spends much effort and memory bandwidth to fetch and decode instructions.

Hence, major questions to be answered by the following experiments are:

1. What is the performance gain possible with the *PicoNut AI* architecture?
2. Is the proposed *PicoNut AI* architecture suitable to overcome the memory bottleneck?
3. Is the proposed *PicoNut AI* architecture suitable to overcome the instruction decode bottleneck?


## LeNet-5 Model Characteristics

{numref}`fig-lenet5-characteristics` shows some general characteristics of the LeNet-5 inference with respect to (required) data memory accesses and computations.

The section "Memory Accesses" shows the number of memory element accesses referring to input, output, weight or bias data, respectively. Independent of the data type, each element access is counted once. Other memory accesses, such as instruction fetches or accesses to local variables are not counted.

The "Scalar" numbers reflect the number of such element accesses issued by an optimized software implementation without vectorization. The line "Ideal" shows the theoretically minimum number of accesses assuming that each input and model value is read exactly once (i.e. the processor has the capability to somehow remember all values if they are needed again later) and each output is written exactly once.

For the scalar implementation, the majority of memory accesses is inferred by the two convolutional layers *Conv-1* and *Conv-2* (approx. 85%), which is due to the frequent accesses to the input matrix required for processing a kernel window. Looking at the ideal numbers, the convolutional layers only require approx. 16% of the memory accesses, and the dominant parts are the fully connected layers (*Full-1*, *Full-2*) which have a large number of weights.

The lines "Vector (32f8)" and "Vector (16f4)" show the number of data element accesses as inferred by our vector implementations described in section [Case Study: LeNet-5](#sec:ai-lenet5).

The numbers show that, with respect to pure data element accesses, our vector implementations are very close to the ideal values and theoretically achievable optimum! The convolutional operations (*Conv-1*, *Conv-2*) use vectors of index values for extracting a window from the sliding window buffer. The accesses to reading these index tables are included in the numbers. Due to a lack of vector registers, the *Conv-2* implementation with the *32f8* data type reads all inputs twice and requires one store and readback of the accumulator after each input channel. Hence, the number of data memory accesses is notably larger than that of its *16f4* counterpart. However, it is still orders of magnitude below the number accesses of the scalar implementation. All other vector implementations require no more than the theoretically required data memory accesses.

The section "Computation" in {numref}`fig-lenet5-characteristics` denotes the number of necessary computations. These are counted as follows:

* for the convolutional and fully connected layers (*Conv-1*, *Conv-2*, *Full-1*, *Full-2*, *Full-3*): the number of accumulations (multiply + add),
* for the activation layers (*ReLU-C1*, *ReLU-C2*, *ReLU-F1*, *ReLU-F2*): the number of comparisons,
* for the pooling layers (*Pool-1*, *Pool-2*): the number of binary maximum operations.

None of the scalar or vector implementations perform unnecessary computations, so that only the ideal numbers are given, which are equal to the actual numbers of computations in all three implementations.

```{figure} figures/lenet5-characteristics.svg
:name: fig-lenet5-characteristics
LeNet-5 Characteristics: Data Memory Accesses and Computational Operations
```



## Execution Times

For the implementations "CPU" (scalar) and "PicoNut AI" (vectorized), the execution times in terms of clock cycles have been determined by simulation using the *PicoNut AI* processor model described in section [The PicoNut AI Processor Architecture](#sec:ai-architecture). For the vectorized variant, two simulations with 1 computational lane and 4 computational lanes have been performed, respectively.

{numref}`fig-lenet5-sim_execution_times-16f4` shows the execution times if the *16f4* datatype (16 bit fixed point, 4 fractional bits) is used. The numbers are denoted relative to the execution time of the CPU implementation, which consumed 8,981,899 (approx. 9 mio) cycles for the single inference.

With just a single computation lane, a speed up of almost 10 can be achieved. With 4 lanes, the total speedup is 25. This measurement includes all computational steps, including those that cannot be vectorized. **These numbers show that there is a high potential for acceleration AI applications using the proposed processor architecture and the RISC-V Vector extension.**

```{figure} figures/lenet5-sim_execution_times-16f4.svg
:name: fig-lenet5-sim_execution_times-16f4
Execution Times (LeNet-5, 16 bit fixed-point)
```

{numref}`fig-lenet5-sim_execution_times-16f4-details` details the distribution of clock cycles over the algorithmic stages. The numbers confirm that the CPU times are dominated by the convolutional layers, followed by the fully connected layers.

```{figure} figures/lenet5-sim_execution_times-16f4-details.svg
:name: fig-lenet5-sim_execution_times-16f4-details
Execution Times per Stage (LeNet-5, 16 bit fixed-point)
```

{numref}`fig-lenet5-sim_execution_times-32f8` shows the execution times if the *32f4* datatype (16 bit fixed point, 4 fractional bits) is used. While there is still a considerable speedup by using vectorized code and the AI unit, the relative speedup of the accelerated runs is less compared to the *16f4* numbers ({numref}`fig-lenet5-sim_execution_times-16f4`). This can be explained by two effects:

1. More data needs to be transferred (32 instead of 16 bits per data element).
2. The CPU implementation is slightly faster in the *32f8* case (8,423,035 opposed to 8,981,899 clock cycles), since 32 bits is the CPU's native data and register width, and the *16f4* implementation may need extra instructions for data format conversions to and from 16 bit values.

Obviously, the vector implementation benefits from a small-width data format, whereas the CPU implementation does not. This can be explained by the instruction decode overhead, which will be analyzed in an extra section below.

```{figure} figures/lenet5-sim_execution_times-32f8.svg
:name: fig-lenet5-sim_execution_times-32f8
Execution Times (LeNet-5, 32 bit fixed-point)
```


## Data Memory Accesses

The numbers in {numref}`fig-lenet5-characteristics` already indicated that with the RISC-V Vector instruction set, the number of data element accesses can be reduced considerably and close to their theoretical optimum.

{numref}`fig-lenet5-memory_data_accesses-elements_counted` and {numref}`fig-lenet5-memory_data_accesses-elements_counted-details` visualize the same numbers graphically. The diagrams show that vectorized software (*PicoNut AI*) can reduce the number of element accesses considerably almost down to their theoretical optimum. {numref}`fig-lenet5-memory_data_accesses-elements_counted-details` shows that with the CPU implementation, the majority of accesses is caused by the convolutional layers and that this is no longer the case thanks to the *sliding window buffer* approach with the vector implementation.

```{figure} figures/lenet5-memory_data_accesses-elements_counted.svg
:name: fig-lenet5-memory_data_accesses-elements_counted
Memory: Data Element Accesses (LeNet-5, relative to optimized CPU implementation)
```

```{figure} figures/lenet5-memory_data_accesses-elements_counted-details.svg
:name: fig-lenet5-memory_data_accesses-elements_counted-details
Memory: Data Element Accesses per Stage (LeNet-5)
```

To validate the real memory access performance and to take *all* data accesses into account - including, for example, accesses to the execution stack or local variable - a series of experiments was performed counting the true number of accesses over the 32 bit wide *DPorts* (scalar and vector).

The results are shown in {numref}`fig-lenet5-memory_data_accesses-simulated_32bit`. As a reference, the ideal minimum number of element accesses is drawn in the last line ("Minimum Elements"), all other numbers are true data memory accesses.

The numbers reveal that with the *16f4* data type fewer data port transactions are required, which is due to the fact that subsequent 16 bit accesses can sometimes be merged to a single 32 bit access by the processor hardware. This effect tends to be stronger for the vector implementation.

```{figure} figures/lenet5-memory_data_accesses-simulated_32bit.svg
:name: fig-lenet5-memory_data_accesses-simulated_32bit
Memory: 32 Bit DPort Accesses (LeNet-5, relative to CPU/32f8)
```

**To summarize, the number of data accesses of RVV implementations executed on the proposed *PicoNut AI* architecture are very close to the ideal values and theoretically achievable optimum!**


## Instruction Counts

Another bottleneck related to CPUs is the *instruction decode* bottleneck, which means that a CPU needs to spend a considerable amount of time and memory accesses on fetching and decoding of instruction. In particular, each instruction needs to be fetched, but not every (scalar) instruction is a load or store instruction. Hence, during the execution of scalar software, the vast majority of memory accesses are instruction fetches and no data accesses.

Since the *PicoNut AI* architecture builds upon a standard CPU architecture (RISC-V) with all the benefits of software programmability, a key consideration is whether we can eliminate the instruction fetch bottleneck. If not, it will hardly be able to compete with pure AI accelerators.

{numref}`fig-lenet5-instructions-16f4` shows the total number of executed instructions of the scalar software implementation and a *PicoNut AI* vector implementation. The numbers are compared to the number of data element accesses and the number of computational instructions. 

The scalar CPU implementation is clearly limited by instruction decodings, the number is larger than the number of data accesses and computational operations. For the *PicoNut AI*, the number of instructions is both smaller than the number of data accesses and smaller than the number computational operations. **Hence, instruction fetching and decoding is no longer a bottleneck with *PicoNut AI*.**

```{figure} figures/lenet5-instructions-16f4.svg
:name: fig-lenet5-instructions-16f4
Executed Instructions (LeNet-5, in relation to required memory data element accesses and computational operations)
```

{numref}`fig-lenet5-instructions-16f4-details` and {numref}`fig-lenet5-instructions-16f4-details-log` show these numbers for the individual stages. The relation between the number of instructions, data accesses and computations varies depending on the actual stage, its underlying algorithm and array sizes. However, two observations hold for all stages:

1. For the CPU implementation, the instruction count is always above the number of data accesses and computations. Hence, the instruction fetch and decode process is a major bottleneck for scalar implementations.
2. For the *PicoNut AI* implementation, the number of data accesses *or* the number of computations is considerably larger than the number of instructions (sometimes both). Hence, all stages are either memory-limited (e.g. pooling or ReLU) or computation-limited (e.g. convolutional layers), but never instruction-limited.

{numref}`fig-lenet5-memory_data_accesses-simulated_32bit`
```{figure} figures/lenet5-instructions-16f4-details.svg
:name: fig-lenet5-instructions-16f4-details
Executed Instructions per Stage (LeNet-5)
```

```{figure} figures/lenet5-instructions-16f4-details-log.svg
:name: fig-lenet5-instructions-16f4-details-log
Executed Instructions per Stage (LeNet-5, Logarithmic Scale)
```



# 7. Conclusion

The *PicoNut AI* project aims to become a toolkit for customer-specific AI systems targeting low-power applications (Embedded & Edge AI).

To develop and evaluate AI modules using Python and *PyTorch*, the *AI Playground* has been developed. It provides functionalities to implement, train and evaluate neural networks and export model data as C code without any library dependencies.

The *PicoNut AI* processor architecture has been implemented as a soft hardware model and allows to perform cycle-accurate simulations at very high simulation performance.

Experimental results are very promising. A speedup of 25 for LeNet-5 inference computations can be achieved, memory data accesses and the number of executed instructions can be reduced considerably, so that the *memory bottleneck* and the *instruction decode bottleneck* are basically eliminated.

By sticking to the RISC-V instruction set, the RISC-V toolchain can be used for developing at the C code level. In the future, libraries and compilers for the efficient inference of neural networks can be leveraged.



# References

[1] RISC-V: [The RISC-V Instruction Set Manual - Volume I: Unprivileged Architecture](https://riscv.org/specifications/ratified), Version 20250508

[2] Michael Platzer,  Peter Puschner: ["Vicuna: A Timing-Predictable RISC-V Vector Coprocessor for Scalable Parallel Computation"](https://github.com/vproc/vicuna), 33rd Euromicro Conference on Real-Time Systems (ECRTS 2021)

[3] Matteo Perotti et al.: ["A 'New Ara' for Vector Computing: An Open Source Highly Efficient RISC-V V 1.0 Vector Processor Design"](https://arxiv.org/pdf/2210.08882v2), 2022 IEEE 33rd International Conference on Application-specific Systems, Architectures and Processors (ASAP), DOI: 10.1109/ASAP54787.2022.00017

[4] V. N. Chander and K. Varghese: ["A Soft RISC-V Vector Processor for Edge-AI"](https://doi.org/10.1109/VLSID2022.2022.00058), 2022 35th International Conference on VLSI Design and 2022 21st International Conference on Embedded Systems (VLSID), Bangalore, India, 2022, pp. 263-268, doi: 10.1109/VLSID2022.2022.00058

[5] Patrick Schmidt et al.: ["RVVe: A Minimal RISC-V Vector Processor for Embedded AI Acceleration"](https://ieeexplore.ieee.org/document/10737723), 2024 IEEE 37th International System-on-Chip Conference (SOCC), DOI:10.1109/SOCC62300.2024.10737723

[6] Philipp van Kempen, Jefferson Parker Jones, Daniel Mueller-Gritschneder, Ulf Schlichtmann: ["muRISCV-NN: Challenging Zve32x Autovectorization with TinyML Inference Library for RISC-V Vector Extension"](https://dl.acm.org/doi/10.1145/3637543.3652878), CF '24 Companion: Proceedings of the 21st ACM International Conference on Computing Frontiers: Workshops and Special Sessions, 2024

[7] muRISCV-NN, [https://github.com/tum-ei-eda/muriscv-nn](https://github.com/tum-ei-eda/muriscv-nn)

[8] CMSIS NN, [https://arm-software.github.io/CMSIS-NN](https://arm-software.github.io/CMSIS-NN)

[9] TensorFlow Lite for Microcontrollers, [https://github.com/tensorflow/tflite-micro](https://github.com/tensorflow/tflite-micro)

[10] TVM - Open Machine Learning Compiler Framework, [https://github.com/apache/tvm](https://github.com/apache/tvm)

[11] Yann LeCun, Léon Bottou, Yoshua Bengio, Patrick Haffner: ["Gradient-Based Learning Applied to Document Recognition"](https://doi.org/10.1109/5.726791), in Proceedings of the IEEE, vol. 86, no. 11, pp. 2278-2324, Nov. 1998, DOI: 10.1109/5.726791

[12] Yann LeCun, Corinna Cortez, Christopher C.J. Burges: ["The MNIST Handwritten Digit Database"](https://web.archive.org/web/20200430193701/http://yann.lecun.com/exdb/mnist/). Yann LeCun's Website yann.lecun.com, archived at web.archive.org

[13] Matthias Pohl, Michael Schäferling, Gundolf Kiefer: ["An Efficient FPGA-based Hardware Framework for Natural Feature Extraction and Related Computer Vision Tasks"](https://doi.org/10.1109/FPL.2014.6927463), 24th International Conference on Field Programmable Logic and Applications (FPL), 2014, DOI: 10.1109/FPL.2014.6927463
