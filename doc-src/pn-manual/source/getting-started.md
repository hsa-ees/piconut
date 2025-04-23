# Getting Started
**Author: Johannes Hofmann 2025**

This chapter provides a quick introduction for working with the PicoNut project
-- both in simulation and on real hardware. It outlines the required tools and
guides you through running a basic “Hello World” example.

## Prerequisites

For simulation of the PicoNut using its SystemC model:
  - SystemC 2.3 or later ([latest release](https://www.accellera.org/downloads/standards/systemc))
  - GtkWave (optional)

For software compilation:
  - RISC-V GNU Toolchain (tested with 12.2.0) ([latest release](https://github.com/sifive/freedom-tools/releases))

For synthesizing:
  - OSS CAD Suite (tested with 2024-07-16) ([latest release](https://github.com/YosysHQ/oss-cad-suite-build/releases/tag/2025-04-14))
  - Intel® Compiler for SystemC (tested with releases 1.25.1 and 1.6.8) ([latest release](https://github.com/intel/systemc-compiler))


Alternatively, you can build a Docker image created for the PicoNut project,
which includes all the necessary software preinstalled.

```{note}
The build process is pretty resource intensive. It is highly recommended
to have at least `~24Gb` of memory as well as `~80Gb` of disk space available.
There is a minimal version available with the option `-m`. It takes less time to build and as well as less disk space. 
```

Navigate to the base directory of the source tree.

```console
$ cd tools/docker
$ ./docker-create-piconut-image.sh
```

To run the examples, a shell in the container can be started as follows:

```console
$ docker compose up -d
$ docker exec -it piconut_container bash
```

```{note}
Older versions of Docker Compose may require to write `docker-compose` instead of `docker compose`.
```

## Running the "Hello World" Example in Simulation

Navigate to the base directory of the source tree.

```console
$ cd sw/applications/hello_newlib
$ make sim
```

If everything works, console should show the output of a simple hello world program.

## Running the "Hello World" Example on ULX3S FPGA Board

```{note}
Currently the ULX3S FPGA development board is supported only.
```

1. Setup ULX3S udev rules. They are found [here](sec:apx:udev:ulx3s).

2. Plug the USB cable from your computer into the US1 USB port of the ULX3S.

3. To flash you need to specify a system that is synthesizable for example
   refdesign_hw. To use other systems change the PN_SYSTEM variable:

Navigate to the base directory of the source tree.

```console
$ cd sw/applications/hello_newlib
$ PN_SYSTEM=refdesign_hw make flash
```

4. To see the printed "Hello World!" output via UART. The UART signal is
   transmitted via the FTDI chip. It has a baudrate of `115200`. The port (typical ttyUSB0)
   is available after the FPGA was flashed. To connect type:

```console
$ picocom -b 115200 /dev/ttyXXXX
```

If everything works, console should show the output of a simple hello world program.

```{note}
To reset the system press the button F1 on the board.
```
