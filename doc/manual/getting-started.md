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
  - OSS CAD Suite (tested with 2025-07-24) ([latest release](https://github.com/YosysHQ/oss-cad-suite-build/releases/tag/2025-07-24))
  - Intel® Compiler for SystemC (tested with releases 1.25.1 and 1.6.8) ([latest release](https://github.com/intel/systemc-compiler))

Alternativly, you can use a docker image with all the necessary software preinstalled.

Navigate to the base directory of the source tree.

```console
$ cd tools/docker
$ ./docker-create-piconut-image.sh --cloud-raw --qt
```

```{note}
You can also build the image yourself with more options for which software to install.
The build process is pretty resource intensive. It is highly recommended
to have at least `~24GB` of memory as well as `~80GB` of disk space available.
For more information use:
```console
$ ./docker-create-piconut-image.sh --help
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
$ make hello
```

If everything works, you should see in the console `PicoNut/RISC-V` in ascii art style.

## Running the "Hello World" Example on ULX3S FPGA Board

```{note}
Currently the ULX3S FPGA development board is supported only.
```

1. Setup ULX3S udev rules. They are found [here](sec:apx:udev:ulx3s).

2. Plug the USB cable from your computer into the US1 USB port of the ULX3S.

```{note}
When using docker make sure to plug in the board before starting docker to make sure all devices
are mounted in the container.
```

Navigate to the base directory of the source tree.

```console
$ make hello-ulx3s
```

4. To see the printed "Hello World!" output via UART. The UART signal is
   transmitted via the FTDI chip. It has a baudrate of `115200`. The port (typical ttyUSB0)
   is available after the FPGA was flashed. To connect type:

```console
$ picocom -b 115200 /dev/ttyXXXX
```

If everything works, you should see in the console `PicoNut/RISC-V` in ascii art style.

```{note}
To reset the system press the button F1 on the board.
```
