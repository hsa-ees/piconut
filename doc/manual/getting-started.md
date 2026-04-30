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


Clone the PicoNut repository and navigate to the base directory of the source tree.

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

## Running the "Hello World" Example on the ULX3S FPGA Board

```{note}
To run the PicoNut on other FPGA boards take a look at [Supported Boards](chp:supported_boards) chapter.
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


## Further Examples

A summary of common targets and command-line variables can be viewed by running

```console
$ make help

```

To build a module navigate to the module directory:

```console
$ make TECHS=sim build      # Build simulation variant of the module
$ make TECHS=syn build      # Synthesize the module
```

To verify a module navigate to the module directory:

```console
$ make TECHS=sim verify                     # Verify module (e.g run the testbench)
$ SIM_TRACE_LEVEL=3 make TECHS=sim verify   # Enable trace file (.vcd) generation
```

Additional targets for systems:

```console
$ make TECHS=sim sim            # Run system simulation
$ make TECHS=syn program        # Program system to a FPGA
```


More information about the build system can be found in the chapter `Project Structure and Build System`.

