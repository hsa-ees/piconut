# Building the GNU Toolchain
**Author: Lukas Bauer 2024**

## General Information

This chapter explains how to install a version of the GNU Toolchain that includes a version of
GDB (GNU Debugger) in which the text user interface (tui) is enabled.

**Prerequisites:**
* libncurses-dev to be able to compile GDB with tui
* The packages listed here: [RISC-V Collab Github](https://github.com/riscv-collab/riscv-gnu-toolchain)
* a symbolic link "python" that points to "python3"

**Information:**
* GCC Version: `12.2.0`
* Buildtime with the `-j4` option: 22 minutes
* Size required for the repository: 11 GB
* Site required for installed Toolchain: 1.4 GB
* Installs the complete "riscv-gnu-toolchain" with an multilb newlib

## Installation

1. Install libncurses (on debian):
``` bash
$ sudo apt install libncurses-dev
```

2. If the "python" command is not available on your system you need to create a symbolic
link named "python" pointing to the "python3" executable.

3. Download the source code from the GitHub repository.
```console
$ git clone https://github.com/riscv/riscv-gnu-toolchain
```

4. Change directory into the newly downloaded repository.
```console
$ cd riscv-gnu-toolchain
```

5. Checkout the Version "2023.01.04" (GCC 12.2.0) with the following command
```console
$ git checkout 2023.01.04
```

6. Configuring the toolchain with the following command (remove the \from the command):
```console
$ ./configure --prefix=/home/<username>/toolchain \
--with-multilib-generator="rv32i-ilp32--;rv32im-ilp32--;rv32ima-ilp32--"
```

```{note}
Note that the given prefix path is only a sugestion and can be changed by the user.
```

7. Compile and install the toolchain.
```console
$ make
```

```{note}
Running the `make` command builds and installs the toolchain. If writing to the
directory specified with `--prefix=` requires root privileges, these need to be
given in order to successfully run the `make` command.
```

