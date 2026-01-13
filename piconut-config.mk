#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#  Description:
#    Global configuration for PicoNut modules and systems.
#
#  --------------------- LICENSE -----------------------------------------------
#  Redistribution and use in source and binary forms, with or without modification,
#  are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice, this
#     list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation and/or
#     other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
#  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#  -----------------------------------------------------------------------------





################################################################################
#                                                                              #
#   Software and system-related settings                                       #
#                                                                              #
################################################################################


# RISC-V ISA and extensions ...
PN_MARCH ?= rv32i_zicsr


# Compiler and Linker toolchain ...
#   On a Debian system, select "/usr" hier
#RISCV := /usr


# General GCC specifications ...
#   On a Debian system to select picolibc, set PN_RISCV_SPECS := picolibc.specs
#RISCV_SPECS := picolibc.specs


# LibC specification ...
#   If set, explizit search paths for headers and libraries are added to PN_SW_CFLAGS
#   and PN_SW_LDFLAGS, respectively. On Debian 13, the following setting
#   adds search paths for PicoLibC (package 'picolibc-riscv64-unknown-elf').
#RISCV_LIBC := /usr/lib/picolibc/riscv64-unknown-elf


# Operating system ...
PN_SW_OS ?= std


# System memory layout ...
#   TBD: Clarify the variables and memory layout:
#        - Where exactly are read-only and uninitialized writeable sections are located?
#        - Where exactly is ROM and RAM to be placed?
PN_CFG_SYS_CODE_SIZE ?= 0x0001FFFF
  # Size of the code section in the linker script.
PN_CFG_SYS_RAM_SIZE ?= 0x0001FFFF
  # Size of the ram section in the linker script.
PN_CFG_SYS_STACK_SIZE ?= 0x00001000
  # Size of the stack section in the linker script.
  # Note: stack size + heap size must be less than ram size.
PN_CFG_SYS_HEAP_SIZE ?= 0x00001000
  # Size of the heap section in the linker script.
  # Note: stack size + heap size must be less than ram size.





################################################################################
#                                                                              #
#   Nucleus: Selection and general settings                                    #
#                                                                              #
################################################################################


# Nucleus selection ...
#   Select the Nucleus variant to be used. This setting must match an existing
#   directory name inside 'hw/cpu/nucleus'.
#   To allow variant-depending code, a C preprocessor symbol with the name
#   PN_CFG_NUCLEUS_IS_<name> is defined automatically, where <name>
#   is PN_CFG_NUCLEUS in uppercase letters. For example, if PN_CFG_NUCLEUS = my_nucleus,
#   differentiating code can be written as follows:
#
#     #ifdef PN_CFG_NUCLEUS_IS_MY_NUCLEUS
#     do_something_if_my_nucleus_is_the_selected_nucleus ();
#     #endif
#
PN_CFG_NUCLEUS ?= nucleus_ref


# CPU/Nucleus options ...
PN_CFG_CPU_CORES ?= 1
	# Number of cores
PN_CFG_CPU_REGFILE_SIZE ?= 32
  # Number of registers in the regfile (set to 16 for the RV32E ISA).

PN_CFG_CPU_RESET_ADR ?= 0x10000000
  # Reset vector





################################################################################
#                                                                              #
#   Membrana: Selection and general settings                                   #
#                                                                              #
################################################################################


# Membrana selection ...
#   Select the Membrana variant to be used. This setting must match an existing
#   directory name inside 'hw/cpu/mebrana' (TBD: was 'hw/piconut/memus').
#   To allow variant-depending code, a C preprocessor symbol with the name
#   PN_CFG_MEMBRANA_IS_<name> is defined automatically, where <name>
#   is PN_CFG_MEMBRANA in uppercase letters. For example, if PN_CFG_MEMBRANA = my_membrana,
#   differentiating code can be written as follows:
#
#     #ifdef PN_CFG_MEMBRANA_IS_MY_MEMBRANA
#     do_something_if_my_membrana_is_the_selected_membrana ();
#     #endif
#
#~ PN_CFG_MEMBRANA ?= membrana_ref
PN_CFG_MEMBRANA ?= membrana_soft
	# TBD: Switch to a synthesizable Membrana (e.g. 'membrana_ref') as the default.


# Internal MEMBRANA memory ...
#   Note: The following settings currently cannot be moved to .h files in the
#         Membrana module(s) (currently: `membrana_hw`), since they are accessed
#         in non-C/C++ code.
PN_CFG_MEMBRANA_EMEM_SIZE ?= 0x40000
  # @brief Size of the local memory inside the Membrana in bytes.
  #
  # The default of 0x40000 corresponds to 256K bytes.


PN_CFG_MEMBRANA_EMEM_BASE ?= 0x10000000
  # @brief Start address of the embedded memory inside the Membrana.





################################################################################
#                                                                              #
#   Debugging                                                                  #
#                                                                              #
################################################################################


# Debugger options ...
#  TBD: Move to debug or CPU module (origin: hw/piconut/config.mk)

# Sets the base address of the DEBUGGER module in address space
# This is the base address where the module is located and can be accessed.
# Note: This valu is fixed according to the `External Debug Support` standard
# and should not be changed.
PN_CFG_DEBUG_DEBUGGER_BASE_ADDRESS ?= 0x00000000

# Start address of the debug handler.
PN_CFG_DEBUG_DEBUG_HANDLER_START_ADDRESS ?= 0x10000004

# Port to which openocd has to connect.
# Note: The config for openocd is found at tools/etc/openocd-*.cfg
PN_CFG_DEBUG_OPENOCD_PORT ?= 9824

# Width address signal of the DMI-bus.
# Note: The DMI-bus is betweem the Debug Transport Module (DTM) and
# one or multiple Debug Modules (DM).
PN_CFG_DEBUG_DMI_BUS_ADR_WIDTH ?= 6





################################################################################
#                                                                              #
#   Board selection and configuration                                          #
#                                                                              #
################################################################################


# The following settings are only required for RTL synthesis (after ICSC).

# Target board ...
#   This setting is also exported to the source code (prefix PN_CFG_*) to make
#   it available there.
PN_CFG_BOARD ?= generic


# Board and system-specific constraints file ...
#   Each board definition comes with a default contraint file, so that the
#   following parameter does not have to be set here. However, systems will
#   often come with their own contraints file, overriding this.
#PN_BOARD_CONSTRAINTS ?=
