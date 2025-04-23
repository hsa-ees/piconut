#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C)      2024 Lukas Bauer <lukas.bauer@hs-ausgsburg.de>
#                2024-2025 Johannes Hofmann <johannes.hofmann1@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#  Description:
#    This file contains the global configuration options for the PicoNut.
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


# Unit factors (do not edit!) ...
KB = 1024
MB = 1024*1024


################# NUCLEUS ######################################################
# Number of registers in the regfile
# 32 - Default value for RV32I
# 16 - Reduced regfile size for RV32E
CFG_REGFILE_SIZE ?= 32

# CFG_MEMU_BRAM_SIZE =1024*1024*1024*1024
# CFG_MEMU_BRAM_SIZE = 102399
CFG_MEMU_BRAM_SIZE = 40960

# Starting adress of the processor. Initual value of PC.
CFG_START_ADDRESS = 0x10000000

################# CSR ##########################################################
# Width of the address in the csr bus.
CFG_CSR_BUS_ADR_WIDTH = 12

# Width of the data in the csr bus.
# Note: Also the width of the csr registers.
CFG_CSR_BUS_DATA_WIDTH = 32

################# DEBUG ########################################################
# Sets the base address of the DEBUGGER module in address space
# This is the base address where the module is located and can be accessed.
# Note: This valu is fixed according to the `External Debug Support` standard
# and should not be changed.
CFG_DEBUG_DEBUGGER_BASE_ADDRESS = 0x00000000

# Start address of the debug handler.
CFG_DEBUG_DEBUG_HANDLER_START_ADDRESS = 0x0000003c

# Port to which openocd has to connect.
# Note: The config for openocd is found at tools/etc/openocd-*.cfg
CFG_DEBUG_OPENOCD_PORT = 9824

# Width address signal of the DMI-bus.
# Note: The DMI-bus is betweem the Debug Transport Module (DTM) and
# one or multiple Debug Modules (DM).
CFG_DEBUG_DMI_BUS_ADR_WIDTH = 6

################# PERIPHERALS ##################################################
# Base address of the wb_uart instance in the system
CFG_WB_UART_BASE_ADDRESS = 0x30000000

# Disables the rx and tx fifo fo the wb_uart
CFG_WB_UART_DISABLE_FIFO = 0
