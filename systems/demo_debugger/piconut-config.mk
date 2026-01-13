
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
#~ PN_MARCH ?= rv32i


# Operating system ...
#~ PN_OS ?= std





################################################################################
#                                                                              #
#   Nucleus: Selection and general settings                                    #
#                                                                              #
################################################################################


# Nucleus selection ...
PN_CFG_NUCLEUS := nucleus_ref




################################################################################
#                                                                              #
#   Membrana: Selection and general settings                                   #
#                                                                              #
################################################################################


# Membrana selection ...
PN_CFG_MEMBRANA := membrana_hw




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
#~ PN_CFG_DEBUG_DEBUGGER_BASE_ADDRESS ?= 0x00000000

# Start address of the debug handler.
#~ PN_CFG_DEBUG_DEBUG_HANDLER_START_ADDRESS ?= 0x0000003c

# Width address signal of the DMI-bus.
# Note: The DMI-bus is betweem the Debug Transport Module (DTM) and
# one or multiple Debug Modules (DM).
#~ PN_CFG_DEBUG_DMI_BUS_ADR_WIDTH ?= 6
