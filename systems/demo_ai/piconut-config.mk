#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#  Description:
#    System-specific configuration for PicoNut modules and systems.
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


# Need more RAM than standard ...
PN_CFG_SYS_CODE_SIZE := 0x00080000
PN_CFG_SYS_RAM_SIZE := 0x00180000

# Voice control app ...
#~ PN_CFG_SYS_CODE_SIZE  := 0x00060000 # VEXT=OFF
#~ PN_CFG_SYS_CODE_SIZE  := 0x000A0000 # VEXT=ON (not tested)
#~ PN_CFG_SYS_RAM_SIZE   := 0x01000000

# Cone detection app ...
#~ PN_CFG_SYS_CODE_SIZE  := 0x10500000 # VEXT=OFF
#~ PN_CFG_SYS_CODE_SIZE  := 0x000A0000 # VEXT=ON (not tested)
#~ PN_CFG_SYS_RAM_SIZE   := 0x10B00000

# AWW app ...
#~ PN_CFG_SYS_CODE_SIZE  := 0x00080000 # VEXT=OFF
#~ PN_CFG_SYS_CODE_SIZE  := 0x000A0000 # VEXT=ON (not tested)
#~ PN_CFG_SYS_RAM_SIZE   := 0x000A0000

# LLM app ...
#~ PN_CFG_SYS_CODE_SIZE  := 0x0A000000 # VEXT=OFF
#~ PN_CFG_SYS_CODE_SIZE  := 0x0A000000 # VEXT=ON (not tested)
#~ PN_CFG_SYS_RAM_SIZE   := 0x03000000


PN_CFG_MEMBRANA_EMEM_SIZE := ( $(PN_CFG_SYS_CODE_SIZE) + $(PN_CFG_SYS_RAM_SIZE) )




################################################################################
#                                                                              #
#   Nucleus: Selection and general settings                                    #
#                                                                              #
################################################################################


# Nucleus selection ...
#   Select the Nucleus variant to be used.
#
PN_CFG_NUCLEUS := nucleus_ai





################################################################################
#                                                                              #
#   Membrana: Selection and general settings                                   #
#                                                                              #
################################################################################


# Membrana selection ...
#   Select the Membrana variant to be used.
#
PN_CFG_MEMBRANA := membrana_ai





################################################################################
#                                                                              #
#   Debugging                                                                  #
#                                                                              #
################################################################################
