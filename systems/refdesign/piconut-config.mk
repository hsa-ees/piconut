#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
#                     Tristan Kundrat <tristan.kundrat@tha.de>
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


# TBD+: Make demo in simulation work with 'membrana_hw'
PN_CFG_MEMBRANA := membrana_soft
# PN_CFG_MEMBRANA := membrana_hw





################################################################################
#                                                                              #
#   Software and system-related settings                                       #
#                                                                              #
################################################################################


## RISC-V ISA and extensions ...
## Changed -march argument for gcc. Comment following lines, if M and Zmmul
## extensions are not used.
# Uncomment following line, if only using Zmmul extension
# PN_MARCH ?= rv32i_zicsr_zmmul
# Uncomment following line, if using M extension
PN_MARCH ?= rv32im_zicsr
# Uncomment following line, if using F extension:
# export PN_MARCH := rv32imf_zicsr



# Operating system ...
#~ PN_OS ?= std





################################################################################
#                                                                              #
#   Nucleus: Selection and general settings                                    #
#                                                                              #
################################################################################


# Nucleus selection ...
#   Select the Nucleus variant to be used.
#
PN_CFG_NUCLEUS := nucleus_ref


# M-Extension ...
#   Enable (1) or Disable (0) M-Extension
PN_CFG_ALU_ENABLE_M_EXTENSION ?= 1
#   Enable (1) or Disable (0) Zmmul-Extension
PN_CFG_ALU_ENABLE_ZMMUL_EXTENSION ?= 1
#   Enable (1) or Disable (0) F-Extension
#   Remember to also set:
#   export PN_MARCH := rv32imf_zicsr
#   for the compiler to actually use the extension
PN_CFG_ENABLE_F_EXTENSION ?= 0


# CPU/CSR ...
PN_CFG_CSR_ADR_WIDTH ?= 12
	# width of the address in the CSR bus.
PN_CFG_CSR_BUS_DATA_WIDTH ?= 32
  # width of the data in the CSR bus.
  # Note: Also the width of the CSR registers.




################################################################################
#                                                                              #
#   Membrana: Selection and general settings                                   #
#                                                                              #
################################################################################


# Membrana selection ...
#   Select the Membrana variant to be used.
#
# TBD: WORKAROUND: Adapt the following lines as soon as the hardware and
#      simulation demos have the same configuration.
#
ifeq ($(PN_BUILD_TECHS),sim)
  PN_CFG_MEMBRANA := membrana_soft
else
  ifeq ($(PN_BUILD_TECHS),syn)
    PN_CFG_MEMBRANA := membrana_hw
  else
    $(error At present, only *one* tech can be built for at a time.)
  endif
endif





################################################################################
#                                                                              #
#   Debugging                                                                  #
#                                                                              #
################################################################################
