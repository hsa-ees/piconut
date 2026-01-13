/*******************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
                     Tristan Kundrat <tristan.kundrat@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    Common definitions for the audio peripheral module shared between hardware
    and software.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation and/or
     other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************/


#pragma once





/************************ Configuration Options *******************************/


#ifndef PN_CFG_AUDIO_BASE_ADDRESS
/** @brief Sets the base address of the AUDIO module in address space
 *  This is the base address where the module is located and can be accessed.
 */
#define PN_CFG_AUDIO_BASE_ADDRESS 0x50000000U
#endif


#ifndef PN_CFG_AUDIO_EXP
/** @brief Number of Audio modules n = 2^(CFG_WB_AUDIO_EXP+1)
 *  example: CFG_WB_AUDIO_EXP 3 -> n = 2^(3+1) = 16
 *  +1 because we want 2^n modules per channel (left, right)
 */
#define PN_CFG_AUDIO_EXP 2U
#endif

/**
 * @brief Number of address bytes occupied by one audio module in Wishbone address space
 */
#define PN_AUDIO_SIZE_BYTE (2U << 2)
