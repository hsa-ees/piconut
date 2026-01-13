/*******************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Niklas Sirch <niklas.sirch1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

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

#ifndef __TIMER_DEFS_H__
#define __TIMER_DEFS_H__

/************************ Configuration Options *******************************/

#ifndef PN_CFG_TIMER_BASE_ADDRESS
/** @brief Sets the base address of the TIMER module in address space
 *  This is the base address where the module is located and can be accessed.
 */
#define PN_CFG_TIMER_BASE_ADDRESS 0x60000000U
#endif

/**
 * @brief TIMER Register Offsets
 * STM32 TIM2 Compatible Register Layout
 */
enum e_timer_regs
{
    M_TIMER_REG_CR1 = 0x00,   // RW - Control Register 1
    M_TIMER_REG_CR2 = 0x04,   // RW - Control Register 2 (dummy)
    M_TIMER_REG_SMCR = 0x08,  // RW - Slave Mode Control Register (dummy)
    M_TIMER_REG_DIER = 0x0C,  // RW - Interrupt Enable Register
    M_TIMER_REG_SR = 0x10,    // RW - Status Register
    M_TIMER_REG_EGR = 0x14,   // WO - Event Generation Register (dummy)
    M_TIMER_REG_CCMR1 = 0x18, // RW - Capture/Compare Mode Register 1 (dummy)
    M_TIMER_REG_CCMR2 = 0x1C, // RW - Capture/Compare Mode Register 2 (dummy)
    M_TIMER_REG_CCER = 0x20,  // RW - Capture/Compare Enable Register (dummy)
    M_TIMER_REG_CNT = 0x24,   // RW - Counter Register
    M_TIMER_REG_PSC = 0x28,   // RW - Prescaler Register
    M_TIMER_REG_ARR = 0x2C,   // RW - Auto-Reload Register
    M_TIMER_REG_CCR1 = 0x34,  // RW - Capture/Compare Register 1
    M_TIMER_REG_CCR2 = 0x38,  // RW - Capture/Compare Register 2
    M_TIMER_REG_CCR3 = 0x3C,  // RW - Capture/Compare Register 3
    M_TIMER_REG_CCR4 = 0x40,  // RW - Capture/Compare Register 4
    M_TIMER_REG_DCR = 0x48,   // RW - DMA Control Register (dummy)
    M_TIMER_REG_DMAR = 0x4C   // RW - DMA Address Register (dummy)
};

#endif