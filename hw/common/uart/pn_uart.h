/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
  - Interface definition of the PicoNut UART
  - Small functions to implement a host-target interface in simulators (e.g. for a soft UART)

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

 *************************************************************************/


#ifndef __PN_UART_H__
#define __PN_UART_H__

#include <piconut_base.h>


/// @brief PicoNut UART I/O registers.
///
typedef struct pn_uart_regs_s {

  /// @brief 0x00: txdata - Transmit data register
  struct {
    unsigned data     :  8;
    unsigned reserved : 23;
    unsigned full     :  1;
  } txdata;
  
  /// @brief 0x04: rxdata - Receive data register
  struct {
    unsigned data     :  8;
    unsigned reserved : 23;
    unsigned empty    :  1;
  } rxdata;
  
  /// @brief 0x08: txctrl - Transmit control register
  struct {
    unsigned txen     :  1;
    unsigned nstop    :  1;
    unsigned reserved1: 14;
    unsigned txcnt    :  3;
    unsigned reserved2: 13;
  } txctrl;
  
  /// @brief 0x0c: rxctrl - Receive control register
  struct {
    unsigned rxen     :  1;
    unsigned reserved1: 15;
    unsigned rxcnt    :  3;
    unsigned reserved2: 13;
  } rxctrl;
  
  /// @brief 0x10: ie - UART interrupt enable
  struct {
    unsigned txwm     :  1;
    unsigned rxwm     :  1;
    unsigned reserved : 30;
  } ie;
      
  /// @brief 0x14: ip - UART interrupt pending
  struct {
    unsigned txwm     :  1;
    unsigned rxwm     :  1;
    unsigned reserved : 30;
  } ip;
  
  /// @brief 0x18: div - Baud rate divisor  
  struct {
    unsigned div      : 16;
    unsigned reserved : 16;
  } div;

} pn_uart_regs_t;


/// @brief Determine relative address of a UART I/O register.
#define PN_UART_OFS(REG) (offsetof(pn_uart_regs_t, pn_uart_regs_t::REG))


/// @brief Helper to check if the UART is addressed.
/// @param ofs is the relative address
static inline bool pn_uart_is_addressed (int ofs) { return ofs >= 0 && ofs < sizeof(pn_uart_regs_t); }


/// @brief Initialize a register set with useful reset defaults.
/// @param regs points to an UART register file.
void pn_uart_init (pn_uart_regs_t *regs);


/// @brief Read from UART.
/// @param regs points to an UART register file.
/// @param ofs is the relative address.
/// @param bsel selects the bytes to be read. Only those bytes with the respective `bsel` bit set are valid
///    in the return value. Other bytes may contain random values.
uint32_t pn_uart_read32 (pn_uart_regs_t *regs, int ofs, int bsel = 0xf);


/// @brief Write to UART and print transmitted bytes to the console.
/// @param regs points to an UART register file.
/// @param ofs is the relative address.
/// @param val contains the value to be written.
/// @param bsel selects the bytes to be changed in the register file.
void pn_uart_write32 (pn_uart_regs_t *regs, int ofs, uint32_t val, int bsel = 0xf);


#endif // __PN_UART_H__
