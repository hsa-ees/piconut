/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description: Host-Target emulated UART - implementation.

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


#include "pn_uart.h"

#include <strings.h>


void pn_uart_init (pn_uart_regs_t *regs) {

  // Initialize regs such that all read accesses deliver plausible data ...
  bzero (regs, sizeof (pn_uart_regs_t));
  regs->rxdata.empty = 1;

  // Static sanity ...
  PN_ASSERT(sizeof (pn_uart_regs_t) == 0x1c);               // check alignment
  PN_ASSERT(* (uint32_t *) &(regs->rxdata) == 0x80000000);  // check endianess and ordering of bit fields
}


uint32_t pn_uart_read32 (pn_uart_regs_t *regs, int ofs, int bsel) {

  // Sanity (any access must be aligned) ...
  PN_ASSERT(pn_uart_is_addressed (ofs) && (ofs & 3) == 0);

  // Return register content ...
  return ((uint32_t *) regs) [ofs];
}


void pn_uart_write32 (pn_uart_regs_t *regs, int ofs, uint32_t val, int bsel) {

  // Sanity (any access must be aligned) ...
  PN_ASSERT(pn_uart_is_addressed (ofs) && (ofs & 3) == 0);

  // Go ahead ...
  switch (ofs) {
    case PN_UART_OFS(txdata):
      // transfer: output the character ...
      putchar (val & 0xff);
      break;
    case PN_UART_OFS(rxdata):
    case PN_UART_OFS(ie):
    case PN_UART_OFS(ip):
      // read-only registers: do nothing
      break;
    default:
      // writeable register: write ...
      for (int i = 0; i < 4; i++) if (bsel & (1 << i))
        ((uint8_t *) regs) [ofs + i] = (val >> (8 * i)) & 0xff;
  }
}
