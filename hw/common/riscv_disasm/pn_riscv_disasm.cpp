/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    PicuNut wrapper for the disassembler of RISC-V Spike.

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


#include "pn_riscv_disasm.h"

#include "../piconut_base.h"

#include "disasm.h"



static std::string last_insn_str;
static isa_parser_t *isa_parser = NULL;
static disassembler_t *disassembler = NULL;


// TBD: Replace this table and use respective table(s) from the Spike modules.
const char *pn_xreg_name[32] = {
  "zero",
  "ra",
  "sp",
  "gp",
  "tp",
  "t0", "t1", "t2",
  "s0", "s1",
  "a0", "a1", "a2", "a3", "a4", "a5", "a6", "a7",
  "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9", "s10", "s11",
  "t3", "t4", "t5", "t6"
};


const char *pn_get_x_reg_name (int reg) {
  PN_ASSERT(reg >= 0 && reg < 32);
  return pn_xreg_name[reg];
}


const char *pn_disassemble (uint32_t _insn) {
  insn_t insn (_insn);

  // Create disassembler, if it does not exist yet ...
  if (!disassembler) {
    PN_FREEO(isa_parser);
    isa_parser = new isa_parser_t ("rv32imv_zvl1024b_zicntr_zihpm", "msu");
      // TBD: Try "rv32gv_zvl1024b"; Make vector length configurable.
    disassembler = new disassembler_t (isa_parser, false);
      // The second parameter 'strict == false' enables opcodes not explicitly defined in the ISA parser.
  }

  // Go ahead ...
  last_insn_str = disassembler->disassemble (insn);
  return last_insn_str.c_str ();
}
