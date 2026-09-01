/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
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

 *************************************************************************/

#include "nucleus_ai.h"

#include <pn_riscv_defs.h>

#include "nucleus_ai_vext.h"





// ********************** Elaboration & Tracing ********************************


m_nucleus_ai::~m_nucleus_ai () {
  PN_FREEO(i_vext);
}


void m_nucleus_ai::init_submodules () {
  PN_FREEO(i_vext);
  if (PN_CFG_NUCLEUS_WITH_V) {
    i_vext = sc_new<m_nucleus_ai_vext>("vext");

    i_vext->clk    (clk);
    i_vext->reset  (reset);

    // CSR bus ...
    i_vext->csr_bus_read_en_in    (csr_bus_read_en_out);
    i_vext->csr_bus_write_en_in   (csr_bus_write_en_out);
    i_vext->csr_bus_write_mode_in (csr_bus_write_mode_out);
    i_vext->csr_bus_adr_in        (csr_bus_adr_out);
    i_vext->csr_bus_wdata_in      (csr_bus_wdata_out);
    i_vext->csr_bus_rdata_out     (csr_bus_rdata_in);

    // General signals ...
    i_vext->base_rdy          (vext_rdy);
    i_vext->base_idle         (vext_idle);
    i_vext->base_insn_stb     (vext_insn_stb);
    i_vext->base_insn         (vext_insn);

    i_vext->base_rs1          (vext_rs1);
    i_vext->base_rs2          (vext_rs2);
    i_vext->base_rd_pending   (vext_rd_pending);
    i_vext->base_rd_stb       (vext_rd_stb);
    i_vext->base_rd_sel       (vext_rd_sel);
    i_vext->base_rd           (vext_rd);

    i_vext->base_mem_pending  (vext_mem_pending);

    // ... DPort ...
    if (PN_CFG_NUCLEUS_AI_USE_VPORT) {
      i_vext->dport_stb   (vport_stb);
      i_vext->dport_we    (vport_we);
      i_vext->dport_ack   (vport_ack);

      i_vext->dport_adr   (vport_adr);
      i_vext->dport_bsel  (vport_bsel);
      i_vext->dport_rdata (vport_rdata);
      i_vext->dport_wdata (vport_wdata);
    }
    else {
      i_vext->dport_stb   (vext_dport_stb);
      i_vext->dport_we    (vext_dport_we);
      i_vext->dport_ack   (vext_dport_ack);

      i_vext->dport_adr   (vext_dport_adr);
      i_vext->dport_bsel  (vext_dport_bsel);
      i_vext->dport_rdata (dport_rdata);
      i_vext->dport_wdata (vext_dport_wdata);
    }
  }

  // CSR module ...
  /*
  PN_FREEO(i_csr);
  if (PN_CFG_NUCLEUS_WITH_ZICSR) {
    i_csr = sc_new<m_pn_csr>("csr");

    i_csr->clk    (clk);
    i_csr->reset  (reset);

    // ... CSR bus ...
    i_csr->csr_bus_read_en_in     (csr_bus_read_en_in);
    i_csr->csr_bus_write_en_in    (csr_bus_write_en_in);
    i_csr->csr_bus_write_mode_in  (csr_bus_write_mode_in);
    i_csr->csr_bus_adr_in         (csr_bus_adr_in);
    i_csr->csr_bus_wdata_in       (csr_bus_wdata_in);
    i_csr->csr_bus_rdata_out      (csr_bus_rdata_out);

    // ... to CSR module: program counter (PC) ...
    i_csr->pc_in                            (reg_pc);
    // ... to CSR module: external interrupts ...
    i_csr->msip_in                          (msip_in);
    i_csr->mtip_in                          (mtip_in);
    i_csr->meip_in                          (meip_in);
    i_csr->mret_in                          (csr_mret_in);
    i_csr->interrupt_in                     (csr_interrupt_in);
    // ... to CSR module: debugging ...
    i_csr->debug_level_enter_ebreak_in      (csr_debug_level_enter_ebreak_in);
    i_csr->debug_level_enter_haltrequest_in (csr_debug_level_enter_haltrequest_in);
    i_csr->debug_level_enter_step_in        (csr_debug_level_enter_step_in);
    i_csr->debug_level_leave_in             (csr_debug_level_leave_in);
    // ... from CSR module: external interrupts ...
    i_csr->mstatus_mie_out                  (csr_mstatus_mie_out);
    i_csr->mie_msie_out                     (csr_mie_msie_out);
    i_csr->mie_mtie_out                     (csr_mie_mtie_out);
    i_csr->mie_meie_out                     (csr_mie_meie_out);
    i_csr->mip_msip_out                     (csr_mip_msip_out);
    i_csr->mip_mtip_out                     (csr_mip_mtip_out);
    i_csr->mip_meip_out                     (csr_mip_meip_out);
    i_csr->mtvec_trap_address_out           (csr_mtvec_trap_address_out);
    i_csr->mepc_out                         (csr_mepc_out);
    i_csr->interrupt_pending_out            (csr_interrupt_pending_out);
    // ... from CSR module: debugging ...
    i_csr->debug_level_enter_out            (csr_debug_level_enter_out);
    i_csr->debug_step_out                   (csr_debug_step_out);
    i_csr->dpc_out                          (csr_dpc_out);
  }
  */
}


void m_nucleus_ai::pn_trace (sc_trace_file * tf, int level) {
  if (level >= 1) {

    // General ports ...
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, pwroff);

    // IPort ...
    PN_TRACE(tf, iport_adr);
    PN_TRACE(tf, iport_bsel);
    PN_TRACE(tf, iport_rdata);
    PN_TRACE(tf, iport_stb);
    PN_TRACE(tf, iport_ack);

    // DPort ...
    PN_TRACE(tf, dport_adr);
    PN_TRACE(tf, dport_bsel);
    PN_TRACE(tf, dport_rdata);
    PN_TRACE(tf, dport_wdata);
    PN_TRACE(tf, dport_stb);
    PN_TRACE(tf, dport_we);
    PN_TRACE(tf, dport_ack);
    PN_TRACE(tf, dport_lrsc);
    PN_TRACE(tf, dport_amo);

    // VPort ...
    PN_TRACE(tf, vport_adr);
    PN_TRACE(tf, vport_bsel);
    PN_TRACE(tf, vport_rdata);
    PN_TRACE(tf, vport_wdata);
    PN_TRACE(tf, vport_stb);
    PN_TRACE(tf, vport_we);
    PN_TRACE(tf, vport_ack);
    PN_TRACE(tf, vport_lrsc);
    PN_TRACE(tf, vport_amo);

    // Registers ...
    PN_TRACE(tf, reg_pc);
    PN_TRACE(tf, reg_ir);
    PN_TRACE_BUS(tf, reg_x, 32);

    // Submodules ...
    if (i_vext) i_vext->pn_trace (tf, level - 1);
  }
}





// ********************** Helpers **********************************************


static uint32_t get_immediate (uint32_t insn) {
  uint32_t ret;
  int insn_opcode = (insn & 0x7f);

  switch (insn_opcode) {

    // I-type ...
    case 0x67:  // 1100111
    case 0x13:  // 0010011
    case 0x03:  // 0000011
      ret = (uint32_t) (((int32_t) insn) >> 20);  // bits 31:20, sign-extended
      break;

    // S-type ...
    case 0x23:  // 0100011
      ret = (BIT_GROUP(insn, 31, 25) <<  5) |
            (BIT_GROUP(insn, 11,  7));
      ret = SIGN_EXTEND(ret, 11);
      break;

    // B-type ...
    case 0x63:  // 1100011
      ret = (BIT_GROUP(insn, 31, 31) << 12) |
            (BIT_GROUP(insn,  7,  7) << 11) |
            (BIT_GROUP(insn, 30, 25) <<  5) |
            (BIT_GROUP(insn, 11,  8) <<  1) |
            0;
      ret = SIGN_EXTEND(ret, 12);
      break;

    // U-type ...
    case 0x37:  // 0110111
    case 0x17:  // 0010111
      ret = insn & ~((1 << 12) - 1);    // get bits 31:12 and reset 11:0
      break;

    // J-type ...
    case 0x6f:  // 1101111
      ret = (BIT_GROUP(insn, 31, 31) << 20) |
            (BIT_GROUP(insn, 19, 12) << 12) |
            (BIT_GROUP(insn, 20, 20) << 11) |
            (BIT_GROUP(insn, 30, 21) <<  1) |
            0;
      ret = SIGN_EXTEND(ret, 20);
      break;

    // Default ...
    default:
      PN_WARNINGF(("Failed to determine immediate value: 0x%08x   %s", insn, pn_disassemble (insn)));
      ret = ~0;
  }
  return ret;
}


static inline sc_uint<PN_DPORT_BSEL_MAX> get_bsel (uint32_t adr, int ld_bytes) {
  switch (ld_bytes) {
    case 0:   // byte
      return 1 << (adr & 3);
    case 1:   // halfword
      if ((adr & 1) == 0) return 3 << (adr & 2);
    case 2:   // word
      if ((adr & 1) == 0) return 0xf;
  }

  // Misaligned access or invalid value for 'ld_bytes' ...
  return 0;
}


static inline uint32_t get_rdata_val (uint32_t adr, int ld_bytes, uint32_t rdata, bool sign_extend) {
  uint32_t ret;

  switch (ld_bytes) {
    case 0:   // byte
      ret = (rdata >> (8 * (adr & 3))) & 0xff;
      if (sign_extend) ret = SIGN_EXTEND(ret, 7);
      break;
    case 1:   // halfword
      ret = (rdata >> (8 * (adr & 2))) & 0xffff;
      if (sign_extend) ret = SIGN_EXTEND(ret, 15);
      break;
    default:   // word
      ret = rdata;
      break;
  }
  return ret;
}


static inline uint32_t get_wdata (uint32_t adr, int ld_bytes, uint32_t val) {
  uint32_t ret;

  switch (ld_bytes) {
    case 0:   // byte
      ret = val & 0xff;
      ret |= (ret << 8);
      ret |= (ret << 16);
      break;
    case 1:   // halfword
      ret = val & 0xffff;
      ret |= (ret << 16);
      break;
    default:   // word
      ret = val;
      break;
  }
  return ret;
}


void m_nucleus_ai::itrace_instruction (uint32_t pc, uint32_t insn) {
  PN_INFOF(("%08x: %08x     %s", pc, insn, pn_disassemble (insn)));
}


void m_nucleus_ai::itrace_print_regs (bool all) {
  int i;

  // Dump all registers ...
  if (all) {
    for (i = 0; i < 8; i++)
      PN_INFOF(("  x%i=0x%08x  %sx%i=0x%08x  x%i=0x%08x  x%i=0x%08x",
                i, reg_x_read (i),
                i+8 < 10 ? " " : "", i+8, reg_x_read (i+8),
                i+16, reg_x_read (i+16),
                i+24, reg_x_read (i+24)));
    for (i = 0; i < 32; i++) itrace_xreg_changed[i] = false;
    if (i_vext) i_vext->itrace_print_vregs (true);
  }

  // Dump changed register only ...
  else {
    for (i = 1; i < 32; i++) if (itrace_xreg_changed[i]) {
      PN_INFOF(("  %s/x%i = 0x%08x", pn_get_x_reg_name(i), i, reg_x_read (i)));
      itrace_xreg_changed[i] = false;
    }
    if (i_vext) i_vext->itrace_print_vregs ();
  }
}





// ********************** Local CSRs *******************************************


uint32_t m_nucleus_ai::csr_read (int idx, bool passive) {
  //~ PN_INFOF (("### csr_read (%i)", idx));
  switch (idx) {

    // Machine Information Registers ...
    case CSR_MVENDORID:
      return PN_RISCV_VENDORID;
    case CSR_MARCHID:
      return PN_RISCV_MARCHID;
    case CSR_MIMPID:
      return PN_RISCV_MIMPID_PICONUT | PN_RISCV_MIMPID_NUCLEUS_AI;
    case CSR_MHARTID:
      return 0;   // just one hart
    case CSR_MCONFIGPTR:
      return 0;   // no configuration data structure

    // Machine Trap Setup (not implemented yet) ...
    /*
    //   Only registers supported by `nucleus_ref` follow.
    case CSR_MSTATUS:   // Machine status register
      break;
    case CSR_MISA:      // ISA and extensions
      break;
    case CSR_MIE:       // Machine interrupt-enable register
      break;
    case CSR_MTVEC:     // Machine trap-handler base address
      break;
    */

    // Machine Trap Handling (not implemented yet) ...
    /*
    //   Only registers supported by `nucleus_ref` follow.
    case CSR_MEPC:      // Machine exception program counter
      break;
    case CSR_MCAUSE:    // Machine trap cause
      break;
    case CSR_MTVAL:     // Machine trap value
      break;
    case CSR_MIP:       // Machine interrupt pending
      break;
    */

    // Debug Mode Registers (only partially implemented) ...
    case CSR_DCSR:      // Debug control and status register
      return reg_csr_dcsr.read ();
    case CSR_DPC:       // Debug program counter
      return reg_csr_dpc.read ();
    case CSR_DSCRATCH0: // Debug scratch register 0
      return reg_csr_dscratch0.read ();
    case CSR_DSCRATCH1: // Debug scratch register 1
      return reg_csr_dscratch1.read ();

    // Unprivileged Counter/Timers ...
    case CSR_CYCLE:     // Cycle counter for RDCYCLE instruction
      //~ PN_INFOF (("### reading CYCLE: %u", (unsigned) reg_csr_cycle.read ()));
      return (uint32_t) reg_csr_cycle.read ();
    case CSR_CYCLEH:    // Upper 32 bits of cycle, RV32 only
      return (uint32_t) (reg_csr_cycle.read () >> 32);
    case CSR_INSTRET:   // Instructions-retired counter for RDINSTRET instruction
      //~ PN_INFOF (("### reading INSTRET: %u", (unsigned) reg_csr_instret.read ()));
      return (uint32_t) reg_csr_instret.read ();
    case CSR_INSTRETH:  // Upper 32 bits of instret, RV32 only
      return (uint32_t) (reg_csr_instret.read () >> 32);
  }
  return 0;
}


void m_nucleus_ai::csr_write (int idx, int mode, uint32_t val) {
  uint32_t vval;   // virtual value as if write mode "all" has been selected

  // Adapt 'val' according to write mode ...
  switch (mode) {
    case pn_csr_write_mode_e::WRITE_CLEAR:
      vval = val & csr_read (idx, true);
      break;
    case pn_csr_write_mode_e::WRITE_SET:
      vval = val | csr_read (idx, true);
      break;
    default:
      vval = val;
  };

  // Actually write ...
  switch (idx) {

    // Machine Trap Setup (not implemented yet) ...
    /*
    //   Only registers supported by `nucleus_ref` follow.
    case CSR_MSTATUS:   // Machine status register
      break;
    case CSR_MISA:      // ISA and extensions
      break;
    case CSR_MIE:       // Machine interrupt-enable register
      break;
    case CSR_MTVEC:     // Machine trap-handler base address
      break;
    */

    // Machine Trap Handling (not implemented yet) ...
    /*
    //   Only registers supported by `nucleus_ref` follow.
    case CSR_MEPC:      // Machine exception program counter
      break;
    case CSR_MCAUSE:    // Machine trap cause
      break;
    case CSR_MTVAL:     // Machine trap value
      break;
    case CSR_MIP:       // Machine interrupt pending
      break;
    */

    // Debug Mode Registers (only partially implemented) ...
    case CSR_DCSR:      // Debug control and status register
      reg_csr_dcsr.write (vval);
      break;
    case CSR_DPC:       // Debug program counter
      reg_csr_dpc.write (vval);
      break;
    case CSR_DSCRATCH0: // Debug scratch register 0
      reg_csr_dscratch0.write (vval);
      break;
    case CSR_DSCRATCH1: // Debug scratch register 1
      reg_csr_dscratch1.write (vval);
      break;
  }
}



// ********************** AI/V Extension ******************************************


void m_nucleus_ai::proc_cmb_dport_direct () {
  dport_stb   = base_dport_stb;
  dport_we    = base_dport_we;
  dport_adr   = base_dport_adr;
  dport_bsel  = base_dport_bsel;
  dport_wdata = base_dport_wdata;

  base_dport_ack = dport_ack;
}


void m_nucleus_ai::proc_cmb_dport_mux () {
  if (reg_dport_vext.read () != 0) {
    dport_stb   = vext_dport_stb;
    dport_we    = vext_dport_we;
    dport_adr   = vext_dport_adr;
    dport_bsel  = vext_dport_bsel;
    dport_wdata = vext_dport_wdata;

    base_dport_ack = 0;
    vext_dport_ack = dport_ack;
  }
  else {
    dport_stb   = base_dport_stb;
    dport_we    = base_dport_we;
    dport_adr   = base_dport_adr;
    dport_bsel  = base_dport_bsel;
    dport_wdata = base_dport_wdata;

    base_dport_ack = dport_ack;
    vext_dport_ack = 0;
  }
}





// ********************** Instruction execution ********************************


#define NEXT_CYCLE main_soft_next_cycle


void m_nucleus_ai::reg_x_write (int i, uint32_t val) {
  if (i > 0) reg_x[i] = val;

  // ITrace bookkeeping ...
  itrace_xreg_changed[i] = true;
}


static uint32_t alu_func (int func, bool funcmod, uint32_t a, uint32_t b) {
  uint32_t ret;

  switch (func) {
    case 0:   // add / sub
      ret = !funcmod ? (a + b) : (a - b);
      break;
    case 1:   // sll
      ret = (a << (b & 0x1f));
      break;
    case 5:   // srl / sra
      ret = !funcmod  ? (a >> (b & 0x1f))
                      : (uint32_t) (((int32_t) a) >> (b & 0x1f));
      break;
    case 2:   // slt
      ret = ((int32_t) a < (int32_t) b) ? 1 : 0;
      break;
    case 3:   // sltu
      ret = (a < b) ? 1 : 0;
      break;
    case 4:   // xor
      ret = (a ^ b);
      break;
    case 6:   // or
      ret = (a | b);
      break;
    case 7:   // and
      ret = (a & b);
      break;
    default:
      // We should never get here.
      PN_ASSERT(false);
  }
  return ret;
}


static uint32_t mext_func (int func, uint32_t a, uint32_t b) {
  uint32_t ret;

  switch (func) {
    case 0:   // mul
      ret = (uint32_t) (a * b);
      break;
    case 1:   // mulh
	      //
	      // Note (jh): Cast uint32 operands to int32 before extending to int64
      	      // otherwise the sign gets ignored (treated like 'normal' highest bit of an uint32).
              // Fix was applied on 'mulh' and 'mulhsu'.
      ret = (uint32_t) (((int64_t)(int32_t) a * (int64_t)(int32_t) b) >> 32);
      break;
    case 2:   // mulhsu
      ret = (uint32_t) (((int64_t)(int32_t) a * (uint64_t) b) >> 32);
      break;
    case 3:   // mulhu
      ret = (uint32_t) (((uint64_t) a * (uint64_t) b) >> 32);
      break;
    case 4:   // div
      if (b == 0) { ret = (uint32_t) -1; break; }   // division by zero
      if (a == (1u << 31) && b == (uint32_t) -1) { ret = (1u << 31); break; }  // overflow
      ret = (uint32_t) ((int32_t) a / (int32_t) b);
      break;
    case 5:   // divu
      if (b == 0) { ret = (uint32_t) -1; break; }   // division by zero
      ret = (uint32_t) ((uint32_t) a / (uint32_t) b);
      break;
    case 6:   // rem
      if (b == 0) { ret = (uint32_t) -1; break; }   // division by zero
      if (a == (1u << 31) && b == (uint32_t) -1) { ret = 0; break; }  // overflow
      ret = (uint32_t) ((int32_t) a % (int32_t) b);
      break;
    case 7:   // remu
      if (b == 0) { ret = (uint32_t) -1; break; }   // division by zero
      ret = (uint32_t) ((uint32_t) a % (uint32_t) b);
      break;
  }

  //~ static const char *obstr[8] = { "MUL", "MULH", "MULHSU", "MULHU", "DIV", "DIVU", "REM", "REMU" };
  //~ PN_INFOF (("### %s %08x * %08x -> %08x", obstr[func], a, b, ret));
  return ret;
};


void m_nucleus_ai::main_soft_execute_insn (uint32_t insn, uint32_t pc, uint32_t *ret_next_pc) {
  sc_uint<PN_DPORT_BSEL_MAX> bsel;
  int insn_opcode, insn_rd, insn_rs1, insn_rs2, insn_funct3, insn_funct7;
  int csr_idx, csr_write_mode;
  uint32_t a, b, next_pc, adr, sys_op;
  uint32_t csr_wdata, csr_rdata;
  bool ok, jump;

  // Decode instruction ...
  insn_opcode = BIT_GROUP(insn,  6,  0);
  insn_rd     = BIT_GROUP(insn, 11,  7);
  insn_rs1    = BIT_GROUP(insn, 19, 15);
  insn_rs2    = BIT_GROUP(insn, 24, 20);
  insn_funct3 = BIT_GROUP(insn, 14, 12);
  insn_funct7 = BIT_GROUP(insn, 31, 25);

  // Execute instruction ...
  next_pc = pc + 4;
  ok = true;
  switch (insn_opcode) {

    // Integer computational ...
    case 0x13:  // 0010011 - ALU_I
      reg_x_write (
        insn_rd,
        alu_func (
          insn_funct3, insn_funct3 == 5 ? ((insn >> 30) & 1) : 0,
            // pass insn[30] as funcmod only on sra/srl, but not for add/sub
          reg_x_read (insn_rs1),
          get_immediate (insn)
        )
      );
      break;
    case 0x33:  // 0110011 - ALU

      // Base (RV32I) ...
      if ((insn_funct7 & 0x5f) == 0) {
        reg_x_write (
          insn_rd,
          alu_func (
            insn_funct3, (insn_funct7 & 0x20) != 0,
            reg_x_read (insn_rs1),
            reg_x_read (insn_rs2)
          )
        );
      }

#if PN_CFG_NUCLEUS_WITH_M

      // M Extension ...
      else if (insn_funct7 == 1) {
        NEXT_CYCLE(insn_funct3 < 4 ? cfg_latency_mul-1 : cfg_latency_div-1);  // simulate latency
        reg_x_write (
          insn_rd,
          mext_func (insn_funct3, reg_x_read (insn_rs1), reg_x_read (insn_rs2))
        );
      }

#endif
      break;

    case 0x37:  // 0110111 - LUI
      reg_x_write (insn_rd, get_immediate (insn));
      break;

    case 0x17:  // 0010111 - AUIPC
      reg_x_write (insn_rd, get_immediate (insn) + reg_pc.read ());
      break;

    // Control flow ...
    case 0x6f:  // 1101111 - JAL
      reg_x_write (insn_rd, next_pc);
      next_pc = reg_pc.read() + get_immediate (insn);
      break;
    case 0x67:  // 1100111 - JALR
      reg_x_write (insn_rd, next_pc);
      next_pc = reg_x_read (insn_rs1) + get_immediate (insn);
      next_pc &= ~1;   // least significant bit to be cleared according to spec
      break;
    case 0x63:  // 1100011 - Branches
      a = reg_x_read (insn_rs1);
      b = reg_x_read (insn_rs2);
      switch (insn_funct3 & 6) {
        case 0: jump = (a == b); break;                     // BEQ/BNE
        case 4: jump = ((int32_t) a < (int32_t) b); break;  // BLT/BGE
        case 6: jump = (a < b); break;                      // BLTU/BGEU
        default: ok = jump = false;
      }
      if (insn_funct3 & 1) jump = !jump;
      if (jump)
        next_pc = reg_pc.read() + get_immediate (insn);
      break;

    // Load & store ...
    case 0x03:  // 0000011 - LB/LH/LW/LBU/LHU
      // If an AI/V extension is present: Wait for the DPort to be released by the extension ...
      // Note: Even if we have a separate VPort, we must wait to get eventual data synchronized.
      if (i_vext) {
        while (vext_mem_pending.read () && !vext_idle.read ()) NEXT_CYCLE(1);
        reg_dport_vext = 0;
      }
      // Calculate adress & byte selection ...
      adr = reg_x_read (insn_rs1) + get_immediate (insn);
      bsel = get_bsel (adr, insn_funct3 & 3);
      if (bsel.value () == 0)
        PN_WARNINGF(("Misaligned load: %08x   %s", reg_pc.read ().to_uint (), pn_disassemble (insn)));
      else {
        // Perform memory access ...
        base_dport_adr = adr & ~3;
        base_dport_bsel = bsel;
        base_dport_we = 0;
        base_dport_stb = 1;
        NEXT_CYCLE(1);
        base_dport_stb = 0;
        while (!base_dport_ack.read ()) NEXT_CYCLE(1);
        // Read the data into register ...
        reg_x_write (insn_rd,
          get_rdata_val (adr, insn_funct3 & 3, dport_rdata.read (), (insn_funct3 & 4) == 0)
        );
      }
      break;
    case 0x23:  // 0100011 - SB/SH/SW
      // If an AI/V extension is present: Wait for the DPort to be released by the extension ...
      // Note: Even if we have a separate VPort, we wait in order to get the correct write order.
      if (i_vext) {
        while (vext_mem_pending.read () && !vext_idle.read ()) NEXT_CYCLE(1);
        reg_dport_vext = 0;
      }
      // Calculate adress & byte selection ...
      adr = reg_x_read (insn_rs1) + get_immediate (insn);
      bsel = get_bsel (adr, insn_funct3 & 3);
      if (bsel.value () == 0)
        PN_WARNINGF(("Misaligned store: %08x   %s", reg_pc.read ().to_uint (), pn_disassemble (insn)));
      else {
        // Perform memory access ...
        base_dport_adr = adr & ~3;
        base_dport_bsel = bsel;
        base_dport_wdata = get_wdata (adr, insn_funct3 & 3, reg_x_read (insn_rs2));
        base_dport_we = 1;
        base_dport_stb = 1;
        NEXT_CYCLE(1);
        base_dport_stb = 0;
        base_dport_we = 0;
        while (!base_dport_ack.read ()) NEXT_CYCLE(1);
      }
      break;

    // ECALL/EBREAK and CSR* ...
    case 0x73:  // 1110011 - SYSTEM (ECALL/EBREAK, CSR*, ...)
      if (insn_funct3 == 0) {

        // ECALL/EBREAK ...
        sys_op = reg_x_read (pn_reg_e::a7);
        switch (sys_op) {
          case pn_syscall_pk_e::sys_exit:
          case pn_syscall_pk_e::sys_exit_group:
            PN_INFOF(("IPORT count %llu", iport_count));
            pwroff = 1;
            NEXT_CYCLE(1);
#ifndef __SYNTHESIS__
            itrace_print_regs (pn_cfg_itrace_level >= 2);
#endif
            while (1) NEXT_CYCLE(1);
          default:
            PN_ERRORF(("0x%08x ecall: Unsupported syscall: %u", reg_pc.read().to_uint (), sys_op));
        }
      }
      else {

#if PN_CFG_NUCLEUS_WITH_ZICSR

        // Zicsr extension (CSR...) ...
        csr_idx = (insn >> 20) & 0x0fff;
        csr_write_mode = insn_funct3 & 3;
        csr_wdata =
          (insn_funct3 % 4) ? (uint32_t) insn_rs1   // CSRRWI / CSRRSI / CSRRCI
                            : reg_x_read (insn_rs1);  // CSRRW / CSRRS / CSRRC

        //   ... post on CSR bus ...
        csr_bus_adr_out = csr_idx;
        csr_bus_wdata_out = csr_wdata;
        csr_bus_write_mode_out = csr_write_mode;
        csr_bus_write_en_out = 1;
        csr_bus_read_en_out = (insn_rd != 0);       // if destination is x0, no read shall happen
        NEXT_CYCLE(1);
        csr_bus_write_en_out = 0;
        csr_bus_read_en_out = 0;
        if (insn_rd != 0) NEXT_CYCLE(1);            // wait for read data from bus

        //   ... access local CSRs ...
        if (insn_rd != 0)
          csr_rdata = csr_read (csr_idx);     // if destination is x0, no read shall happen
        else csr_rdata = 0;
        csr_write (csr_idx, csr_write_mode, csr_wdata);

        // Read CSR bus reply & write result ...
        //~ PN_INFOF (("### csr_bus_rdata_in = %08x", csr_bus_rdata_in.read ().to_uint ()));
        reg_x_write (insn_rd, csr_rdata | csr_bus_rdata_in.read ());

#else // PN_CFG_NUCLEUS_WITH_ZICSR
        ok = false;
#endif // PN_CFG_NUCLEUS_WITH_ZICSR

      }
      break;

#if PN_CFG_NUCLEUS_WITH_A
#error "A extension is not supported (yet)."

    // A extension ...

#endif // PN_CFG_NUCLEUS_WITH_A

    // Other ...
    case 0x0f:  // 0001111 - FENCE
      // do nothing
      break;

    default:
      ok = false;
  }

  // V extension ...
  if (!ok && i_vext) {
    if (i_vext->insn_is_v (insn)) {
      while (vext_rdy.read () == 0) NEXT_CYCLE(1);
      if (!PN_CFG_NUCLEUS_AI_USE_VPORT) reg_dport_vext = 1;   // grant DPort to Vext
      vext_insn = insn;
      vext_rs1 = reg_x_read (insn_rs1);
      vext_rs2 = reg_x_read (insn_rs2);
      vext_insn_stb = 1;
      NEXT_CYCLE(1);
      vext_insn_stb = 0;
      //~ PN_INFOF(("### vext_rd_pending = %i, vext_idle = %i", (int) vext_rd_pending.read (), (int) vext_idle.read ()));
      while (vext_rd_pending.read ()) {
        // V instruction has something to write back to a destination register ...
        if (vext_rd_stb.read () == 1) {
          reg_x_write (vext_rd_sel.read ().to_uint (), vext_rd.read ().to_uint ());
          break;
        }
        NEXT_CYCLE(1);
      }
      ok = true;
    }
  }

  // Done ...
  if (!ok) PN_ERRORF(("Invalid instruction: %08x   %s", reg_pc.read ().to_uint (), pn_disassemble (insn)));
  *ret_next_pc = next_pc;
}





// ********************** Main Process *****************************************


void m_nucleus_ai::main_soft_next_cycle (int n) {
  reg_csr_cycle = reg_csr_cycle.read () + n;
  for (; n > 0; n--) {
    wait (1);
    if (iport_ack.read ()) {
      got_iport_ack = true;
      got_iport_rdata = iport_rdata.read ();
    }
  }
}


void m_nucleus_ai::proc_clk_main_soft () {
  uint32_t insn, next_pc, prefetch_pc;

  // Reset ...

  //   ... reset all outputs ...
  pwroff = 0;
  iport_stb = 0;
  iport_bsel = 0xf;
  base_dport_stb = 0;
  base_dport_we = 0;
  dport_lrsc = 0;
  dport_amo = 0;
  debug_halt_ack = 0;

  //   ... reset all internal control signals ...
  csr_bus_read_en_out = 0;
  csr_bus_write_en_out = 0;
  vext_insn_stb = 0;

  //   ... reset registers ...
  reg_pc = PN_CFG_CPU_RESET_ADR;
  reg_dport_vext = 0;

  //   ... reset CSRs ...
  reg_csr_dcsr = 0;
  reg_csr_dpc = 0;
  reg_csr_dscratch0 = 0;
  reg_csr_dscratch1 = 0;

  reg_csr_cycle = 0;
  reg_csr_instret = 0;

  //   ... reset variables ...
  got_iport_ack = false;

  // Wait until other components are reset ...
  NEXT_CYCLE(1);

  next_pc = reg_pc.read ();

  // With IPort pipelining: Initiate first instruction fetch ...
  if (PN_CFG_NUCLEUS_USE_PORT_PIPELINING) {
    prefetch_pc = next_pc;
    iport_adr = next_pc;
    iport_stb = 1;
    NEXT_CYCLE(1);
    iport_stb = 0;
  }

  // Main loop ...
  insn = 0;
  while (1) {

    // TBD: Handle interrupts ...

    // Fetch instruction ...
    if (!PN_CFG_NUCLEUS_USE_PORT_PIPELINING) {

      // No IPort pipelining ...

      // Fetch the current instruction ...
      //   'next_pc' contains the value written to 'reg_pc' in the current clock cycle.
      iport_adr = next_pc;
      iport_stb = 1;
      NEXT_CYCLE(1);
      iport_stb = 0;
      while (!iport_ack.read ()) NEXT_CYCLE(1);
      insn = iport_rdata.read ();
    }
    else {

      // With IPort pipelining ...

      // Handle incorrect (speculative) prefetch ...
      if (prefetch_pc != next_pc) {
        iport_adr = next_pc;      // fetch the correct insn ...
        iport_stb = 1;
        if (got_iport_ack || iport_ack.read ()) {
          // The previous incorrect fetch was ack'ed in this cycle or earlier:
          // Go to next cycle ...
          got_iport_ack = false;
          NEXT_CYCLE(1);
          iport_stb = 0;
        }
        else {
          // The previous incorrect fetch was not ack'ed yet:
          // Wait for the ack, while maintaining the correct fetch for exactly one cycle ...
          NEXT_CYCLE(1);
          iport_stb = 0;
          if (!got_iport_ack)
            while (!iport_ack.read ()) NEXT_CYCLE(1);
          got_iport_ack = false;
        }
      }

      // At this point, we have a pipelined correct prefetch either
      // a) pending with got_iport_ack == 0 or
      // b) completed with got_iport_ack == 1, got_iport_rdata set.

      // Issue the next (speculative) prefetch ...
      prefetch_pc = next_pc + 4;
      iport_adr = prefetch_pc;
      iport_stb = 1;
      NEXT_CYCLE(1);
      iport_stb = 0;

      // Wait for the current fetch to complete ...
      while (!got_iport_ack) NEXT_CYCLE (1);
      insn = got_iport_rdata;
      got_iport_ack = false;
    }
    reg_ir = insn;

    // ITrace ...
#ifndef __SYNTHESIS__
    if (pn_cfg_itrace_level >= 1)
      iport_count++;
    if (pn_cfg_itrace_level >= 2) {
      itrace_print_regs (pn_cfg_itrace_level >= 2); // print register changes of the previous instruction (needed to wait for a clock edge, so this could not happen earlier)
      itrace_instruction (reg_pc.read (), insn);    // print current instruction
    }
#endif

    // Execute instruction ...
    main_soft_execute_insn (insn, reg_pc.read (), &next_pc);

    // Write back next PC ...
    reg_pc = next_pc;
    reg_csr_instret = reg_csr_instret.read () + 1;
  }
}
