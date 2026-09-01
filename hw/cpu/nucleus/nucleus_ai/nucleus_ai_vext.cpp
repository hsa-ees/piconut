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

#include "nucleus_ai_vext.h"

#include <pn_riscv_defs.h>

#include "nucleus_ai.h"





// ********************** Helpers **********************************************





// ********************** m_csr_slave: CSR Slave Interface *********************


void m_csr_slave::pn_trace (sc_trace_file *tf, int level) {
  if (level >= 1) {
    PN_TRACE(tf, csr_bus_read_en_in);
    PN_TRACE(tf, csr_bus_write_en_in);
    PN_TRACE(tf, csr_bus_write_mode_in);
    PN_TRACE(tf, csr_bus_adr_in);
    PN_TRACE(tf, csr_bus_wdata_in);
    PN_TRACE(tf, csr_bus_rdata_out);
  }
}


void m_csr_slave::on_clk () {
  if (csr_bus_read_en_in.read () == 1) {
    //~ PN_INFOF (("### m_csr_slave::on_clk: Reading csr_bus_rdata_out = %08x", csr_read (csr_bus_adr_in.read ().to_int ())));
    csr_bus_rdata_out = csr_read (csr_bus_adr_in.read ().to_int ());
  }
  if (csr_bus_write_en_in.read ()) {
    //~ PN_INFOF (("### m_csr_slave::on_clk: Writing"));
    csr_write (
      csr_bus_adr_in.read ().to_int (),
      csr_bus_write_mode_in.read ().to_int (),
      csr_bus_wdata_in.read ().to_int ()
    );
  }
}


void m_csr_slave::proc_clk_main () {
  while (1) {
    wait (1);
    // on_clk ();
  }
}





// ********************** Elaboration & Tracing ********************************


m_nucleus_ai_vext::~m_nucleus_ai_vext () {
  mem_done ();
}


void m_nucleus_ai_vext::pn_trace (sc_trace_file * tf, int level) {
  if (level >= 1) {

    // CSR ...
    m_csr_slave::pn_trace (tf, level);

    // General ports ...
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);

    // Base processor communications ...
    PN_TRACE(tf, base_rdy);
    PN_TRACE(tf, base_idle);
    PN_TRACE(tf, base_insn_stb);
    PN_TRACE(tf, base_insn);

    PN_TRACE(tf, base_rs1);
    PN_TRACE(tf, base_rs2);
    PN_TRACE(tf, base_rd_pending);
    PN_TRACE(tf, base_rd_stb);
    PN_TRACE(tf, base_rd_sel);
    PN_TRACE(tf, base_rd);

    PN_TRACE(tf, base_mem_pending);

    // DPort ...
    PN_TRACE(tf, dport_adr);
    PN_TRACE(tf, dport_bsel);
    PN_TRACE(tf, dport_rdata);
    PN_TRACE(tf, dport_wdata);
    PN_TRACE(tf, dport_stb);
    PN_TRACE(tf, dport_we);
    PN_TRACE(tf, dport_ack);

    // Registers ...
    PN_TRACE_BUS(tf, reg_v, 32);

    PN_TRACE(tf, reg_vstart);
    PN_TRACE(tf, reg_vl);
    PN_TRACE(tf, reg_vtype_vill);
    PN_TRACE(tf, reg_vtype_vma);
    PN_TRACE(tf, reg_vtype_vta);
    PN_TRACE(tf, reg_vtype_vsew);
    PN_TRACE(tf, reg_vtype_vlmul);
    PN_TRACE(tf, reg_vxrm);
    PN_TRACE(tf, reg_vxsat);

    // Submodules ...
    //~ if (i_csr) i_csr->pn_trace (tf, level - 1);
  }
}


void m_nucleus_ai_vext::itrace_init () {
  for (int i = 0; i < 32; i++) itrace_vreg_changed[i] = false;
}


void m_nucleus_ai_vext::itrace_print_vregs (bool all) {
  char buf[PN_CFG_VLEN*4 + 10], *p;
  uint32_t val;
  int v, e0, e;

  for (v = 0; v < 32; v++) if (all || itrace_vreg_changed[v]) {
    for (e0 = 0; e0 < PN_CFG_VLEN/32; e0 += 8) {
      buf[0] = '\0';
      p = buf;
      for (e = e0; e < e0 + 8 && e < PN_CFG_VLEN/32; e++) {
        while (*p && p < buf + sizeof(buf)) p++;
        PN_ASSERT(p < buf + sizeof(buf) - 10);    // avoid buffer overflow
        if (reg_v_changed[v]) val = reg_v_next[v] (32 * e + 31, 32 * e).to_uint ();
        else                  val = reg_v[v].read () (32 * e + 31, 32 * e).to_uint ();
        sprintf (p, " %08x", val);
      }
      PN_INFOF(("%sv%i[%2i..%2i] = %s", v < 10 ? " " : "", v, e0, e0 + 7, buf));
    }
    itrace_vreg_changed[v] = false;
  }
}





// ********************** Local CSRs *******************************************


uint32_t m_nucleus_ai_vext::csr_read (int idx, bool passive) {
  //~ PN_INFOF (("### VExt: csr_read (%i)", idx));
  switch (idx) {
    case CSR_VSTART:    // vector start position
      return reg_vstart.read ();
    case CSR_VL:        // vector length
      return reg_vl.read ();
    case CSR_VTYPE:     // vector data type register
      return  (reg_vtype_vill.read () ? (1 << 31) : 0) |
              (reg_vtype_vma.read () ? (1 << 7) : 0) |
              (reg_vtype_vta.read () ? (1 << 6) : 0) |
              (reg_vtype_vsew.read ().to_uint () << 3) |
              (reg_vtype_vlmul.read ().to_uint ());
    case CSR_VLENB:     // vector register length in bytes
      //~ PN_INFOF (("### VExt: csr_read (VLENB)", idx));
      return PN_CFG_VLEN / 8;
    case CSR_VCSR:      // Vector control and status register
      return (reg_vxrm.read (), reg_vxsat.read ());
    case CSR_VXRM:      // fixed-point rounding mode
      return reg_vxrm.read ();
    case CSR_VXSAT:     // fixed-point accrued saturation flag
      return reg_vxsat.read () ? 1 : 0;
  }
  return 0;
}


void m_nucleus_ai_vext::csr_write (int idx, int mode, uint32_t val) {
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
    case CSR_VSTART:    // vector start position
      reg_vstart = vval;
      break;
    case CSR_VL:        // vector length
      reg_vl = vval;
      break;
    case CSR_VTYPE:     // vector data type register
      reg_vtype_vma = (bool) ((vval >> 7) & 1);
      reg_vtype_vta = (bool) ((vval >> 6) & 1);
      reg_vtype_vsew = (vval >> 3) & 3;
      reg_vtype_vlmul = vval & 7;
      reg_vtype_vill = ((vval >> 3) & 3) > 2;   // SEW > 32 not allowed
      break;
    case CSR_VCSR:      // vector control and status register
      reg_vxrm = (vval >> 1) & 3;
      reg_vxsat = (bool) (vval & 1);
      break;
    case CSR_VXRM:      // vector fixed-point rounding mode
      reg_vxrm = vval & 3;
      break;
    case CSR_VXSAT:     // vector fixed-point saturation flag
      reg_vxsat = (bool) (vval & 1);
      break;
  }
}





// ********************** Register Access **************************************


void m_nucleus_ai_vext::reg_v_next_cycle () {
  for (int v = 0; v < 32; v++) if (reg_v_changed[v]) {
    reg_v[v].write (reg_v_next[v]);
    reg_v_changed[v] = false;
  }
}


void m_nucleus_ai_vext::reg_v_write_full (int v, sc_biguint<PN_CFG_VLEN> mask) {
  reg_v_next[v] = mask;
  reg_v_changed[v] = true;

  itrace_vreg_changed[v] = true;
}


uint32_t m_nucleus_ai_vext::reg_v_read (int vsew, int v, int e) {
  int sew = (1 << (vsew + 3));

  v += e / (PN_CFG_VLEN >> (3 + vsew));
  e %= (PN_CFG_VLEN >> (3 + vsew));
  PN_ASSERT (v >= 0 && v < 32);
  return reg_v[v].read () ((e + 1) * sew - 1, e * sew).to_uint ();
}


void m_nucleus_ai_vext::reg_v_write (int vsew, int v, int e, uint32_t val) {
  int sew = (1 << (vsew + 3));

  // Adapt v, e according for eventual register group addressing ...
  v += e / (PN_CFG_VLEN >> (3 + vsew));
  e %= (PN_CFG_VLEN >> (3 + vsew));
  PN_ASSERT (v >= 0 && v < 32);

  // Pre-read into 'reg_v_next' if adequate ...
  if (!reg_v_changed[v]) reg_v_next[v] = reg_v[v].read ();

  // Change the correct portion in 'reg_v_next' ...
  reg_v_next[v]((e + 1) * sew - 1, e * sew) = val;
  reg_v_changed[v] = true;

  // ITrace house-keeping ...
  itrace_vreg_changed[v] = true;
}





// ********************** Memory Access (Load/Store) ***************************


/* In the simulation soft model, memory accesses are enqueued in a linked list
 * of unlimited size, and the actual accesses are performed during completion.
 * In a hardware implementation, submission and completion should happen by
 * two pieces of hardware operating in parallel.
 */


#define NEXT_CYCLE main_soft_next_cycle

#define BSEL2MASK(bsel) (((bsel & 1) ? 0xff : 0) | ((bsel & 2) ? 0xff00 : 0) | ((bsel & 4) ? 0xff0000 : 0) | ((bsel & 8) ? 0xff000000 : 0))

#define DPORT_DLEN 32     // TBD: make configurable
#define DPORT_DLEN_LD 5

#define DPORT_ADR_MASK (~((1 << (DPORT_DLEN_LD - 3)) - 1))


static bool mem_access_is_alignable (mem_access_t *la) {
  uint32_t eff_bsel = la->bsel;
  eff_bsel <<= (la->adr & ~DPORT_ADR_MASK);
  return (eff_bsel & ~((1 << (DPORT_DLEN/8)) - 1)) == 0;
}


static inline void dport_access_clear (mem_access_t *pa) { pa->bsel = 0; }


static inline bool dport_access_is_valid (mem_access_t *pa) { return pa->bsel != 0; }


static bool dport_access_can_merge (mem_access_t *pa, mem_access_t *la) {
  if (!dport_access_is_valid (pa)) return true;   // an invalid port access can be assumed anything
  if (pa->write != la->write) return false;       // different access type
  if (pa->adr != (la->adr & DPORT_ADR_MASK)) return false;    // incompatible address
  return true;
}


static bool dport_access_covers (mem_access_t *pa, mem_access_t *la) {
  if (!dport_access_can_merge (pa, la)) return false;
  return (pa->bsel | (la->bsel << (la->adr & ~DPORT_ADR_MASK))) == pa->bsel;
}


static void dport_access_get_data (mem_access_t *pa, mem_access_t *la) {
  if (la->write == false)
    la->data = (pa->data.to_uint () >> (8 * (la->adr & ~DPORT_ADR_MASK))) & BSEL2MASK(la->bsel);
}


static void dport_access_merge (mem_access_t *pa, mem_access_t *la) {
  PN_ASSERT (dport_access_can_merge (pa, la));
  PN_ASSERT (mem_access_is_alignable (la));
  if (!dport_access_is_valid (pa)) {
    pa->write = la->write;
    pa->adr = la->adr & DPORT_ADR_MASK;
    pa->bsel = 0;
    pa->data = 0;
  }
  pa->bsel |= (la->bsel << (la->adr & ~DPORT_ADR_MASK));
  if (pa->write) pa->data |= (la->data << (8 * (la->adr & ~DPORT_ADR_MASK)));
}


void m_nucleus_ai_vext::mem_init () {
  mem_first = mem_completed = NULL;
  mem_p_last = &mem_first;
  mem_clear ();
}


void m_nucleus_ai_vext::mem_clear () {
  mem_access_t *ma;

  PN_ASSERTM(mem_first == NULL, "Memory submission leaked.");
  PN_ASSERT(!dport_access_is_valid (&mem_dport_strobed));    // pending DPort access is not supported and should not happen.

  PN_FREEP(mem_completed);
  while (mem_first) {
    ma = mem_first;
    mem_first = mem_first->next;
    PN_FREEP(ma);
  }
  mem_p_last = &mem_first;

  dport_access_clear (&mem_dport_completed);
  dport_access_clear (&mem_dport_strobed);
}


void m_nucleus_ai_vext::mem_submit (uint32_t _adr, int _bsel, bool _write, uint32_t _data) {
  /* Store a memory access for later execution. This method does not switch to a new clock cycle.
   * _adr will be truncated to a multiple of 4, and _bsel and _data will be shift left according to _adr & 3.
   * On misalignmnent,
   */
  mem_access_t *ma = PN_MALLOC(mem_access_t, 1);

  // Fill element ...
  ma->adr = _adr;
  ma->bsel = _bsel;
  ma->write = _write;
  ma->data = _data;

  // Check alignment ...
  if (!mem_access_is_alignable (ma)) {
    PN_ERRORF((_write ? "Misaligned memory access: w(%08x/%x) = %08x" : "Misaligned memory access: r(%08x/%x)", _adr, _bsel, _data));
    ma->bsel = 0;
  }

  // Append to queue ...
  //~ PN_INFOF(("### mem: submitted %c(%08x/%01x)", ma->write ? 'w' : 'r', ma->adr, ma->bsel));
  ma->next = NULL;
  *mem_p_last = ma;
  mem_p_last = &(ma->next);
}


mem_access_t *m_nucleus_ai_vext::mem_complete () {
  /* Run the memory (DPort) protocol until a new memory request has completed.
   * Returns the completed request with the `data` field containing the read value on reads.
   * Returned pointer is valid until the next call of this method.
   */
  mem_access_t *ma;

  //~ PN_INFO("### mem: complete ...");

  // Move current access from queue to `mem_completed` ...
  PN_ASSERT(mem_first != NULL);
  PN_FREEP(mem_completed);
  mem_completed = mem_first;
  if (mem_p_last == &(mem_first->next)) mem_p_last = &mem_first;
  mem_first = mem_first->next;

  // If current access is covered by a combined dport access: We're done. Return the result ...
  if (dport_access_covers (&mem_dport_completed, mem_completed)) {
    dport_access_get_data (&mem_dport_completed, mem_completed);
    //~ PN_INFOF(("### mem: completed %c(%08x/%01x) = %x", mem_completed->write ? 'w' : 'r', mem_completed->adr, mem_completed->bsel, mem_completed->data.to_uint ()));
    return mem_completed;
  }

  // The current completed dport access has been fully delivered: Clear it ...
  dport_access_clear (&mem_dport_completed);

  // Strobe current access, unless it has already been strobed due to bus pipelining ...
  if (dport_access_is_valid (&mem_dport_strobed)) {
    // already strobed ...
    mem_dport_completed = mem_dport_strobed;    // pre-strobed becomes the current access
    dport_access_clear (&mem_dport_strobed);    // clear the prestrobed acess
  }
  else {

    // Merge the current with as many additional requests from the beginning of the list ...
    dport_access_merge (&mem_dport_completed, mem_completed);
    for (ma = mem_first; ma; ma = ma->next) {
      if (!dport_access_can_merge (&mem_dport_completed, ma)) break;
      dport_access_merge (&mem_dport_completed, ma);
    }

    // Strobe the access ...
    dport_adr = mem_dport_completed.adr;
    dport_bsel = mem_dport_completed.bsel;
    if (mem_dport_completed.write) dport_wdata = mem_dport_completed.data;
    dport_we = mem_dport_completed.write;
    dport_stb = 1;
    //~ PN_INFOF(("### mem: strobed %c(%08x/%01x) = %x", mem_dport_completed.write ? 'w' : 'r', mem_dport_completed.adr, mem_dport_completed.bsel, mem_dport_completed.write ? mem_dport_completed.data.to_uint () : 0));
    NEXT_CYCLE(1);
    dport_stb = 0;
    dport_we = 0;
  }

  // Pre-strobe next access if bus pipelining is used on DPort ...
  if (PN_CFG_NUCLEUS_USE_PORT_PIPELINING && !dport_access_is_valid (&mem_dport_strobed)) {

    // Find first logical access not covered by the current DPort access ...
    ma = mem_first;
    while (ma && dport_access_covers (&mem_dport_completed, ma)) ma = ma->next;

    // Merge the next entries as far as possible ...
    while (ma && dport_access_can_merge (&mem_dport_strobed, ma)) {
      dport_access_merge (&mem_dport_strobed, ma);
      ma = ma->next;
    }

    // Strobe ...
    if (dport_access_is_valid (&mem_dport_strobed)) {
      dport_adr = mem_dport_strobed.adr;
      dport_bsel = mem_dport_strobed.bsel;
      if (mem_dport_strobed.write)
        dport_wdata = mem_dport_strobed.data;
      dport_we = mem_dport_strobed.write;
      dport_stb = 1;
      //~ PN_INFOF(("### mem: pre-strobed %c(%08x/%01x) = %x", mem_dport_strobed.write ? 'w' : 'r', mem_dport_strobed.adr, mem_dport_strobed.bsel, mem_dport_strobed.write ? mem_dport_strobed.data.to_uint() : 0));
    }
  }

  // Wait one cycle and release strobe ...
  NEXT_CYCLE(1);
  dport_stb = 0;
  dport_we = 0;

  // Wait for current access to complete and read the bus result ...
  while (dport_ack.read () == 0) NEXT_CYCLE(1);
  if (!mem_dport_completed.write) {
    mem_dport_completed.data = dport_rdata.read ();
    //~ PN_INFOF (("###    dport.data = %08x, ", mem_dport_completed.data.to_uint ()));
  }

  // Extract the desired logical data ...
  PN_ASSERT (dport_access_covers (&mem_dport_completed, mem_completed));
  dport_access_get_data (&mem_dport_completed, mem_completed);
  //~ PN_INFOF (("###    dport.data = %08x, completed.data = %08x", mem_dport_completed.data.to_uint (), mem_completed->data.to_uint ()));

  // Done ...
  //~ PN_INFOF(("### mem: completed %c(%08x/%01x) = %x", mem_completed->write ? 'w' : 'r', mem_completed->adr, mem_completed->bsel, mem_completed->data.to_uint ()));
  return mem_completed;
}





// ********************** Instruction helpers **********************************


bool m_nucleus_ai_vext::insn_is_v (uint32_t insn) {
  int   insn_opcode = BIT_GROUP(insn,  6,  0);
  int   insn_funct3 = BIT_GROUP(insn, 14, 12);
  bool  insn_mew    = BIT(insn, 28);    // extended memory element width (RISC-V Vol. 1, section 30.7.3)

  // Configuration-setting instructions (vset[i]vl[i]) ...
  if (insn_opcode == 0x57 && insn_funct3 == 7) return true;

  // Vector load / store ...
  if (insn_opcode == 0x07 || insn_opcode == 0x27)
    if (insn_mew == 0 && (insn_funct3 == 0 || insn_funct3 >= 5)) return true;

  // Vector integer computational instructions ...
  if (insn_opcode == 0x57) return true;

  // None of the above ...
  return false;
}





// ********************** Instruction execution (soft) *************************


// Vector Arithmetic Instruction encoding: operand type and source locations (funct3) ...
#define OPIVV 0
#define OPFVV 1
#define OPMVV 2
#define OPIVI 3
#define OPIVX 4
#define OPFVF 5
#define OPMVX 6
#define OPCFG 7


void m_nucleus_ai_vext::main_soft_next_cycle (int n) {
  reg_v_next_cycle ();


  //
  // Quick fix by Johannes
  // Remove second cthread so writes to csr signals happen ONLY in ONE cthread. => No two seperate drivers.
  //
  for(int i = 0; i < n; ++i)
  {
    m_csr_slave::on_clk ();
    wait (1);
  }
}


bool m_nucleus_ai_vext::main_soft_execute_insn (uint32_t insn, uint32_t rs1_val, uint32_t rs2_val, bool *ret_rd_writeback, uint32_t *ret_rd_val) {
  // Returns 'false' in case of an illegal Vext instruction.
  //
  // Error handling:
  // - Instruction is not a Vext instruction: Abort (insn_is_v() should have prevented this).
  // - Instruction is an invalid Vext instruction: Emit a warning and return false
  // - Instruction is not implemented, but part of Zve32x: Abort
  //
  // Accessing ports:
  // - DPort signals are exclusively accessed by mem_*() methods, which are called from here.
  // - If the instruction has no scalar destination register, 'base_rd_pending' should be released
  //   as soon as possible.
  // - If the instruction does not perform a memory access, 'base_mem_pending' should be released as
  //   soon as possible.
  // - Other port signal accesses are not allowed here.
  //
  // Accessing registers:
  // - Vector registers (reg_v) may be accessed here, usually by reg_v_*() methods.
  // - CSRs may be accessed here.
  //
  int i;

  // Decode instruction ...
  int   insn_opcode = BIT_GROUP(insn,  6,  0);
  int   insn_vd     = BIT_GROUP(insn, 11,  7);
  int   insn_vs1    = BIT_GROUP(insn, 19, 15);
  int   insn_vs2    = BIT_GROUP(insn, 24, 20);
  int   insn_funct3 = BIT_GROUP(insn, 14, 12);
  bool  insn_vm     = BIT(insn, 25);      // for load/store and computational instructions

  // Determine common parameters ...
  int vsew = reg_vtype_vsew.read ();      // ld of SEW/8
  int sew = 8 << vsew;                    // SEW = selected element width, set in VTYPE register
  int vlmul = reg_vtype_vlmul.read ();
  vlmul = SIGN_EXTEND(vlmul, 2);
  int vlmax = PN_CFG_VLEN >> (3 + vsew - SIGN_EXTEND(vlmul, 2));
  int vstart = reg_vstart.read ();
  int vl = reg_vl.read ();

  // Effective mask (for load/store and computational instructions) ...
  sc_biguint<PN_CFG_VLEN> eff_mask;
  if (insn_vm) eff_mask = -1;             // all-1
  else eff_mask = reg_v_read_full (0);    // mask register

  // Execute instruction ...
  bool insn_handled = false;              // mark whether the instruction has been executed.

  //~ PN_INFOF(("### Executing: %08x    %s", insn, pn_disassemble (insn)));
  //~ PN_INFOF(("### eff_mask = %08x, insn_vm = %i", eff_mask(31,0).to_uint (), int (insn_vm)));

  // ***** Configuration-Setting Instructions (vset[i]vl[i]) *****

  if (insn_opcode == 0x57 && insn_funct3 == 7) {
    int avl, new_vlmax, new_vl, new_vtype, new_vsew, new_vlmul;

    // Release DPort ...
    base_mem_pending = 0;

    // Decode instruction ...
    int insn_vtypei = BIT_GROUP(insn, 31, 20);
    int insn_rs1    = insn_vs1;
    int insn_rd     = insn_vd;

    // Get application vector length (AVL) and new vtype ...
    if (insn_vtypei < 0x800) {        // vsetvli ...
      avl = rs1_val;
      new_vtype = insn_vtypei & 0x7ff;
    }
    else if (insn_vtypei < 0xc00) {   // vsetvl ...
      avl = rs1_val;
      new_vtype = rs2_val;
    }
    else {                            // vsetivli ...
      uint32_t insn_uimm = insn_rs1;
      avl = insn_uimm;
      new_vtype = insn_vtypei & 0x3ff;
    }

    // Set vtype register and determine vlmax, new_vl ...
    reg_vtype_vma = BIT_GROUP(new_vtype, 7, 7);
    reg_vtype_vta = BIT_GROUP(new_vtype, 6, 6);
    new_vsew      = BIT_GROUP(new_vtype, 5, 3);
    new_vlmul     = BIT_GROUP(new_vtype, 2, 0);
    if (new_vsew > 3 || BIT_GROUP(new_vtype, 31, 8) != 0) {        // Error ...
      reg_vtype_vill  = 1;
      reg_vtype_vsew  = 0;
      reg_vtype_vlmul = 0;

      new_vl = 0;
    }
    else {        // No error ...
      reg_vtype_vill  = 0;
      reg_vtype_vsew  = new_vsew;
      reg_vtype_vlmul = new_vlmul;

      new_vlmax = PN_CFG_VLEN >> (3 + new_vsew - SIGN_EXTEND(new_vlmul, 2));
      new_vl = MIN(new_vlmax, avl);
      if (insn_vtypei < 0xc00 && insn_rs1 == 0) {     // special cases for vsetvl and vsetvli with rs1 == x0...
        if (insn_rd == 0) new_vl = vl;
        else new_vl = new_vlmax;
      }
      //~ PN_INFOF (("### VL = %i (VLmax = %i), SEW=%i, LMUL=2^%i", new_vl, new_vlmax, (8 << new_vsew), SIGN_EXTEND(new_vlmul, 2)));
    }
    reg_vl = new_vl;

    // Write back rd ...
    if (insn_rd != 0) {
      *ret_rd_val = (uint32_t) new_vl;
      *ret_rd_writeback = true;
    }
    else {
      base_rd_pending = 0;    // release writeback port
      NEXT_CYCLE(1);          // generate a cycle with 'base_rd_pending' unset
    }

    // Done...
    insn_handled = true;
  }


  // ***** Vector Load / Store *****

  if (insn_opcode == 0x07 || insn_opcode == 0x27) {
    uint32_t adr, data;

    // Release RD writeback ...
    base_rd_pending = 0;

    // Decode instruction fields ...
    int   insn_vs3    = insn_vd;          // store data
    int   insn_width  = insn_funct3;
    bool  insn_mew    = BIT(insn, 28);    // extended memory element width (RISC-V Vol. 1, section 30.7.3)
    int   insn_mop    = BIT_GROUP(insn, 27, 26);  // memory addressing mode
    int   insn_nf     = BIT_GROUP(insn, 31, 29);  // number of fields in each segment, for segment load/stores
    int   insn_umop   = insn_vs2;         // additional fields encoding variants of unit-stride instructions

    // Sanity ...
    if (reg_vtype_vill.read ()) {
      PN_WARNING("VTYPE invalid (VILL bit set)");
      return false;
    }
    PN_ASSERT(vl < PN_CFG_VLEN);    // illegal vector length

    PN_ASSERTM(insn_mew == 0 && (insn_width == 0 || insn_width >= 5), "Opcode represents an FP operation");
    PN_ASSERTM(insn_nf < 2, "Vector load/store segment instructions are not implemented");
    if (insn_mop == 0) {
      PN_ASSERTM(insn_umop != 8, "Unit-stride whole register load/store is not implemented");
      PN_ASSERTM(insn_umop != 11, "Unit-stride mask load/store, EEW=8, is not implemented");
      PN_ASSERTM(insn_umop != 16, "Unit-stride fault-only-first load is not implemented");
    }

    // Determine more parameters ...
    int veew = insn_width & 3;              // ld of EEW/8
    int eew = 8 << veew;                    // EEW = effective element width, set in instruction

    bool write = (insn_opcode == 0x27);
    uint32_t adr0 = rs1_val;

    // Go ahead ...
    mem_clear ();
    for (int pass = 0; pass < 2; pass++) {        // pass 0: submission, pass 1: completion
      for (i = vstart; i < vl; i++) if (eff_mask[i] == 1) {

        // ... address calculation ...
        switch (insn_mop) {
          case 0: // unit-stride
            if (insn_umop != 0) {
              PN_WARNING("Invalid LUMOP/SUMOP value in vector load/store");
              return false;
            }
            adr = adr0 + (i << veew);
            break;
          case 2: // strided
            adr = adr0 + i * rs2_val;
            break;
          case 1: // indexed-unordered
          case 3: // indexed-ordered
            adr = adr0 + reg_v_read (veew, insn_vs2, i);
            break;
          default: PN_ASSERT(false);
        }
        if (pass ==  0) {

          // ... submission ...
          if (write) data = reg_v_read ((insn_mop & 1) == 0 ? veew : vsew, insn_vs3, i);
          else data = 0;
          switch (eew) {
            case 32: mem_submit32 (adr, write, data); break;
            case 16: mem_submit16 (adr, write, data); break;
            case  8: mem_submit8  (adr, write, data); break;
            default: PN_ASSERT(false);
          }
        }
        else {

          // ... completion ...
          switch (eew) {
            case 32: data = mem_complete32 (); break;
            case 16: data = mem_complete16 (adr); break;
            case  8: data = mem_complete8  (adr); break;
            default: PN_ASSERT(false); data = 0;
          }
          if (!write)
            reg_v_write ((insn_mop & 1) == 0 ? veew : vsew, insn_vd, i, data);
        }
      }
    }

    // Done...
    base_mem_pending = 0;   // release DPort
    insn_handled = true;
  }


  // ***** Vector Integer Computational Instructions (including fixed-point, reduction, mask and permutation) *****

  if (insn_opcode == 0x57 && insn_funct3 < 7) {
    uint32_t val, idx;
    int sign_pos;

    // Release DPort ...
    base_mem_pending = 0;

    // Decode specific instruction fields ...
    int   insn_funct6 = BIT_GROUP(insn, 31, 26);
    int   insn_imm    = insn_vs1;
    int   insn_rd     = insn_vd;

    // Sanity ...
    if (reg_vtype_vill.read ()) {
      PN_WARNING("VTYPE invalid (VILL bit set)");
      return false;
    }
    PN_ASSERT(vl < PN_CFG_VLEN);    // illegal vector length

    // Effective operation ...
    int op_funct6 = insn_funct6       // = insn_funct6 + (0x000 for OPI* | 0x100 for OPV* | ...)
                    | ((insn_funct3 == OPIVV || insn_funct3 == OPIVX || insn_funct3 == OPIVI) ? 0x000 : 0)
                    | ((insn_funct3 == OPMVV || insn_funct3 == OPMVX)                         ? 0x100 : 0)
                    | ((insn_funct3 == OPFVV || insn_funct3 == OPFVF)                         ? 0x200 : 0)
                    | ((insn_funct3 == OPCFG)                                                 ? 0x300 : 0);

    // Wide source or destination (for widening and narrowing operations) ...
    bool wide_vd = false, wide_vs2 = false;
    if (insn_funct6 >= 0x30) {     // widening operations (vw...) ...
      wide_vd = true;
      if (insn_funct6 >= 0x34 && insn_funct6 <= 0x37) wide_vs2 = true;
    }
    switch (insn_funct3) {
      case OPIVV: case OPIVX: case OPIVI:
        if (insn_funct6 >= 0x2c && insn_funct6 <= 0x2f) wide_vs2 = true;
        break;
      case OPMVV: case OPMVX:
        if (insn_funct6 == 0x2b) wide_vs2 = true;      // vnmsub
        break;
    }
    if ((wide_vd || wide_vs2) && sew > 16) {
      PN_WARNING ("Narrowing/widening operand with SEW > 16 are not supported.");
      return false;
    }

    // Get scalar value ...
    uint32_t scalar_u = 0;
    int32_t scalar_s = 0;
    switch (insn_funct3) {
      case OPIVV: case OPMVV:           // vector-vector operands (OPIVV, OPMVV)
        break;
      case OPFVV:
        PN_WARNING("FP operations are not implemented; OPFVV operands are not allowed.");
        return false;
      case OPIVI:                       // vector-immediate (OPIVI)
        scalar_u = insn_imm;
        scalar_s = SIGN_EXTEND(scalar_u, 4);
        break;
      case OPIVX: case OPMVX:           // vector-scalar register (OPIVX, OPMVX)
        scalar_s = scalar_u = rs1_val;
        break;
      case OPFVF:
        PN_WARNING("FP operations are not implemented; OPFVF (vector-scalar) operands are not allowed.");
        return false;
      case OPCFG:
        PN_ASSERTM(false, "OPCFG (scalars-imms) operands are not implemented.");
      default:
        PN_ASSERT(false);
    }

    // Prepare rd, clear rd writeback ...
    uint32_t  rd_val = 0;
    bool      rd_writeback = false;

    // Read vs1.mask, vs2.mask and v0.mask (for vector mask instructions), clear vd.mask writeback ...
    sc_biguint<PN_CFG_VLEN> v0_mask   = reg_v_read_full (0);        // vector #0 mask ("v0.t") (for vmerge)
    sc_biguint<PN_CFG_VLEN> vs1_mask  = reg_v_read_full (insn_vs1); // vector mask source 1
    sc_biguint<PN_CFG_VLEN> vs2_mask  = reg_v_read_full (insn_vs2); // vector mask source 2
    sc_biguint<PN_CFG_VLEN> vd_mask   = reg_v_read_full (insn_vd);  // vector mask destination, read to preserve (upper) unchanged bits
    bool                    vd_mask_writeback = false;              // vector mask writeback flag

    // Read vd[0] (for reduction operations), clear reduction result writeback flags ...
    //    Note: In hardware, red_u/red_s will be the same register(s)/signal(s).
    uint32_t  red_u = reg_v_read (vsew, insn_vs1, 0);     // starting value for (unsigned) reduction operations
    int32_t   red_s = SIGN_EXTEND(red_u, sew - 1);        // starting value for (signed) reduction operations
    bool      red_u_writeback, red_s_writeback;           // reduction accumulator writeback flags (at most one will be set)
    red_u_writeback = red_s_writeback = false;

    // *** Handle element-oriented operations (vector integer, fixed-point and reduction) ...

    uint32_t  es1_u, es2_u;   // vector source elements, extended to 32 bits (parts of vs1/vs2, unsigned)
    int32_t   es1_s, es2_s;   // vector source elements, extended to 32 bits (parts of vs1/vs2, signed)
    uint32_t  ed;             // vector element eventually to write back to vd
    bool      ed_writeback;   // write back flag for element-wise operations

    insn_handled = true;
    for (i = vstart; i < vl; i++) if (eff_mask[i] == 1) {
      ed = 0;
      ed_writeback = true;    // This will be set identically in each iteration.
        // Note: It defaults to 'true', computational instructions not writing back into VD must
        //       explicitly set it to 'false'.

      // ... get source element ...
      if (insn_funct3 == OPIVI || insn_funct3 == OPIVX || insn_funct3 == OPMVX) {
        es1_u = scalar_u;
        es1_s = scalar_s;
      }
      else {
        es1_u = reg_v_read (vsew, insn_vs1, i);
        es1_s = SIGN_EXTEND(es1_u, sew - 1);
      }
      es2_u = reg_v_read (vsew + (wide_vs2 ? 1 : 0), insn_vs2, i);
      es2_s = SIGN_EXTEND(es2_u, sew - 1);

      // ... perform operation ...
      switch (op_funct6) {

        // Integer add/sub, including widening variants ...
        case 0x000:  // vadd
        case 0x130:  // vwaddu
        case 0x134:  // vwaddu.w
          ed = es2_u + es1_u;
          break;
        case 0x131:  // vwadd
        case 0x135:  // vwadd.w
          ed = (uint32_t) (es2_s + es1_s);
          break;
        case 0x002:  // vsub
        case 0x132:  // vwsubu
        case 0x136:  // vwsubu.w
          ed = es2_u - es1_u;
          break;
        case 0x133:  // vwsub
        case 0x137:  // vwsub.w
          ed = (uint32_t) (es2_s - es1_s);
          break;
        case 0x003:  // vrsub
          ed = -es2_u + es1_u;
          break;

        // Integer extension ...
        case 0x112:  // VXUNARY0 (V)
          // operation is encoded in 'insn_vs1': 2 = vzext.vf8, 3 = vsext.vf8, ... 6 = vzext.vf2, 7 = vsext.vf2
          sign_pos = (sew >> (4 - (insn_vs1 >> 1))) - 1;
          ed = ZERO_EXTEND(es2_u, sign_pos);
          if (insn_vs1 & 1) ed = SIGN_EXTEND(ed, sign_pos);
          break;

        // TBD: Integer add-with-carry / subtract-with-borrow ...
        //   Note: These operations process all elements, even if masked out.
        case 0x010:  // vadc
          PN_ASSERTM(false, "Operation not implemented");
          break;
        case 0x011:  // vmadc
          PN_ASSERTM(false, "Operation not implemented");
          break;
        case 0x012:  // vsbc
          PN_ASSERTM(false, "Operation not implemented");
          break;
        case 0x013:  // vmsbc
          PN_ASSERTM(false, "Operation not implemented");
          break;

        // Bitwise logical ...
        case 0x009:  // vand
          ed = es2_u & es1_u;
          break;
        case 0x00a:  // vor
          ed = es2_u | es1_u;
          break;
        case 0x00b:  // vxor
          ed = es2_u ^ es1_u;
          break;

        // Shift (single-width and narrowing) ...
        case 0x025:  // vsll
          ed = es2_u << ZERO_EXTEND(es1_u, vsew + 3);
          break;
        case 0x028:  // vsrl
        case 0x02c:  // vnsrl
          ed = es2_u >> ZERO_EXTEND(es1_u, vsew + 3);
          break;
        case 0x029:  // vsra
        case 0x02d:  // vnsra
          ed = es2_s >> ZERO_EXTEND(es1_u, vsew + 3);
          //~ PN_INFOF (("### vsra: es2 = %08x, es1 = %x, ed = %08x, vsew = %i, sew = %i, ZERO_EXTEND(es1_u, vsew + 3) = %i", es2_s, es1_u, ed, vsew, sew, ZERO_EXTEND(es1_u, vsew + 3)));
          break;

        // Integer compare ...
        case 0x018:  // vmseq
          reg_v_mask_write (insn_vd, i, es2_u == es1_u);
          ed_writeback = false;
          break;
        case 0x019:  // vmsne
          reg_v_mask_write (insn_vd, i, es2_u != es1_u);
          ed_writeback = false;
          break;
        case 0x01a:  // vmsltu
          reg_v_mask_write (insn_vd, i, es2_u < es1_u);
          ed_writeback = false;
          break;
        case 0x01b:  // vmslt
          reg_v_mask_write (insn_vd, i, es2_s < es1_s);
          ed_writeback = false;
          break;
        case 0x01c:  // vmsleu
          reg_v_mask_write (insn_vd, i, es2_u <= es1_u);
          ed_writeback = false;
          break;
        case 0x01d:  // vmsle
          reg_v_mask_write (insn_vd, i, es2_s <= es1_s);
          ed_writeback = false;
          break;
        case 0x01e:  // vmsgtu
          reg_v_mask_write (insn_vd, i, es2_u > es1_u);
          ed_writeback = false;
          break;
        case 0x01f:  // vmsgt
          reg_v_mask_write (insn_vd, i, es2_s > es1_s);
          ed_writeback = false;
          break;

        // Integer min/max ...
        case 0x004:  // vminu
          ed = MIN(es2_u, es1_u);
          break;
        case 0x005:  // vmin
          ed = MIN(es2_s, es1_s);
          break;
        case 0x006:  // vmaxu
          ed = MAX(es2_u, es1_u);
          break;
        case 0x007:  // vmax
          ed = MAX(es2_s, es1_s);
          break;

        // Integer multiplication, including widening multiply ...
        case 0x125:  // vmul
        case 0x13b:  // vwmul
          ed = (uint32_t) (es2_s * es1_s);
          break;
        case 0x138:  // vwmulu
          ed = es2_u * es1_u;
          break;
        case 0x127:  // vmulh
          ed = (uint32_t) (((int64_t) es2_s * (int64_t) es1_s) >> sew);
          break;
        case 0x124:  // vmulhu
          ed = (uint32_t) (((uint64_t) es2_u * (uint64_t) es1_u) >> sew);
          break;
        case 0x126:  // vmulhsu
        case 0x13a:  // vwmulsu
          ed = (uint32_t) (((int64_t) es2_s * (uint64_t) es1_u) >> sew);
          break;

        // Integer division ...
        case 0x120:  // vdivu
          ed = es2_u / es1_u;
          break;
        case 0x121:  // vdiv
          ed = (uint32_t) (es2_s / es1_s);
          break;
        case 0x122:  // vremu
          ed = es2_u % es1_u;
          break;
        case 0x123:  // vrem
          ed = (uint32_t) (es2_s % es1_s);
          break;

        // TBD: Multiply-add instructions ...
        //   Note: These need to read insn_vd as a third source operand, which is hard to implement.
        //         A future implementation should probably emulate them by a sequence of a vmul and a vadd
        //         instruction.
        case 0x129:  // vmadd
          PN_ASSERTM(false, "Operation not implemented");
          break;
        case 0x12b:  // vnmsub
          PN_ASSERTM(false, "Operation not implemented");
          break;
        case 0x12d:  // vmacc
          PN_ASSERTM(false, "Operation not implemented");
          break;
        case 0x12f:  // vnmsac
          PN_ASSERTM(false, "Operation not implemented");
          break;
        case 0x13c:  // vwmaccu
          PN_ASSERTM(false, "Operation not implemented");
          break;
        case 0x13d:  // vwmacc
          PN_ASSERTM(false, "Operation not implemented");
          break;
        case 0x13e:  // vwmaccus
          PN_ASSERTM(false, "Operation not implemented");
          break;
        case 0x13f:  // vwmaccsu
          PN_ASSERTM(false, "Operation not implemented");
          break;

        // Fixed-point arithmetic ...
        case 0x020:  // vsaddu
          PN_ASSERTM(false, "Operation not implemented");
          break;
        case 0x021:  // vsadd
          PN_ASSERTM(false, "Operation not implemented");
          break;
        case 0x022:  // vssubu
          PN_ASSERTM(false, "Operation not implemented");
          break;
        case 0x023:  // vssub
          PN_ASSERTM(false, "Operation not implemented");
          break;

        case 0x108:  // vaaddu
          PN_ASSERTM(false, "Operation not implemented");
          break;
        case 0x109:  // vaadd
          PN_ASSERTM(false, "Operation not implemented");
          break;
        case 0x10a:  // vasubu
          PN_ASSERTM(false, "Operation not implemented");
          break;
        case 0x10b:  // vasub
          PN_ASSERTM(false, "Operation not implemented");
          break;

        case 0x027:  // vsmul (V, X) / vmv<nr>r (I)
          PN_ASSERTM(false, "Operation not implemented");
          break;

        case 0x02a:  // vssrl
          PN_ASSERTM(false, "Operation not implemented");
          break;
        case 0x02b:  // vssra
          PN_ASSERTM(false, "Operation not implemented");
          break;

        case 0x02e:  // vnclipu
          PN_ASSERTM(false, "Operation not implemented");
          break;
        case 0x02f:  // vnclip
          PN_ASSERTM(false, "Operation not implemented");
          break;

        // Vector reduction operations ...
        case 0x100:  // vredsum
        case 0x030:  // vwredsumu
          red_u += es2_u;
          red_u_writeback = true;
          break;
        case 0x031:  // vwredsum
          red_s += es2_s;
          red_s_writeback = true;
          break;
          break;
        case 0x101:  // vredand
          red_u &= es2_u;
          red_u_writeback = true;
          break;
        case 0x102:  // vredor
          red_u |= es2_u;
          red_u_writeback = true;
          break;
        case 0x103:  // vredxor
          red_u ^= es2_u;
          red_u_writeback = true;
          break;
        case 0x104:  // vredminu
          red_u = MIN(red_u, es2_u);
          red_u_writeback = true;
          break;
        case 0x105:  // vredmin
          red_s = MIN(red_s, es2_s);
          red_s_writeback = true;
          break;
        case 0x106:  // vredmaxu
          red_u = MAX(red_u, es2_u);
          red_u_writeback = true;
          break;
        case 0x107:  // vredmax
          red_s = MAX(red_s, es2_s);
          red_s_writeback = true;
          break;

        default:
          insn_handled = false;
      } // switch (op_funct6)

      // ... write back to destination register ...
      if (ed_writeback && !red_u_writeback && !red_s_writeback)
        reg_v_write (vsew + (wide_vd ? 1 : 0), insn_vd, i, ed);

    } // for (i = vstart; i < vl; i++) if (eff_mask[i])

    // *** Handle vector mask instructions ...

    if (!insn_handled) {
      insn_handled = true;
      vd_mask_writeback = true;
      switch (op_funct6) {
        case 0x118:  // vmandn
          vd_mask = vs2_mask & ~vs1_mask;
          break;
        case 0x119:  // vmand
          vd_mask = vs2_mask & vs1_mask;
          break;
        case 0x11a:  // vmor
          vd_mask = vs2_mask | vs1_mask;
          break;
        case 0x11b:  // vmxor
          vd_mask = vs2_mask ^ vs1_mask;
          break;
        case 0x11c:  // vmorn
          vd_mask = vs2_mask | ~vs1_mask;
          break;
        case 0x11d:  // vmnand
          vd_mask = ~(vs2_mask & vs1_mask);
          break;
        case 0x11e:  // vmnor
          vd_mask = ~(vs2_mask | vs1_mask);
          break;
        case 0x11f:  // vmxnor
          vd_mask = ~(vs2_mask ^ vs1_mask);
          break;
        case 0x110:  // VWXUNARY0 (V), (VRXUNARY0 (X))
          switch (insn_vs1) {
            case 16:  // vcpop (count population in mask)
              val = 0;
              for (i = vstart; i < vl; i++)
                if (eff_mask[i] && vs2_mask[i]) val++;
              rd_val = val;
              rd_writeback = true;
              break;
            case 17:  // vfirst (find-first-set mask bit)
              val = (uint32_t) -1;
              for (i = vstart; i < vl; i++)
                if (eff_mask[i] && vs2_mask[i]) { val = i; break; }
              rd_val = val;
              rd_writeback = true;
              break;
            default:
              insn_handled = vd_mask_writeback = false;
          }
          break;
        case 0x114:  // VMUNARY0 (V)
          switch (insn_vs1) {
            case 1:   // vmsbf (set before first mask bit)
              for (i = vstart; i < vl; i++) if (eff_mask[i]) vd_mask[i] = 0;
              for (i = vstart; i < vl; i++) if (eff_mask[i]) {
                if (vs2_mask[i]) break;
                vs2_mask[i] = 1;
              }
              vd_mask_writeback = true;
              break;
            case 2:   // vmsof (set only first mask bit)
              for (i = vstart; i < vl; i++) if (eff_mask[i]) vd_mask[i] = 0;
              for (i = vstart; i < vl; i++) if (eff_mask[i]) {
                if (vs2_mask[i]) { vs2_mask[i] = 1; break; }
              }
              vd_mask_writeback = true;
              break;
            case 3:   // vmsif (set including first mask bit)
              for (i = vstart; i < vl; i++) if (eff_mask[i]) vd_mask[i] = 0;
              for (i = vstart; i < vl; i++) if (eff_mask[i]) {
                vs2_mask[i] = 1;
                if (vs2_mask[i]) break;
              }
              vd_mask_writeback = true;
              break;

            case 16:  // viota
              val = 0;
              for (i = vstart; i < vl; i++) if (eff_mask[i]) {
                reg_v_write (vsew, insn_vd, i, val);
                val++;
              }
              break;
            case 17:  // vid
              for (i = vstart; i < vl; i++) if (eff_mask[i])
                reg_v_write (vsew, insn_vd, i, i);
              break;
            default:
              insn_handled = vd_mask_writeback = false;
          }
          break;
        default:
          insn_handled = vd_mask_writeback = false;
      };
    }

    // *** Handle vector permutation instructions ...

    if (!insn_handled) {
      insn_handled = true;
      switch (op_funct6) {

        case 0x017:  // vmerge / vmv
          // vmerge is encoded with 'insn_vm == 0'. However, ALL elements are processed,
          //        and v0_mask decides which source to select.
          // vmv    is encoded with 'insn_vm == 1'. all elements are processed correctly.
          //
          for (i = vstart; i < vl; i++) {
            val = eff_mask[i] ? (uint32_t) es1_u : es2_u;
            reg_v_write (vsew, insn_vd, i, val);
          }
          break;

        case 0x110:  // VWXUNARY0 (V), VRXUNARY0 (X)
          if (insn_funct3 == OPMVV && insn_vs1 == 0) {        // vmv.x.s
            rd_val = reg_v_read (vsew, insn_vs2, 0);
            rd_writeback = true;
          }
          else if (insn_funct3 == OPMVX && insn_vs2 == 0) {   // vmv.s.x
            for (i = vstart; i < vl; i++) if (eff_mask[i])
              reg_v_write (vsew, insn_vd, i, rs1_val);
          }
          else insn_handled = false;
          break;

        case 0x00c:  // vrgather
          for (i = vstart; i < vl; i++) if (eff_mask[i]) {
            idx = reg_v_read (vsew, insn_vs1, i);
            if (idx < (uint32_t) vlmax) val = reg_v_read (vsew, insn_vs2, idx);
            else                        val = 0;
            reg_v_write (vsew, insn_vd, i, val);
          }
          break;
        case 0x00e:  // vrgatherei16 (V), vslideup (X, I)
          if (insn_funct3 == OPIVV) {     // vrgatherei16
            for (i = vstart; i < vl; i++) if (eff_mask[i]) {
              idx = reg_v_read (1, insn_vs1, i);    // EEW = 16 => VEEW = 1
              if (idx < (uint32_t) vlmax) val = reg_v_read (vsew, insn_vs2, idx);
              else                        val = 0;
              reg_v_write (vsew, insn_vd, i, val);
            }
          }
          else {                          // vslideup
            // Note 1: Unlike the other slide operations, empty elements (vstart ... scalar_u)
            //         are left unchanged and not filled with 0. This is inconsistent, but
            //         complies with the spec.
            // Note 2: It would be allowed to change the order of the following loop back to
            //         positive order, since the spec does not allow overlapping vs2 and vd.
            for (i = vl - 1; (uint32_t) i >= MAX((uint32_t) vstart, scalar_u); i--) if (eff_mask[i]) {
              val = reg_v_read (vsew, insn_vs2, i - scalar_u);
              reg_v_write (vsew, insn_vd, i, val);
            }
          }
          break;
        case 0x00f:  // vslidedown (X, I)
          for (i = vstart; i < vl; i++) if (eff_mask[i]) {
            if (i + scalar_u < (uint32_t) vl)
              val = reg_v_read (vsew, insn_vs2, i + scalar_u);
            else
              val = 0;
            reg_v_write (vsew, insn_vd, i, val);
          }
          break;
        case 0x10e:  // vslide1up
          // Note: It would be allowed to change the order of the following loop back to
          //       positive order, since the spec does not allow overlapping vs2 and vd.
          for (i = vl - 1; i >= vstart; i--) if (eff_mask[i]) {
            if (i > 0)
              val = reg_v_read (vsew, insn_vs2, i - 1);
            else
              val = rs1_val;
            reg_v_write (vsew, insn_vd, i, val);
          }
          break;
        case 0x10f:  // vslide1down
          for (i = vstart; i < vl; i++) if (eff_mask[i]) {
            if (i + 1 < vl)
              val = reg_v_read (vsew, insn_vs2, i + 1);
            else
              val = rs1_val;
            reg_v_write (vsew, insn_vd, i, val);
          }
          break;

        case 0x117:  // vcompress
          idx = 0;
          for (i = vstart; i < vl; i++) if (vs1_mask[i]) {
            val = reg_v_read (vsew, insn_vs2, i);
            reg_v_write (vsew, insn_vd, idx, val);
            idx++;
          }
          break;

        default:
          insn_handled = false;
      };
    }

    // *** (End of operation handling)

    // Write back reduction accumulator (if applicable) ...
    if (red_u_writeback || red_s_writeback) {
      if (red_s_writeback) red_u = (uint32_t) red_s;
      reg_v_write (vsew + (wide_vd ? 1 : 0), insn_vd, 0, red_u);
    }

    // Write back mask (if applicable) ...
    if (vd_mask_writeback) {
      sc_biguint<PN_CFG_VLEN> data = reg_v_read_full (insn_vd);
      data(PN_CFG_VLEN/8 - 1, 0) = vd_mask;
      reg_v_write_full (insn_vd, data);
    }

    // Write back to scalar register (if applicable) ...
    if (insn_rd == 0) rd_writeback = false;     // do not write to x0
    if (rd_writeback) {
      *ret_rd_val = rd_val;
      *ret_rd_writeback = rd_writeback;
    }
    else
      base_rd_pending = 0;    // release writeback port

    // Simulate initiation delay ...
    //   simulate the duration for submitting all vector elements to the pipeline ...
    int max_parallel_elements = (PN_CFG_PIPE_LANES) * (PN_CFG_ELEN) / sew;
    //~ PN_INFOF (("### els = %i/%i", vl, max_parallel_elements));
    NEXT_CYCLE((vl + max_parallel_elements - 1) / max_parallel_elements);
    if (rd_writeback) {
      // have to write back an x register:
      //   simulate the duration of waiting for the pipeline latency ...
      NEXT_CYCLE(PN_CFG_PIPE_LATENCY);
    }
  }     // Vector Integer Computational Instructions (including fixed-point, reduction, mask and permutation)

  // Reset vstart ...
  reg_vstart = 0;

  // Done ...
  //~ PN_INFO("###"); itrace_print_vregs (false);

  if (!insn_handled) PN_WARNING("Unknown instruction");
  return insn_handled;
}





// ********************** Main Process *****************************************


void m_nucleus_ai_vext::proc_clk_main_soft () {
  uint32_t insn, rd_val;
  bool rd_writeback;

  // Reset ...
  //   ... reset all outputs ...
  base_rdy = 0;
  base_idle = 0;
  base_rd_pending = 0;
  base_rd_stb = 0;
  base_mem_pending = 0;
  dport_stb = 0;

  //   ... reset registers & CSRs ...
  reg_vstart = 0;
  reg_vl = 0;
  reg_vtype_vill = 1;     // see recommendations in section 30.3.11. of the RISC-V Manual Vol. 1
  reg_vtype_vma = 0;
  reg_vtype_vta = 0;
  reg_vtype_vsew = 0;
  reg_vtype_vlmul = 0;
  reg_vxrm = 0;           // round to nearest up (add +0.5 to LSB)
  reg_vxsat = 0;          // accrued saturation (init with "none")

  // Main loop ...
  while (1) {
    base_rdy = 1;
    base_rd_pending = 1;  // keep a hand on an eventual X reg writeback
    base_mem_pending = 1; // keep a hand on an eventual memory access
    base_idle = 1;
    NEXT_CYCLE(1);
    if (base_insn_stb.read ()) {
      base_rdy = 0;
      base_idle = 0;

      // Execute instruction ...
      rd_writeback = false;
      insn = base_insn.read ();
      main_soft_execute_insn (
          insn,
          base_rs1.read ().to_uint (),
          base_rs2.read ().to_uint (),
          &rd_writeback, &rd_val
        );

      // Release DPort (if applicable) ...
      base_mem_pending = 0;

      // Writeback register (if applicable) ...
      if (rd_writeback) {
        base_rd_sel = BIT_GROUP(insn, 11, 7);
        base_rd = rd_val;
        base_rd_stb = 1;
        NEXT_CYCLE(1);
        base_rd_stb = 0;
        base_rd_pending = 0;
        NEXT_CYCLE(1);      // make sure that base_rd_pending is 0 for at least one clock cycle
      }
    }
  }
}
