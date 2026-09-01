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


/** @brief Nucleus AI - Vector Extension Unit
 *
 * The vector extension unit implements the Zve32x extension for variable VLEN
 * values.
 *
 * Known limitations are:
 *
 * - The module is not exhaustively tested and just meant as a proof-of-concept.
 *
 * - The following load/store operatons are not implemented (yet):
 *     - Vector load/store segment instructions
 *     - Unit-stride whole register load/store
 *     - Unit-stride mask load/store, EEW=8
 *     - Unit-stride fault-only-first load
 *
 * - Fixed-point arithmetic operations are not implemented (yet).
 *
 * - No exceptions are generated.
 *
 * Not-implemented-yet Zve32x instructions raise assertions, so that this becomes
 * immediately visible.
 *
 * Note on timing of the soft module:
 *
 * - The model does not simulate the behaviour of a pipeline, but models execution
 *   times leading to a timing close to such a behaviour.
 *
 * - The duration of memory accesses depends on the accesses themselves, DPort
 *   accesses are accurate.
 *
 * - The duration of all computational operations is simulated as
 *
 *     ceil (VL / (PN_CFG_PIPE_LANES * ELEN/SEW))
 *
 *   clocks, which is the initiation interval of the computation pipeline,
 *   depending on the selected vector length (VL, including LMUL), the
 *   data type / single element width (SEW), the hardware element width (ELEN,
 *   usually =32) and the number of parallel pipeline lanes (PN_CFG_PIPE_LANES)
 *   For instruction with a scalar destination (RD), the pipeline latency
 *   (PN_CFG_PIPE_LATENCY) is added.
 */


#pragma once

#include <piconut.h>





// ********************** Configuraton *****************************************


#define PN_CFG_VLEN 1024
#define PN_CFG_ELEN   32        // Note: Values different from 32 are currently not supported

#define PN_CFG_PIPE_LANES 4     // Number of parallel pipeline lanes, each ELEN wide
#define PN_CFG_PIPE_LATENCY 10  // Pipeline latency (for performance estimation)





// ********************** Helpers **********************************************


#define BIT_GROUP(val, i1, i0) (((val) >> i0) & ((1 << (i1 + 1 - i0)) - 1))
  ///< Get the group of bits (i1, i0), both i1 and i0 included.
#define BIT(val, i) (((val) >> i) & 1)
  ///< Get bit i (delivers 0 or 1).
#define ZERO_EXTEND(val, i) ((val) & ((1 << (i + 1)) - 1))
  ///< Zero-extend a value, i is the index of the first bit to keep.
#define SIGN_EXTEND(val, i) ((val) - (((val) & (1 << (i))) << 1))
  ///< Sign-extend a value, i is the index of the sign bit.





// ********************** CSR Slave Interface **********************************


class m_csr_slave {
  public:

    // CSR bus ports ...
    sc_in<bool>         PN_NAME(csr_bus_read_en_in);     // read enable
    sc_in<bool>         PN_NAME(csr_bus_write_en_in);    // write enable
    sc_in<sc_uint<2>>   PN_NAME(csr_bus_write_mode_in);  // write mode (see `pn_csr_write_mode_e`)
    sc_in<sc_uint<12>>  PN_NAME(csr_bus_adr_in);         // address
    sc_in<sc_uint<32>>  PN_NAME(csr_bus_wdata_in);       // data to write
    sc_out<sc_uint<32>> PN_NAME(csr_bus_rdata_out);      // data read (wired "or")

  protected:
    void pn_trace (sc_trace_file *tf, int level = 1);
      ///< @brief trace the signals, to be called from subclass's pn_trace().
    void on_clk ();
      ///< @brief Handle all actions to be performed on a clock edge.
      ///
      /// This method can be called from CTHREADs and performs all tasks to be
      /// to done to server the CSR bus interface. It calls csr_read() and csr_write()
      /// internally as adequate.
      ///
      /// This method is synthesizable given that csr_read() and csr_write() do
      /// not contain non-synthesizable code.
    void proc_clk_main ();
      ///< @brief Main SystemC process (optional).
      ///
      /// Can be activated a s SC_CTHREAD and will then autonomously call on_clk().

    virtual uint32_t csr_read (int idx, bool passive = false) = 0;
      ///< @brief Read a CSR.
      ///
      /// If 'passive' is set, do not perform any side actions (e.g. just get the current value for internal purposes)
    virtual void csr_write (int idx, int mode, uint32_t val) = 0;
      ///< @brief write to a CSR.
};





// ********************** V Extension Component ********************************


typedef struct mem_access_s {
  bool write;
  uint32_t adr;
  uint32_t bsel;
  pn_dport_dat_t data;

  struct mem_access_s *next;
} mem_access_t;


class m_nucleus_ai_vext: sc_module, public m_csr_slave {
  public:

    // Ports ...

    // ... general ...
    sc_in_clk               PN_NAME(clk);
    sc_in<bool>             PN_NAME(reset);

    // ... base processor communication ...
    sc_out<bool>            PN_NAME(base_rdy);        ///< is ready to accept/issue a new instruction
    sc_out<bool>            PN_NAME(base_idle);       ///< all pending instructions are completed
    sc_in<bool>             PN_NAME(base_insn_stb);   ///< start a new vector instruction (passed on `base_insn`)
    sc_in<sc_uint<32>>      PN_NAME(base_insn);       ///< instruction word (valid on `insn_stb` = 1)

    sc_in<sc_uint<32>>      PN_NAME(base_rs1);        ///< scalar source register 1 content (valid on `insn_stb` set)
    sc_in<sc_uint<32>>      PN_NAME(base_rs2);        ///< scalar source register 2 content (valid on `insn_stb` set)
    sc_out<bool>            PN_NAME(base_rd_pending); ///< instruction(s) with a scalar destination may be pending
    sc_out<bool>            PN_NAME(base_rd_stb);     ///< scalar destination register strobe
    sc_out<sc_uint<5>>      PN_NAME(base_rd_sel);     ///< scalar destination register number
    sc_out<sc_uint<32>>     PN_NAME(base_rd);         ///< scalar destination register content

    sc_out<bool>            PN_NAME(base_mem_pending);   ///< memory write may be pending

    // ... DPort ...
    sc_out<bool>            PN_NAME(dport_stb);
    sc_out<bool>            PN_NAME(dport_we);
    sc_in<bool>             PN_NAME(dport_ack);

    sc_out<pn_dport_adr_t>  PN_NAME(dport_adr);
    sc_out<pn_dport_bsel_t> PN_NAME(dport_bsel);
    sc_in<pn_dport_dat_t>   PN_NAME(dport_rdata);
    sc_out<pn_dport_dat_t>  PN_NAME(dport_wdata);

    // Constructor & destructor ...
    SC_CTOR(m_nucleus_ai_vext) {
      //~ init_submodules ();

      SC_CTHREAD(proc_clk_csr_slave, clk.pos());
      reset_signal_is (reset, true);

      SC_CTHREAD(proc_clk_main_soft, clk.pos());
      reset_signal_is (reset, true);

      mem_init ();
      reg_v_init ();

      itrace_init ();
    }
    ~m_nucleus_ai_vext ();

    // Configuration (elaboration phase) ...
    //~ void conf () { cfg_ = ; }

    // Tracing ...
    void pn_trace (sc_trace_file * tf, int level = 1);

    void itrace_init ();
    void itrace_print_vregs (bool all = false);   // by default, only registers changed after last call are printed

    // Processes ...
    void proc_clk_csr_slave () { m_csr_slave::proc_clk_main (); }
    void proc_clk_main_soft ();

    // Methods (synthesizable) ...
    bool insn_is_v (uint32_t insn);   // return whether an instruction is part of the V extension

  protected:

    // Configuration ...

    // Tracing ...
    bool itrace_vreg_changed[32];

    // Submodules ...
    //~ void init_submodules ();      // instantiate and connect all submodules

    // Main vector registers ...
    sc_vector<sc_signal<sc_biguint<PN_CFG_VLEN>>> PN_NAME_VEC(reg_v, 32);

    // State registers / CSRs ...
    sc_signal<sc_uint<32>>  reg_vstart;
    sc_signal<sc_uint<32>>  reg_vl;
    //~ sc_signal<sc_uint<32>> reg_vtype;
    sc_signal<bool>         reg_vtype_vill;   // illegal vtype
    sc_signal<bool>         reg_vtype_vma;    // vector mask agnostic (presently unused - "undisturbed" mode used)
    sc_signal<bool>         reg_vtype_vta;    // vector tail agnostic (presently unused - "undisturbed" mode used)
    sc_signal<sc_uint<2>>   reg_vtype_vsew;   // selected element width (SEW) (8, 16, 32, 64)
    sc_signal<sc_uint<3>>   reg_vtype_vlmul;  // vector length multiplier (VLMUL); LMUL = 2^VLMUL
    sc_signal<sc_uint<2>>   reg_vxrm;         // vector fixed-point rounding mode
    sc_signal<bool>         reg_vxsat;        // vector fixed-point saturation flag

    // CSR methods ...
    uint32_t csr_read (int idx, bool passive = false);
      // read a CSR; if 'passive' is set, do not perform any side actions (e.g. just get the current value for internal purposes)
    void csr_write (int idx, int mode, uint32_t val);
      // write to a CSR

    // Soft model: data memory access ...
    //   In a hardware implementation, there may be two modules operating in parallel:
    //   The address generator may behave like mem_submit(), and the data receiving module may
    //   behave similar to mem_complete(). In the soft model, all submissions are generated first
    //   and stored in a linked list, followed by all completions.
    mem_access_t *mem_first, **mem_p_last, *mem_completed;  // logical accesses
    mem_access_t mem_dport_completed, mem_dport_strobed;    // current physical accesses (merged & aligned) (bsel == 0 <=> invalid)

    void mem_init ();
      ///< Init the memory access submodule.
    void mem_done () { mem_clear (); }
      ///< Cleanup the memory access submodule.
    void mem_clear ();
      ///< Assert that there are no pending DPort accesses and cleanup internal data structures.
    void mem_submit (uint32_t _adr, int bsel, bool _write, uint32_t _data = 0);
      ///< Submit a memory access (read or write). In this implementation, this never consumes clock cycles.
    void mem_submit32 (uint32_t _adr, bool _write, uint32_t _data = 0) { mem_submit (_adr, 0xf, _write, _data); }
    void mem_submit16 (uint32_t _adr, bool _write, uint32_t _data = 0) { mem_submit (_adr, 3, _write, _data); }
    void mem_submit8  (uint32_t _adr, bool _write, uint32_t _data = 0) { mem_submit (_adr, 1, _write, _data); }
    mem_access_t *mem_complete ();
      ///< Wait for a memory access (read or write) to complete and return its structure, which may contain the read data.
      /// This may consume clock cycles, depending on the DPorts peer behaviour.
    uint32_t mem_complete32 () { return mem_complete ()->data.to_uint (); }
    uint16_t mem_complete16 (uint32_t adr) { return (uint16_t) mem_complete ()->data.to_uint (); }
    uint8_t mem_complete8 (uint32_t adr) { return (uint8_t) mem_complete ()->data.to_uint (); }

    // Soft model: vector register file access ...
    sc_biguint<PN_CFG_VLEN> reg_v_next[32];
    bool reg_v_changed[32];

    void reg_v_init () { for (int v = 0; v < 32; v++) reg_v_changed[v] = false; }
    void reg_v_next_cycle ();   // writeback 'reg_v_next' into real registers on a clock cycle
    sc_biguint<PN_CFG_VLEN> reg_v_read_full (int v) { return reg_v[v].read (); }
    void reg_v_write_full (int v, sc_biguint<PN_CFG_VLEN> mask);
    uint32_t reg_v_read (int vsew, int v, int e);
      ///< Read element `e` of vector register `v`. Result is zero-extended.
      /// Register group accesses are allowed , `e` is not validated against LMUL.
    void reg_v_write (int vsew, int v, int e, uint32_t val);
      ///< Write element `e` of vector register `v`.
      /// Register group accesses are allowed, `e` is not validated against LMUL.
    bool reg_v_mask_read (int v, int e) { return reg_v_read (-3, v, e) ? true : false; }
    void reg_v_mask_write (int v, int e, bool val) { reg_v_write (-3, v, e, val ? 1 : 0); }

    // Soft model: main mathods ...
    void main_soft_next_cycle (int n = 1);
    bool main_soft_execute_insn (uint32_t insn, uint32_t rs1_val, uint32_t rs2_val, bool *ret_rd_writeback, uint32_t *ret_rd_val);
};
