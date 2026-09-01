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

/** @brief Nucleus AI
 *
 * The Nucleus AI is optimized for AI applications with the following properties:
 * - high simulation performance in its soft hardware variant
 * - pipelined memory access at IPort
 * - pipelined memory access at DPort for vector operations
 * - prepared to support Vector (RVV) and matrix extensions (IME/AME/VME)
 * - instruction trace support
 * - optional extensions: Zicsr, M
 *
 */

#pragma once

#include <piconut.h>





// ********************** Configuraton *****************************************


// TBD: In the future, these settings can also be overwritten at runtime.


/// @brief Nucleus supports the Zicsr extension.
#ifndef PN_CFG_NUCLEUS_WITH_ZICSR
#define PN_CFG_NUCLEUS_WITH_ZICSR 1
#endif


/// @brief Nucleus supports the M extension.
#ifndef PN_CFG_NUCLEUS_WITH_M
#define PN_CFG_NUCLEUS_WITH_M 1
#endif


/// @brief Nucleus supports the V extension (only Zve32x).
#ifndef PN_CFG_NUCLEUS_WITH_V
#define PN_CFG_NUCLEUS_WITH_V 1
#endif


/// @brief Nucleus supports the A extension (not implemented).
#ifndef PN_CFG_NUCLEUS_WITH_A
#define PN_CFG_NUCLEUS_WITH_A 0
#endif


/// @brief Use IPort/DPort pipeline mode (the Membrana must support that).
#ifndef PN_CFG_NUCLEUS_USE_PORT_PIPELINING
#define PN_CFG_NUCLEUS_USE_PORT_PIPELINING 1
#endif


/// @brief Use second DPort for vector data accesses (the Membrana must support that).
#ifndef PN_CFG_NUCLEUS_AI_USE_VPORT
#define PN_CFG_NUCLEUS_AI_USE_VPORT 1
#endif





// ********************** Nucleus AI Component *********************************


SC_MODULE(m_nucleus_ai) {
  public:

    // Ports ...

    // ... general ...
    sc_in_clk               PN_NAME(clk);
    sc_in<bool>             PN_NAME(reset);
    sc_out<bool>            PN_NAME(pwroff);  ///< '1' indicates that the CPU is in a final halt state, and power can be turned off

    // ... IPort ...
    sc_out<bool>            PN_NAME(iport_stb);
    sc_in<bool>             PN_NAME(iport_ack);

    sc_out<pn_iport_adr_t>  PN_NAME(iport_adr);
    sc_out<pn_iport_bsel_t> PN_NAME(iport_bsel);
    sc_in<pn_iport_dat_t>   PN_NAME(iport_rdata);

    // ... DPort ...
    sc_out<bool>            PN_NAME(dport_stb);
    sc_out<bool>            PN_NAME(dport_we);
    sc_in<bool>             PN_NAME(dport_ack);
    sc_out<bool>            PN_NAME(dport_lrsc);
    sc_out<bool>            PN_NAME(dport_amo);

    sc_out<pn_dport_adr_t>  PN_NAME(dport_adr);
    sc_out<pn_dport_bsel_t> PN_NAME(dport_bsel);
    sc_in<pn_dport_dat_t>   PN_NAME(dport_rdata);
    sc_out<pn_dport_dat_t>  PN_NAME(dport_wdata);

    // ... VPort ...
    sc_out<bool>            PN_NAME(vport_stb);
    sc_out<bool>            PN_NAME(vport_we);
    sc_in<bool>             PN_NAME(vport_ack);
    sc_out<bool>            PN_NAME(vport_lrsc);
    sc_out<bool>            PN_NAME(vport_amo);

    sc_out<pn_dport_adr_t>  PN_NAME(vport_adr);
    sc_out<pn_dport_bsel_t> PN_NAME(vport_bsel);
    sc_in<pn_dport_dat_t>   PN_NAME(vport_rdata);
    sc_out<pn_dport_dat_t>  PN_NAME(vport_wdata);

    // ... interrupts (CLINT) ...
    sc_in<bool>             PN_NAME(msip_in);
    sc_in<bool>             PN_NAME(mtip_in);
    sc_in<bool>             PN_NAME(meip_in);

    // ... debugging ...
    sc_in<bool>             PN_NAME(debug_halt_req);
    sc_out<bool>            PN_NAME(debug_halt_ack);

#if !PN_PRESYNTHESIZED_H_ONLY(NUCLEUS_AI)

    // Constructor & destructor ...
    SC_CTOR(m_nucleus_ai) {
      //~ PN_INFO("### m_nucleus_ai created.");
      cfg_latency_mul = 3;
      cfg_latency_div = 32;

      i_vext = NULL;
      init_submodules ();

      SC_CTHREAD(proc_clk_main_soft, clk.pos());
      reset_signal_is (reset, true);

      if (PN_CFG_NUCLEUS_AI_USE_VPORT) {
        SC_METHOD(proc_cmb_dport_direct);
          sensitive << base_dport_stb << base_dport_we << base_dport_adr << base_dport_bsel << base_dport_wdata
                    << dport_ack;
      }
      else {
        SC_METHOD(proc_cmb_dport_mux);
          sensitive << base_dport_stb << base_dport_we << base_dport_adr << base_dport_bsel << base_dport_wdata
                    << vext_dport_stb << vext_dport_we << vext_dport_adr << vext_dport_bsel << vext_dport_wdata
                    << dport_ack
                    << reg_dport_vext;
      }

      itrace_init ();
    }
    ~m_nucleus_ai ();

    // Configuration (elaboration phase) ...
    void conf_m (int _cfg_latency_mul, int _cfg_latency_div) { cfg_latency_mul = _cfg_latency_mul; cfg_latency_div = _cfg_latency_div; }
      ///< @brief Configure the M extension.

    // Tracing ...
    void pn_trace (sc_trace_file * tf, int level = 1);

    // Processes ...
    void proc_clk_main_soft ();
    void proc_cmb_dport_direct ();
    void proc_cmb_dport_mux ();

#else // !PN_PRESYNTHESIZED_H_ONLY(NUCLEUS_AI)

    SC_CTOR(m_nucleus_ai) {}
    void pn_trace(sc_trace_file * tf, int level = 1) {}

#endif // !PN_PRESYNTHESIZED_H_ONLY(NUCLEUS_AI)

  protected:

#if !PN_PRESYNTHESIZED_H_ONLY(NUCLEUS_AI)

    // Configuration ...
    int cfg_latency_mul, cfg_latency_div;

    // Tracing ...
    bool itrace_xreg_changed[32];

    void itrace_init () { for (int i = 0; i < 32; i++) itrace_xreg_changed[i] = false; }
    void itrace_instruction (uint32_t pc, uint32_t insn);   // print instruction trace entry, depending on `pn_cfg_itrace_level`
    void itrace_print_regs (bool all = false);             // by default, only registers changed after last call are printed                      // print registers for instruction trace, depending on `pn_cfg_itrace_level`

    // Submodules ...
    void init_submodules ();      // instantiate and connect all submodules

    // DPort signals of the core processor ...
    sc_signal<bool>             PN_NAME(base_dport_stb);
    sc_signal<bool>             PN_NAME(base_dport_ack);
    sc_signal<bool>             PN_NAME(base_dport_we);
    sc_signal<pn_dport_adr_t>   PN_NAME(base_dport_adr);
    sc_signal<pn_dport_bsel_t>  PN_NAME(base_dport_bsel);
    sc_signal<pn_dport_dat_t>   PN_NAME(base_dport_wdata);

    // Main processor registers ...
    sc_signal<sc_uint<32>>            PN_NAME(reg_pc);
    sc_signal<sc_uint<32>>            PN_NAME(reg_ir);
    sc_vector<sc_signal<sc_uint<32>>> PN_NAME_VEC(reg_x, 32);

    sc_signal<bool>         PN_NAME(reg_dport_vext);      // DPort is assigned to VExt submodule

    // CSR registers (local) ...
    //   ... Debug Mode Registers ...
    sc_signal<sc_uint<32>>  PN_NAME(reg_csr_dcsr);
    sc_signal<sc_uint<32>>  PN_NAME(reg_csr_dpc);
    sc_signal<sc_uint<32>>  PN_NAME(reg_csr_dscratch0);
    sc_signal<sc_uint<32>>  PN_NAME(reg_csr_dscratch1);

    sc_signal<sc_uint<64>>  PN_NAME(reg_csr_cycle);
    sc_signal<sc_uint<64>>  PN_NAME(reg_csr_instret);

    // CSR bus master signals (for internal modules) ...
    sc_signal<bool>         PN_NAME(csr_bus_read_en_out);     // read enable
    sc_signal<bool>         PN_NAME(csr_bus_write_en_out);    // write enable
    sc_signal<sc_uint<2>>   PN_NAME(csr_bus_write_mode_out);  // write mode (see `pn_csr_write_mode_e`)
    sc_signal<sc_uint<12>>  PN_NAME(csr_bus_adr_out);         // address
    sc_signal<sc_uint<32>>  PN_NAME(csr_bus_wdata_out);       // data to write
    sc_signal<sc_uint<32>>  PN_NAME(csr_bus_rdata_in);        // data read (wired "or")

    // AI Extension interface ...
    class m_nucleus_ai_vext *i_vext;

    sc_signal<bool>         PN_NAME(vext_rdy);        ///< extension is ready to accept/issue a new instruction
    sc_signal<bool>         PN_NAME(vext_idle);       ///< all pending instructions are completed
    sc_signal<bool>         PN_NAME(vext_insn_stb);   ///< start a new vector instruction (passed on `vext_insn`)
    sc_signal<sc_uint<32>>  PN_NAME(vext_insn);       ///< RISC-V instruction word (valid on stb = 1)

    sc_signal<sc_uint<32>>  PN_NAME(vext_rs1);        ///< scalar source register 1 content (valid on `insn_stb` set)
    sc_signal<sc_uint<32>>  PN_NAME(vext_rs2);        ///< scalar source register 2 content (valid on `insn_stb` set)
    sc_signal<bool>         PN_NAME(vext_rd_pending); ///< instruction(s) with a scalar destination pending
    sc_signal<bool>         PN_NAME(vext_rd_stb);     ///< scalar destination register strobe
    sc_signal<sc_uint<5>>   PN_NAME(vext_rd_sel);     ///< scalar destination register number
    sc_signal<sc_uint<32>>  PN_NAME(vext_rd);         ///< scalar destination register content

    sc_signal<bool>         PN_NAME(vext_mem_pending);///< memory write pending

    //~ sc_signal<sc_uint<32>>      PN_NAME(vext_csr_bus_rdata_in);   // data read from CSR bus (presently short-cut to `csr_bus_rdata_in`, since this is the only submodule with CSRs)

    sc_signal<bool>             PN_NAME(vext_dport_stb);
    sc_signal<bool>             PN_NAME(vext_dport_ack);
    sc_signal<bool>             PN_NAME(vext_dport_we);
    sc_signal<pn_dport_adr_t>   PN_NAME(vext_dport_adr);
    sc_signal<pn_dport_bsel_t>  PN_NAME(vext_dport_bsel);
    sc_signal<pn_dport_dat_t>   PN_NAME(vext_dport_wdata);

    // CSR methods ...
    uint32_t csr_read (int idx, bool passive = false);
      // read a CSR; if 'passive' is set, do not perform any side actions (e.g. just get the current value for internal purposes)
    void csr_write (int idx, int mode, uint32_t val);
      // write to a CSR

    // Soft model helpers and variables ...
    uint32_t reg_x_read (int i) { return i > 0 ? reg_x[i].read().to_uint() : (uint32_t) 0; }
    void reg_x_write (int i, uint32_t val);

    void main_soft_next_cycle (int n = 1);
    void main_soft_execute_insn (uint32_t insn, uint32_t pc, uint32_t *ret_next_pc);

    bool got_iport_ack;
    uint32_t got_iport_rdata;
    
    // Profile instruction memory access ...
    unsigned long long iport_count = 0;

#else  // !PN_PRESYNTHESIZED_H_ONLY(NUCLEUS_AI)
    PN_PRESYNTHESIZED;
#endif // !PN_PRESYNTHESIZED_H_ONLY(NUCLEUS_AI)
};
