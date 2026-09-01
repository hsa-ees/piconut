/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description: Headers of the AI-optimized Membrana

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

/*
  MembranaAI has the following features and properties:
  - support for a configurable and wide DPort data bus for fast vecorized transfers
  - support for a second DPort (VPort) for vector accesses with a width independent of the man DPort's witdh
  - support for overlapping transfers
  - soft variant optimized for simuliation performance
  - soft variant with integrated soft UART

  The following features are not implemented (yet):
  - synthesis
  - support for RV64
  - support for more than one core
  - Amo extension: Conditional stores always succeed
*/

#ifndef __MEMBRANA_AI_H__
#define __MEMBRANA_AI_H__

#include <piconut.h>





/** @brief Base address of the UART for basic I/O.
 */
#ifndef PN_CFG_UART_BASE
#define PN_CFG_UART_BASE 0x30000000
#endif


// Sanity ...
//   Clarify supported configurations ...
#if PN_CFG_CPU_CORES != 1
#error "Multiple cores are presently unsupported."
#endif


SC_MODULE(m_membrana_ai) {
  public:

    // General ports ...
    sc_in_clk     PN_NAME(clk);
    sc_in<bool>   PN_NAME(reset);

    // Instruction Ports (IPort) ...
    sc_vector<sc_in<pn_iport_adr_t>>  PN_NAME_VEC(iport_adr, PN_CFG_CPU_CORES);
    sc_vector<sc_in<pn_iport_bsel_t>> PN_NAME_VEC(iport_bsel, PN_CFG_CPU_CORES);
    sc_vector<sc_out<pn_iport_dat_t>> PN_NAME_VEC(iport_rdata, PN_CFG_CPU_CORES);

    sc_vector<sc_in<bool>>            PN_NAME_VEC(iport_stb, PN_CFG_CPU_CORES);
    sc_vector<sc_out<bool>>           PN_NAME_VEC(iport_ack, PN_CFG_CPU_CORES);

    // Data Ports (DPort) ...
    sc_vector<sc_in<pn_dport_adr_t>>  PN_NAME_VEC(dport_adr, PN_CFG_CPU_CORES);
    sc_vector<sc_in<pn_dport_bsel_t>> PN_NAME_VEC(dport_bsel, PN_CFG_CPU_CORES);
    sc_vector<sc_out<pn_dport_dat_t>> PN_NAME_VEC(dport_rdata, PN_CFG_CPU_CORES);
    sc_vector<sc_in<pn_dport_dat_t>>  PN_NAME_VEC(dport_wdata, PN_CFG_CPU_CORES);

    sc_vector<sc_in<bool>>            PN_NAME_VEC(dport_stb, PN_CFG_CPU_CORES);
    sc_vector<sc_in<bool>>            PN_NAME_VEC(dport_we, PN_CFG_CPU_CORES);
    sc_vector<sc_out<bool>>           PN_NAME_VEC(dport_ack, PN_CFG_CPU_CORES);

    sc_vector<sc_in<bool>>            PN_NAME_VEC(dport_lrsc, PN_CFG_CPU_CORES);
    sc_vector<sc_in<bool>>            PN_NAME_VEC(dport_amo, PN_CFG_CPU_CORES);

    // Vector Data Ports (VPort) ...
    sc_vector<sc_in<pn_dport_adr_t>>  PN_NAME_VEC(vport_adr, PN_CFG_CPU_CORES);
    sc_vector<sc_in<pn_dport_bsel_t>> PN_NAME_VEC(vport_bsel, PN_CFG_CPU_CORES);
    sc_vector<sc_out<pn_dport_dat_t>> PN_NAME_VEC(vport_rdata, PN_CFG_CPU_CORES);
    sc_vector<sc_in<pn_dport_dat_t>>  PN_NAME_VEC(vport_wdata, PN_CFG_CPU_CORES);

    sc_vector<sc_in<bool>>            PN_NAME_VEC(vport_stb, PN_CFG_CPU_CORES);
    sc_vector<sc_in<bool>>            PN_NAME_VEC(vport_we, PN_CFG_CPU_CORES);
    sc_vector<sc_out<bool>>           PN_NAME_VEC(vport_ack, PN_CFG_CPU_CORES);

    sc_vector<sc_in<bool>>            PN_NAME_VEC(vport_lrsc, PN_CFG_CPU_CORES);
    sc_vector<sc_in<bool>>            PN_NAME_VEC(vport_amo, PN_CFG_CPU_CORES);

    // Constructor & destructor (inline) ...
    SC_HAS_PROCESS (m_membrana_ai);
    m_membrana_ai (
        sc_module_name name,
        int _iport_alen = 32, int _iport_dlen = 32,
        int _dport_alen = 32, int _dport_dlen = 32, int _vport_dlen = 32
      )
      : sc_module (name) {

      SC_CTHREAD (proc_clk_main_soft, clk.pos());
      reset_signal_is (reset, true);

      uart_regs = NULL;
      init (_iport_alen, _iport_dlen, _dport_alen, _dport_dlen, _vport_dlen);
    }
    ~m_membrana_ai ();

    // Tracing ...
    void pn_trace (sc_trace_file * tf, int level = 1);

    // Initialize memory from ELF file ...
    void load_elf (const char *elf_name);

    // Main process ...
    void proc_clk_main_soft ();

  protected:

    // Init / done ...
    void init (int _iport_alen = 32, int _iport_dlen = 32, int _dport_alen = 32, int _dport_dlen = 32, int _vport_dlen = 32);

    // Access helpers ...
    //   Encapsulate memory accesses, optimized for 32 bit accesses.
    uint32_t read32 (uint32_t adr, int bsel);
    void write32 (uint32_t adr, int bsel, uint32_t val);

    // Configuration ...
    int iport_alen, iport_dlen;
    int dport_alen, dport_dlen, vport_dlen;

    // Local memory contents ...
    uint8_t *emem;
    uint32_t emem_base, emem_end;

    // Integrated (soft) UART ...
    struct pn_uart_regs_s *uart_regs;

    // Profile data memory access ...
    unsigned long long dport_count = 0;

};   // SC_MODULE(m_membrana_ai)


#endif // __MEMBRANA_AI_H__
