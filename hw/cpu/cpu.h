/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lukas Bauer <lukas.bauer1@tha.de>
                     2024 Marco Milenkovic <Marco.Milenkovic@tha.de>
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

#ifndef __CPU_H__
#define __CPU_H__

#include <piconut.h>

// Include the header files of the submodules

#ifdef PN_CFG_NUCLEUS_IS_NUCLEUS_REF
#include <nucleus_ref.h>
#endif

#ifdef PN_CFG_NUCLEUS_IS_NUCLEUS_AI
#include <nucleus_ai.h>
#endif

#ifdef PN_CFG_MEMBRANA_IS_MEMBRANA_HW
#include <membrana_hw.h>
#endif

#ifdef PN_CFG_MEMBRANA_IS_MEMBRANA_SOFT
#include <membrana_soft.h>
#endif

#ifdef PN_CFG_MEMBRANA_IS_MEMBRANA_AI
#include <membrana_ai.h>
#endif


SC_MODULE(m_cpu), pn_module_if
{
public:
    // General ports ...
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    // Debug ...
    sc_in<bool> PN_NAME(debug_haltrequest_in);
    sc_out<bool> PN_NAME(debug_haltrequest_ack_out);

    // Interrupts ...
    sc_in<bool> PN_NAME(msip_in);
    sc_in<bool> PN_NAME(mtip_in);
    sc_in<bool> PN_NAME(meip_in);

    // Wishbone interface ...
#ifdef PN_CFG_MEMBRANA_IS_MEMBRANA_HW
    pn_wishbone_master_t wb_master;
#endif

    // CLINT signals ...
    sc_signal<bool> signal_clint_stb;
    sc_signal<bool> signal_clint_we;
    sc_signal<sc_uint<4>> signal_clint_bsel;
    sc_signal<sc_uint<32>> signal_clint_addr;
    sc_signal<sc_uint<32>> signal_clint_wdata;
    sc_signal<sc_uint<32>> signal_clint_rdata;
    sc_signal<bool> signal_clint_ack;

    // Constructor/Destructors ...
    SC_CTOR(m_cpu)
#ifdef PN_CFG_MEMBRANA_IS_MEMBRANA_HW
        : wb_master{
              .alen = 32,
              .dlen = 32}
#endif

    {
#ifdef PN_CFG_MEMBRANA_IS_MEMBRANA_HW
        pn_add_wishbone_master(&wb_master);
#endif

#if !PN_PRESYNTHESIZED_H_ONLY(CPU)

#ifdef PN_CFG_NUCLEUS_IS_NUCLEUS_REF
        nucleus = sc_new<m_nucleus_ref>("nucleus");
#endif

#ifdef PN_CFG_NUCLEUS_IS_NUCLEUS_AI
        nucleus = sc_new<m_nucleus_ai>("nucleus");
#endif

#ifdef PN_CFG_MEMBRANA_IS_MEMBRANA_HW
        membrana = sc_new<m_membrana_hw>("membrana");
#endif

#ifdef PN_CFG_MEMBRANA_IS_MEMBRANA_SOFT
        membrana = sc_new<m_membrana_soft>("membrana");
#endif

#ifdef PN_CFG_MEMBRANA_IS_MEMBRANA_AI
        membrana = sc_new<m_membrana_ai>("membrana");
#endif

        init_submodules();
        // Add the connections between the submodules here. Could be nessessary instead of init_submodules();
    }

    // Functions
    void pn_trace(sc_trace_file * tf, int level = 1);
    bool state_is_not_halt();

    // Processes

    // Submodules
#ifdef PN_CFG_NUCLEUS_IS_NUCLEUS_REF
    m_nucleus_ref* nucleus;
#endif

#ifdef PN_CFG_NUCLEUS_IS_NUCLEUS_AI
    m_nucleus_ai *nucleus;
#endif

#ifdef PN_CFG_MEMBRANA_IS_MEMBRANA_SOFT
    m_membrana_soft* membrana;
#endif

#ifdef PN_CFG_MEMBRANA_IS_MEMBRANA_HW
    m_membrana_hw* membrana;
#endif

#ifdef PN_CFG_MEMBRANA_IS_MEMBRANA_AI
    m_membrana_ai *membrana;
#endif

#else // !PN_PRESYNTHESIZED_H_ONLY(CPU)
    }

    void pn_trace(sc_trace_file * tf, int level = 1) {}

#endif // !PN_PRESYNTHESIZED_H_ONLY(CPU)

protected:
#if !PN_PRESYNTHESIZED_H_ONLY(CPU)

    // Internal Signals
    // ------------ Interconnect Signals ------------
    // ------------ IPort Signals ------------
    sc_vector<sc_signal<bool>> PN_NAME_VEC(stb_iport, PN_CFG_CPU_CORES);
    sc_vector<sc_signal<bool>> PN_NAME_VEC(ack_iport, PN_CFG_CPU_CORES);
    sc_vector<sc_signal<sc_uint<32>>> PN_NAME_VEC(adr_iport, PN_CFG_CPU_CORES);
    sc_vector<sc_signal<sc_uint<32>>> PN_NAME_VEC(rdata_iport, PN_CFG_CPU_CORES);
    sc_vector<sc_signal<sc_uint<4>>> PN_NAME_VEC(bsel_iport, PN_CFG_CPU_CORES);

    // ------------ DPort Signals ------------
    sc_vector<sc_signal<bool>> PN_NAME_VEC(stb_dport, PN_CFG_CPU_CORES);
    sc_vector<sc_signal<bool>> PN_NAME_VEC(we_dport, PN_CFG_CPU_CORES);
    sc_vector<sc_signal<bool>> PN_NAME_VEC(lrsc_dport, PN_CFG_CPU_CORES);
    sc_vector<sc_signal<bool>> PN_NAME_VEC(amo_dport, PN_CFG_CPU_CORES);
    sc_vector<sc_signal<bool>> PN_NAME_VEC(ack_dport, PN_CFG_CPU_CORES);
    sc_vector<sc_signal<sc_uint<32>>> PN_NAME_VEC(adr_dport, PN_CFG_CPU_CORES);
    sc_vector<sc_signal<sc_uint<32>>> PN_NAME_VEC(wdata_dport, PN_CFG_CPU_CORES);
    sc_vector<sc_signal<sc_uint<32>>> PN_NAME_VEC(rdata_dport, PN_CFG_CPU_CORES);
    sc_vector<sc_signal<sc_uint<4>>> PN_NAME_VEC(bsel_dport, PN_CFG_CPU_CORES);

    // Methods
    void init_submodules();

#else  // !PN_PRESYNTHESIZED_H_ONLY(CPU)
    PN_PRESYNTHESIZED;
#endif // !PN_PRESYNTHESIZED_H_ONLY(CPU)
};

#endif // __PICONUT_H__
