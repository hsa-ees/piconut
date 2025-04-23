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

#ifndef __PICONUT_H__
#define __PICONUT_H__

#include <systemc.h>
#include "elab_alloc.h"
#include "base.h"

// Include the header files of the submodules

#ifdef __SIMONLYMEMU__
#include "softmemu.h"
#endif

#ifdef __HW_MEMU__
#include "hw_memu.h"
#endif

#ifdef __MINIMALNUCLEUS__
#include "nucleus.h"
#endif

SC_MODULE(m_piconut)
{
public:
    // ------------ Ports ------------
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    // Debug
    sc_in<bool> PN_NAME(debug_haltrequest_in);
    sc_out<bool> PN_NAME(debug_haltrequest_ack_out);

    // ------------ Wishbone Interface ------------
#ifdef __HW_MEMU__
    sc_in<bool> PN_NAME(wb_ack_i);
    sc_in<sc_uint<32>> PN_NAME(wb_dat_i);

    sc_out<sc_uint<32>> PN_NAME(wb_adr_o);
    sc_out<sc_uint<32>> PN_NAME(wb_dat_o);
    sc_out<bool> PN_NAME(wb_we_o);
    sc_out<bool> PN_NAME(wb_stb_o);
    sc_out<bool> PN_NAME(wb_cyc_o);
    sc_out<sc_uint<4>> PN_NAME(wb_sel_o);
#endif

    // Constructor/Destructors
    SC_CTOR(m_piconut)
    {
#ifdef __MINIMALNUCLEUS__
        nucleus = sc_new<m_nucleus>("nucleus");
#endif

#ifdef __HW_MEMU__
        hw_memu = sc_new<m_hw_memu>("hw_memu");
#endif

#ifdef __SIMONLYMEMU__
        simmemu = sc_new<m_soft_memu>("simmemu");
#endif
        init_submodules();
        // Add the connections between the submodules here. Could be nessessary instead of init_submodules();
    }

    // Functions
    void Trace(sc_trace_file * tf, int level = 1);
    bool state_is_not_halt();

    // Processes

    // Submodules
#ifdef __SIMONLYMEMU__
    m_soft_memu* simmemu;
#endif

#ifdef __MINIMALNUCLEUS__
    m_nucleus* nucleus;
#endif

#ifdef __HW_MEMU__
    m_hw_memu* hw_memu;
#endif

protected:
    // Internal Signals
    // ------------ Interconnect Signals ------------
    // ------------ IPort Signals ------------
    sc_signal<bool> PN_NAME(stb_iport);
    sc_signal<bool> PN_NAME(ack_iport);
    sc_signal<sc_uint<32>> PN_NAME(adr_iport);
    sc_signal<sc_uint<32>> PN_NAME(rdata_iport);
    sc_signal<sc_uint<4>> PN_NAME(bsel_iport);

    // ------------ DPort Signals ------------
    sc_signal<bool> PN_NAME(stb_dport);
    sc_signal<bool> PN_NAME(we_dport);
    sc_signal<bool> PN_NAME(ack_dport);
    sc_signal<sc_uint<32>> PN_NAME(adr_dport);
    sc_signal<sc_uint<32>> PN_NAME(wdata_dport);
    sc_signal<sc_uint<32>> PN_NAME(rdata_dport);
    sc_signal<sc_uint<4>> PN_NAME(bsel_dport);

    // Methods
    void init_submodules();
};

#endif // __PICONUT_H__