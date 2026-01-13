/**
 * @file wb_debugger.h
 * @brief This file contains the declaration of the m_debugger module.
 * @author Johannes Hofmann
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the declaration of the m_wb_debugger module.

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

/**
 * @addtogroup m_wb_debugger
 * TODO
 */

#ifndef __DEBUGGER_H__
#define __DEBUGGER_H__

#include <systemc.h>

#include <piconut.h>

SC_MODULE(m_debugger)
{
public:
    // --------------- System ---------------
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    // --------------- Jtag ---------------
    sc_in<bool> PN_NAME(tck_i);
    sc_in<bool> PN_NAME(tms_i);
    sc_in<bool> PN_NAME(tdi_i);
    sc_in<bool> PN_NAME(trst_n_i); // optional
    sc_out<bool> PN_NAME(tdo_o);

    // --------------- Wishbone ---------------
    sc_in<bool> PN_NAME(wb_stb_i);            // strobe input
    sc_in<bool> PN_NAME(wb_cyc_i);            // cycle valid input
    sc_in<bool> PN_NAME(wb_we_i);             // indicates write transfer
    sc_in<sc_uint<3>> PN_NAME(wb_cti_i);      // cycle type identifier (optional, for registered feedback)
    sc_in<sc_uint<2>> PN_NAME(wb_bte_i);      // burst type extension (optional, for registered feedback)
    sc_in<sc_uint<32 / 8>> PN_NAME(wb_sel_i); // byte select inputs
    sc_out<bool> PN_NAME(wb_ack_o);           // normal termination
    sc_out<bool> PN_NAME(wb_err_o);           // termination w/ error (optional)
    sc_out<bool> PN_NAME(wb_rty_o);           // termination w/ retry (optional)
    sc_in<sc_uint<32>> PN_NAME(wb_adr_i);     // address
    sc_in<sc_uint<32>> PN_NAME(wb_dat_i);     // data in
    sc_out<sc_uint<32>> PN_NAME(wb_dat_o);    // data out

    // --------------- Debug Interrupt ---------------
    sc_out<bool> PN_NAME(debug_haltrequest_o);
    sc_in<bool> PN_NAME(debug_haltrequest_ack_i);

#if !PN_PRESYNTHESIZED_H_ONLY(DEBUGGER)
    // Constructor ...
    SC_CTOR(m_debugger)
    {
        init_submodules();
    }

    // Functions ...
    void pn_trace(sc_trace_file * tf, int levels = 1);

    // Processes ...

    // Submodules ...
    void init_submodules();

    class m_dtm* dtm;
    class m_dm* dm;

#else // !PN_PRESYNTHESIZED_H_ONLY(NUCLEUS_REF)

    SC_CTOR(m_debugger) {}
    void pn_trace(sc_trace_file * tf, int level = 1) {}

#endif // !PN_PRESYNTHESIZED_H_ONLY(NUCLEUS_REF)

protected:
#if !PN_PRESYNTHESIZED_H_ONLY(DEBUGGER)
    // Registers ...

    // Signals ...
    sc_signal<sc_uint<PN_CFG_DEBUG_DMI_BUS_ADR_WIDTH>> PN_NAME(dmi_adr);
    sc_signal<sc_uint<32>> PN_NAME(dmi_dat_w);
    sc_signal<sc_uint<32>> PN_NAME(dmi_dat_r);
    sc_signal<bool> PN_NAME(dmi_re);
    sc_signal<bool> PN_NAME(dmi_we);

#else
    PN_PRESYNTHESIZED;
#endif
};

#endif
