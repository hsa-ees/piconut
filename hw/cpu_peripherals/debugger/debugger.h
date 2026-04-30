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

SC_MODULE(m_debugger), pn_module_if
{
public:
    // --------------- System ---------------
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    pn_wishbone_slave_t wb_slave;
    pn_debug_master_t debug_master;

    // --------------- Jtag ---------------
    sc_in<bool> PN_NAME(tck_i);
    sc_in<bool> PN_NAME(tms_i);
    sc_in<bool> PN_NAME(tdi_i);
    sc_in<bool> PN_NAME(trst_n_i); // optional
    sc_out<bool> PN_NAME(tdo_o);

    // Constructor ...
    SC_HAS_PROCESS(m_debugger);
    m_debugger(
        sc_module_name name)
        : sc_module(name)
        , wb_slave{
              .alen = 32,
              .dlen = 32,
              .base_address = 0,
              .size = 0x3C}
    {
        pn_add_wishbone_slave(&wb_slave);
        pn_add_debug_master(&debug_master);

#if !PN_PRESYNTHESIZED_H_ONLY(DEBUGGER)

        init_submodules();
    }

    // Functions ...
    void pn_trace(sc_trace_file * tf, int levels = 1);

    // Processes ...

    // Submodules ...
    void init_submodules();

    class m_dtm* dtm;
    class m_dm* dm;

#else // !PN_PRESYNTHESIZED_H_ONLY(DEBUGGER)
    }

    void pn_trace(sc_trace_file * tf, int level = 1) {}

#endif // !PN_PRESYNTHESIZED_H_ONLY(DEBUGGER)

protected:
#if !PN_PRESYNTHESIZED_H_ONLY(DEBUGGER)
    // Registers ...

    // Signals ...
    sc_signal<sc_uint<PN_CFG_DEBUG_DMI_BUS_ADR_WIDTH>> PN_NAME(dmi_adr);
    sc_signal<sc_uint<32>> PN_NAME(dmi_dat_w);
    sc_signal<sc_uint<32>> PN_NAME(dmi_dat_r);
    sc_signal<bool> PN_NAME(dmi_re);
    sc_signal<bool> PN_NAME(dmi_we);

#else // !PN_PRESYNTHESIZED_H_ONLY(DEBUGGER)

    // Declare the module to be pre-synthesized ...
    PN_PRESYNTHESIZED;

#endif // !PN_PRESYNTHESIZED_H_ONLY(DEBUGGER)
};

#endif
