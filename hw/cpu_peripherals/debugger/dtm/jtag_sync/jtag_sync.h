/**
 * @file jtag_sync.h
 * @brief This file contains the declaration of the m_jtag_sync module.
 * @author Johannes Hofmann
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the declaration of the m_jtag_sync module.

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
 * @addtogroup m_jtag_sync
 * This module is used to prepare the jtag input signals in order to use them in
 * the m_dtm module. This includes sync the input signals (tck, tms, tdi, trst_n)
 * as well as an edge detection for tck and trst_n to be used as enable signales
 * later.
 *
 *
 * TODO: ports
 *
 */

#ifndef __JTAG_SYNC_H__
#define __JTAG_SYNC_H__

#include <systemc.h>

#include <piconut.h>

SC_MODULE(m_jtag_sync)
{
public:
    // --------------- System ---------------
    sc_in_clk clk;
    sc_in<bool> reset;

    // --------------- Input ---------------
    sc_in<bool> tck_i;
    sc_in<bool> tms_i;
    sc_in<bool> tdi_i;
    sc_in<bool> trst_n_i;

    // --------------- Output ---------------
    sc_out<bool> tck_sync_en_o;
    sc_out<bool> tms_sync_o;
    sc_out<bool> tdi_sync_o;
    sc_out<bool> trst_n_sync_en_o;

#if !PN_PRESYNTHESIZED_H_ONLY(JTAG_SYNC)
    // Constructor ...
    SC_CTOR(m_jtag_sync)
    {
        SC_CTHREAD(proc_clk, clk.pos());

        SC_METHOD(proc_cmb);
        sensitive << tck_sync_reg[0] << tck_sync_reg[1] // edge detection
                  << tms_sync_reg[0]
                  << tdi_sync_reg[0]
                  << trst_n_sync_reg[0] << trst_n_sync_reg[1]; // edge detection
    }

    // Functions ...
    void pn_trace(sc_trace_file * tf, int levels = 1);

    // Processes ...
    void proc_clk();
    void proc_cmb();

#else

    SC_CTOR(m_jtag_sync) {}
    void pn_trace(sc_trace_file * tf, int level = 1) {}

#endif

protected:
#if !PN_PRESYNTHESIZED_H_ONLY(JTAG_SYNC)

    sc_vector<sc_signal<bool>> PN_NAME_VEC(tck_sync_reg, 3); // edge detection
    sc_vector<sc_signal<bool>> PN_NAME_VEC(tdi_sync_reg, 2);
    sc_vector<sc_signal<bool>> PN_NAME_VEC(tms_sync_reg, 2);
    sc_vector<sc_signal<bool>> PN_NAME_VEC(trst_n_sync_reg, 3); // edge detection

#else
    PN_PRESYNTHESIZED;
#endif
};

#endif
