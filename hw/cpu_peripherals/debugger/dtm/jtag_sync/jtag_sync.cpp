/**
 * @file jtag_sync.cpp
 * @brief This file contains the definition of the m_jtag_sync module.
 * @author Johannes Hofmann
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the definition of the m_jtag_sync module.

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

#include "jtag_sync.h"

void m_jtag_sync::pn_trace(sc_trace_file* tf, int level)
{
    // Ports ...
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);

    PN_TRACE(tf, tck_i);
    PN_TRACE(tf, tms_i);
    PN_TRACE(tf, tdi_i);
    PN_TRACE(tf, trst_n_i);

    PN_TRACE(tf, tck_sync_en_o);
    PN_TRACE(tf, tms_sync_o);
    PN_TRACE(tf, tdi_sync_o);
    PN_TRACE(tf, trst_n_sync_en_o);

    // Internal ...
    PN_TRACE_BUS(tf, tck_sync_reg, 3);
    PN_TRACE_BUS(tf, tms_sync_reg, 2);
    PN_TRACE_BUS(tf, tdi_sync_reg, 2);
    PN_TRACE_BUS(tf, trst_n_sync_reg, 3);
}

void m_jtag_sync::proc_clk()
{
    while(true)
    {
        if(reset.read() == 1)
        {
            for(size_t i = 0; i < 3; ++i)
            {
                tck_sync_reg[i] = 0;
                trst_n_sync_reg[i] = 1;
            }

            for(size_t i = 0; i < 2; ++i)
            {
                tms_sync_reg[i] = 0;
                tdi_sync_reg[i] = 0;
            }
        }
        else
        {
            tck_sync_reg[2] = tck_i.read();
            tms_sync_reg[1] = tms_i.read();
            tdi_sync_reg[1] = tdi_i.read();
            trst_n_sync_reg[2] = trst_n_i.read();

            tck_sync_reg[1] = tck_sync_reg[2];
            tms_sync_reg[0] = tms_sync_reg[1];
            tdi_sync_reg[0] = tdi_sync_reg[1];
            trst_n_sync_reg[1] = trst_n_sync_reg[2];

            tck_sync_reg[0] = tck_sync_reg[1];
            trst_n_sync_reg[0] = trst_n_sync_reg[1];
        }

        wait();
    }
}

void m_jtag_sync::proc_cmb()
{
    tck_sync_en_o = !tck_sync_reg[0].read() && tck_sync_reg[1];
    tms_sync_o = tms_sync_reg[0].read();
    tdi_sync_o = tdi_sync_reg[0].read();

    // Edge detection for active low signal is reversed
    trst_n_sync_en_o = !(trst_n_sync_reg[0].read() && !trst_n_sync_reg[1]);
}
