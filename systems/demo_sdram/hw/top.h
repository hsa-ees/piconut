/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2026 Hermann Zoha <hermann.zoha@tha.de>
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

#ifndef __TOP_H__
#define __TOP_H__

#include <systemc.h>
#include <piconut.h>

// PicoNut modules used ...
#include <pn_interconnect.h>
#include <cpu.h>
#include <sdram_controller.h>
#include <uart.h>

SC_MODULE(m_demo_sdram)
{
public:
    // Ports ...
    sc_in_clk       PN_NAME(clk);
    sc_in<bool>     PN_NAME(reset);

    sc_out<bool>            PN_NAME(sdram_clk);               // Clock to Sdram
    sc_out<bool>            PN_NAME(sdram_clk_en);            // Clock enable
    sc_out<bool>            PN_NAME(sdram_cp_sel_n);          // Chip select active low
    sc_out<bool>            PN_NAME(sdram_wr_en_n);           // Write Enable SDRAM active low
    sc_out<bool>            PN_NAME(sdram_col_ac_sel_n);      // Column Active Select Sdram active low
    sc_out<bool>            PN_NAME(sdram_row_ac_sel_n);      // Row Active Select Sdram active low
    sc_out<sc_uint<13>>     PN_NAME(sdram_addr);              // sdram address clom & row
    sc_out<sc_uint<2>>      PN_NAME(sdram_ba);                // sdram bank  
    sc_out<bool>            PN_NAME(c_we_o);                  // Write enable signal for sdram_switch
    sc_in<sc_uint<16>>      PN_NAME(sdram_dq_i);              // data bus for sdram_switch
    sc_out<sc_uint<16>>     PN_NAME(sdram_dq_o);              // data bus for sdram_switch
    sc_out<sc_uint<2>>      PN_NAME(sdram_dqm);               // sdram data mask

    sc_in<bool>     PN_NAME(rx_i);
    sc_out<bool>    PN_NAME(tx_o);

    // Constructor/Destructors
    SC_CTOR(m_demo_sdram)
    {
        SC_METHOD(proc_cmb);

        init_submodules();
    }

    // Functions
    void pn_trace(sc_trace_file * tf, int level = 1);

    // Processes
    void proc_cmb();

    // Submodules
    m_pn_interconnect*  pn_interconnect;
    m_cpu*  cpu;
    m_uart* uart;
    m_sdram_controller* sdram_controller;

protected:
    // Internal Signals
    sc_signal<bool> PN_NAME(dummy_low);

    // Methods
    void init_submodules();
};

#endif // __TOP_H__
