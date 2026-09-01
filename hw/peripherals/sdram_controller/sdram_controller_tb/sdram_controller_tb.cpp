/*******************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2026 Hermann Zoha <hermann.zoha@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    Testbench for the SDRAM-Controller Module.

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

 ******************************************************************************/

//
// This testbansh is a dummy for synthesis form SystenC to Verilog.
//



#include "../sdram_controller.h"
#include <piconut.h>
#include <vector>
#include <functional>


sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

// wishbone signals
sc_signal<bool> PN_NAME(stb_i);        
sc_signal<bool> PN_NAME(cyc_i);        
sc_signal<bool> PN_NAME(we_i);         
sc_signal<pn_wb_sel_t> PN_NAME(sel_i); 

sc_signal<bool> PN_NAME(ack_o); 
sc_signal<bool> PN_NAME(err_o); 
sc_signal<bool> PN_NAME(rty_o); 

sc_signal<pn_wb_adr_t> PN_NAME(addr_i); 
sc_signal<pn_wb_dat_t> PN_NAME(dat_i);  
sc_signal<pn_wb_dat_t> PN_NAME(dat_o);  

// SDRAM-Controller signals
sc_signal<bool> PN_NAME(sdram_clk);
sc_signal<bool> PN_NAME(sdram_clk_en);

sc_signal<bool> PN_NAME(sdram_cp_sel_n);
sc_signal<bool> PN_NAME(sdram_wr_en_n);
sc_signal<bool> PN_NAME(sdram_col_ac_sel_n);
sc_signal<bool> PN_NAME(sdram_row_ac_sel_n);

sc_signal<sc_uint<13>> PN_NAME(sdram_addr);
sc_signal<sc_uint<2>> PN_NAME(sdram_ba);
sc_signal<bool> PN_NAME(c_we_o);
sc_signal<sc_uint<16>> PN_NAME(sdram_dq_i);
sc_signal<sc_uint<16>> PN_NAME(sdram_dq_o);
sc_signal<sc_uint<2>> PN_NAME(sdram_dqm);


int sc_main(int argc, char** argv)
{
  // Instantiate DUT 
    m_sdram_controller i_dut{"i_dut"};

    // Connect Signals
    i_dut.clk(clk);
    i_dut.reset(reset);

    // Connect Wishbone
    i_dut.wb_slave.adr_i (addr_i);
    i_dut.wb_slave.dat_i (dat_i);
    i_dut.wb_slave.dat_o (dat_o);
    i_dut.wb_slave.we_i (we_i);
    i_dut.wb_slave.stb_i (stb_i);
    i_dut.wb_slave.ack_o (ack_o);
    i_dut.wb_slave.cyc_i (cyc_i);
    i_dut.wb_slave.sel_i (sel_i);
    i_dut.wb_slave.rty_o (rty_o);
    i_dut.wb_slave.err_o (err_o);

    // Connect SDRAM_Controller
    i_dut.sdram_clk (sdram_clk);
    i_dut.sdram_clk_en (sdram_clk_en);
    i_dut.sdram_cp_sel_n(sdram_cp_sel_n);
    i_dut.sdram_wr_en_n (sdram_wr_en_n);
    i_dut.sdram_col_ac_sel_n(sdram_col_ac_sel_n);
    i_dut.sdram_row_ac_sel_n(sdram_row_ac_sel_n);
    i_dut.sdram_addr(sdram_addr);
    i_dut.sdram_ba(sdram_ba);
    i_dut.c_we_o(c_we_o);
    i_dut.sdram_dq_i(sdram_dq_i);
    i_dut.sdram_dq_o(sdram_dq_o);
    i_dut.sdram_dqm(sdram_dqm);

    sc_start(SC_ZERO_TIME);
    
    return 0;
}