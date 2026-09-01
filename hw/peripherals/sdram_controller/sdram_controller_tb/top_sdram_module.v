/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)     2025 Hermann Zoha <hermann.zoha@tha.de> 
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


`include "mt48lc16m16a2.v"
`include "sdram_tri_state.v"
`include "sdram_controller.v"

module top_sdram_module(clk, reset, adr_i, dat_i,
                         dat_o, sel_i, stb_i, cyc_i,
                         we_i, ack_o, rty_o, err_o);
    input wire clk;
    input wire reset;
    input wire [63:0] adr_i;
    input wire [63:0] dat_i;
    output wire [63:0] dat_o;
    input wire [7:0] sel_i;
    input wire stb_i;
    input wire cyc_i;
    input wire we_i;
    output  rty_o;
    output  ack_o;
    output err_o;

    wire clk_to_sdram;
    wire clk_enable;
    wire chip_select;
    wire write_enable;
    wire column_active_select;
    wire row_active_select;
    wire [12:0] address_to_sdram;
    wire [1:0] bank;
    wire c_we;
    wire [15:0]dq_s;
    wire [15:0]sdram_dq_i_1;
    wire [15:0]sdram_dq_o_1;
    wire [1:0]dqml;

m_sdram_controller sdram_controller(
	.clk(clk),
	.reset(reset),
	.wb_slave_adr_i(adr_i),
	.wb_slave_dat_i(dat_i),
	.wb_slave_dat_o(dat_o),
	.wb_slave_sel_i(sel_i),
	.wb_slave_stb_i(stb_i),
	.wb_slave_cyc_i(cyc_i),
	.wb_slave_we_i(we_i),
	.wb_slave_ack_o(ack_o),
	.wb_slave_rty_o(rty_o),
	.wb_slave_err_o(err),
	.sdram_clk(clk_to_sdram),
	.sdram_clk_en(clk_enable),
	.sdram_cp_sel_n(chip_select),
	.sdram_wr_en_n(write_enable),
	.sdram_col_ac_sel_n(column_active_select),
	.sdram_row_ac_sel_n(row_active_select),
	.sdram_addr(address_to_sdram),
	.sdram_ba(bank),
	.c_we_o(c_we),
	.sdram_dq_i(sdram_dq_i_1),
	.sdram_dq_o(sdram_dq_o_1),
	.sdram_dqm(dqml)
);

tri_state tri_state(
    .dq_s_io(dq_s),
    .dq_c_i(sdram_dq_o_1),
    .dq_c_o(sdram_dq_i_1),
    .c_we(c_we)
);

mt48lc16m16a2 sdram(
    .Dq(dq_s),
    .Addr(address_to_sdram),
    .Ba(bank),
    .Clk(clk_to_sdram),
    .Cke(clk_enable),
    .Cs_n(chip_select),
    .Ras_n(row_active_select),
    .Cas_n(column_active_select),
    .We_n(write_enable),
    .Dqm(dqml)
);


endmodule