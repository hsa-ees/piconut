/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Claus Janicher <claus.janicher@tha.de>
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


// Design template
#include "wishbone_template.h"
#include <stdint.h>
#include <systemc.h>


// defines for testscenario
#define PERIOD_NS 10.0

// #define CFG_WB_SLAVE_ADDRESS 0x00000000

// initialize TB signals

// change to PN_NAME

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

// signals for wishbone bus
sc_signal<sc_uint<32>>     PN_NAME(wb_adr_i);
sc_signal<sc_uint<32>>     PN_NAME(wb_dat_o);
sc_signal<sc_uint<32>>     PN_NAME(wb_dat_i);
sc_signal<sc_uint<4>>     PN_NAME(wb_sel_i);
sc_signal<bool>            PN_NAME(wb_we_i);
sc_signal<bool>            PN_NAME(wb_stb_i);
sc_signal<bool>            PN_NAME(wb_ack_o);
sc_signal<bool>            PN_NAME(wb_cyc_i);

sc_signal<bool>            PN_NAME(wb_rty_o);
sc_signal<bool>            PN_NAME(wb_err_o);




void run_cycle(int cycles = 1){
    for (int i = 0; i < cycles; i++){
        clk = 0;
        sc_start(PERIOD_NS/2, SC_NS);
        clk = 1;
        sc_start(PERIOD_NS/2, SC_NS);
    }

}

// write to the wishbone module some data into the registers with the offset addresses.
// simulate the wishbone master:
void write_data(){

    wb_cyc_i = 1;
    wb_adr_i = (CFG_WB_SLAVE_ADDRESS + 0x0);
    wb_stb_i = 1;


    wb_dat_i = 0xAABBCCDD;
    wb_we_i = 1;
    wb_sel_i = 0xFF; // write all 32 bits of the data line into the dummy_register
    run_cycle(5);
    wb_adr_i = (CFG_WB_SLAVE_ADDRESS + 0x4);
    run_cycle(5);
    wb_adr_i = (CFG_WB_SLAVE_ADDRESS + 0x8);
    wb_dat_i = 0xEEEEEEEE;

    run_cycle(10);
    wb_we_i = 0;
    wb_dat_i = 0;

}

void read_data () {
    // try to read from the same register to get the same data back.
    wb_cyc_i = 1;
    wb_stb_i = 1;

    wb_adr_i = (CFG_WB_SLAVE_ADDRESS + 0x0);
    wb_stb_i = 1;
    wb_sel_i = 0xC; // write all 32 bits of the data line into the dummy_register

    run_cycle(4);
    if (wb_ack_o.read() == 1) {
        PN_ASSERTM (wb_dat_o.read() == 0xAABB0000, "Wishbone Write then Read not passed");
    }

    wb_adr_i = (CFG_WB_SLAVE_ADDRESS + 0x4);
    wb_sel_i = 0x4;     // select the second highest word.
    run_cycle(4);

    if (wb_ack_o.read() == 1) {
        PN_ASSERTM (wb_dat_o.read() == 0x00BB0000, "Wishbone Write then Read  0x00BB not passed");
    }

    wb_sel_i = 0x2; // select the second lowest word.
    run_cycle(2);

    if (wb_ack_o.read() == 1) {
        PN_ASSERTM (wb_dat_o.read() == 0x0000CC00, "Wishbone Write then Read not passed");

    }

    run_cycle(2);
    wb_sel_i = 0x6; // select the middle two words.
    wb_adr_i = (CFG_WB_SLAVE_ADDRESS + 0x4);
    run_cycle(4);

    if (wb_ack_o.read() == 1) {
        PN_ASSERTM (wb_dat_o.read() == 0x00BBCC00, "Wishbone Write then Read for addr 0xC offset and 0xF not passed");
    }

    run_cycle(2);
    wb_sel_i = 0xF; // select all four words
    wb_adr_i = (CFG_WB_SLAVE_ADDRESS + 0x8);
    run_cycle(4);

    if (wb_ack_o.read() == 1) {
        PN_ASSERTM (wb_dat_o.read() == 0xEEEEEEEE, "Wishbone Write then Read for addr  offset and  not passed");
    }

    wb_sel_i = 0b1010; // select the highest and the second lowest word.
    wb_adr_i = (CFG_WB_SLAVE_ADDRESS);
    run_cycle(4);

    if (wb_ack_o.read() == 1) {
        PN_ASSERTM (wb_dat_o.read() == 0xAA00CC00, "Wishbone Write then Read for addr  offset and  not passed");
    }


    run_cycle(10);

}



int sc_main(int argc, char **argv)
{

    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file *tf = PN_BEGIN_TRACE("wishbone_template_tb");


    // Initialize the Design under Testing (DUT)
    m_wishbone_t dut_inst{"dut_inst"};

    // connect the wishbone signals

    dut_inst.clk(clk);
    dut_inst.reset(reset);
    dut_inst.wb_adr_i(wb_adr_i);
    dut_inst.wb_dat_i(wb_dat_i);
    dut_inst.wb_dat_o(wb_dat_o);
    dut_inst.wb_we_i(wb_we_i);
    dut_inst.wb_stb_i(wb_stb_i);
    dut_inst.wb_ack_o(wb_ack_o);
    dut_inst.wb_cyc_i(wb_cyc_i);
    dut_inst.wb_sel_i(wb_sel_i);
    dut_inst.wb_err_o(wb_err_o);
    dut_inst.wb_rty_o(wb_rty_o);





    // Traces of signals here

    dut_inst.Trace(tf, pn_cfg_vcd_level); // trace signals of

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Testbench code here

    // reset
    run_cycle(1);
    reset = 1;
    PN_INFO("Reset was set");
    run_cycle(10);
    reset = 0;
    PN_INFO("Reset no longer set");
    run_cycle(10);


    write_data();

    read_data();

    run_cycle(10);


    run_cycle(); // end with a wait

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}