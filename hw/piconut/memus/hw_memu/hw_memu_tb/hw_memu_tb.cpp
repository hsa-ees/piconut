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
#include "hw_memu.h"
#include <stdint.h>
#include <systemc.h>

#define PERIOD_NS 10.0

// defines for full mem test
// insert memory size correctly to avoid core dumps
#define full_mem_test 1
#define blockram_mem_test 1
#define mem_size 0x000001FF

// initialize TB signals


sc_signal<bool>         PN_NAME(clk);
sc_signal<bool>         PN_NAME(reset);

sc_signal<sc_uint<4>>      PN_NAME(wea); // data width is fixed to 32bit -> 4 byte enables
sc_signal<sc_uint<4>>      PN_NAME(web); // data width is fixed to 32bit -> 4 byte enables
sc_signal<bool>         PN_NAME(ena); // enable signal for blockram a
sc_signal<bool>         PN_NAME(enb); // enable signal for blockram b

sc_signal<sc_uint<32>>      PN_NAME(addra); // blockram a address
sc_signal<sc_uint<32>>      PN_NAME(addrb); // blockram b address
sc_signal<sc_uint<32>>      PN_NAME(dia); // blockram a data in
sc_signal<sc_uint<32>>      PN_NAME(dib); // blockram b data in
sc_signal<sc_uint<32>>      PN_NAME(doa); // blockram a data out
sc_signal<sc_uint<32>>      PN_NAME(dob); // blockram b data out

sc_signal<bool>         PN_NAME(ip_stb); // strobe
sc_signal<sc_uint<4>>      PN_NAME(ip_bsel); // byte select bsel[n]? 4 == 32bit, 8 == 64bit, 16 == 128bit
sc_signal<sc_uint<32>>      PN_NAME(ip_adr); // adress <=30
sc_signal<bool>        PN_NAME(ip_ack); // acknowledge
sc_signal<sc_uint<32>>    PN_NAME(ip_rdata); // hands data to the core 32/64/128 bit width
//signals DataPort
sc_signal<bool>         PN_NAME(dp_stb); // strobe
sc_signal<bool>         PN_NAME(dp_we); // write enable DPort only?
sc_signal<sc_uint<4>>      PN_NAME(dp_bsel);  // byte select bsel[n]? 4 == 32bit, 8 == 64bit, 16 == 128bit
sc_signal<sc_uint<32>>      PN_NAME(dp_adr); // adress <=30
sc_signal<sc_uint<32>>     PN_NAME(dp_wdata); // write data from core to memU 32/64/128 bit width DPort only?
sc_signal<bool>        PN_NAME(dp_ack); // acknowledge
sc_signal<sc_uint<32>>    PN_NAME(dp_rdata); // hands data to the core 32/64/128 bit width

// signals for wishbone bus
sc_signal<sc_uint<32>>     PN_NAME(wb_adr_o);
sc_signal<sc_uint<32>>     PN_NAME(wb_dat_o);
sc_signal<sc_uint<32>>     PN_NAME(wb_dat_i);
sc_signal<bool>            PN_NAME(wb_we_o);
sc_signal<bool>            PN_NAME(wb_stb_o);
sc_signal<bool>            PN_NAME(wb_ack_i);
sc_signal<bool>            PN_NAME(wb_cyc_o);
sc_signal<sc_uint<4>> PN_NAME(wb_sel_o);



void run_cycle(int cycles = 1){
    for (int i = 0; i < cycles; i++){
        clk = 0;
        sc_start(PERIOD_NS/2, SC_NS);
        clk = 1;
        sc_start(PERIOD_NS/2, SC_NS);
    }

}



int sc_main(int argc, char **argv)
{

    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file *tf = PN_BEGIN_TRACE("hw_memu_tb");

    pn_cfg_application_path = "hello_newlib";

    // Initialliaze the Design under Testing (DUT)

    m_hw_memu dut_inst{"dut_inst"};

    // connect the wishbone signals
    dut_inst.clk(clk);
    dut_inst.reset(reset);
    dut_inst.wb_adr_o(wb_adr_o);
    dut_inst.wb_dat_i(wb_dat_i);
    dut_inst.wb_dat_o(wb_dat_o);
    dut_inst.wb_we_o(wb_we_o);
    dut_inst.wb_stb_o(wb_stb_o);
    dut_inst.wb_ack_i(wb_ack_i);
    dut_inst.wb_cyc_o(wb_cyc_o);
    dut_inst.wb_sel_o(wb_sel_o);

    // connects signals from TOP to TB
    dut_inst.ip_stb(ip_stb);
    dut_inst.ip_bsel(ip_bsel);
    dut_inst.ip_adr(ip_adr);
    dut_inst.ip_ack(ip_ack);
    dut_inst.ip_rdata(ip_rdata);

    dut_inst.dp_stb(dp_stb);
    dut_inst.dp_we(dp_we);
    dut_inst.dp_bsel(dp_bsel);
    dut_inst.dp_adr(dp_adr);
    dut_inst.dp_wdata(dp_wdata);
    dut_inst.dp_ack(dp_ack);
    dut_inst.dp_rdata(dp_rdata);



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



    // DPort write

    PN_INFO("Writing data with DPort Write");
    dp_stb = 1;
    dp_we = 1;
    dp_adr = 0xFFFFFFFF;
    dp_wdata = 0xDECA1001;
    dp_bsel = 0xF;
    run_cycle(1);
    cout << std::hex << wb_dat_o.read() << endl;
    cout << std::hex << wb_adr_o.read() << endl;


    run_cycle(5);
    PN_INFO("Reading data from wb_dat_o");

    cout << std::hex << wb_dat_o.read() << endl;



    if ((wb_dat_o.read() == 0xDECA1001) && (wb_stb_o.read() == 0x1) && (wb_adr_o.read() == 0xFFFFFFFF)) {

        run_cycle(2);
        wb_ack_i.write(true);
        run_cycle(2);
        PN_ASSERTM (wb_stb_o.read() == 0x0, "Memory Wishbone Write then Read not passed");
    } else PN_INFO("Memory Wishbone Write then Read test failed");
    dp_stb = 0;
    dp_we = 0;
    run_cycle(10);


    run_cycle(); // end with a wait

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}