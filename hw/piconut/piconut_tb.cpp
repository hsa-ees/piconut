/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lukas Bauer <lukas.bauer1@tha.de>
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

#include <stdint.h>
#include <systemc.h>
#include "piconut.h"
// This is only needed for current debugging
#include "c_soft_peripheral.h"
#include "c_soft_memory.h"
//-------------------------------------------

#define PERIOD_NS 10.0

// initialize TB signals
sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

// ------------ Initialize DUT signals ------------
// Debug
sc_signal<bool> PN_NAME(debug_haltrequest);
sc_signal<bool> PN_NAME(debug_haltrequest_ack);

// ------------ IPort Signals------------
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

// ------------ Wishbone Interface ------------
#ifdef __HW_MEMU__
sc_signal<bool> PN_NAME(wb_ack_i);
sc_signal<sc_uint<32>> PN_NAME(wb_dat_i);

sc_signal<sc_uint<32>> PN_NAME(wb_adr_o);
sc_signal<sc_uint<32>> PN_NAME(wb_dat_o);
sc_signal<bool> PN_NAME(wb_we_o);
sc_signal<bool> PN_NAME(wb_stb_o);
sc_signal<bool> PN_NAME(wb_cyc_o);
sc_signal<sc_uint<4>> PN_NAME(wb_sel_o);
#endif

void run_cycle(int cycles = 1)
{
    for(int i = 0; i < cycles; i++)
    {
        clk = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
        clk = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

int sc_main(int argc, char** argv)
{

#ifdef __SIMONLYMEMU__
    pn_parse_enable_trace_core = 1; // enable core trace dump

    PN_PARSE_CMD_ARGS(argc, argv);                    // parse command line arguments
    sc_trace_file* tf = PN_BEGIN_TRACE("piconut_tb"); // create trace file

    // Initialliaze the Design under Testing (DUT)
    m_piconut dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB
    dut_inst.clk(clk);
    dut_inst.reset(reset);
    dut_inst.debug_haltrequest_in(debug_haltrequest);
    dut_inst.debug_haltrequest_ack_out(debug_haltrequest_ack);
    dut_inst.simmemu->load_elf("hello_newlib");

    // Traces of Signals
    dut_inst.Trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT
    // traces of local signals here

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Testbench code here
    reset = 1;
    run_cycle(); // end with a wait
    reset = 0;
    run_cycle(2);
    run_cycle(20000);

    uint64_t memory_address = 0x10000000;
    c_soft_peripheral* found_peripheral = dut_inst.simmemu->find_peripheral(memory_address); // search for peripheral in the list of peripherals
    if(found_peripheral)
    {
        c_soft_memory* memory = dynamic_cast<c_soft_memory*>(found_peripheral); // cast the found peripheral to Memory object
        if(memory)
        {
            // If the peripheral is correctly identified as a Memory object, dump its contents
            memory->dump_memory("memory_dump.txt");
            std::cout << "Memory dump successful." << std::endl;
        }
        else
        {
            std::cerr << "Found peripheral is not a Memory object." << std::endl;
        }
    }
    else
    {
        std::cerr << "No peripheral found at address 0x" << std::hex << memory_address << "." << std::endl;
    }

#endif

#ifdef __HW_MEMU__

    pn_parse_enable_trace_core = 1; // enable core trace dump

    PN_PARSE_CMD_ARGS(argc, argv);                    // parse command line arguments
    sc_trace_file* tf = PN_BEGIN_TRACE("piconut_tb"); // create trace file

    pn_cfg_application_path = "hello_newlib";

    // Initialliaze the Design under Testing (DUT)
    m_piconut dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB
    dut_inst.clk(clk);
    dut_inst.reset(reset);

    dut_inst.debug_haltrequest_in(debug_haltrequest);
    dut_inst.debug_haltrequest_ack_out(debug_haltrequest_ack);

    // connects signals from TOP to DUT
    dut_inst.wb_ack_i(wb_ack_i);
    dut_inst.wb_dat_i(wb_dat_i);
    dut_inst.wb_adr_o(wb_adr_o);
    dut_inst.wb_dat_o(wb_dat_o);
    dut_inst.wb_we_o(wb_we_o);
    dut_inst.wb_stb_o(wb_stb_o);
    dut_inst.wb_cyc_o(wb_cyc_o);
    dut_inst.wb_sel_o(wb_sel_o);

    // Traces of Signals
    dut_inst.Trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT
    // traces of local signals here

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Testbench code here
    reset = 1;
    run_cycle(); // end with a wait
    reset = 0;
    run_cycle(2);
    run_cycle(30000);

#endif

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}