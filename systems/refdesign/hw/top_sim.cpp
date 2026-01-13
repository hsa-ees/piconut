/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Marco Milenkovic <Marco.Milenkovic@hs-augsburg.de>
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

#include "top.h"

// This is only needed for current debugging
//~ #include "c_soft_peripheral.h"
//~ #include "c_soft_uart.h"
//~ #include "c_soft_memory.h"

//-------------------------------------------

#define PERIOD_NS 10.0
#define UART_BASE_ADDR 0x30000000
#define debug

// initialize TB signals
sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

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

    pn_parse_enable_trace_core = 1;     // enable core trace dump
    pn_cfg_enable_application_path = 1; // enable application path in program args
    PN_PARSE_CMD_ARGS(argc, argv);      // parse command line arguments

    sc_trace_file* tf = PN_BEGIN_TRACE("piconut_tb"); // create trace file

    // Initialliaze the Design under Testing (DUT)
    m_refdesign i_dut{"i_dut"}; // this is the Design name needed by the svc_too

    // Connect the signals
    i_dut.clk(clk);
    i_dut.reset(reset);

    std::unique_ptr<c_soft_uart> uart = std::make_unique<c_soft_uart>(0x22, UART_BASE_ADDR);
    i_dut.cpu->membrana->add_peripheral(UART_BASE_ADDR, std::move(uart));

    // connects signals from TOP to TB
    i_dut.cpu->membrana->load_elf(pn_cfg_application_path);

    i_dut.cpu->membrana->list_all_peripherals();

    // Traces of Signals
    i_dut.pn_trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT
    // traces of local signals here

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Testbench code here
    reset = 1;
    run_cycle(); // end with a wait
    reset = 0;
    run_cycle(2);

    while(i_dut.cpu->state_is_not_halt())
    {
        run_cycle(1);
    }
    run_cycle(1);

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    uint64_t memory_address = 0x10000000;
    c_soft_peripheral* mem_peripheral = i_dut.cpu->membrana->find_peripheral(memory_address); // search for peripheral in the list of peripherals
    if(mem_peripheral)
    {
        c_soft_memory* memory = dynamic_cast<c_soft_memory*>(mem_peripheral); // cast the found peripheral to Memory object
        if(pn_cfg_dump_memory && memory)
        {
            char abs_path[PATH_MAX];
            char dump_filename[] = "memory_post_exe.dump";
            memory->dump_memory(dump_filename);
            realpath(dump_filename, abs_path); // as abs to allow terminal Ctrl+Click
            PN_INFOF(("Dumped memory at %s", abs_path));
        }
    }
    else
    {
        PN_INFO("No peripheral found at address at given address");
    }

    return 0;
}
