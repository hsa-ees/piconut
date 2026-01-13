/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Lukas Bauer <lukas.bauer1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description: This file contains the implementation of the c_soft_uart for simulation ONLY

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
#include <piconut.h>

#include "../uart_soft.h"



#define PERIOD_NS 10.0
#define UART_BASE_ADR 0x30000000

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
    pn_parse_enable_trace_core = 1;                      // enable core trace dump
    PN_PARSE_CMD_ARGS(argc, argv);                       // parse command line arguments
    sc_trace_file* tf = PN_BEGIN_TRACE("soft_uart_tb");   // create trace file
    //~ FILE* coreDumpFile = fopen (NUCLEUS_TRACE_FILE, "a"); // open coreDump file for core requests

    // Initiate soft uart directly
    c_soft_uart uart(0x22, UART_BASE_ADR); 

    sc_start(); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // ************** Add testbench code here ****************

    // *************** Reset test ***************
    run_cycle(2);
    reset = 1;
    PN_INFO("Set Reset");
    run_cycle(2);
    reset = 0;
    PN_INFO("Reset De-asserted");
    run_cycle(2);

    // *************** Memory Interface test ***************

    // Enable the UART peripheral

    uint32_t data = 0x00000001;
    uart.write32( UART_BASE_ADR + 0x08, data); // Enable the UART peripheral

    // Read the UART peripheral status

    uint32_t status = uart.read32(UART_BASE_ADR + 0x08);

    // Check if the UART peripheral is enabled

    PN_INFOF(("UART peripheral status: 0x%08X", status));
    PN_ASSERTM(status == 0x00000001, "UART peripheral not enabled");

    // *************** UART test ***************

    // Write a character to the UART peripheral

    uint32_t snd = 0x00000041; // ASCII code for 'A'
    uart.write32(UART_BASE_ADR + 0x00, snd);

    // disable the UART peripheral

    data = 0x00000000;
    uart.write32(UART_BASE_ADR + 0x08, data); // Disable the UART peripheral

    // Read the UART peripheral status

    status = uart.read32(UART_BASE_ADR + 0x08);

    // Check if the UART peripheral is disabled

    PN_INFOF(("UART peripheral status: 0x%08X", status));
    PN_ASSERTM(status == 0x00000000, "UART peripheral not disabled");

    // send hello to the UART peripheral

    const char* hello = "Hello!\n!";

    for(int i = 0; i < 8; i++)
    {
        uart.write32(UART_BASE_ADR + 0x00, hello[i]);
    }

    // enable the UART peripheral

    data = 0x00000001;
    uart.write32(UART_BASE_ADR + 0x08, data); // Enable the UART peripheral

    // Read the UART peripheral status

    status = uart.read32(UART_BASE_ADR + 0x08);

    // Check if the UART peripheral is enabled

    PN_INFOF(("UART peripheral status: 0x%08X", status));
    PN_ASSERTM(status == 0x00000001, "UART peripheral not enabled");

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}
