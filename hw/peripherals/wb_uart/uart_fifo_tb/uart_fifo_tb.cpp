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


// Design template
#include "uart_fifo.h"
#include <stdint.h>
#include <systemc.h>

#define PERIOD_NS 10.0

// initialize TB signals
sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);


sc_signal<bool>                             PN_NAME(clear); // Clear the content of the FIFO
sc_signal<bool>                             PN_NAME(write_sig); // Write enable for the FIFO
sc_signal<bool>                             PN_NAME(read_sig);  // Read enable for the FIFO
sc_signal<sc_uint<UART_FIFO_WIDTH>>         PN_NAME(data_in);  // Data to be written to the FIFO
sc_signal<sc_uint<UART_FIFO_WIDTH>>         PN_NAME(data_out);  // Data to be read from the FIFO
sc_signal<bool>                             PN_NAME(full);  // FIFO is full
sc_signal<bool>                             PN_NAME(empty); // FIFO is empty
sc_signal<sc_uint<UART_FIFO_SIZE_2E + 1>>   PN_NAME(usage); // Usage of the FIFO





void run_cycle(int cycles = 1){
    for (int i = 0; i < cycles; i++){
        clk = 0;
        sc_start(PERIOD_NS/2, SC_NS);
        clk = 1;
        sc_start(PERIOD_NS/2, SC_NS);
    }

}

void write_fifo(uint8_t data){
    // setting data and write signal
    write_sig = 1;
    data_in = data;
    run_cycle();

    // clearing write signal
    write_sig = 0;
    run_cycle();
}

uint8_t read_fifo(){
    // setting read signal
    read_sig = 1;
    run_cycle();

    // reading data and clearing read signal
    uint8_t data = (uint8_t)data_out.read();
    read_sig = 0;
    run_cycle();

    return data;
}

int sc_main(int argc, char **argv)
{

    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file *tf = PN_BEGIN_TRACE("uart_fifo_tb");


    // Initialliaze the Design under Testing (DUT)
    m_uart_fifo dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB
    dut_inst.clk(clk);
    dut_inst.reset(reset);
    dut_inst.clear_i(clear);
    dut_inst.write_i(write_sig);
    dut_inst.read_i(read_sig);
    dut_inst.data_i(data_in);
    dut_inst.data_o(data_out);
    dut_inst.full_o(full);
    dut_inst.empty_o(empty);
    dut_inst.usage_o(usage);


    // Traces of signals here

    dut_inst.Trace(tf, pn_cfg_vcd_level); // trace signals of

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Testbench code here

    // Reset
    PN_INFO("Resetting ...");
    reset = 1;
    run_cycle();
    reset = 0;
    run_cycle();

    // Check if empty is set if no data is written after the reset
    PN_INFO("Checking if empty is set ...");
    PN_INFOF(("Empty is %d", (uint8_t)empty.read()));
    PN_ASSERTM((uint8_t)empty.read() == 1, "Empty is not set");

    // Write 2 Words to FIFO and check if the usage is
    // set to the number 2
    PN_INFO("TEST: Writing 2 Words to FIFO ...");
    write_fifo(0x55);
    write_fifo(0xAA);

    PN_INFO("Checking if usage is 2 ...");
    PN_INFOF(("Usage is %d", (uint8_t)usage.read()));
    PN_ASSERTM((uint8_t)usage.read() == 2, "Usage is not 2");

    // Checking if clear works
    // this should reset the FIFO setting empty to high and usage to 0
    PN_INFO("TEST: Clearing FIFO ...");
    clear = 1;
    run_cycle();
    clear = 0;
    run_cycle();

    PN_INFO("Checking if empty is set ...");
    PN_INFOF(("Empty is %d", (uint8_t)empty.read()));
    PN_ASSERTM((uint8_t)empty.read() == 1, "Empty is not set");
    PN_INFO("Checking if usage is 0 ...");
    PN_INFOF(("Usage is %d", (uint8_t)usage.read()));
    PN_ASSERTM((uint8_t)usage.read() == 0, "Usage is not 0");

    // Write till full is set by writing 8 words
    // this should set the full signal
    PN_INFO("TEST: Writing till full is set ...");

    for (int i = 0; i < (1 << UART_FIFO_SIZE_2E); i++){
        write_fifo(i);
    }

    run_cycle(); // end with a wait
    PN_INFO("Checking if full is set ...");
    PN_INFOF(("Full is %d", (uint8_t)full.read()));
    PN_ASSERTM((uint8_t)full.read() == 1, "Full is not set");
    PN_INFO("Checking if usage is 8 ...");
    PN_INFOF(("Usage is %d", (uint8_t)usage.read()));
    PN_ASSERTM((uint8_t)usage.read() == 8, "Usage is not 8");

    // Read till empty is set
    // by reading 8 words and then checking if empty is set
    PN_INFO("TEST: Reading till empty is set ...");
    for(int i = 0; i < (1 << UART_FIFO_SIZE_2E); i++){
        uint8_t data = read_fifo();
        PN_INFOF(("Data read: %d", data));
        PN_ASSERTF(data == i, ("Data read is not correct Should be %d but is %d", i, data));
    }

    PN_INFO("Checking if empty is set ...");
    PN_INFOF(("Empty is %d", (uint8_t)empty.read()));
    PN_ASSERTM((uint8_t)empty.read() == 1, "Empty is not set");
    PN_INFO("Checking if usage is 0 ...");
    PN_INFOF(("Usage is %d", (uint8_t)usage.read()));
    PN_ASSERTM((uint8_t)usage.read() == 0, "Usage is not 0");

    // Checking writing after reading from full fifo
    // this should not write any additional data
    // and when reading should have the same data as written into it
    PN_INFO("TEST: Writing after reading from full fifo ...");

    for (int i = 0; i < (1 << UART_FIFO_SIZE_2E); i++){
        write_fifo(i);
    }

    PN_INFO("Checking if full is set ...");
    PN_INFOF(("Full is %d", (uint8_t)full.read()));
    PN_ASSERTM((uint8_t)full.read() == 1, "Full is not set");

    for(int i = 0; i < (1 << UART_FIFO_SIZE_2E); i++){
        uint8_t data = read_fifo();
        PN_INFOF(("Data read: %d", data));
        PN_ASSERTF(data == i, ("Data read is not correct Should be %d but is %d", i, data));
        PN_INFO("Checking if full is not set ...");
        PN_INFOF(("Full is %d", (uint8_t)full.read()));
        PN_ASSERTM((uint8_t)full.read() == 0, "Full is set");
        PN_INFO("Checking if usage is 7 ...");
        PN_INFOF(("Usage is %d", (uint8_t)usage.read()));
        PN_ASSERTM((uint8_t)usage.read() == (1 << UART_FIFO_SIZE_2E) - 1, "Usage is not correct");

        // Write one more
        write_fifo(i+8);
        PN_INFO("Checking if full is set ...");
        PN_INFOF(("Full is %d", (uint8_t)full.read()));
        PN_ASSERTM((uint8_t)full.read() == 1, "Full is not set");
        PN_INFO("Checking if usage is 8 ...");
        PN_INFOF(("Usage is %d", (uint8_t)usage.read()));
        PN_ASSERTM((uint8_t)usage.read() == (1 << UART_FIFO_SIZE_2E), "Usage is not correct");
    }

    run_cycle(); // end with a wait

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}