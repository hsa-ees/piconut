/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Daniel Dakhno <Daniel.Dakhno1@tha.de>
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
#include <membrana_soft.h>
#include <piconut.h>
#include "c_soft_uart.h"

#define PERIOD_NS 10.0
#define UART_BASE_ADR 0x30000000

// initialize TB signals
sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

// IPort Signals ...
sc_signal<bool> PN_NAME(stb_iport);
sc_signal<sc_uint<32>> PN_NAME(adr_iport);
sc_signal<sc_uint<4>> PN_NAME(bsel_iport);
sc_signal<sc_uint<32>> PN_NAME(rdata_iport);
sc_signal<bool> PN_NAME(ack_iport);
// DPort Signals ...
sc_signal<bool> PN_NAME(stb_dport);
sc_signal<bool> PN_NAME(we_dport);
sc_signal<sc_uint<32>> PN_NAME(adr_dport);
sc_signal<sc_uint<32>> PN_NAME(wdata_dport);
sc_signal<sc_uint<4>> PN_NAME(bsel_dport);
sc_signal<sc_uint<32>> PN_NAME(rdata_dport);
sc_signal<bool> PN_NAME(ack_dport);

/**
 * @brief simulating a read request from core
 *
 * @param adr address for the read request
 * @param bsel byte select for the read request
 * @param IorD if true, read from IPort, else read from DPort
 * @return uint32_t
 */
uint32_t read_request(uint64_t adr, uint8_t bsel, bool IorD);

/**
 * @brief simualting a write request from core
 *
 * @param data data to write
 * @param adr address to write to
 * @param bsel byte select for the write request
 */
void write_request(uint32_t data, uint64_t adr, uint8_t bsel);

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
    sc_trace_file* tf = PN_BEGIN_TRACE("audio_soft_tb"); // create trace file
    FILE* coreDumpFile = fopen(NUCLEUS_TRACE_FILE, "a"); // open coreDump file for core requests

    // Initialliaze the Design under Testing (DUT)
    m_membrana_soft dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB
    dut_inst.clk(clk);
    dut_inst.reset(reset);

    // IPort Signal Map...
    dut_inst.stb_iport(stb_iport);
    dut_inst.adr_iport(adr_iport);
    dut_inst.bsel_iport(bsel_iport);

    dut_inst.rdata_iport(rdata_iport);
    dut_inst.ack_iport(ack_iport);
    // DPort Signal Map...
    dut_inst.stb_dport(stb_dport);
    dut_inst.we_dport(we_dport);
    dut_inst.adr_dport(adr_dport);
    dut_inst.wdata_dport(wdata_dport);
    dut_inst.bsel_dport(bsel_dport);

    dut_inst.rdata_dport(rdata_dport);
    dut_inst.ack_dport(ack_dport);

    // Traces of Signals
    dut_inst.pn_trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT

    // create peripheral
    std::unique_ptr<c_soft_uart> uart = std::make_unique<c_soft_uart>(0x22, UART_BASE_ADR);

    // traces of local signals here

    sc_start(); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // ************** Add testbench code here ****************

    dut_inst.add_peripheral(UART_BASE_ADR, std::move(uart)); // add the peripheral to the DUT

    dut_inst.list_all_peripherals();
    // Attempt to find the Memory peripheral by its address
    uint64_t uart_address = UART_BASE_ADR;                                        // Address where the memory peripheral is expected
    c_soft_peripheral* found_preipheral = dut_inst.find_peripheral(uart_address); // search for peripheral in the list of peripherals
    if(found_preipheral)
    {
        c_soft_uart* uart_ptr = dynamic_cast<c_soft_uart*>(found_preipheral); // cast the found peripheral to Memory object
        if(uart_ptr)
        {
            // If the peripheral is correctly identified as a Memory object, dump its contents
            std::cout << "Found UART at address 0x" << std::hex << uart_address << "." << std::endl;
        }
        else
        {
            std::cerr << "Found peripheral is not a UART object." << std::endl;
        }
    }
    else
    {
        std::cerr << "No peripheral found at address 0x" << std::hex << uart_address << "." << std::endl;
    }

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
    write_request(data, UART_BASE_ADR + 0x08, 0xF); // Enable the UART peripheral

    // Read the UART peripheral status

    uint32_t status = read_request(UART_BASE_ADR + 0x08, 0xF, true);

    // Check if the UART peripheral is enabled

    PN_INFOF(("UART peripheral status: 0x%08X", status));
    PN_ASSERTM(status == 0x00000001, "UART peripheral not enabled");

    // *************** UART test ***************

    // Write a character to the UART peripheral
    uint32_t snd = 0x00000041; // ASCII code for 'A'
    write_request(snd, UART_BASE_ADR + 0x00, 0xF);

    // disable the UART peripheral

    data = 0x00000000;
    write_request(data, UART_BASE_ADR + 0x08, 0xF); // Enable the UART peripheral

    // Read the UART peripheral status

    status = read_request(UART_BASE_ADR + 0x08, 0xF, true);

    // Check if the UART peripheral is disabled

    PN_INFOF(("UART peripheral status: 0x%08X", status));
    PN_ASSERTM(status == 0x00000000, "UART peripheral not disabled");

    // send hello to the UART peripheral

    const char* hello = "Hello!\n!";

    for(int i = 0; i < 8; i++)
    {
        write_request(hello[i], UART_BASE_ADR + 0x00, 0xF);
    }

    // enable the UART peripheral

    data = 0x00000001;
    write_request(data, UART_BASE_ADR + 0x08, 0xF); // Enable the UART peripheral

    // Read the UART peripheral status

    status = read_request(UART_BASE_ADR + 0x08, 0xF, true);

    // Check if the UART peripheral is enabled

    PN_INFOF(("UART peripheral status: 0x%08X", status));
    PN_ASSERTM(status == 0x00000001, "UART peripheral not enabled");

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}

uint32_t read_request(uint64_t adr, uint8_t bsel, bool IorD)
{
    uint32_t data_var;
    if(IorD)
    {
        // IPort read from memory ...
        stb_iport = 1;     // set the strobe signal
        adr_iport = adr;   // set the address
        bsel_iport = bsel; // set to 32 bit read
        run_cycle(1);
        stb_iport = 0; // reset stb signal
        run_cycle(1);
        data_var = rdata_iport.read(); // read the data
    }
    else
    {
        stb_dport = 1;     // set the strobe signal
        we_dport = 0;      // reset the write enable signal
        adr_dport = adr;   // set the address
        bsel_dport = bsel; // set to 32 bit read
        run_cycle(1);
        stb_dport = 0; // reset stb signal
        run_cycle(1);
        data_var = rdata_dport.read(); // read the data
    }

    return data_var;
}

void write_request(uint32_t data, uint64_t adr, uint8_t bsel)
{

    // DPort write to memory ...
    stb_dport = 1;      // set the strobe signal
    we_dport = 1;       // set the write enable signal
    adr_dport = adr;    // set the address
    wdata_dport = data; // set the data to write
    bsel_dport = bsel;  // set to 32 bit write
    run_cycle(1);
    stb_dport = 0; // reset stb signal
    we_dport = 0;  // reset we signal
    run_cycle(1);
}
