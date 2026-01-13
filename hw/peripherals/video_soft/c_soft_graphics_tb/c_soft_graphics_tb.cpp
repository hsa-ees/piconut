/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Konrad Armbrecht <konrad.armbrecht@tha.de>
      Technische Hochschule Augsburg, University of Applied Sciences

  Description: This file contains the implementation of the c_soft_graphics for simulation ONLY

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
#include "c_soft_graphics.h"

#define PERIOD_NS 10.0

// Testbench global variable for validating flush callback trigger
uint32_t flush_triggered_times = 0;

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
sc_signal<bool> PN_NAME(_lrsc_dport);
sc_signal<bool> PN_NAME(_amo_dport);

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

/**
 * @brief simulating a read request from core
 *
 * @param adr address for the read request
 * @param bsel byte select for the read request
 * @param IorD if true, read from IPort, else read from DPort
 * @return uint32_t the requested data
 */
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
        // RUN CYCLE SET TO "2" TO AVOID MEMORY WRITE FAILURE
        run_cycle(2);
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
        // RUN CYCLE SET TO "2" TO AVOID MEMORY WRITE FAILURE
        run_cycle(2);
        data_var = rdata_dport.read(); // read the data
    }
    return data_var;
}

/**
 * @brief simulating a write request from core
 *
 * @param data data to write
 * @param adr address to write to
 * @param bsel byte select for the write request
 */

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
    run_cycle(2);
}

/**
 * @brief Sends framebuffer data to the console.
 *
 * @param framebuffer Pointer to the framebuffer data.
 */

void flush_to_gui(uint32_t* framebuffer, uint32_t framebuffer_size)
{
    // No flushing to the GUI, as Qt is not available in the graphics periphery, but only in top_tb.
    // Counting function calls instead to check counter value in testbench code below to ensure proper callback trigger function.
    flush_triggered_times++;
}

int sc_main(int argc, char** argv)
{

    pn_parse_enable_trace_core = 1;                         // enable core trace dump
    PN_PARSE_CMD_ARGS(argc, argv);                          // parse command line arguments
    sc_trace_file* tf = PN_BEGIN_TRACE("membrana_soft_tb"); // create trace file
    FILE* coreDumpFile = fopen(NUCLEUS_TRACE_FILE, "a");    // open coreDump file for core requests

    // Initialiaze the Design under Testing (DUT)
    m_membrana_soft dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB
    dut_inst.clk(clk);
    dut_inst.reset(reset);

    // IPort Signal Map...
    dut_inst.stb_iport[0](stb_iport);
    dut_inst.adr_iport[0](adr_iport);
    dut_inst.bsel_iport[0](bsel_iport);

    dut_inst.rdata_iport[0](rdata_iport);
    dut_inst.ack_iport[0](ack_iport);
    // DPort Signal Map...
    dut_inst.stb_dport[0](stb_dport);
    dut_inst.we_dport[0](we_dport);
    dut_inst.adr_dport[0](adr_dport);
    dut_inst.wdata_dport[0](wdata_dport);
    dut_inst.bsel_dport[0](bsel_dport);
    dut_inst.rdata_dport[0](rdata_dport);
    dut_inst.ack_dport[0](ack_dport);
    dut_inst.load_reserve[0](_lrsc_dport);
    dut_inst.amo_dport[0](_amo_dport);

    // Traces of Signals
    dut_inst.pn_trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT
    // traces of local signals here

    uint32_t clock_frequency = (1.0 / (PERIOD_NS * 1e-9));

    // create peripheral for simulator:
    std::unique_ptr<c_soft_graphics> graphics = std::make_unique<c_soft_graphics>(
        CFG_GRAPHICS_BASE_ADDRESS,
        c_soft_graphics::ResolutionMode::Mode_80x60,
        c_soft_graphics::ColorMode::Mode_RGB888,
        clock_frequency,
        flush_to_gui);

    // start simulation
    sc_start();
    PN_INFO("\n\t\t*****Simulation started*****");

    // ************** Add testbench code here ****************

    // add the peripheral to the DUT
    dut_inst.add_peripheral(CFG_GRAPHICS_BASE_ADDRESS, std::move(graphics));
    dut_inst.list_all_peripherals();

    // search for peripheral in the list of peripherals
    c_soft_peripheral* found_peripheral = dut_inst.find_peripheral(CFG_GRAPHICS_BASE_ADDRESS);
    if(found_peripheral)
    {
        // cast the found peripheral to graphics object
        c_soft_graphics* graphics_ptr = dynamic_cast<c_soft_graphics*>(found_peripheral);
        if(graphics_ptr)
        {
            // If the peripheral is correctly identified as a graphics object, dump its contents
            PN_INFOF(("Found graphics peripheral at address 0x%llx.", CFG_GRAPHICS_BASE_ADDRESS));
        }
        else
        {
            // std::cerr << "Found peripheral is not a graphics object." << std::endl;
            PN_ERROR("Found peripheral is not a graphics object.");
        }
    }
    else
    {
        PN_ERRORF(("No graphics peripheral found at address 0x%llx.", CFG_GRAPHICS_BASE_ADDRESS));
    }

    // *************** Testbench setup ***************

    uint32_t width = read_request(CFG_GRAPHICS_BASE_ADDRESS + c_soft_graphics::WIDTH, 0xF, true);
    uint32_t height = read_request(CFG_GRAPHICS_BASE_ADDRESS + c_soft_graphics::HEIGHT, 0xF, true);
    uint32_t framebuffer_size = width * height;

    PN_INFO("");
    PN_INFO("==== Graphics Peripheral Parameters ====");
    PN_INFOF(("Framebuffer Width  : %d px", width));
    PN_INFOF(("Framebuffer Height : %d px", height));
    PN_INFOF(("Framebuffer Size   : %d pixels", framebuffer_size));
    PN_INFOF(("Framebuffer Size   : 0x%zx pixels (hex)", framebuffer_size));
    PN_INFO("========================================");
    PN_INFO("");
    PN_INFO(" *************** Running Graphics tests *************** ");
    PN_INFO("");

    // write first pixel of framebuffer
    PN_INFO("Test: write first pixel of framebuffer");
    write_request(0x00FFFFFF, CFG_GRAPHICS_BASE_ADDRESS + c_soft_graphics::FRAMEBUFFER, 0xF);
    uint32_t data_after_write = read_request(CFG_GRAPHICS_BASE_ADDRESS + c_soft_graphics::FRAMEBUFFER, 0xF, true);
    PN_ASSERTM(data_after_write == 0x00FFFFFF, "First pixel not written to framebuffer.");

    // write last pixel of framebuffer
    PN_INFO("Test: write last pixel of framebuffer");
    write_request(0x00123456, CFG_GRAPHICS_BASE_ADDRESS + c_soft_graphics::FRAMEBUFFER + (framebuffer_size - 1) * 4, 0xF); // final pixel in framebuffer
    data_after_write = read_request(CFG_GRAPHICS_BASE_ADDRESS + c_soft_graphics::FRAMEBUFFER + (framebuffer_size - 1) * 4, 0xF, true);
    PN_ASSERTM(data_after_write == 0x00123456, "Last pixel not written to framebuffer.");

    // write to registers
    // control
    PN_INFO("Test: write to registers");
    uint32_t data_before_write = read_request(CFG_GRAPHICS_BASE_ADDRESS + c_soft_graphics::CONTROL, 0xF, true);
    write_request(0x00000001, 0x40000000, 0xF); // write control register - warning expected
    data_after_write = read_request(CFG_GRAPHICS_BASE_ADDRESS + c_soft_graphics::CONTROL, 0xF, true);
    PN_ASSERTM(data_before_write == data_after_write, "Write to control register was possible. Should not be. Register is currently read only.");

    // color mode
    data_before_write = read_request(CFG_GRAPHICS_BASE_ADDRESS + c_soft_graphics::COLOR_MODE, 0xF, true);
    write_request(50, CFG_GRAPHICS_BASE_ADDRESS + c_soft_graphics::COLOR_MODE, 0xF); // invalid color mode
    data_after_write = read_request(CFG_GRAPHICS_BASE_ADDRESS + c_soft_graphics::COLOR_MODE, 0xF, true);
    PN_ASSERTM(data_before_write == data_after_write, "Write '50' to color mode register was possible. Should not be. '50' is not a valid color mode.");

    write_request(0, CFG_GRAPHICS_BASE_ADDRESS + c_soft_graphics::COLOR_MODE, 0xF); // invalid color mode
    data_after_write = read_request(CFG_GRAPHICS_BASE_ADDRESS + c_soft_graphics::COLOR_MODE, 0xF, true);
    PN_ASSERTM(data_after_write == 0, "Write '0' to color mode register was not possible. Should be. '0' is a valid color mode.");

    // resolution mode
    data_before_write = read_request(CFG_GRAPHICS_BASE_ADDRESS + c_soft_graphics::RESOLUTION_MODE, 0xF, true);
    write_request(50, CFG_GRAPHICS_BASE_ADDRESS + c_soft_graphics::RESOLUTION_MODE, 0xF); // invalid resolution mode
    data_after_write = read_request(CFG_GRAPHICS_BASE_ADDRESS + c_soft_graphics::RESOLUTION_MODE, 0xF, true);
    PN_ASSERTM(data_before_write == data_after_write, "Write '50' to resolution mode register was possible. Should not be. '50' is not a valid resolution mode.");

    // resolution mode
    write_request(0x00000000, CFG_GRAPHICS_BASE_ADDRESS + c_soft_graphics::RESOLUTION_MODE, 0xF);
    data_after_write = read_request(CFG_GRAPHICS_BASE_ADDRESS + c_soft_graphics::RESOLUTION_MODE, 0xF, true);
    PN_ASSERTM(data_after_write == 0x00000000, "Write '0x00000000' to resolution mode register was not possible. Should be. '0x00000000' is a valid resolution mode.");

    // resolution mode
    write_request(0x00000001, CFG_GRAPHICS_BASE_ADDRESS + c_soft_graphics::RESOLUTION_MODE, 0xF);
    data_after_write = read_request(CFG_GRAPHICS_BASE_ADDRESS + c_soft_graphics::RESOLUTION_MODE, 0xF, true);
    PN_ASSERTM(data_after_write == 0x00000001, "Write '0x00000001' to resolution mode register was not possible. Should be. '0x00000001' is a valid resolution mode.");

    // wait for flush callback
    PN_ASSERTM(flush_triggered_times == 0, "Flush callback triggered too early");
    run_cycle(1700000);
    PN_ASSERTM(flush_triggered_times >= 1, "Flush callback trigger missing (first time).");
    run_cycle(1700000);
    PN_ASSERTM(flush_triggered_times >= 2, "Flush callback trigger missing (second time)");

    PN_INFO("");
    PN_INFO(" *************** Finished Graphics tests successfully *************** \n");
    return 0;
}
