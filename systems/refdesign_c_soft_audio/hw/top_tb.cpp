/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Daniel Dakhno <daniel.dakhno1@tha.de>
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
#include <piconut-config.h>
#include "top.h"

#include "c_soft_uart.h"
#include "c_soft_graphics.h"
#include "gui.h"
#include "c_soft_audio.h"
#include "memory.h"

//-------------------------------------------

#define PERIOD_NS 10.0
#define debug

// initialize TB signals
sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

// Add these signal declarations near your other signal declarations
sc_signal<bool> mtip_signal; // Timer interrupt signal
sc_signal<bool> msip_signal; // Software interrupt signal
sc_signal<bool> meip_signal; // External interrupt signal

// Setup GUI
gui* gui_window = nullptr;

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

// Sends framebuffer data to GUI to display it
void flush_to_gui(uint32_t* framebuffer, uint32_t framebuffer_size_in_px)
{
    // unneeded
}

int sc_main(int argc, char** argv)
{
    pn_parse_enable_trace_core = 1;                   // enable core trace dump
    pn_cfg_enable_application_path = 1;               // enable application path in program args
    PN_PARSE_CMD_ARGS(argc, argv);                    // parse command line arguments
    sc_trace_file* tf = PN_BEGIN_TRACE("piconut_tb"); // create trace file

    int argc2 = 0;
    gui_window = new gui(argc2, nullptr);

    // Initialliaze the Design under Testing (DUT)
    m_top dut_inst{"dut_inst"}; // this is the Design name needed by the svc_too

    // Connect the signals
    dut_inst.clk(clk);
    dut_inst.reset(reset);

    std::unique_ptr<c_soft_uart> uart = std::make_unique<c_soft_uart>(0x22, CFG_WB_UART_BASE_ADDRESS);
    dut_inst.piconut->membrana_soft->add_peripheral(CFG_WB_UART_BASE_ADDRESS, std::move(uart));

    uint32_t clock_frequency = (1.0 / (PERIOD_NS * 1e-9));

    std::unique_ptr<c_soft_graphics> graphics = std::make_unique<c_soft_graphics>(
        CFG_GRAPHICS_BASE_ADDRESS,
        c_soft_graphics::ResolutionMode::Mode_80x60,
        c_soft_graphics::ColorMode::Mode_RGB888,
        clock_frequency,
        flush_to_gui);

    dut_inst.piconut->membrana_soft->add_peripheral(CFG_GRAPHICS_BASE_ADDRESS, std::move(graphics));

    std::unique_ptr<c_soft_audio> audio = std::make_unique<c_soft_audio>(0x64, CFG_WB_AUDIO_BASE_ADDRESS, dut_inst.piconut->membrana_soft, [](bool interrupt_status) {
        meip_signal = interrupt_status;
    });

    c_soft_audio* audioPtr = audio.get();

    dut_inst.piconut->membrana_soft->add_peripheral(CFG_WB_AUDIO_BASE_ADDRESS, std::move(audio));

    gui_window->setActionCallback([audioPtr](GUI_ACTION action) -> void {
        if(action == GUI_ACTION::SETTINGS_AUDIO)
        {
            audioPtr->showControlsWindow();
        }
    });

    // Connect the interrupt signals to the PicoNut processor
    dut_inst.piconut->mtip_in(mtip_signal);
    dut_inst.piconut->msip_in(msip_signal);
    dut_inst.piconut->meip_in(meip_signal);

    // connects signals from TOP to TB
    dut_inst.piconut->membrana_soft->load_elf(pn_cfg_application_path);

    dut_inst.piconut->membrana_soft->list_all_peripherals();

    // Traces of Signals
    dut_inst.pn_trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT
    // traces of local signals here

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Testbench code here
    reset = 1;
    run_cycle(); // end with a wait
    reset = 0;
    run_cycle(2);

    while(dut_inst.piconut->state_is_not_halt() && gui_window->is_gui_window_open())
    {
        gui_window->processEvents();
        run_cycle();
    }
    run_cycle(1);

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}
