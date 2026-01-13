/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Martin Erichsen <martin.erichsen@tha.de>
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

#include "color_translator.h"
#include <systemc.h>

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

sc_signal<sc_uint<5>> PN_NAME(color_mode);
sc_signal<sc_uint<32>> PN_NAME(color_in);
sc_signal<sc_uint<24>> PN_NAME(color_out);

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

void run_test(uint32_t input, uint32_t expected)
{
    color_in = input;
    run_cycle();
    PN_ASSERTF(color_out.read() == expected, ("Failed: Input: %08X; Output: %08X; Expected: %08X", input, (uint32_t)color_out.read(), expected));
}

int sc_main(int argc, char** argv)
{
    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("color_translator");

    m_color_translator dut_inst{"dut_inst"};
    dut_inst.clk(clk);
    dut_inst.reset(reset);
    dut_inst.color_mode(color_mode);
    dut_inst.color_in(color_in);
    dut_inst.color_out(color_out);

    dut_inst.pn_trace(tf, pn_cfg_vcd_level);

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Reset
    reset = 1;
    run_cycle(1);
    reset = 0;
    run_cycle(1);

    // COLOR_MODE_1_MONO
    PN_INFO("Testing COLOR_MODE_1_MONO");
    color_mode = COLOR_MODE_1_MONO;
    run_test(0x00, 0x000000);
    run_test(0x01, 0xFFFFFF);
    run_test(0xAA, 0x000000);
    run_test(0xFF, 0xFFFFFF);

    // COLOR_MODE_2_GRAY
    PN_INFO("Testing COLOR_MODE_2_GRAY");
    color_mode = COLOR_MODE_2_GRAY;
    run_test(0x00, 0x000000);
    run_test(0x01, 0x555555);
    run_test(0x02, 0xAAAAAA);
    run_test(0x03, 0xFFFFFF);
    run_test(0xFC, 0x000000);

    // COLOR_MODE_3_RGB
    PN_INFO("Testing COLOR_MODE_3_RGB");
    color_mode = COLOR_MODE_3_RGB;
    run_test(0x00, 0x000000);
    run_test(0x01, 0x0000FF);
    run_test(0x02, 0x00FF00);
    run_test(0x04, 0xFF0000);
    run_test(0xF8, 0x000000);

    // COLOR_MODE_3_GRAY
    PN_INFO("Testing COLOR_MODE_3_GRAY");
    color_mode = COLOR_MODE_3_GRAY;
    run_test(0x00, 0x000000);
    run_test(0x01, 0x242424);
    run_test(0x02, 0x494949);
    run_test(0x03, 0x6D6D6D);
    run_test(0x04, 0x929292);
    run_test(0x07, 0xFFFFFF);
    run_test(0xF8, 0x000000);

    // COLOR_MODE_4_RGBI
    PN_INFO("Testing COLOR_MODE_4_RGBI");
    color_mode = COLOR_MODE_4_RGBI;
    run_test(0x00, 0x000000);
    run_test(0x04, 0xAA0000);
    run_test(0x02, 0x00AA00);
    run_test(0x01, 0x0000AA);
    run_test(0x0C, 0xFF5555);
    run_test(0x0A, 0x55FF55);
    run_test(0x09, 0x5555FF);
    run_test(0x0F, 0xFFFFFF);
    run_test(0xF0, 0x000000);

    // COLOR_MODE_4_GRAY
    PN_INFO("Testing COLOR_MODE_4_GRAY");
    color_mode = COLOR_MODE_4_GRAY;
    run_test(0x00, 0x000000);
    run_test(0x0A, 0xAAAAAA);
    run_test(0x0F, 0xFFFFFF);
    run_test(0xFFF0, 0x000000);

    // COLOR_MODE_8_RGB332
    PN_INFO("Testing COLOR_MODE_8_RGB332");
    color_mode = COLOR_MODE_8_RGB332;
    run_test(0x00, 0x000000);
    run_test(0x20, 0x240000);
    run_test(0x04, 0x002400);
    run_test(0x01, 0x000055);
    run_test(0xFF, 0xFFFFFF);
    run_test(0xFF00, 0x000000);

    // COLOR_MODE_8_GRAY
    PN_INFO("Testing COLOR_MODE_8_GRAY");
    color_mode = COLOR_MODE_8_GRAY;
    run_test(0x00, 0x000000);
    run_test(0x42, 0x424242);
    run_test(0xFF00, 0x000000);

    // COLOR_MODE_16_RGB565
    PN_INFO("Testing COLOR_MODE_16_RGB565");
    color_mode = COLOR_MODE_16_RGB565;
    run_test(0x0000, 0x000000);
    run_test(0x0800, 0x080000);
    run_test(0x0020, 0x000400);
    run_test(0x0001, 0x000008);
    run_test(0xFFFF, 0xFFFFFF);
    run_test(0xFFFF0000, 0x000000);

    // COLOR_MODE_32_RGB
    PN_INFO("Testing COLOR_MODE_32_RGB");
    color_mode = COLOR_MODE_32_RGB;
    run_test(0x000000, 0x000000);
    run_test(0x123456, 0x123456);
    run_test(0xFF000000, 0x000000);

    // ---------------
    // Not implemented
    PN_INFO("Testing unimplemented modes:");

    // COLOR_MODE_2_MAP
    PN_INFO("Testing COLOR_MODE_2_MAP");
    color_mode = COLOR_MODE_2_MAP;
    run_test(0x00, 0x000000);
    run_test(0xFF, 0x000000);

    // COLOR_MODE_3_MAP
    PN_INFO("Testing COLOR_MODE_3_MAP");
    color_mode = COLOR_MODE_3_MAP;
    run_test(0x00, 0x000000);
    run_test(0xFF, 0x000000);

    // COLOR_MODE_4_MAP
    PN_INFO("Testing COLOR_MODE_4_MAP");
    color_mode = COLOR_MODE_4_MAP;
    run_test(0x00, 0x000000);
    run_test(0xFF, 0x000000);

    // COLOR_MODE_8_MAP
    PN_INFO("Testing COLOR_MODE_8_MAP");
    color_mode = COLOR_MODE_8_MAP;
    run_test(0x00, 0x000000);
    run_test(0xFF, 0x000000);

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}
