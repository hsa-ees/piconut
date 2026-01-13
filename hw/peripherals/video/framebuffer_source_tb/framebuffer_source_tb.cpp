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

#include "framebuffer_source.h"
#include <systemc.h>

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(ctl_clk);
sc_signal<bool> PN_NAME(ctl_reset);
sc_signal<sc_uint<FB_RAM_ADDR_WIDTH>> PN_NAME(ctl_addr);
sc_signal<sc_uint<FB_RAM_DATA_WIDTH>> PN_NAME(ctl_data_in);
sc_signal<sc_uint<FB_RAM_DATA_WIDTH>> PN_NAME(ctl_data_out);
sc_signal<bool> PN_NAME(ctl_write_en);

sc_signal<bool> PN_NAME(vid_clk);
sc_signal<bool> PN_NAME(vid_enable);
sc_signal<sc_uint<H_COUNTER_WIDTH>> PN_NAME(vid_column);
sc_signal<sc_uint<V_COUNTER_WIDTH>> PN_NAME(vid_line);
sc_signal<sc_uint<32>> PN_NAME(vid_output);

void run_cycle_ctl(int cycles = 1)
{
    for(int i = 0; i < cycles; i++)
    {
        ctl_clk = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
        ctl_clk = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

void run_cycle_vid(int cycles = 1)
{
    for(int i = 0; i < cycles; i++)
    {
        vid_clk = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
        vid_clk = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

int sc_main(int argc, char** argv)
{
    const unsigned int TEST_SEED = 0x4242;
    const int data_mask = (1 << FB_RAM_DATA_WIDTH) - 1;

    unsigned int test_buffer[40 * 30];

    srand(TEST_SEED);
    for(int i = 0; i < 40 * 30; i++)
    {
        test_buffer[i] = rand() & data_mask;
    }

    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("framebuffer_source");

    m_framebuffer_source dut_inst{"dut_inst"};
    dut_inst.ctl_clk(ctl_clk);
    dut_inst.ctl_addr(ctl_addr);
    dut_inst.ctl_reset(ctl_reset);
    dut_inst.ctl_data_in(ctl_data_in);
    dut_inst.ctl_data_out(ctl_data_out);
    dut_inst.ctl_write_en(ctl_write_en);

    dut_inst.vid_clk(vid_clk);
    dut_inst.vid_enable(vid_enable);
    dut_inst.vid_column(vid_column);
    dut_inst.vid_line(vid_line);
    dut_inst.vid_output(vid_output);

    dut_inst.pn_trace(tf, pn_cfg_vcd_level);

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Reset
    ctl_reset = 1;
    run_cycle_ctl(1);
    ctl_reset = 0;
    run_cycle_ctl(1);

    vid_enable = 1;

    run_cycle_vid(1);

    PN_INFO("[Writing Buffer]");

    for(int i = 0; i < 40 * 30; i++)
    {
        int data = test_buffer[i];

        ctl_addr = i;
        ctl_data_in = data;
        ctl_write_en = 1;
        run_cycle_ctl();
        PN_ASSERT(ctl_data_in.read() == ctl_data_out.read());
    }

    PN_INFO("[Reading Buffer CTL]");

    for(int i = 0; i < 40 * 30; i++)
    {
        int data = test_buffer[i];

        ctl_addr = i;
        ctl_data_in = 0;
        ctl_write_en = 0;
        run_cycle_ctl();
        PN_ASSERTF(ctl_data_out.read() == data, ("Failed reading at %08X; Expected: %d, Got: %d", i, data, (unsigned int)ctl_data_out.read()));
    }

    PN_INFO("[Reading Buffer VID]");

    for(int y = 0; y < 480; y++)
    {
        for(int x = 0; x < 640; x++)
        {
            int data = test_buffer[(x >> 4) + (y >> 4) * 40];

            vid_column = x;
            vid_line = y;
            run_cycle_vid();
            PN_ASSERTF(vid_output.read() == data, ("Failed reading at %dx%d; Expected: %d, Got: %d", x, y, data, (unsigned int)vid_output.read()));
        }
    }

    PN_INFO("[Disabling video port]");
    vid_enable = 0;
    run_cycle_vid();
    PN_ASSERTF(vid_output.read() == 0x00, ("Failed disabling video port; Got: %d", vid_output.read()));

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}
