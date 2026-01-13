/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Tristan Kundrat <tristan.kundrat@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description: This is the testbench generating an audio signal using the wb_audio
               module. For a full test run, use the following command:
                   cd .. && make run-tb-trace && ./compare_vcd.py

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
#include "../audio.h"
#include "../audio_single.h"
#include <stdint.h>
#include <systemc.h>

// defines for testscenario
#define PERIOD_NS 2000.0
#define CLK_FREQ_HZ ((1000 * 1000 * 1000) / PERIOD_NS)

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

// signals for wishbone bus
sc_signal<sc_uint<32>> PN_NAME(wb_adr_i);
sc_signal<sc_uint<32>> PN_NAME(wb_dat_o);
sc_signal<sc_uint<32>> PN_NAME(wb_dat_i);
sc_signal<sc_uint<4>> PN_NAME(wb_sel_i);
sc_signal<bool> PN_NAME(wb_we_i);
sc_signal<bool> PN_NAME(wb_stb_i);
sc_signal<bool> PN_NAME(wb_ack_o);
sc_signal<bool> PN_NAME(wb_cyc_i);

sc_signal<bool> PN_NAME(wb_rty_o);
sc_signal<bool> PN_NAME(wb_err_o);

// signals for audio
sc_signal<sc_uint<16>> PN_NAME(audio_r);
sc_signal<sc_uint<16>> PN_NAME(audio_l);

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

void write_register(int offset, sc_uint<32> data, sc_uint<4> mask)
{
    wb_adr_i = (PN_CFG_AUDIO_BASE_ADDRESS + offset);
    wb_dat_i = data;
    wb_sel_i = mask;
    wb_cyc_i = 1;
    wb_we_i = 1;
    wb_stb_i = 1;

    while(wb_ack_o.read() == 0)
        run_cycle();

    wb_stb_i = 0;
    wb_we_i = 0;
    wb_cyc_i = 0;
}

void write_audio_enable(bool enable, int voice)
{
    int offset = 0x0 + (0x8 * voice);
    write_register(offset, (sc_uint<32>)enable, 0b0001);
}

void write_audio_waveform_type(sc_uint<8> waveform_type, int voice)
{
    int offset = 0x0 + (0x8 * voice);
    write_register(offset, (sc_uint<32>)(waveform_type << 8), 0b0010);
}

void write_audio_step_height(sc_uint<16> step_height, int voice)
{
    int offset = 0x0 + (0x8 * voice);
    write_register(offset, (sc_uint<32>)(((sc_uint<16>)step_height) << 16), 0b1100);
}

void write_audio_frequency_divisor(sc_uint<32> frequency_divisor, int voice)
{
    int offset = 0x4 + (0x8 * voice);
    write_register(offset, frequency_divisor, 0b1111);
}

void write_audio_frequency(int freq_hz, int voice)
{
    write_audio_frequency_divisor((sc_uint<32>)(CLK_FREQ_HZ / freq_hz), voice);
}

sc_uint<32> read_register(int offset, sc_uint<4> mask)
{
    sc_uint<32> data = 0;

    wb_adr_i = (PN_CFG_AUDIO_BASE_ADDRESS + offset);
    wb_sel_i = mask;
    wb_cyc_i = 1;
    wb_we_i = 0;
    wb_stb_i = 1;

    while(wb_ack_o.read() == 0)
        run_cycle();

    data = wb_dat_o.read();

    wb_stb_i = 0;
    wb_we_i = 0;
    wb_cyc_i = 0;

    return data;
}

bool read_audio_enable(int voice)
{
    int offset = 0x0 + (0x8 * voice);
    return (bool)(read_register(offset, 0b0001) & 0b1);
}

sc_uint<8> read_audio_waveform_type(int voice)
{
    int offset = 0x0 + (0x8 * voice);
    return (sc_uint<8>)(read_register(offset, 0b0010) >> 8);
}

sc_uint<16> read_audio_step_height(int voice)
{
    int offset = 0x0 + (0x8 * voice);
    return (sc_uint<16>)(read_register(offset, 0b1100) >> 16);
}

sc_uint<32> read_audio_frequency_divisor(int voice)
{
    int offset = 0x4 + (0x8 * voice);
    return read_register(offset, 0b1111);
}

unsigned int read_audio_frequency(int voice)
{
    return (unsigned int)(read_audio_frequency_divisor(voice) / CLK_FREQ_HZ);
}

int sc_main(int argc, char** argv)
{

    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("audio_tb");

    // Initialize the Design under Testing (DUT)
    m_audio dut_inst{"i_dut"};

    // connect the wishbone signals

    dut_inst.clk(clk);
    dut_inst.reset(reset);
    dut_inst.wb_adr_i(wb_adr_i);
    dut_inst.wb_dat_i(wb_dat_i);
    dut_inst.wb_dat_o(wb_dat_o);
    dut_inst.wb_we_i(wb_we_i);
    dut_inst.wb_stb_i(wb_stb_i);
    dut_inst.wb_ack_o(wb_ack_o);
    dut_inst.wb_cyc_i(wb_cyc_i);
    dut_inst.wb_sel_i(wb_sel_i);
    dut_inst.wb_err_o(wb_err_o);
    dut_inst.wb_rty_o(wb_rty_o);
    dut_inst.audio_l(audio_l);
    dut_inst.audio_r(audio_r);

    // Traces of signals here

    dut_inst.pn_trace(tf, pn_cfg_vcd_level); // trace signals of

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

    const double C = 32.7;
    const double D = 36.7;
    const double E = 41.2;
    const double F = 43.7;
    const double G = 50.0;
    const double A = 55.0;
    const double H = 61.7;
    const double notes[8] = {C, D, E, F, G, A, H, C * 2};

    int amplitude = 0xFFFFFF;

    for(uint voice = 0; voice < NUM_AUDIO - 2; voice++)
    {
        for(uint i = 0; i < 8; i++)
        {
            double freq = notes[i] * (1 << i);

            int freqdiv = (int)(CLK_FREQ_HZ / freq);

            int step_height_tri = (int)(amplitude * 2 / freqdiv);
            int step_height_saw = (int)(amplitude / freqdiv);
            int step_height_square = amplitude;

            write_audio_step_height((sc_uint<16>)step_height_tri, voice);
            run_cycle(3);
            write_audio_step_height((sc_uint<16>)step_height_saw, voice + 1);
            run_cycle(3);
            write_audio_step_height((sc_uint<16>)step_height_square, voice + 2);
            run_cycle(3);

            write_audio_waveform_type(WF_TRIANGLE, voice);
            run_cycle(3);
            write_audio_waveform_type(WF_SAWTOOTH, voice + 1);
            run_cycle(3);
            write_audio_waveform_type(WF_SQUARE, voice + 2);
            run_cycle(3);

            write_audio_frequency_divisor((sc_uint<32>)freqdiv, voice);
            run_cycle(3);
            write_audio_frequency_divisor((sc_uint<32>)freqdiv, voice + 1);
            run_cycle(3);
            write_audio_frequency_divisor((sc_uint<32>)freqdiv, voice + 2);
            run_cycle(3);

            write_audio_enable(true, voice);
            run_cycle(3);
            write_audio_enable(true, voice + 1);
            run_cycle(3);
            write_audio_enable(true, voice + 2);
            run_cycle(3);

            run_cycle((int)(CLK_FREQ_HZ * 5 / freq));

            write_audio_enable(false, voice);
            run_cycle(3);
            write_audio_enable(false, voice + 1);
            run_cycle(3);
            write_audio_enable(false, voice + 2);
            run_cycle(3);
        }
    }

    run_cycle(); // end with a wait

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}
