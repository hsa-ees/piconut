/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Tristan Kundrat <tristan.kundrat@tha.de>
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

#include "audio.h"

#include "audio_single.h"

/**
 * @brief this function is used to generate a tracefile
 * you may use this function to add Signals and Ports or even Members of Structs to your .vcd tracefile
 * use PN_TRACE(tf, signalname) here to add whatever signal u need to trace.
 * @param tf this is the tracefile object
 * @param level is used as a selector when to enable a trace*/
void m_audio::pn_trace(sc_trace_file* tf, int level)
{

    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, wb_ack_o);
    PN_TRACE(tf, wb_adr_i);
    PN_TRACE(tf, wb_cyc_i);
    PN_TRACE(tf, wb_dat_i);
    PN_TRACE(tf, wb_dat_o);
    PN_TRACE(tf, wb_sel_i);
    PN_TRACE(tf, wb_we_i);
    PN_TRACE(tf, wb_err_o);

    PN_TRACE(tf, audio_l);
    PN_TRACE(tf, audio_r);

    PN_TRACE_BUS(tf, audio_singles_ack_o, NUM_AUDIO);
    PN_TRACE_BUS(tf, audio_singles_err_o, NUM_AUDIO);
    PN_TRACE_BUS(tf, audio_singles_rty_o, NUM_AUDIO);
    PN_TRACE_BUS(tf, audio_singles_dat_o, NUM_AUDIO);
    PN_TRACE_BUS(tf, audio_singles_audio, NUM_AUDIO);

    for(unsigned int i = 0; i < NUM_AUDIO; i++)
    {
        audio_singles[i]->pn_trace(tf, level);
    }
}

void m_audio::proc_comb_wb()
{
    sc_uint<PN_CFG_AUDIO_EXP + 1> audio_sel =
        ((wb_adr_i.read() - PN_CFG_AUDIO_BASE_ADDRESS) >> 3);
    wb_ack_o = audio_singles_ack_o[audio_sel].read();
    wb_dat_o = audio_singles_dat_o[audio_sel].read();
    wb_err_o = audio_singles_err_o[audio_sel].read();
    wb_rty_o = audio_singles_rty_o[audio_sel].read();

    for(unsigned int i = 0; i < NUM_AUDIO; i++)
    {
        // 4 bytes per word, 2 words, i * 4 * 2 == i << 3
        audio_singles_base_offsets[i] = (sc_uint<8>)(i << 3);
    }
}

void m_audio::proc_comb_audio()
{
    sc_uint<16 + PN_CFG_AUDIO_EXP> audio_l_accurate = 0;
    sc_uint<16 + PN_CFG_AUDIO_EXP> audio_r_accurate = 0;
    for(unsigned int i = 0; i < NUM_AUDIO; i++)
    {
        if(i & 0b1) // odd
        {
            audio_r_accurate = audio_r_accurate +
                               audio_singles_audio[i].read();
        }
        else // even
        {
            audio_l_accurate = audio_l_accurate +
                               audio_singles_audio[i].read();
        }
    }
    audio_r = (sc_uint<16>)(audio_r_accurate >> PN_CFG_AUDIO_EXP);
    audio_l = (sc_uint<16>)(audio_l_accurate >> PN_CFG_AUDIO_EXP);
}

void m_audio::init_submodules()
{
    // create NUM_AUDIO Audio modules total
    for(unsigned int i = 0; i < NUM_AUDIO; i++)
    {
        std::string name = "audio_single_" + std::to_string(i);
        audio_singles[i] = sc_new<m_audio_single>(name.c_str());
        audio_singles[i]->clk(clk);
        audio_singles[i]->reset(reset);
        audio_singles[i]->wb_stb_i(wb_stb_i);
        audio_singles[i]->wb_cyc_i(wb_cyc_i);
        audio_singles[i]->wb_we_i(wb_we_i);
        audio_singles[i]->wb_sel_i(wb_sel_i);
        audio_singles[i]->wb_ack_o(audio_singles_ack_o[i]);
        audio_singles[i]->wb_err_o(audio_singles_err_o[i]);
        audio_singles[i]->wb_rty_o(audio_singles_rty_o[i]);
        audio_singles[i]->wb_adr_i(wb_adr_i);
        audio_singles[i]->wb_dat_i(wb_dat_i);
        audio_singles[i]->wb_dat_o(audio_singles_dat_o[i]);
        audio_singles[i]->audio_o(audio_singles_audio[i]);
        audio_singles[i]->base_offset_i(audio_singles_base_offsets[i]);
    }
}
