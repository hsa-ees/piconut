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

/**
 * @fn SC_MODULE(m_audio)
 * @author Tristan Kundrat
 * @brief Audio module combining multiple synthesizer voices
 *
 * This module instantiates multiple `m_audio_single` modules and adds their
 * `audio_o` outputs, resulting in a 16 bit `audio_l` and `audio_r` signal.
 * The number is configured using the `PN_CFG_AUDIO_EXP` macro, such that the
 * number of wanted voices per channel (left/right) m = 2^(`PN_CFG_AUDIO_EXP`).
 * 
 *
 *
 *
 * @par Ports:
 * @param[in] clk clock of the module
 * @param[in] reset reset of the module
 * @param[in] wb_adr_i Wishbone Adress input
 * @param[out] wb_dat_o Wishbone Data out
 * @param[in] wb_dat_i Wishbone Data in
 * @param[in] wb_we_i Wishbone Write-Enable in
 * @param[in] wb_stb_i Wishbone Strobe in
 * @param[in] wb_cyc_i Wishbone Cycle in
 * @param[out] wb_ack_o Wishbone ACK input
 * @param[in] wb_cti_i // Wishbone cycle type identifier (optional, for registered feedback)
 * @param[in] wb_bte_i // Wishbone burst type extension  (optional, for registered feedback)
 * @param[out] wb_error_o // Wishbone termination w/ error (optional)
 * @param[out] wb_rty_o // Wishbone termination w/ retry (optional)
 * @param[out] audio_r 16 bit unsigned audio output (right channel)
 * @param[out] audio_l 16 bit unsigned audio output (left channel)
 *
 *
 */

#ifndef __AUDIO_H__
#define __AUDIO_H__

#include <sysc/utils/sc_vector.h>
#include <systemc.h>
#include <piconut.h>
#include <piconut-config.h>
#include "audio_defs.h"

#ifndef WB_DAT_WIDTH
#define WB_DAT_WIDTH 32
#endif

#ifndef WB_ADR_WIDTH
#define WB_ADR_WIDTH 32
#endif

#define NUM_AUDIO (1U << (PN_CFG_AUDIO_EXP + 1))

SC_MODULE(m_audio)
{
    // wishbone slave ports ...
    sc_in_clk PN_NAME(clk);     // clock input
    sc_in<bool> PN_NAME(reset); // reset

    sc_in<bool> PN_NAME(wb_stb_i); // strobe input
    sc_in<bool> PN_NAME(wb_cyc_i); // cycle valid input
    sc_in<bool> PN_NAME(wb_we_i);  // indicates write transfer
    // sc_in<sc_uint<3>> PN_NAME(wb_cti_i);                // cycle type identifier (optional, for registered feedback)
    // sc_in<sc_uint<2>> PN_NAME(wb_bte_i);                // burst type extension (optional, for registered feedback)
    sc_in<sc_uint<WB_DAT_WIDTH / 8>> PN_NAME(wb_sel_i); // byte select inputs
    sc_out<bool> PN_NAME(wb_ack_o);                     // normal termination
    sc_out<bool> PN_NAME(wb_err_o);                     // termination w/ error (optional)
    sc_out<bool> PN_NAME(wb_rty_o);                     // termination w/ retry (optional)

    sc_in<sc_uint<WB_ADR_WIDTH>> PN_NAME(wb_adr_i);  // address bus inputs
    sc_in<sc_uint<WB_DAT_WIDTH>> PN_NAME(wb_dat_i);  // input data bus
    sc_out<sc_uint<WB_DAT_WIDTH>> PN_NAME(wb_dat_o); // output data bus

    // 16 bit unsigned digital audio output
    sc_out<sc_uint<16>> PN_NAME(audio_l);
    sc_out<sc_uint<16>> PN_NAME(audio_r);

#if !PN_PRESYNTHESIZED_H_ONLY(AUDIO)

    SC_CTOR(m_audio)
    {
        SC_METHOD(proc_comb_wb);
        sensitive << wb_stb_i
                  << wb_cyc_i
                  << wb_we_i
                  << wb_sel_i
                  << wb_ack_o
                  << wb_err_o
                  << wb_rty_o
                  << wb_adr_i
                  << wb_dat_i
                  << wb_dat_o;
        for(unsigned int i = 0; i < NUM_AUDIO; i++)
        {
            sensitive << audio_singles_ack_o[i]
                      << audio_singles_dat_o[i]
                      << audio_singles_err_o[i]
                      << audio_singles_rty_o[i];
        }

        SC_METHOD(proc_comb_audio);
        for(unsigned int i = 0; i < NUM_AUDIO; i++)
        {
            sensitive << audio_singles_audio[i];
        }

        init_submodules();
    }
    void pn_trace(
        sc_trace_file * tf,
        int level = 1);

    // Processes
    void proc_comb_wb();
    void proc_comb_audio();

    // Submodules
    class m_audio_single* audio_singles[NUM_AUDIO];

#else // !PN_PRESYNTHESIZED_H_ONLY(AUDIO)

    // Header-only variant: Implement all syntactically required methods by empty inliners ...
    SC_CTOR(m_audio) {}
    void pn_trace(sc_trace_file * tf, int level = 1) {}

#endif // !PN_PRESYNTHESIZED_H_ONLY(AUDIO)

protected:
#if !PN_PRESYNTHESIZED_H_ONLY(AUDIO)
    // Registers
    sc_vector<sc_signal<sc_uint<8>>> PN_NAME_VEC(audio_singles_base_offsets, NUM_AUDIO);
    sc_vector<sc_signal<bool>> PN_NAME_VEC(audio_singles_ack_o, NUM_AUDIO);
    sc_vector<sc_signal<bool>> PN_NAME_VEC(audio_singles_err_o, NUM_AUDIO);
    sc_vector<sc_signal<bool>> PN_NAME_VEC(audio_singles_rty_o, NUM_AUDIO);
    sc_vector<sc_signal<sc_uint<WB_DAT_WIDTH>>> PN_NAME_VEC(audio_singles_dat_o, NUM_AUDIO);
    sc_vector<sc_signal<sc_uint<16>>> PN_NAME_VEC(audio_singles_audio, NUM_AUDIO);
    // Methods
    void init_submodules();
#else // !PN_PRESYNTHESIZED_H_ONLY(AUDIO)

    // Declare the module to be pre-synthesized ...
    PN_PRESYNTHESIZED;

#endif // !PN_PRESYNTHESIZED_H_ONLY(AUDIO)
};

#endif //__AUDIO_H__
