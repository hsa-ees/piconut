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
 * @fn SC_MODULE(m_audio_single)
 * @author Tristan Kundrat
 * @brief Wishbone Slave Module for one audio voice
 *
 *
 * This module is the hardware-wishbone SLAVE unit for one voice of the audio synthesizer peripheral.
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
 * @param[in] base_offset_i base address offset, at which the modules registers can be accessed
 * @param[out] audio_o 16 bit unsigned audio output
 *
 *
 */

#ifndef __AUDIO_SINGLE_H__
#define __AUDIO_SINGLE_H__

#include <piconut.h> // contains all PN_<> Macros and is part of the PicoNut

#include "audio_defs.h"

// header files of submodules
#include "squarewave.h"
#include "trianglewave.h"
#include "sawtoothwave.h"
#include "sinewave.h"

#ifndef WB_DAT_WIDTH
#define WB_DAT_WIDTH 32
#endif

#ifndef WB_ADR_WIDTH
#define WB_ADR_WIDTH 32
#endif

// waveform_type
typedef enum
{
    WF_SQUARE = 0,
    WF_TRIANGLE,
    WF_SAWTOOTH,
    WF_SINE,
} e_waveform_t;

SC_MODULE(m_audio_single)
{

public:
    enum e_wb_state
    {
        WB_IDLE = 0,
        WB_READ,
        WB_WRITE1,
        WB_WRITE2

    };

    enum e_wb_addressrange
    {
        ADR_REG_0 = 0x0,
        ADR_REG_1 = 0x4,

    };

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
    sc_out<sc_uint<16>> PN_NAME(audio_o);

    // base address offset
    sc_in<sc_uint<8>> PN_NAME(base_offset_i);

    SC_CTOR(m_audio_single)
    {
        SC_METHOD(proc_comb_wb_slave);
        sensitive << wb_stb_i
                  << wb_cyc_i
                  << wb_we_i
                  << wb_adr_i
                  << wb_dat_i
                  << wb_current_state
                  << wb_sel_i
                  << register_0
                  << register_1
                  << base_offset_i;

        SC_METHOD(proc_comb_voice_select);
        sensitive << enable
                  << waveform_type
                  << step_height
                  << frequency_divisor
                  << squarewave_audio
                  << trianglewave_audio
                  << sawtoothwave_audio
                  << sinewave_audio;

        SC_METHOD(proc_comb_reg_to_sig);
        sensitive << register_0
                  << register_1;

        SC_CTHREAD(proc_clk_state, clk.pos()); // create clock sensitive process
        reset_signal_is(reset, true);

        SC_CTHREAD(proc_clk_wb_slave, clk.pos()); // create clock sensitive process
        reset_signal_is(reset, true);

        init_submodules();
    }

    void pn_trace(
        sc_trace_file * tf,
        int level = 1);

    /**
     * @brief Handels the wishbone slave functions */
    void proc_clk_wb_slave();
    void proc_clk_state();

    void proc_comb_wb_slave();

    void proc_comb_voice_select();
    void proc_comb_reg_to_sig();

    // Submodules
    m_squarewave* squarewave;
    m_trianglewave* trianglewave;
    m_sawtoothwave* sawtoothwave;
    m_sinewave* sinewave;

    /**
     * @brief This function is for the write byte select functionality of the wishbone bus.
     * @param input_word Hand over the Dataword you would like to get changed.
     */
    sc_uint<32> write_with_byte_select(sc_uint<32> input_word);

    /**
     * @brief This function is for the read byte select functionality of the wishbone bus.
     * @param input_word Hand over the Dataword you would like to get changed.
     */
    sc_uint<32> read_with_byte_select(sc_uint<32> input_word);

protected:
    // Registers
    sc_signal<sc_uint<3>> PN_NAME(wb_current_state);
    sc_signal<sc_uint<3>> PN_NAME(wb_next_state);

    sc_signal<sc_uint<32>> PN_NAME(register_0);
    sc_signal<sc_uint<32>> PN_NAME(register_1);

    sc_signal<bool> PN_NAME(c_wb_write_en);

    sc_signal<sc_uint<32>> PN_NAME(frequency_divisor);
    sc_signal<sc_uint<16>> PN_NAME(step_height);
    sc_signal<bool> PN_NAME(enable);

    sc_signal<sc_uint<16>> PN_NAME(squarewave_audio);
    sc_signal<bool> PN_NAME(squarewave_enable);

    sc_signal<sc_uint<16>> PN_NAME(trianglewave_audio);
    sc_signal<bool> PN_NAME(trianglewave_enable);

    sc_signal<sc_uint<16>> PN_NAME(sawtoothwave_audio);
    sc_signal<bool> PN_NAME(sawtoothwave_enable);

    sc_signal<sc_uint<16>> PN_NAME(sinewave_audio);
    sc_signal<bool> PN_NAME(sinewave_enable);

    sc_signal<sc_uint<8>> PN_NAME(waveform_type);

    // Methods
    void init_submodules();
};

#endif //__AUDIO_SINGLE_H__
