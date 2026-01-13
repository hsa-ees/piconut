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

#include "audio_single.h"

/**
 * @brief this function is used to generate a tracefile
 * you may use this function to add Signals and Ports or even Members of Structs to your .vcd tracefile
 * use PN_TRACE(tf, signalname) here to add whatever signal u need to trace.
 * @param tf this is the tracefile object
 * @param level is used as a selector when to enable a trace*/
void m_audio_single::pn_trace(sc_trace_file* tf, int level)
{

    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, wb_ack_o);
    PN_TRACE(tf, wb_adr_i);
    PN_TRACE(tf, wb_current_state);
    PN_TRACE(tf, wb_cyc_i);
    PN_TRACE(tf, wb_dat_i);
    PN_TRACE(tf, wb_dat_o);
    PN_TRACE(tf, wb_sel_i);
    PN_TRACE(tf, wb_we_i);
    PN_TRACE(tf, wb_err_o);

    PN_TRACE(tf, register_0);
    PN_TRACE(tf, register_1);

    PN_TRACE(tf, c_wb_write_en);

    PN_TRACE(tf, frequency_divisor);
    PN_TRACE(tf, step_height);
    PN_TRACE(tf, enable);
    PN_TRACE(tf, audio_o);

    PN_TRACE(tf, squarewave_audio);
    PN_TRACE(tf, squarewave_enable);

    PN_TRACE(tf, trianglewave_audio);
    PN_TRACE(tf, trianglewave_enable);

    PN_TRACE(tf, sawtoothwave_audio);
    PN_TRACE(tf, sawtoothwave_enable);

    PN_TRACE(tf, sinewave_audio);
    PN_TRACE(tf, sinewave_enable);
}

void m_audio_single::proc_comb_wb_slave() // Wishbone Statetransition logic (combinatory)
{
    // set idle Outputs to default
    wb_ack_o = 0;
    wb_dat_o = 0;
    wb_err_o = 0;
    wb_rty_o = 0;

    c_wb_write_en = 0;

    wb_next_state = wb_current_state.read(); // default next state is current state

    switch(wb_current_state.read())
    {
        case WB_IDLE: // Wishbone Idle State to check if we have a valid transaction

            if(wb_stb_i.read() == 1 && wb_cyc_i.read() == 1) // WB Strobe and cycle valid,
            {
                if(((PN_CFG_AUDIO_BASE_ADDRESS + base_offset_i.read()) <= wb_adr_i.read()) &&
                    (wb_adr_i.read() < (PN_CFG_AUDIO_BASE_ADDRESS + base_offset_i.read() + PN_AUDIO_SIZE_BYTE)))
                {
                    if(wb_we_i.read() == 1)
                    {
                        wb_next_state = WB_WRITE1; // Master writes to us (Slave)
                    }
                    else
                    {
                        wb_next_state = WB_READ; // Master reads from us (Slave)
                    }
                }
            }
            break;

        case WB_WRITE1: // Master writes to us -> read from input, write to our register

            c_wb_write_en = 1;

            // set the next wishbone state because writing is not finished yet
            wb_next_state = WB_WRITE2;
            break;

        case WB_WRITE2: // Write Acknowledge

            c_wb_write_en = 1;
            wb_ack_o = 1;

            // after setting the wishbone ack the writing is finished and we can go back to the idle state
            if(wb_stb_i.read() == 0 && wb_cyc_i.read() == 0)
            {
                wb_next_state = WB_IDLE;
            }
            break;

        case WB_READ: // Master Reads from us -> Write data to the bus output

            switch(wb_adr_i.read() - (PN_CFG_AUDIO_BASE_ADDRESS + base_offset_i.read())) // use Address offset to determine target register
            {

                // check which address is being addressed and write the address back as data value
                case e_wb_addressrange::ADR_REG_0:
                    // read the data from the wishbone bus addressed by the offset to the first dummy register with the byte select functionality.

                    wb_dat_o =
                        read_with_byte_select(register_0.read());
                    break;
                case e_wb_addressrange::ADR_REG_1:

                    wb_dat_o =
                        read_with_byte_select(register_1.read());
                    break;
            }

            wb_ack_o = 1;

            if(wb_stb_i.read() == 0 && wb_cyc_i.read() == 0)
            {
                wb_next_state = WB_IDLE;
            }
            break;
    }
}

void m_audio_single::proc_clk_wb_slave()
{
    register_0 = 0;
    register_1 = 0;

    while(true)
    {
        wait();

        if(c_wb_write_en.read() == 1)
        {
            switch(wb_adr_i.read() - (PN_CFG_AUDIO_BASE_ADDRESS + base_offset_i.read())) // use Address offset to determine target register
            {
                case e_wb_addressrange::ADR_REG_0:
                    // write seomthing in the ADR_REG_0 addressrange
                    register_0 =
                        write_with_byte_select(register_0.read());
                    break;
                case e_wb_addressrange::ADR_REG_1:
                    // write something in the ADR_REG_1 addressrange
                    register_1 =
                        write_with_byte_select(register_1.read());
                    break;
            }
        }
    }
}

// state transition process
void m_audio_single::proc_clk_state()
{
    wb_current_state = WB_IDLE;

    while(true)
    {
        wait();

        wb_current_state = wb_next_state;
    }
}

sc_uint<WB_DAT_WIDTH> m_audio_single::write_with_byte_select(
    sc_uint<WB_DAT_WIDTH> input_word)
{
    sc_uint<WB_DAT_WIDTH> mask = 0;
    sc_uint<WB_DAT_WIDTH / 8> wb_sel_i_var = wb_sel_i.read();

    // Selective byte masking
    if(wb_sel_i_var & 0b0001)
    {
        mask |= 0x000000FF; // Byte 0
    }
    if(wb_sel_i_var & 0b0010)
    {
        mask |= 0x0000FF00; // Byte 1
    }
    if(wb_sel_i_var & 0b0100)
    {
        mask |= 0x00FF0000; // Byte 2
    }
    if(wb_sel_i_var & 0b1000)
    {
        mask |= 0xFF000000; // Byte 3
    }

    return (input_word & ~mask) | (wb_dat_i.read() & mask);
}

sc_uint<WB_DAT_WIDTH> m_audio_single::read_with_byte_select(
    sc_uint<WB_DAT_WIDTH> input_word)
{
    sc_uint<WB_DAT_WIDTH> mask = 0;
    sc_uint<WB_DAT_WIDTH / 8> wb_sel_i_var = wb_sel_i.read();

    // Selective byte masking
    if(wb_sel_i_var & 0b0001)
    {
        mask |= 0x000000FF; // Byte 0
    }
    if(wb_sel_i_var & 0b0010)
    {
        mask |= 0x0000FF00; // Byte 1
    }
    if(wb_sel_i_var & 0b0100)
    {
        mask |= 0x00FF0000; // Byte 2
    }
    if(wb_sel_i_var & 0b1000)
    {
        mask |= 0xFF000000; // Byte 3
    }

    return (input_word & mask);
}

void m_audio_single::proc_comb_reg_to_sig()
{
    // connecting registers to signals, take this C struct as a reference:
    /*
    typedef struct {
        unsigned int enable            : 1;
        unsigned int reserved          : 7;
        unsigned int waveform_type     : 8;
        unsigned int step_height       :16;
        unsigned int frequency_divisor :32;
    } synth_regs_t;
    */

    enable = (bool)register_0.read().range(0, 0);
    waveform_type = (sc_uint<8>)register_0.read().range(15, 8);
    step_height = (sc_uint<16>)register_0.read().range(31, 16);
    frequency_divisor = register_1.read();
}

void m_audio_single::proc_comb_voice_select()
{
    // default signals
    squarewave_enable = 0;
    trianglewave_enable = 0;
    sawtoothwave_enable = 0;
    sinewave_enable = 0;
    audio_o = 0;

    if(enable.read())
    {
        switch(waveform_type.read())
        {
            case WF_SQUARE:
                squarewave_enable = 1;
                audio_o = squarewave_audio.read();
                break;
            case WF_TRIANGLE:
                trianglewave_enable = 1;
                audio_o = trianglewave_audio.read();
                break;
            case WF_SAWTOOTH:
                sawtoothwave_enable = 1;
                audio_o = sawtoothwave_audio.read();
                break;
            case WF_SINE:
                sinewave_enable = 1;
                audio_o = sinewave_audio.read();
                break;
            default:
                break;
        }
    }
}

void m_audio_single::init_submodules()
{
    squarewave = sc_new<m_squarewave>("squarewave");
    squarewave->clk(clk);
    squarewave->reset(reset);
    squarewave->enable(squarewave_enable);
    squarewave->frequency_divisor(frequency_divisor);
    squarewave->audio_signal(squarewave_audio);
    squarewave->step_height(step_height);

    trianglewave = sc_new<m_trianglewave>("trianglewave");
    trianglewave->clk(clk);
    trianglewave->reset(reset);
    trianglewave->enable(trianglewave_enable);
    trianglewave->frequency_divisor(frequency_divisor);
    trianglewave->audio_signal(trianglewave_audio);
    trianglewave->step_height(step_height);

    sawtoothwave = sc_new<m_sawtoothwave>("sawtoothwave");
    sawtoothwave->clk(clk);
    sawtoothwave->reset(reset);
    sawtoothwave->enable(sawtoothwave_enable);
    sawtoothwave->frequency_divisor(frequency_divisor);
    sawtoothwave->audio_signal(sawtoothwave_audio);
    sawtoothwave->step_height(step_height);

    sinewave = sc_new<m_sinewave>("sinewave");
    sinewave->clk(clk);
    sinewave->reset(reset);
    sinewave->enable(sinewave_enable);
    sinewave->frequency_divisor(frequency_divisor);
    sinewave->audio_signal(sinewave_audio);
    sinewave->step_height(step_height);
}
