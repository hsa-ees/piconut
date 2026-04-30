/**
 * @file i2c_data_control_tb.cpp
 * @brief
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)     2025 Eikya Lagisetti <eikya.lagisetti1@tha.de>
                    2025 Sebastian Ebenhöh <sebastian.moritz.ebenhoeh@tha.de>
                    2025 Johannes Fleiner <Johannes.Fleiner1@tha.de>

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

#include <systemc.h>
#include "../i2c_data_control.h"

#define PERIOD_NS 40
#define DATA_CLK_DIV 20
#define TRISE_DELAY 5

// clock & reset
sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

// controller
sc_signal<bool> PN_NAME(generate_start_cond_in);
sc_signal<bool> PN_NAME(start_tx_in);
sc_signal<bool> PN_NAME(start_rx_in);
sc_signal<bool> PN_NAME(send_ack_in);
sc_signal<bool> PN_NAME(send_nack_in);
sc_signal<sc_uint<8>> PN_NAME(tx_data_in);
sc_signal<bool> PN_NAME(start_ready_out);
sc_signal<bool> PN_NAME(shift_loaded_out);
sc_signal<bool> PN_NAME(ack_received_out);
sc_signal<bool> PN_NAME(nack_received_out);
sc_signal<bool> PN_NAME(byte_loaded_in_DR_out);
sc_signal<bool> PN_NAME(sending_data_out);
sc_signal<sc_uint<8>> PN_NAME(shift_data_out);

// clk_control
sc_signal<bool> PN_NAME(data_clk_pulse_write_in);
sc_signal<bool> PN_NAME(data_clk_pulse_read_in);
sc_signal<bool> PN_NAME(stop_ready_in);

// wb
sc_signal<bool> PN_NAME(enable_dr_write_out);

// data_control
sc_signal<bool> PN_NAME(sda_out);
sc_signal<bool> PN_NAME(sda_in);

/**
 * @fn run_cycle(int cycles, bool enable_data_clk)
 *
 * Advances the simulation by a given number of clock cycles.
 * Optionally generates data clock pulses.
 *
 * @param[in] cycles Number of clock cycles to simulate
 * @param[in] enable_data_clk Enable data clock pulse generation
 */
void run_cycle(int cycles = 1, bool enable_data_clk = false)
{
    static int cycle_cnt = -1;
    static int trise_cnt = 0;
    static bool pending = false;
    static bool toggle = true;

    for(int i = 0; i < cycles; i++)
    {
        bool pulse_write = false;
        bool pulse_read = false;

        if(enable_data_clk)
        {
            cycle_cnt++;

            pulse_write = ((cycle_cnt % DATA_CLK_DIV) == 0);
        }

        if(pulse_write)
        {
            toggle = !toggle;

            if(toggle)
            {
                pending = true;
                trise_cnt = TRISE_DELAY;
            }
            else
            {
                pulse_read = true;
            }
        }

        if(pending)
        {
            if(trise_cnt != 0)
            {
                trise_cnt--;
            }
            else
            {
                pulse_read = true;
                pending = false;
            }
        }

        data_clk_pulse_write_in.write(pulse_write);
        data_clk_pulse_read_in.write(pulse_read);

        clk = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
        clk = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

/**
 * @fn test_write_write()
 *
 * This test checks two consecutive I2C write operations.
 * It verifies START generation, transmission of two data bytes,
 * ACK after the first byte, NACK after the second byte, and the STOP condition.
 */
void test_write_write()
{
    run_cycle(1, false);

    reset.write(true);
    generate_start_cond_in.write(false);
    stop_ready_in.write(false);
    start_tx_in.write(false);
    start_rx_in.write(false);
    send_ack_in.write(false);
    send_nack_in.write(false);
    data_clk_pulse_write_in.write(false);
    data_clk_pulse_read_in.write(false);
    sda_in.write(false);    // initial ACK
    tx_data_in.write(0x71); // 0111 0001

    PN_INFO("m_i2c_data_control_tb: run test_write_write() ...");

    run_cycle(200, false);
    reset.write(false);
    run_cycle(1, false);

    generate_start_cond_in.write(true);
    run_cycle(3, false);
    generate_start_cond_in.write(false);

    PN_ASSERTM(sda_out.read() == false, "SDA not driven low during START condition");

    int cnt = 0;
    while(start_ready_out.read() == 0)
    {
        run_cycle(1, false);
        if(++cnt > 200)
            PN_ASSERTM(false, "START not completed (start_ready_out not asserted)");
    }

    start_tx_in.write(true);
    run_cycle(1, true);
    start_tx_in.write(false);

    cnt = 0;
    while(shift_loaded_out.read() == 0)
    {
        run_cycle(1, true);
        if(++cnt > 200)
            PN_ASSERTM(false, "TX did not start (shift_loaded_out not asserted)");
    }

    cnt = 0;
    while(sending_data_out.read() == 0)
    {
        run_cycle(1, true);
        if(++cnt > 200)
            PN_ASSERTM(false, "sending_data_out not asserted during TX phase");
    }

    cnt = 0;
    while(sda_out.read() == 1)
    {
        run_cycle(1, true);
        if(++cnt > 500)
            PN_ASSERTM(false, "SDA never went low during TX phase");
    }

    cnt = 0;
    while(sda_out.read() == 0)
    {
        run_cycle(1, true);
        if(++cnt > 500)
            PN_ASSERTM(false, "SDA never went high again before ACK phase");
    }

    cnt = 0;
    while(ack_received_out.read() == 0)
    {
        run_cycle(1, true);
        if(++cnt > 2000)
            PN_ASSERTM(false, "Expected ACK after first TX byte (ack_received_out never asserted)");
    }
    PN_ASSERTM(nack_received_out.read() == 0, "Unexpected NACK after first TX byte");


    tx_data_in.write(0x55);
    run_cycle(1, true);

    start_tx_in.write(true);
    run_cycle(15, true);
    start_tx_in.write(false);

    cnt = 0;
    while(shift_loaded_out.read() == 0)
    {
        run_cycle(1, true);
        if(++cnt > 200)
            PN_ASSERTM(false, "TX did not start (shift_loaded_out not asserted)");
    }

    cnt = 0;
    while(sending_data_out.read() == 0)
    {
        run_cycle(1, true);
        if(++cnt > 200)
            PN_ASSERTM(false, "sending_data_out not asserted during TX phase");
    }

    sda_in.write(true);
    run_cycle(1, true);

    cnt = 0;
    while(nack_received_out.read() == 0)
    {
        run_cycle(1, true);
        if(++cnt > 2000)
            PN_ASSERTM(false, "Expected NACK after first TX byte (nack_received_out never asserted)");
    }
    PN_ASSERTM(ack_received_out.read() == 0, "Unexpected ACK after first TX byte");

    run_cycle(50, true);

    stop_ready_in.write(true);
    run_cycle(50, true);

    PN_ASSERTM(sda_out.read() == true,"SDA not released after STOP (sda_out should be high)");

    PN_INFO("m_i2c_data_control_tb: test passed");
}

/**
 * @fn test_write_read()
 *
 * This test checks an I2C write followed by a read operation.
 * It verifies address transmission, data reception, ACK generation,
 * and the STOP condition.
 */
void test_write_read()
{
    run_cycle(1, false);

    reset.write(true);
    generate_start_cond_in.write(false);
    stop_ready_in.write(false);
    start_tx_in.write(false);
    start_rx_in.write(false);
    send_ack_in.write(false);
    send_nack_in.write(false);
    data_clk_pulse_write_in.write(false);
    data_clk_pulse_read_in.write(false);
    sda_in.write(false);    // initial ACK
    tx_data_in.write(0x71); // 0111 0001

    PN_INFO("m_i2c_data_control_tb: run test_write_read() ...");

    run_cycle(20, false);
    reset.write(false);
    run_cycle(1, false);

    generate_start_cond_in.write(true);
    run_cycle(3, false);
    generate_start_cond_in.write(false);

    PN_ASSERTM(sda_out.read() == false, "SDA not driven low during START condition");

    int cnt = 0;
    while(start_ready_out.read() == 0)
    {
        run_cycle(1, true);
        cnt++;
        if(cnt > 200)
        {
            PN_ASSERTM(false, "START not completed (start_ready_out not asserted)");
        }
    }

    start_tx_in.write(true);
    run_cycle(1, true);
    start_tx_in.write(false);

    cnt = 0;
    while(shift_loaded_out.read() == 0)
    {
        run_cycle(1, true);
        cnt++;
        if(cnt > 200)
        {
            PN_ASSERTM(false, "TX did not start (shift_loaded_out not asserted)");
        }
    }

    cnt = 0;
    while(ack_received_out.read() == 0)
    {
        run_cycle(1, true);
        cnt++;
        if(cnt > 2000)
        {
            PN_ASSERTM(false, "Expected ACK after TX (ack_received_out never asserted)");
        }
    }

    sda_in.write(true);

    run_cycle(2, true);
    start_rx_in.write(true);
    run_cycle(20, true);
    start_rx_in.write(false);

    cnt = 0;
    while(shift_loaded_out.read() == 0)
    {
        run_cycle(1, true);
        cnt++;
        if(cnt > 3000)
        {
            PN_ASSERTM(false, "RX byte not completed (shift_loaded_out never asserted)");
        }
    }

    send_ack_in.write(true);
    run_cycle(20, true);
    send_ack_in.write(false);

    PN_ASSERTM(sda_out.read() == false, "SDA not driven low during ACK phase");

    cnt = 0;
    while(byte_loaded_in_DR_out.read() == 0)
    {
        run_cycle(1, true);
        cnt++;
        if(cnt > 200)
        {
            PN_ASSERTM(false, "RX byte not forwarded to DR (byte_loaded_in_DR_out never asserted)");
        }
    }

    PN_ASSERTM(enable_dr_write_out.read() == 1, "enable_dr_write_out not asserted when byte_loaded_in_DR_out asserted");
    PN_ASSERTF(shift_data_out.read() == 0xFF, ("Unexpected shift_data_out. Expected 0xFF, Actual 0x%02x", (unsigned)shift_data_out.read()));

    stop_ready_in.write(true);
    run_cycle(100, true);

    PN_INFO("m_i2c_data_control_tb: test passed");
}

int sc_main(int argc, char* argv[])
{
    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("m_i2c_data_control");

    m_i2c_data_control i_dut("i_dut");

    // ports
    i_dut.clk(clk);
    i_dut.reset(reset);
    i_dut.generate_start_cond_in(generate_start_cond_in);
    i_dut.stop_ready_in(stop_ready_in);
    i_dut.start_tx_in(start_tx_in);
    i_dut.start_rx_in(start_rx_in);
    i_dut.tx_data_in(tx_data_in);
    i_dut.send_ack_in(send_ack_in);
    i_dut.send_nack_in(send_nack_in);
    i_dut.data_clk_pulse_write_in(data_clk_pulse_write_in);
    i_dut.data_clk_pulse_read_in(data_clk_pulse_read_in);
    i_dut.sda_in(sda_in);
    i_dut.sda_out(sda_out);
    i_dut.ack_received_out(ack_received_out);
    i_dut.nack_received_out(nack_received_out);
    i_dut.shift_loaded_out(shift_loaded_out);
    i_dut.start_ready_out(start_ready_out);
    i_dut.sending_data_out(sending_data_out);
    i_dut.byte_loaded_in_DR_out(byte_loaded_in_DR_out);
    i_dut.shift_data_out(shift_data_out);
    i_dut.enable_dr_write_out(enable_dr_write_out);

    i_dut.pn_trace(tf, pn_cfg_vcd_level);

    sc_start(SC_ZERO_TIME);

    PN_INFO("m_i2c_data_control_tb: simulation started");

    test_write_read();

    test_write_write();

    PN_END_TRACE();
    PN_INFO("m_i2c_data_control_tb: simulation complete");

    return 0;
}
