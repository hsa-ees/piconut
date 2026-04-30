/**
 * @file i2c_clk_control.h
 * @brief
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Eikya Lagisetti <eikya.lagisetti1@tha.de>
                     2025 Johannes Fleiner <johannes.fleiner1@tha.de>
                     2025 Sebastian Ebenhöh <sebastian.moritz.ebenhöh@tha.de>
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
 * @fn SC_MODULE(m_i2c_clk_control)
 *
 * This SystemC module handles the low-level timing and generation of the
 * I2C clock signal (SCL). It calculates high/low periods based on CCR/TRISE
 * register settings, manages clock stretching (synchronization), and controls
 * the precise timing sequences for START and STOP conditions.
 *
 *
 * @par
 * @param[in] clk                           Clock input.
 * @param[in] reset                         Asynchronous reset (active high).
 *
 * @param[in] ccr_in                        CCR register value.
 * @param[in] trise_in                      TRISE register value.
 *
 * @param[in] scl_on_in                     Enables SCL generation.
 * @param[in] master_stretch_in             Request to force a clock stretch.
 * @param[in] init_master_in                Initialization signal for master mode.
 * @param[out] scl_ready_out                Indicates SCL Master is ready for START condition.
 *
 * @param[in]  start_ready_in               Trigger to begin START condition.
 * @param[out] start_finished_out           Indicates START sequence is completed.
 * @param[in]  generate_stop_in             Trigger to begin STOP condition.
 * @param[out] stop_ready_out               Indicates first part of STOP generation is ready.
 * @param[out] stop_finished_out            Indicates STOP sequence is fully completed.
 *
 * @param[in]  scl_in                       SCL line input.
 * @param[out] scl_out                      SCL line output.
 *
 * @param[out] data_clk_pulse_read_out      Sample trigger pulse for RX data.
 * @param[out] data_clk_pulse_write_out     Shift trigger pulse for TX data.
 */

#ifndef I2C_CLKCONTROL_H
#define I2C_CLKCONTROL_H

#include <systemc.h>
#include <piconut.h>

SC_MODULE(m_i2c_clk_control)
{

public:
    // Global Control
    sc_in<bool> PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    // Register Interface
    sc_in<sc_uint<16>> PN_NAME(ccr_in);
    sc_in<sc_uint<6>> PN_NAME(trise_in);

    // Control Signals
    sc_in<bool> PN_NAME(scl_on_in);
    sc_in<bool> PN_NAME(master_stretch_in);
    sc_in<bool> PN_NAME(init_master_in);
    sc_out<bool> PN_NAME(scl_ready_out);

    // START/STOP Sequencing
    sc_in<bool> PN_NAME(start_ready_in);
    sc_out<bool> PN_NAME(start_finished_out);
    sc_in<bool> PN_NAME(generate_stop_in);
    sc_out<bool> PN_NAME(stop_ready_out);
    sc_out<bool> PN_NAME(stop_finished_out);

    // Physical Interface (SCL)
    sc_in<bool> PN_NAME(scl_in);
    sc_out<bool> PN_NAME(scl_out);

    // Status & Synchronization
    sc_out<bool> PN_NAME(data_clk_pulse_read_out);
    sc_out<bool> PN_NAME(data_clk_pulse_write_out);


    /**
     * @brief Constructor for the I2C clk_control.
     *
     * Registers the clocked FSM process and the combinational process.
     */
    SC_CTOR(m_i2c_clk_control)
    {
        SC_CTHREAD(proc_clk_clk_control, clk.pos());
        reset_signal_is(reset, true);

        SC_METHOD(proc_cmb_clk_control);
        sensitive << state
                  << counter
                  << low_period
                  << high_period
                  << trise_delay
                  << ccr_in
                  << trise_in
                  << scl_on_in
                  << init_master_in
                  << master_stretch_in
                  << scl_in
                  << start_hold_period
                  << stop_hold_period
                  << start_ready_in
                  << generate_stop_in;
    }

    void pn_trace(sc_trace_file * tf, int level = 1);

private:
    /**
     * @brief Enumeration of SCL generation FSM states.
     *
     * These states manage the low-level physical layer timing for the clock line, including:
     * - Idle state and parameter initialization.
     * - Cyclic generation of SCL Low and High periods based on CCR settings.
     * - Synchronization logic (Clock Stretching detection and TRISE waiting).
     * - Detailed timing sequences for START and STOP conditions.
     */
    typedef enum
    {
        STATE_IDLE,
        STATE_INIT_MASTER,
        STATE_SCL_START_HOLD,
        STATE_START_FINISH,
        STATE_SCL_START_LOW,
        STATE_SCL_LOW,
        STATE_SCL_START_HIGH,
        STATE_SCL_WAIT_TRISE,
        STATE_SCL_CHECK_STRETCH,
        STATE_SCL_HIGH,
        STATE_STOP_CON,
        STATE_STOP_READY,
        STATE_SAFETY_WAIT,
        STATE_CHECK_MASTER_STRETCH,
        STATE_STOP_FINISHED_1,
        STATE_STOP_FINISHED_2
    } m_i2c_clk_control_states;

    // Width of the state signal in bits.
    static constexpr unsigned int state_width = 4;

    sc_signal<sc_uint<state_width>> PN_NAME(state);
    sc_signal<sc_uint<16>> PN_NAME(counter);
    sc_signal<sc_uint<16>> PN_NAME(low_period);
    sc_signal<sc_uint<16>> PN_NAME(high_period);
    sc_signal<sc_uint<16>> PN_NAME(trise_delay);

    sc_signal<sc_uint<state_width>> PN_NAME(next_state);
    sc_signal<sc_uint<16>> PN_NAME(next_counter);
    sc_signal<sc_uint<16>> PN_NAME(next_low_period);
    sc_signal<sc_uint<16>> PN_NAME(next_high_period);
    sc_signal<sc_uint<16>> PN_NAME(next_trise_delay);

    sc_signal<sc_uint<16>> PN_NAME(start_hold_period);
    sc_signal<sc_uint<16>> PN_NAME(stop_hold_period);
    sc_signal<sc_uint<16>> PN_NAME(next_start_hold_period);
    sc_signal<sc_uint<16>> PN_NAME(next_stop_hold_period);

    /* Process functions */
    void proc_clk_clk_control();
    void proc_cmb_clk_control();
};

#endif // I2C_CLKCONTROL_H