/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Daniel Sommerfeldt <daniel.sommerfeldt1@tha.de>
                2026 Johannes Hofmann <johannes.hofmann1@tha.de>
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
 * @file gpio.h
 *
 * @defgroup gpio GPIO Module
 * @{
 */

#ifndef __GPIO_H__
#define __GPIO_H__

#include <systemc.h>
#include <piconut.h>
#include "gpio_defs.h"

/**
 * @class m_gpio
 * @brief GPIO peripheral module.
 *
 * @details
 * This module implements a configurable General Purpose Input/Output (GPIO) controller
 * with a Wishbone slave interface for register access.
 *
 * **Ports**:
 * @param[in] clk       Clock of the module
 * @param[in] reset     Reset of the module
 * @param[in, out] wb_slave  Wishbone slave interface
 * @param[in] input     GPIO input (Bits after num_inputs are ignored)
 * @param[in] output    GPIO output port (Bits after num_outputs are ignored)
 */
SC_MODULE(m_gpio), pn_module_if
{
public:
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    pn_wishbone_slave_t wb_slave;

    sc_in<sc_uint<GPIO_MAX_PINS>> PN_NAME(input);
    sc_out<sc_uint<GPIO_MAX_PINS>> PN_NAME(output);

    /**
     * @brief Constructor.
     *
     * Initializes the GPIO module with the specified base address and pin configuration.
     *
     * @fn m_gpio(sc_module_name, pn_wb_adr_t, const uint8_t, const uint8_t)
     *
     * @param[in] name           Module name for identification in waveform dumps
     * @param[in] base_address   Base address for Wishbone register space
     * @param[in] num_inputs     Number of input pins to enable (default: 0)
     * @param[in] num_outputs    Number of output pins to enable (default: 0)
     *
     * @pre num_inputs <= GPIO_MAX_PINS
     * @pre num_outputs <= GPIO_MAX_PINS
     */
    SC_HAS_PROCESS(m_gpio);
    m_gpio(
        sc_module_name name,
        pn_wb_adr_t base_address,
        const uint8_t num_inputs = 0,
        const uint8_t num_outputs = 0)
        : sc_module(name)
        , wb_slave{
              .alen = GPIO_WB_ADR_WIDTH,
              .dlen = GPIO_MAX_PINS,
              .base_address = base_address,
              .size = GPIO_SIZE}
        , num_inputs{num_inputs}
        , num_outputs{num_outputs}
    {
        pn_add_wishbone_slave(&wb_slave);

        PN_ASSERT(num_inputs <= GPIO_MAX_PINS);
        PN_ASSERT(num_outputs <= GPIO_MAX_PINS);

        SC_METHOD(proc_cmb_wb_slave);
        sensitive << wb_slave.stb_i
                  << wb_slave.cyc_i
                  << wb_slave.we_i
                  << wb_slave.adr_i
                  << wb_slave.sel_i
                  << r_input_val
                  << r_input_en
                  << r_output_en
                  << r_output_val
                  << r_wb_current_state;

        SC_CTHREAD(proc_clk_state, clk.pos());
        SC_CTHREAD(proc_clk_wb_slave, clk.pos());

        SC_METHOD(proc_cmb_gpio_output);
        sensitive << r_output_val
                  << r_output_en;

        SC_CTHREAD(proc_clk_gpio_input, clk.pos());
    };

    /**
     * @brief Generate waveform trace.
     *
     * Records all internal signals and ports to a trace file.
     *
     * @param[in,out] tf  Pointer to open sc_trace_file.
     * @param[in] level   Trace level (default: 1).
     */
    void pn_trace(sc_trace_file * tf, int level = 1);

    /**
     * @name Processes
     * @{
     */

    /**
     * @brief Wishbone state register.
     */
    void proc_clk_state();

    /**
     * @brief Wishbone slave logic.
     */
    void proc_cmb_wb_slave();

    /**
     * @brief Wishbone interface register.
     *
     * Registers interfaced by wishbone:
     *  - GPIO_REG_INPUT_EN
     *  - GPIO_REG_OUTPUT_EN
     *  - GPIO_REG_OUTPUT_VAL
     */
    void proc_clk_wb_slave();

    /**
     * @brief GPIO output driver.
     *
     * Write output to GPIO_REG_OUTPUT_VAL register for all bits set in
     * GPIO_REG_OUTPUT_EN register.
     */
    void proc_cmb_gpio_output();

    /**
     * @brief GPIO_REG_INPUT_VAL register.
     *
     * Reads input and writes to GPIO_REG_INPUT_VAL register for all bits set in
     * GPIO_REG_INPUT_EN register.
     */
    void proc_clk_gpio_input();

    /** @} */ // end of Processes

protected:
    enum e_wb_state
    {
        WB_IDLE = 0,
        WB_WRITE,
        WB_READ,
    };

    const uint8_t num_inputs;
    const uint8_t num_outputs;

    sc_signal<sc_uint<GPIO_MAX_PINS>> PN_NAME(r_input_val);
    sc_signal<sc_uint<GPIO_MAX_PINS>> PN_NAME(r_input_en);
    sc_signal<sc_uint<GPIO_MAX_PINS>> PN_NAME(r_output_en);
    sc_signal<sc_uint<GPIO_MAX_PINS>> PN_NAME(r_output_val);

    sc_signal<bool> PN_NAME(c_wb_write_en);
    sc_signal<sc_uint<2>> PN_NAME(r_wb_current_state);
    sc_signal<sc_uint<2>> PN_NAME(wb_next_state);
};

#endif // __GPIO_H__

/** @} */ // end of gpio
