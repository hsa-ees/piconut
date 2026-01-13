/**
 * @brief This file contains the definition of the c_dtm class.
 *        For simulation ONLY.
 * @author Johannes Hofmann
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Johannes Hofmann <johannes.hofmann@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the definition of the c_dtm class.
      For simulation ONLY.

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
 * @addtogroup c_dtm
 *
 * The `Debug Transport Module (DTM)` provide access to the `Debug Module (DM)`
 * over JTAG. In the "External Debug Support" standard, the specification for
 * a `DTM` with a JTAG interface is defined. This is based on the definition of
 * a `TAP` in the JTAG standard, which enables access to custom-defined JTAG registers.
 * There are four registers implemented to accomplish this.
 *
 * | Name   | Address |  Description                                                                                                   |
 * | ------ | ------- | -------------------------------------------------------------------------------------------------------------- |
 * | BYPASS | 0x00    | This register has 1-bit length and has no effect on. Used for bypass data. (JTAG standard)                     |
 * | IDCODE | 0x01    | This register is read-only. It holds the value `0xdeadbeef`. Used for identification purposes. (JTAG standard) |
 * | DTMCS  | 0x10    | This register is used to control the `DMI-bus` and to reset the `DMI-bus`.                                     |
 * | DMI    | 0x11    | This register is used to access the `DM` via the `DMI-bus`.                                                    |
 *
 * The `DMI-bus` is used to read or write to registers inside the `DM`.
 *
 * To set the state of the JTAG input pin (tck, tms, tdi) call the functon `jtag_set_input_pins(jtag_input_pins_t)`.
 * The get the state of the JTAG output pin tdo call the function `jtag_get_output_pin()`.
 * To set the JTAG reset pin call the function `jtag_reset()`.
 * The type `jtag_input_pins_t` is a struct holding the tck, tms, tdi signals as booleans.
 *
 * The `DTM` is the master in the `DMI-bus` and drives all the operations happening.
 * The callback functions for write, read or reset the `DMI-bus` must be provided when
 * a `c_dtm` instance is created to the constructor.
 *
 * Note: The module is not synthesizable and is only used for simulation purposes.
 *
 */

#ifndef __C_DTM_H__
#define __C_DTM_H__

#include <piconut-config.h>

#include <cstddef>
#include <cstdint>
#include <functional>

struct jtag_input_pins_t
{
    bool tck;
    bool tms;
    bool tdi;
};

class c_dtm
{
public:
    enum e_data_reg_address
    {
        BYPASS = 0x0,
        IDCODE = 0x1,
        DTMCS = 0x10,
        DMI = 0x11,
    };

public:
    /**
     * @brief Constructor.
     *
     * @param callback_dmi_write Callback function called when a write
     * is performed on dmi bus.
     * @param callback_dmi_read Callback function called when a read
     * is performed on dmi bus.
     * @param callback_dmi_reset Callback function called when a reset
     * is performed on dmi bus.
     */
    c_dtm(
        std::function<void(uint8_t, uint32_t)> callback_dmi_write,
        std::function<uint32_t(uint8_t)> callback_dmi_read,
        std::function<void(void)> callback_dmi_reset);

    /**
     * @brief Destructor.
     */
    ~c_dtm();

    /**
     * @brief Set JTAG input pins.
     *
     * @param jtag_input_pins JTAG input pins states.
     */
    void jtag_set_input_pins(jtag_input_pins_t jtag_input_pins);

    /**
     * @brief Get output JTAG tdo.
     *
     * @return State of JTAG tdo pin.
     */
    bool jtag_get_output_pin() const;

    /**
     * @brief Jtag reset.
     */
    void jtag_reset();

    /**
     * @brief Get the width of the instruction register.
     *
     * @return Width of the instruction register.
     */
    size_t get_instruction_reg_width() const;

    /**
     * @brief Get the width of the given data register.
     *
     * @param data_reg Given data register.
     * @return Width of the given data register.
     */
    size_t get_data_reg_width(e_data_reg_address data_reg) const;

private:
    void _update_tap();
    void _tap_test_logic_reset();
    void _tap_capture_dr();
    void _tap_shift_dr();
    void _tap_update_dr();
    void _tap_shift_ir();

private:
    enum class e_tap_state : int
    {
        TEST_LOGIC_RESET,
        RUN_TEST_IDLE,
        SELECT_DR_SCAN,
        CAPTURE_DR,
        SHIFT_DR,
        EXIT1_DR,
        PAUSE_DR,
        EXIT2_DR,
        UPDATE_DR,
        SELECT_IR_SCAN,
        CAPTURE_IR,
        SHIFT_IR,
        EXIT1_IR,
        PAUSE_IR,
        EXIT2_IR,
        UPDATE_IR
    };

private:
    const size_t instruction_reg_width = 5;
    const size_t data_reg_width_bypass = 1;
    const size_t data_reg_width_idcode = 32;
    const size_t data_reg_width_dtmcs = 32;
    const size_t data_reg_width_dmi = PN_CFG_DEBUG_DMI_BUS_ADR_WIDTH + 34;

    // Jtag
    bool jtag_tck;
    bool jtag_tms;
    bool jtag_tdi;
    bool jtag_tdo;

    // Dmi callbacks
    std::function<void(uint8_t, uint32_t)> callback_dmi_write;
    std::function<uint32_t(uint8_t)> callback_dmi_read;
    std::function<void(void)> callback_dmi_reset;

    // Tap state machine
    e_tap_state tap_state;

    uint32_t instruction_reg;

    uint64_t selected_data_reg;
    uint64_t selected_data_reg_width;

    uint32_t dmi_last_read;
};

#endif
