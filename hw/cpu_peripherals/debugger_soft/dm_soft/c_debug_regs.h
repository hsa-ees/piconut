/**
 * @file c_debug_regs.h
 * @brief This file contains the definition of the c_debug_regs class.
 *        For simulation ONLY.
 * @author Johannes Hofmann
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the definition of the c_debug_regs class.
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
 * @addtogroup c_debug_regs
 *
 * The `c_debug_regs` module is a submodule of the `c_soft_dm` module and
 * implements the necessary registers exposed to the host. The registers are
 * implemented according to the `RISC-V External Debug Support` specification.
 * The registers are accessed by the `DMI-bus`.
 *
 * Currently there are following registers implemented:
 *
 * | Name             | Address | Description                                                                                                 |
 * | ---------------- | ------- | ----------------------------------------------------------------------------------------------------------- |
 * | DATA0            | 0x04    | For data exchange between the host and the hart. Accessed by both the hart and the host.                    |
 * | DATA1            | 0x05    | For data exchange between the host and the hart. Accessed by both the hart and the host.                    |
 * | DMCONTROL        | 0x10    | Debug Module Control. Indicates if the hart should be `halted`, `resumed` and how many harts are available. |
 * | DMSTATUS         | 0x11    | Debug Module Status. Indicates if the hart is currently `halted`, `running` and `have reset`.               |
 * | ABSTRACTCS       | 0x16    | Abstract Control Status. Indicates if the hart is busy or an error occured related to `abstract commands`.  |
 * | COMMAND          | 0x17    | The `abstract command` is written in here.                                                                  |
 * | ABSTRACTAUTO     | 0x18    | (Under construction).                                                                                       |
 * | PROGRAM_BUFFER0  | 0x20    | Holds an assembler command coming from the host. Accessed by both the hart and the host.                    |
 * | PROGRAM_BUFFER1  | 0x21    | Holds an assembler command coming from the host. Accessed by both the hart and the host.                    |
 * | PROGRAM_BUFFER2  | 0x22    | Holds an assembler command coming from the host. Accessed by both the hart and the host.                    |
 *
 * The `DATAx` and `PROGRAM_BUFFERx` registers are mirrored to the registers
 * accessed by the hart.
 *
 * More complex registers like `COMMAND` and `ABSTRACTCS` are implemented in their
 * own module.
 *
 * Note: The module is not synthesizable and is only used for simulation purposes.
 *
 */

#ifndef __C_DEBUG_REGS_H__
#define __C_DEBUG_REGS_H__

#include "c_debug_reg_abstractcs.h"
#include "c_debug_reg_command.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

class c_soft_dm;

class c_debug_regs
{
    friend class c_soft_dm;
    friend class c_debug_reg_command;

public:
    enum class e_reg_address : int
    {
        data0 = 0x4,
        data1,

        dmcontrol = 0x10,
        dmstatus = 0x11,

        abstractcs = 0x16,
        command = 0x17,
        abstractauto = 0x18,

        progbuf0 = 0x20,
        progbuf1,
        progbuf2,

        haltsum0 = 0x40,
    };

    // Abstract Data 0 (data0, at 0x04)
    enum e_data_reg_index
    {
        DATA0 = 0,
        DATA1,
        DATA2,
        DATA3,
        DATA4,
        DATA5,
        DATA6,
        DATA7,
        DATA8,
        DATA9,
        DATA10,
        DATA11
    };

    struct data_t
    {
        uint32_t data : 32;
    };

    // Debug Module Control (dmcontrol, at 0x10)
    struct dmcontrol_t
    {
        uint32_t dmactive : 1;
        uint32_t ndmreset : 1;
        uint32_t clrresethaltreq : 1;
        uint32_t setresethaltreq : 1;
        uint32_t reserved0 : 2; // Reserved
        uint32_t hartselhi : 10;
        uint32_t hartsello : 10;
        uint32_t hasel : 1;
        uint32_t reserved1 : 1; // Reserved
        uint32_t ackhavereset : 1;
        uint32_t hartreset : 1;
        uint32_t resumereq : 1;
        uint32_t haltreq : 1;
    };

    // Debug Module Status (dmstatus, at 0x11)
    struct dmstatus_t
    {
        uint32_t version : 4;
        uint32_t confstrptrvalid : 1;
        uint32_t hasresethaltreq : 1;
        uint32_t authbusy : 1;
        uint32_t authenticated : 1;
        uint32_t anyhalted : 1;
        uint32_t allhalted : 1;
        uint32_t anyrunning : 1;
        uint32_t allrunning : 1;
        uint32_t anyunavail : 1;
        uint32_t allunavail : 1;
        uint32_t anynonexistent : 1;
        uint32_t allnonexistent : 1;
        uint32_t anyresumeack : 1;
        uint32_t allresumeack : 1;
        uint32_t anyhavereset : 1;
        uint32_t allhavereset : 1;
        uint32_t reserved0 : 2; // Reserved
        uint32_t impebreak : 1;
        uint32_t reserved1 : 9; // Reserved
    };

    // Abstract Control and Status (abstractcs, at 0x16)
    // Note: Implemented in c_debug_reg_abstractcs.h

    // Abstract Command (command, at 0x17)
    // Note: Implemented in c_debug_reg_commands.h

    // Abstract Command Autoexec (abstractauto, at 0x18)
    struct abstractauto_t
    {
        uint32_t autoexecdata : 12;
        uint32_t reserved0 : 4; // Reserved
        uint32_t autoexecprogbuf : 16;
    };

    // Program Buffer 0 (progbuf0, at 0x20)
    enum class e_progbuf_reg_index : uint32_t
    {
        PROGBUF0 = 0,
        PROGBUF1 = 1,
        PROGBUF2 = 2,
        PROGBUF3 = 3,
        PROGBUF4 = 4,
        PROGBUF5 = 5,
    };

    struct progbuf_t
    {
        uint32_t data : 32;
    };

    // Halt Summary 0 (haltsum0, at 0x40)
    struct haltsum0_t
    {
        uint32_t haltsum0 : 32;
    };

public:
    /**
     * @brief Constructor
     */
    c_debug_regs(
        c_soft_dm* dm,
        const size_t hart_count,
        const size_t data_size,
        const size_t progbuf_size,
        std::function<void(bool)> callback_signal_debug_haltrequest);

    /**
     * @brief Destructor
     */
    ~c_debug_regs() = default;

    /**
     * @brief Write 32-bit integer to register.
     * @param address Address of the register according to debug spec.
     * @param data Data that will be written into the register.
     */
    void write_reg(uint8_t address, uint32_t data);

    /**
     * @brief Read register as 32-bit integer.
     * @param address Address of the register according to debug spec.
     * @return Register data.
     */
    uint32_t read_reg(uint8_t address);

    void write_reg_data(e_data_reg_index index, data_t reg);
    void write_reg_dmcontrol(dmcontrol_t reg);
    void write_reg_dmstatus(dmstatus_t reg);
    void write_reg_abstractcs(c_debug_reg_abstractcs::reg_t reg);
    void write_reg_command(c_debug_reg_command::reg_t reg);
    void write_reg_abstractauto(abstractauto_t reg);
    void write_reg_progbuf(e_progbuf_reg_index index, progbuf_t reg);
    void write_reg_haltsum0(haltsum0_t reg);

    data_t read_reg_data(e_data_reg_index index);
    dmcontrol_t read_reg_dmcontrol();
    dmstatus_t read_reg_dmstatus();
    c_debug_reg_abstractcs::reg_t read_reg_abstractcs();
    c_debug_reg_command::reg_t read_reg_command();
    abstractauto_t read_reg_abstractauto();
    progbuf_t read_reg_progbuf(e_progbuf_reg_index index);
    haltsum0_t read_reg_haltsum0();

private:
    void _check_hartsellen_detection();
    void _check_hart_exists();
    void _check_hart_halt_request();
    void _check_hart_resume_request();

    void _handle_reg_access_while_command_running();

private:
    c_soft_dm* dm;
    const uint8_t hartsellen;
    const size_t data_size;
    const size_t progbuf_size;

    std::function<void(bool)> callback_signal_debug_haltrequest;

    dmstatus_t dmstatus;
    dmcontrol_t dmcontrol;
    std::vector<data_t> data;
    c_debug_reg_abstractcs abstractcs;
    c_debug_reg_command command;
    abstractauto_t abstractauto;
    std::vector<progbuf_t> progbuf;
    haltsum0_t haltsum0;
};

#endif