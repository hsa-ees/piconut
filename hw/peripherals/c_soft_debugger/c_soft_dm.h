/**
 * @file c_soft_dm.h
 * @brief This file contains the definition of the c_soft_dm class.
 *        For simulation ONLY.
 * @author Johannes Hofmann
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Johannes Hofmann <johannes.hofmann@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the definition of the c_soft_dm class.
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
 * @addtogroup c_soft_dm
 *
 * The module translates the instructions it receives from the `DMI-bus` into
 * commands. It must be able to handle the following requests:
 *  - Provide the debugger with information about the hardware implementation.
 *  - Halt and start hart.
 *  - Indicate if the hart is currently halted.
 *  - Read and write "General Purpose Registers" (GPR).
 *  - Enable debugging from the very first assembler instruction.
 *
 * The `DM` has registers accessed by the `DTM` called `Debug registers` and
 * normal or system registers. The `c_soft_dm` module derives from `c_soft_peripheral` so
 * the system registers can be accessed by the processor.
 * The registers exposed to the system are:
 *
 * | Name                | Address | Description                                                                                                        |
 * | ------------------- | ------- | ------------------------------------------------------------------------------------------------------------------ |
 * | DATA0               | 0x00    | For data exchange between the host and the processor. Accessed by both the hart and the host.                      |
 * | DATA1               | 0x04    | For data exchange between the host and the processor. Accessed by both the hart and the host.                      |
 * | PROGRAM_BUFFER0     | 0x08    | Holds an assembler command coming from the host. Accessed by both the hart and the host.                           |
 * | PROGRAM_BUFFER1     | 0x0c    | Holds an assembler command coming from the host. Accessed by both the hart and the host.                           |
 * | PROGRAM_BUFFER2     | 0x10    | Holds an assembler command coming from the host. Accessed by both the hart and the host.                           |
 * | ABSTRACT_COMMAND0   | 0x14    | Holds a command coming from the host and is translated into an assembler command.                                  |
 * | ABSTRACT_COMMAND1   | 0x18    | Holds a command coming from the host and is translated into an assembler command.                                  |
 * | ABSTRACT_COMMAND2   | 0x1c    | Holds a command coming from the host and is translated into an assembler command.                                  |
 * | ABSTRACT_COMMAND3   | 0x20    | Holds a command coming from the host and is translated into an assembler command.                                  |
 * | ABSTRACT_COMMAND4   | 0x24    | Holds a command coming from the host and is translated into an assembler command.                                  |
 * | ABSTRACT_COMMAND5   | 0x28    | Holds a command coming from the host and is translated into an assembler command.                                  |
 * | ABSTRACT_COMMAND6   | 0x2c    | Holds a command coming from the host and is translated into an assembler command.                                  |
 * | ABSTRACT_COMMAND7   | 0x30    | Holds a command coming from the host and is translated into an assembler command.                                  |
 * | HARTCONTROL         | 0x34    | Control register that tells the processor if commands should be executed or if the debug handler should be exited. |
 * | HARTSTATUS          | 0x38    | Status register where the processor tells the `DM` if he is `running`, `halted`, executing `commands`.             |
 * | DEBUG_HANDLER_START | 0x3c    | Start of the debug handler program.                                                                                |
 *
 * The registers accessed by the `DTM` are implemented and described  in the
 * `c_debug_regs` module which is part of this module.
 *
 * Halting a hart can be done in two ways.
 *  - The hart reads an `EBREAK` instruction
 *  - `haltrequst` from the host.
 * If a hart is halted it starts executing the `debug handler` program.
 * The `haltrequst` from the host is an actual signal going into the hart to halt it.
 *
 * If the host has a request to run some custom commands it will either write the
 * assembler comamnds directly into the `PRORGAM_BUFFERx` registers or send and
 * `abstract commands`. The `abstract command` holds the instruction in a more
 * abstract kind. It has the be processed to one or multiple assembler commands.
 * They are then saved in the `ABSTRACT_COMMANDx` registers. After the commands
 * are updated in the registers there is the `run_commandsreq` bit set in the
 * `HARTCONTROL` register which tells the `dabug handler` to executed the new commands.
 *
 * To resume the execution of the main program there is the `resumereq` bit in the
 * `HARTCONTROL` register that tells the `debug handler` to resume the main
 * program.
 *
 * The `HARTSTATUS` register is used as a feedback so the host can be sure that
 * the hart is in the requested state.
 *
 * The base address of the `c_soft_dm` module is fixed set to `zero` (0x00000000)
 * following to the `External Debug Support` standard.
 *
 * The module has one output signals, `signal_debug_haltrequest`. There is one
 * callback function as parameters to the constructor which are invoked, if the
 * state of the signal is changed. Additionaly, the module has one input signal,
 * `signal_debug_haltrequest_ack`. The state of the signal is set by invokeing
 * `set_signal_debug_haltrequest_ack`.
 *
 * Note: The module is not synthesizable and is only used for simulation purposes.
 *
 */

#ifndef __C_SOFT_DM_H__
#define __C_SOFT_DM_H__

#include "c_debug_regs.h"
#include "c_soft_peripheral.h"

#include <piconut-config.h>

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

class c_soft_dm : public c_soft_peripheral
{
    friend class c_debug_regs;
    friend class c_debug_reg_command;

public:
    enum class e_regs : uint32_t
    {
        DATA0 = 0x00,
        DATA1 = 0x04,

        PROGRAM_BUFFER0 = 0x08,
        PROGRAM_BUFFER1 = 0x0C,
        PROGRAM_BUFFER2 = 0x10,

        ABSTRACT_COMMAND0 = 0x14,
        ABSTRACT_COMMAND1 = 0x18,
        ABSTRACT_COMMAND2 = 0x1C,
        ABSTRACT_COMMAND3 = 0x20,
        ABSTRACT_COMMAND4 = 0x24,
        ABSTRACT_COMMAND5 = 0x28,
        ABSTRACT_COMMAND6 = 0x2C,
        ABSTRACT_COMMAND7 = 0x30,

        HARTCONTROL = 0x34,
        HARTSTATUS = 0x38,

        // Debug rom reaches from CFG_DEBUG_DEBUG_HANDLER_START_ADDRESS to
        // CFG_DEBUG_DEBUG_HANDLER_START_ADDRESS + debug_handler::binary_size.
        DEBUG_HANDLER_START = CFG_DEBUG_DEBUG_HANDLER_START_ADDRESS,
    };

    enum class e_flag_reg_aamsize : uint32_t
    {
        INCREMENT_BYTES_1 = 0,
        INCREMENT_BYTES_2 = 1,
        INCREMENT_BYTES_4 = 2,
    };

    struct hartcontrol_t
    {
        // haltreq via debug request signal
        uint32_t resumereq : 1;
        // resetreq via debug reset signal
        uint32_t run_commandsreq : 1;
        uint32_t memincer : 1;
        e_flag_reg_aamsize aamsize : 2;
        uint32_t reserved0 : 27;
    };

    struct hartstatus_t
    {
        uint32_t halted : 1;
        uint32_t running : 1;
        uint32_t havereset : 1;
        uint32_t commands_running : 1;
        uint32_t reserved0 : 28;
    };

public:
    /**
     * @brief Constructor.
     *
     * @param callback_signal_debug_haltrequest Callback function called when a
     * debug hatlrequest is perfomed or done.
     */
    c_soft_dm(
        std::function<void(bool)> callback_signal_debug_haltrequest);

    /**
     * @brief Destructor.
     */
    ~c_soft_dm() = default;

    /**
     * @brief Short info text about this module.
     *
     * @return Info text about this module.
     */
    const char* get_info() override;

    /**
     * @brief Returns true if adr is in  range of the module.
     *
     * @param adr Address to check.
     * @return True if adr is in range of this module else false.
     */
    bool is_addressed(uint64_t adr) override;

    /**
     * @brief Read register word.
     *
     * @param adr Address of register.
     * @return Register data.
     */
    uint32_t read32(uint64_t adr) override;

    /**
     * @brief Write register word.
     *
     * @param adr Address of register.
     * @param data Data to be written.
     */
    void write32(uint64_t adr, uint32_t data) override;

    /**
     * @brief Set the value of the hardware signal debug_haltrequest_ack.
     *
     * @param value Binary value of the signal.
     */
    void set_signal_debug_haltrequest_ack(bool value);

    /**
     * @brief Write operation from dmi bus.
     *
     * @param address Address of the module/register.
     * @param data Data to write.
     */
    void dmi_write(uint8_t address, uint32_t data);

    /**
     * @brief Read operation from dmi bus.
     *
     * @param address Address of the module/register.
     * @return Data to read.
     */
    uint32_t dmi_read(uint8_t address);

    /**
     * @brief Reset from dmi bus.
     */
    void dmi_reset();

private:
    uint32_t _absolute_reg_address(e_regs reg) const;
    void _set_abstract_commands(std::vector<uint32_t> commands);

    bool _is_debug_handler_addressed(uint32_t internal_address) const;
    uint32_t _debug_handler_instruction(uint32_t internal_address) const;

    hartcontrol_t _read_reg_hartcontrol();
    void _write_reg_hartstatus(hartstatus_t hartstatus);

private:
    const uint32_t base_address;
    const size_t size;
    const char name[32]{"Debug Module"};

    c_debug_regs debug_regs;

    const size_t abstract_command_regs_size;
    std::vector<uint32_t> abstract_command_regs;

    hartcontrol_t hartcontrol_reg;
    hartstatus_t hartstatus_reg;
};

#endif