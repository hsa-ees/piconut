/**
 * @file debug_handler.h
 * @brief This file contains the instructions contained in the Debug Modules
 * Debug Handler.
 * @author Johannes Hofmann
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    This file contains the instructions contained in the Debug Modules
    Debug Handler.

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
 *
 * @addtogroup debug_handler
 *
 * The debug handler is a routine executed by the processor when entering debug mode.
 * Its purpose is to process commands received from the host system and resume
 * the main program if requested. Written in RISC-V assembly, it operates in
 * four stages: `_entry`, `_loop`, `_run_cmd`, and `_resume`. Execution begins
 * at `_entry`, then moves to `_loop,` where it waits for a `resume request` or a
 * `run command request`. If either is set, the handler jumps to the corresponding
 * stage. The `_run_cmd` stage concludes by setting the program counter to the
 * first `abstract command` register of the `DM`.
 * According to the External Debug Support standard, the last command from the
 * host, whether an `abstract command` or `progbuf` is an `EBREAK` instruction.
 * As a result, the debug handler is executed again after the last command.
 *
 * Note: Currently there is no exception handling implemented. The debug handler
 * needs to be extended when the piconut supports exception handling.
 *
 */

#ifndef __DEBUG_HANDLER_H__
#define __DEBUG_HANDLER_H__

#include <cstddef>
#include <cstdint>

namespace debug_handler {

static constexpr size_t binary_wsize = 16;
static constexpr size_t binary_size = binary_wsize * 4;

static constexpr uint32_t binary[binary_wsize] = {
    // _entry:
    0x7b241073, // 0x00 csrw x0, CSR_DSCRATCH0, s0      -> Save s0 in dscratch0 csr
    0x00100413, // 0x04 addi s0, zero, 1                ->
    0x02802c23, // 0x08 sw s0, 0x38(zero)               -> Write 1 to hartstatus reg -> hart is halted

    // _loop:
    0x03402403, // 0x0c lw s0, 0x34(zero)               -> Load hartcontrol in s0
    0x00347413, // 0x10 andi s0, s0, 3                  -> Check if resumereq or commandreq is set in hartcontrol..
    0xfe800de3, // 0x14 beq s0, zero, _loop             -> If not jump to _loop

    // _run_commands:
    0x00247413, // 0x18 andi s0, s0, 0x2                -> Check if commandreq is set
    0x00040a63, // 0x1c beq s0, zero, _resume           -> If not jump to _resume
    0x00900413, // 0x20 addi s0, zero, 9                ->
    0x02802c23, // 0x24 sw s0, 0x38(zero)               -> Write 9 to hartstatus reg. Indicates that the hart is running commands now
    0x7b201473, // 0x28 csrw s0, CSR_DSCRATCH0, zero    -> Restore s0
    0x01400067, // 0x2c jalr zero, 0x14(zero)           -> Jump to first abstract command

    // _resume:
    0x00200413, // 0x30 addi s0, zero, 2                ->
    0x02802c23, // 0x34 sw s0, 0x38(zero)               -> Write 2 to hartstatus reg -> hart is running now
    0x7b201473, // 0x38 csrw s0, CSR_DSCRATCH0, zero    -> Restore s0
    0x7b200073, // 0x3c dret
};

} // namespace debug_handler

#endif
