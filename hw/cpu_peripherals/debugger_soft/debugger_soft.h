/**
 * @file debugger_soft.h
 * @brief This file contains the definition of the c_soft_debugger class.
 *        For simulation ONLY.
 * @author Johannes Hofmann
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Johannes Hofmann <Johannes.Hofmann@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the implementation of the c_soft_debugger class.
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
 * @addtogroup c_soft_debugger
 *
 * This module serves as a wrapper for all c_soft_debugger modules.
 * It instanciates a `c_remote_bitbang`, a `c_dtm` and a `c_soft_dm`.
 * The `c_soft_peripheral` interface is forwarded to the `c_soft_dm`.
 *
 * Note: The module is not synthesizable and is only used for simulation purposes.
 *
 */

#ifndef __C_SOFT_DEBUGGER_H__
#define __C_SOFT_DEBUGGER_H__

#include "c_soft_peripheral.h"

#include "dm_soft.h"
#include "dtm_soft.h"
#include "remote_bitbang.h"

#include <cstdint>
#include <memory>
#include <functional>

class c_soft_debugger : public c_soft_peripheral
{
public:
    /**
     * @brief Constructor.
     *
     * @param callback_signal_debug_haltrequest Callback function called when a
     * debug hatlrequest is perfomed or done. Needed by the debug module.
     * @param openocd_port Por to which OpenOCD has to connect. Needed by the remote
     * bitbang module.
     */
    c_soft_debugger(
        std::function<void(bool)> callback_signal_debug_haltrequest,
        uint16_t openocd_port = PN_CFG_DEBUG_OPENOCD_PORT);

    /**
     * @brief Destructor
     */
    ~c_soft_debugger() = default;

    void set_signal_debug_haltrequest_ack(bool value);

    const char* get_info() override;
    bool is_addressed(uint64_t adr) override;

    uint32_t read32(uint64_t adr) override;
    void write32(uint64_t adr, uint32_t data) override;

    void tick();

private:
    std::unique_ptr<c_soft_dm> dm;
    std::unique_ptr<c_dtm> dtm;
    std::unique_ptr<c_remote_bitbang> remote_bitbang;
};

#endif
