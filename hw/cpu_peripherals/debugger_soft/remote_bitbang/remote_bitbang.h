/**
 * @brief This file contains the definition of the c_remote_bitbang class.
 *        For simulation ONLY.
 * @author Johannes Hofmann
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Johannes Hofmann <johannes.hofmann@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the implementation of the c_remote_bitbang class.
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
 * @addtogroup c_remote_bitbang
 *
 * This module implements remote bitbang with a socket server that allows
 * a client to connect and send or recieve data following the remote bitbang
 * specification:
 *
 * | Command | Event                          |
 * |---------|--------------------------------|
 * | B       | LED on (Not supported)         |
 * | b       | LED off (Not supported)        |
 * | r       | JTAG reset                     |
 * | 0       | tck = 0, tms = 0, tdi = 0      |
 * | 1       | tck = 0, tms = 0, tdi = 1      |
 * | 2       | tck = 0, tms = 1, tdi = 0      |
 * | 3       | tck = 0, tms = 1, tdi = 1      |
 * | 4       | tck = 1, tms = 0, tdi = 0      |
 * | 5       | tck = 1, tms = 0, tdi = 1      |
 * | 6       | tck = 1, tms = 1, tdi = 0      |
 * | 7       | tck = 1, tms = 1, tdi = 1      |
 * | R       | Read tdo                       |
 * | Q       | Quit (Close socket connection) |
 *
 * Commands that are not listed are ignored.
 *
 * The JTAG interface is connected through callback functions that respond to
 * specific client commands:
 *     - `void callback_jtag_set_inputs(bool tck, bool tms, bool tdi)`: Invoked when
 *        a new JTAG input pin command is received.
 *     - `void callback_jtag_get_output()`: Invoked when the current state when
 *        `tdo` is requested.
 *     - `void callback_jtag_trigger_reset()`: Invoked when a JTAG reset command
 *        is received.
 *
 * Note: The module is not synthesizable and is only used for simulation purposes.
 *
 */

#ifndef __REMOTE_BITBANG_H__
#define __REMOTE_BITBANG_H__

#include "remote_bitbang_defs.h"

#include <cstdint>
#include <functional>

class c_remote_bitbang
{
public:
    /**
     * @brief Constructor.
     * @param callback_jtag_set_inputs Callback function called when new JTAG
     *        write input command recieved.
     * @param callback_jtag_get_output Callback function called when new JTAG
     *        read output command recieved.
     * @param callback_jtag_trigger_reset Callback function called when JTAG trigger
     *        reset command recieved.
     * @param port Port to which will be listen (openocd have to connect to it).
     */
    c_remote_bitbang(
        std::function<void(bool, bool, bool)> callback_jtag_set_inputs,
        std::function<bool(void)> callback_jtag_get_output,
        std::function<void(void)> callback_jtag_trigger_reset,
        uint16_t port = PN_CFG_DEBUG_OPENOCD_PORT);

    /**
     * @brief Destructor.
     */
    ~c_remote_bitbang();

    /**
     * @brief Performs all update functions.
     *        Note: Should be called inside a while loop or similar.
     */
    void process();

    /**
     * @brief Checks if another socket is connected.
     * @return True if another socket has connected else false.
     */
    bool connected();

private:
    void _socket_accept();
    void _socket_close();
    void _socket_handle_read();
    void _decode_rx(char* rx_message);

private:
    std::function<void(bool, bool, bool)> callback_jtag_set_inputs;
    std::function<bool(void)> callback_jtag_get_output;
    std::function<void(void)> callback_jtag_trigger_reset;

    // socket
    uint16_t port;
    int socket_fd;
    int client_fd;
};

#endif
