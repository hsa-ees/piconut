/**
 * @file c_fake_client.h
 * @brief This file contains the definition of the c_fake_client class.
 *        For simulation ONLY.
 * @author Johannes Hofmann
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Johannes Hofmann <johannes.hofmann@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the implementation of the c_fake_client class.
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
 * This module is a simple socket client that connects to a socket on localhost
 * to send and receive data.
 *
 * Note: The module is not synthesizable and is only used for simulation purposes.
 *
 */

#ifndef __C_FAKE_CLIENT_H__
#define __C_FAKE_CLIENT_H__

#include <cstdint>
#include <string>
#include <optional>

class c_fake_client
{
public:
    /**
     * @brief Constructor.
     */
    c_fake_client();

    /**
     * @brief Destructor.
     */
    ~c_fake_client();

    /**
     * @brief Connect socket to given port on localhost.
     * @param port Port to connect.
     * @return True if operation was successful else false.
     */
    bool connect(uint16_t port);

    /**
     * @brief Returns true if another socket has connected.
     * @return True if connected to another socket else false.
     */
    bool connected();

    /**
     * @brief Send a message to the connected socket.
     * @param message Message to be send.
     */
    bool send(std::string message);

    /**
     * @brief Recieve message from connected socket.
     * @return Maybe recieved message.
     */
    std::optional<std::string> recieve();

    /**
     * @brief Close connection.
     */
    void close();

private:
    uint16_t port;
    int socket_fd;
};

#endif