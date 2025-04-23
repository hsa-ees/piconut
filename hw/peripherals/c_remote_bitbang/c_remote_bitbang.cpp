/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Johannes Hofmann <johannes.hofmann1@tha.de>
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

#include "c_remote_bitbang.h"

#include <base.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

c_remote_bitbang::c_remote_bitbang(
    std::function<void(bool, bool, bool)> callback_jtag_set_inputs,
    std::function<bool(void)> callback_jtag_get_output,
    std::function<void(void)> callback_jtag_trigger_reset,
    uint16_t port)
    : callback_jtag_set_inputs{callback_jtag_set_inputs}
    , callback_jtag_get_output{callback_jtag_get_output}
    , callback_jtag_trigger_reset{callback_jtag_trigger_reset}
    , port{port}
    , socket_fd{0}
    , client_fd{0}
{
    socket_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if(socket_fd == -1)
    {
        PN_ASSERTF(false,
            ("c_remote_bitbang: Failed to make socket: %s (%d)",
                strerror(errno),
                errno));
    }

    fcntl(socket_fd, F_SETFL, O_NONBLOCK);

    struct sockaddr_in address;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if(::bind(socket_fd, (struct sockaddr*)&address, sizeof(address)) == -1)
    {
        PN_ASSERTF(false,
            ("c_remote_bitbang: Failed to bind socket: %s (%d)",
                strerror(errno),
                errno));
    }

    if(::listen(socket_fd, 1) == -1)
    {
        PN_ASSERTF(false,
            ("c_remote_bitbang: Failed to listen on socket: %s (%d)",
                strerror(errno),
                errno));
    }

    socklen_t addrlen = sizeof(address);
    if(::getsockname(socket_fd, (struct sockaddr*)&address, &addrlen) == -1)
    {
        PN_ASSERTF(false,
            ("c_remote_bitbang: Failed getsockname: %s (%d)",
                strerror(errno),
                errno));
    }

    PN_INFOF(("Listening for remote bitbang connection on port %d.",
        ntohs(address.sin_port)));
    fflush(stdout);
}

c_remote_bitbang::~c_remote_bitbang()
{
    ::close(socket_fd);

    if(connected())
    {
        ::close(client_fd);
    }
}

void c_remote_bitbang::_socket_accept()
{
    client_fd = ::accept(socket_fd, NULL, NULL);
    if(client_fd == -1)
    {
        // Ignore error caused by non-blocking mode
        if(errno != EAGAIN)
        {
            PN_ASSERTF(false,
                ("c_remote_bitbang: Failed to accept on socket: %s (%d)",
                    strerror(errno),
                    errno));
        }

        return;
    }

    PN_INFOF(("c_remote_bitbang: client connected"));
    fcntl(client_fd, F_SETFL, O_NONBLOCK);
}

void c_remote_bitbang::process()
{
    if(!connected())
    {
        _socket_accept();
        return;
    }

    _socket_handle_read();
}

bool c_remote_bitbang::connected()
{
    return client_fd > 0;
}

void c_remote_bitbang::_socket_handle_read()
{
    if(!connected())
    {
        PN_ASSERT(false);
        return;
    }

    char rx_buffer[1024] = {0};

    const int result = ::read(client_fd, rx_buffer, sizeof(rx_buffer));
    if(result == -1)
    {
        // Ignore error caused by non-blocking mode
        if(errno != EAGAIN)
        {
            PN_WARNINGF(("c_remote_bitbang: Failed recieved: %s (%d)",
                strerror(errno),
                errno));
        }
        return;
    }

    if(result == 0)
    {
        _socket_close();
        return;
    }

    _decode_rx(rx_buffer);
}

void c_remote_bitbang::_decode_rx(char* rx_message)
{
    uint8_t tx_buffer[1024] = {0};
    uint64_t tx_buffer_index = 0;

    int rx_message_size = strlen(rx_message);

    for(int i = 0; i < rx_message_size; i++)
    {
        uint8_t current_command = rx_message[i];

        // clang-format off
        switch (current_command) {
          case 'B': /*PN_INFO ("*BLINK*");*/ break;
          case 'b': /*PN_INFO ("_______");*/ break;
          case 'r': callback_jtag_trigger_reset(); break;
          case '0': callback_jtag_set_inputs(0, 0, 0); break;
          case '1': callback_jtag_set_inputs(0, 0, 1); break;
          case '2': callback_jtag_set_inputs(0, 1, 0); break;
          case '3': callback_jtag_set_inputs(0, 1, 1); break;
          case '4': callback_jtag_set_inputs(1, 0, 0); break;
          case '5': callback_jtag_set_inputs(1, 0, 1); break;
          case '6': callback_jtag_set_inputs(1, 1, 0); break;
          case '7': callback_jtag_set_inputs(1, 1, 1); break;
          case 'R': tx_buffer[tx_buffer_index++] =
                        callback_jtag_get_output() ? '1' : '0'; break;
          case 'Q': _socket_close(); return;
          default:
                PN_WARNINGF(("c_remote_bitbang: Got unsupported command: '%c'",
                    current_command));
                break;
        }
        // clang-format on
    }

    // Send tdo back to openocd
    unsigned bytes_sent = 0;
    while(bytes_sent < tx_buffer_index)
    {
        ssize_t bytes = ::write(client_fd, tx_buffer + bytes_sent, tx_buffer_index);
        if(bytes == -1)
        {
            PN_ASSERTF(false,
                ("c_remote_bitbang: Failed to write to socket: %s (%d)",
                    strerror(errno),
                    errno));
        }
        bytes_sent += bytes;
    }
}

void c_remote_bitbang::_socket_close()
{
    if(::close(client_fd) == -1)
    {
        // Ignore error caused by non-blocking mode
        if(errno != EAGAIN)
        {
            PN_WARNINGF(("c_remote_bitbang: Close failed: %s (%d)",
                strerror(errno),
                errno));
        }
        return;
    }

    PN_INFOF(("c_remote_bitbang: Socket closed"));
    client_fd = 0;
}
