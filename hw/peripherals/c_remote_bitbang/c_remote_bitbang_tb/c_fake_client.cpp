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

#include "c_fake_client.h"

#include <base.h>

#include <cstring>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

c_fake_client::c_fake_client()
{
}

c_fake_client::~c_fake_client()
{
    close();
}

bool c_fake_client::connect(uint16_t port)
{
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);

    if(socket_fd == -1)
    {
        PN_ERRORF(("c_fake_client: Failed to make socket: %s (%d)",
            strerror(errno),
            errno));
        return false;
    }

    sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(port);
    server_address.sin_addr.s_addr = INADDR_ANY;

    if(::connect(socket_fd, (struct sockaddr*)&server_address, sizeof(server_address)) == -1)
    {
        PN_ERRORF(("c_fake_client: Failed to connect: %s (%d)",
            strerror(errno),
            errno));
        return false;
    }

    return true;
}

bool c_fake_client::connected()
{
    return socket_fd > 0;
}

bool c_fake_client::send(std::string message)
{
    if(!connected())
    {
        PN_INFO("c_fake_client: Tried to send while socket was not connected!");
        return false;
    }

    const char* msg = message.c_str();
    const size_t msg_length = message.size();

    if(::send(socket_fd, msg, msg_length, 0) == -1)
    {
        PN_WARNINGF(("c_fake_client: Failed to send: %s (%d)",
            strerror(errno),
            errno));
        return false;
    }

    return true;
}

std::optional<std::string> c_fake_client::recieve()
{
    if(!connected())
    {
        PN_INFO("c_fake_client: Tried to recieve while socket was not connected!");
        return {};
    }

    char buffer[1024] = {0};

    const int result = ::recv(socket_fd, buffer, sizeof(buffer), 0);
    if(result == -1)
    {
        PN_WARNINGF(("c_fake_client: Failed to recieve: %s (%d)",
            strerror(errno),
            errno));
        return {};
    }

    if(result == 0)
    {
        PN_INFO("c_fake_client: Connection closed by remote");
        return {};
    }

    return {std::string{buffer}};
}

void c_fake_client::close()
{
    if(!connected())
    {
        return;
    }

    if(::close(socket_fd) == -1)
    {
        PN_WARNINGF(("c_fake_client: Failed to close: %s (%d)",
            strerror(errno),
            errno));
        return;
    }
    socket_fd = 0;
}
