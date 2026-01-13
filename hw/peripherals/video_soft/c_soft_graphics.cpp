/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Konrad Armbrecht <konrad.armbrecht@tha.de>
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

#include "c_soft_graphics.h"
#include <iostream>
#include <cstring> // Für std::memcpy

c_soft_graphics::c_soft_graphics(
    uint64_t base_address,
    ResolutionMode resolution_mode,
    ColorMode color_mode,
    uint32_t clock_frequency,
    void (*flush_to_gui)(uint32_t* framebuffer, uint32_t framebuffer_size_in_px),
    std::function<void(bool)> callback_signal_graphics_interrupt)
    : base_address{base_address}
    , clock_frequency(clock_frequency)
    , flush_callback(flush_to_gui)
    , callback_signal_graphics_interrupt(callback_signal_graphics_interrupt)
{
    // init
    registers.framebuffer = nullptr;
    registers.control = 0;

    // set number of last possible resolution_mode-case here (adapt to number of resolution modes, see switch case in write32 function)
    // based on resolution_mode, framebuffer is created in resize_framebuffer function, called by write32 function.
    registers.resolution_mode_support = 1;
    write32(base_address + RESOLUTION_MODE, static_cast<uint32_t>(resolution_mode));

    // set number of last possible color_mode-case here (adapt to number of color modes, see switch case in write32 function)
    registers.color_mode_support = 0;
    write32(base_address + COLOR_MODE, static_cast<uint32_t>(color_mode));

    // fb_size_in_px gets calulated in write32 function. Add ~100 for 25 registers, even if less are used. Mult by 4 to get size in bytes.
    size = (framebuffer_size_in_px + 100) * 4;

    
    // v-sync counter, used in on_rising_edge_clock function
    // in theory, you'd calculate 60Hz like this, but the simulation is too slow -> max_counter is set to a static practical value
    // counter = 0;
    // v_sync_frequency = 60; // Hz
    // max_counter = clock_frequency / v_sync_frequency;
    max_counter = 10000;

    // Initial state: no interrupts pending
    if(callback_signal_graphics_interrupt)
    {
        callback_signal_graphics_interrupt(false);
    }
}

c_soft_graphics::~c_soft_graphics()
{
    delete[] registers.framebuffer;
}

const char* c_soft_graphics::get_info()
{
    static char info[128]; // Static to avoid memory management issues
    snprintf(info, sizeof(info), "Name: %s, Size: %d B\n", name, size);
    return info;
}

bool c_soft_graphics::is_addressed(uint64_t adr)
{
    return adr >= base_address && adr < size + base_address;
}

uint32_t c_soft_graphics::read32(uint64_t adr)
{
    uint32_t internal_address = adr - this->base_address;

    // set last two bits to 0
    internal_address &= ~0b11;

    // skip switch case when framebuffer is addressed
    if(internal_address > 0x18)
    {
        return registers.framebuffer[(internal_address - FRAMEBUFFER) >> 2];
    }

    // Read only from start of register
    switch(internal_address)
    {
        case CONTROL:
            PN_WARNING("Read from control register. Register is not in use.");
            return registers.control;
            break;
        case WIDTH:
            return registers.width;
            break;
        case HEIGHT:
            return registers.height;
            break;
        case RESOLUTION_MODE:
            return registers.resolution_mode;
            break;
        case RESOLUTION_MODE_SUPPORT:
            return registers.resolution_mode_support;
            break;
        case COLOR_MODE:
            return registers.color_mode;
            break;
        case COLOR_MODE_SUPPORT:
            return registers.color_mode_support;
            break;
        default:
            PN_WARNING("Invalid read32 address");
            return 0;
    }
}

void c_soft_graphics::write32(uint64_t adr, uint32_t data)
{
    // set last two bits to 0
    uint32_t internal_address = (adr - this->base_address) & ~(0b11);

    // skip switch if adr is in framebuffer
    if((internal_address >= 0x1C) && (internal_address <= 0x1C + registers.width * registers.height * 4))
    {
        registers.framebuffer[(internal_address - FRAMEBUFFER) >> 2] = data;
    }
    else
    {
        // Write only from start of register
        switch(internal_address)
        {
            case CONTROL:
                PN_WARNING("Written to control register. Register is not in use.");
                break;
            case WIDTH:
                registers.width = data;
                break;
            case HEIGHT:
                registers.height = data;
                break;
            case RESOLUTION_MODE:
                if(data <= registers.resolution_mode_support)
                {
                    registers.resolution_mode = data;
                    switch(data)
                    {
                        case 0:
                            create_framebuffer(3 * 2);
                            registers.width = 3;
                            registers.height = 2;
                            framebuffer_size_in_px = registers.width * registers.height;
                            PN_INFOF(("Graphics resolution mode %d set. Width: %d, height: %d, framebuffer size in px: %d", data, registers.width, registers.height, framebuffer_size_in_px));
                            break;
                        case 1:
                            create_framebuffer(80 * 60);
                            registers.width = 80;
                            registers.height = 60;
                            framebuffer_size_in_px = registers.width * registers.height;
                            PN_INFOF(("Graphics resolution mode %d set. Width: %d, height: %d, framebuffer size in px: %d", data, registers.width, registers.height, framebuffer_size_in_px));
                            break;
                        default:
                            PN_WARNING("Invalid resolution_mode.");
                            return;
                    }
                }
                else
                {
                    PN_WARNINGF(("Attempted to write invalid resolution_mode '%d'. Aborted. Previously set value '%d' remains.", data, registers.resolution_mode));
                }
                break;
            case COLOR_MODE:
                if(data <= registers.color_mode_support)
                {
                    registers.color_mode = data;
                    switch(data)
                    {
                        case 0:
                            // 24 bit true color, alpha channel not used -> 32 bit per pixel for simple memory fit
                            registers.color_mode = 0;
                            PN_INFO("Graphics color mode 0 set. Color depth in bit: 24 (RGB888)");
                            break;
                        default:
                            PN_WARNING("Invalid color_mode.");
                            return;
                    }
                }
                else
                {
                    PN_WARNINGF(("Attempted to write invalid color_mode '%d'. Aborted. Previously set value '%d' remains.", data, registers.color_mode));
                }
                break;
            default:
                PN_WARNING("Invalid write32 address");
                return;
        }
    }
}

void c_soft_graphics::flush_graphics(void (*callback)(uint32_t*, uint32_t), uint32_t framebuffer_size)
{
    if(flush_callback)
    {
        callback(registers.framebuffer, framebuffer_size_in_px);
    }
}

void c_soft_graphics::clear_framebuffer()
{
    std::fill(registers.framebuffer, registers.framebuffer + (registers.width * registers.height), 0);
    PN_INFO("Framebuffer cleared.");
}

void c_soft_graphics::create_framebuffer(uint32_t new_size)
{
    uint32_t* new_framebuffer = new uint32_t[new_size]();
    if(registers.framebuffer != nullptr)
    {
        PN_WARNING("Changing framebuffer size during runtime not supported. Please restart simulation and change resolution mode in costructor, or call clear_framebuffer() when changing resolution.");
    }
    registers.framebuffer = new_framebuffer;
    framebuffer_size_in_px = new_size;
}

void c_soft_graphics::on_rising_edge_clock()
{
    // Increment the graphics v-sync counter
    counter++;

    // If the counter counted to 1/60s (60Hz), trigger interrupt.
    // In this case max counter is reduced to 1000 because software simulation runs too slow and a cpu cycle needs more than 10ns -> 60Hz
    if(counter >= max_counter)
    {
        if(callback_signal_graphics_interrupt)
        {
            callback_signal_graphics_interrupt(true);
            PN_INFO("Graphics interrupt triggered");
        }

        flush_graphics(flush_callback, framebuffer_size_in_px);
        counter = 0;
    }
    else
    {
        if(callback_signal_graphics_interrupt)
        {
            callback_signal_graphics_interrupt(false);
        }
    }
}

void c_soft_graphics::register_meip_callback(std::function<void(bool)> callback)
{
    callback_signal_graphics_interrupt = callback;
    PN_INFOF(("c_soft_graphics: Registered MEIP callback"));
}