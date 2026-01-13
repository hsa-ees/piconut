/**
 * @file c_soft_audio.cpp
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Daniel Dakhno <Daniel.Dakhno1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description: This file contains the implementation of the c_soft_audio for simulation ONLY

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

#include "c_soft_audio.h"

int64 read_memory(char* data, int64 length, void* ctx)
{
    audio_context* context = static_cast<audio_context*>(ctx);

    uint32_t current_buffer_start = context->current_buffer_address;
    uint32_t current_buffer_size = context->current_buffer_size;

    uint32_t current_buffer_remaining = current_buffer_size - context->regs->read_head;

    if(current_buffer_remaining == 0)
    {
        if(context->regs->next_buffer_size == 0)
        {
            // upcomming buffer 0, stop playback
            context->audio->stopPlayback();
            context->sample_playback_enabled = false;
            return 0;
        }
        context->current_buffer_address = current_buffer_start = context->regs->next_buffer_address;
        context->current_buffer_size = current_buffer_size = current_buffer_remaining = context->regs->next_buffer_size;
        context->regs->read_head = 0;

        if(context->interrupt_enables.buffer_switch)
        {
            // set interrupt
            context->interrupt_flags.buffer_switch = true;
            *context->interrupt_requested_flag = true;
        }
    }

    // only copy to end of buffer
    uint32_t copied_size = MIN(length, current_buffer_remaining);

    for(uint32_t i = 0; i < copied_size; i++)
    {
        uint32_t address = current_buffer_start + context->regs->read_head + i;
        data[i] = context->membrana->read_peripheral(address, 0x01);
    }

    context->regs->read_head += copied_size;

    return copied_size;
}

c_soft_audio::c_soft_audio(uint64_t size, uint64_t base_address, m_membrana_soft* membrana_soft, void (*interrupt_callback)(bool))
    : size(size)
    , audio_registers(audio_registers)
    , base_address(base_address)
    , membrana_soft(membrana_soft)
    , interrupt_callback(interrupt_callback)
{
    context.regs = &audio_registers;
    context.membrana = membrana_soft;
    context.interrupt_requested_flag = &interrupt_requested;
    audio = new AudioTest(read_memory, &context);
    context.audio = audio;
}
c_soft_audio::~c_soft_audio() {}

const char* c_soft_audio::get_info()
{
    static char info[128]; // Static to avoid memory management issues
    snprintf(info, sizeof(info), "Name: %s, Size: %d B\n", name, size);
    return info;
}

void c_soft_audio::check_playback()
{
    if(this->play_general && this->context.sample_playback_enabled)
    {
        this->context.current_buffer_address = this->audio_registers.next_buffer_address;
        this->context.current_buffer_size = this->audio_registers.next_buffer_size;

        if(this->context.interrupt_enables.buffer_switch)
        {
            // set interrupt
            this->context.interrupt_flags.buffer_switch = true;
            *(this->context.interrupt_requested_flag) = true;
        }

        audio->startPlayback(sample_rate, context.stereo, this->bits_per_sample, this->signed_samples, this->float_samples);
    }
    else
    {
        audio->stopPlayback();
    }
}

void c_soft_audio::write32(uint64_t adr, uint32_t data)
{
    uint8_t internal_address = adr - this->base_address;

    switch(internal_address)
    {
        case REGISTER_GENERAL_CONTROL_OFFSET: {
            this->play_general = CHECK_BIT(data, 0);
            break;
        }
        case REGISTER_SAMPLE_CONTROL_OFFSET: {
            context.stereo = CHECK_BIT(data, STATUS_CONTROL_OFFSET_ENABLE_STEREO);
            this->context.sample_playback_enabled = CHECK_BIT(data, STATUS_CONTROL_OFFSET_ENABLE_PLAYING);
            context.interrupt_enables.buffer_switch = CHECK_BIT(data, STATUS_CONTROL_OFFSET_ENABLE_BUFFER_SWITCH_INTERRUPT);
            context.interrupt_flags.buffer_switch = CHECK_BIT(data, STATUS_CONTROL_OFFSET_INTERRUPT_FIRED);

            // TODO handle interrupt status reset situation

            this->check_playback();
            break;
        }
        case REGISTER_PRESCALER_OFFSET: {
            // need to calculate this here
            // fixed to 44100
            audio_registers.cycles_per_sample = data;
            this->sample_rate = 12000000 / data;
            break;
        }
        case REGISTER_FORMAT_OFFSET: {
            this->float_samples = false;
            this->signed_samples = false;

            if(CHECK_BIT(data, 0))
            {
                this->bits_per_sample = 8;
            }
            else if(CHECK_BIT(data, 1))
            {
                this->bits_per_sample = 8;
                this->signed_samples = true;
            }
            else if(CHECK_BIT(data, 2))
            {
                this->bits_per_sample = 16;
            }
            else if(CHECK_BIT(data, 3))
            {
                this->bits_per_sample = 16;
                this->signed_samples = true;
            }
            else if(CHECK_BIT(data, 4))
            {
                this->bits_per_sample = 24;
            }
            else if(CHECK_BIT(data, 5))
            {
                this->bits_per_sample = 24;
                this->signed_samples = true;
            }
            else if(CHECK_BIT(data, 6))
            {
                this->float_samples = true;
                break;
            }

            this->audio_registers.format = data;

            break;
        }
        case REGISTER_NEXT_BUFFER_OFFSET: {
            audio_registers.next_buffer_address = data;
            break;
        }
        case REGISTER_NEXT_BUFFER_SIZE_OFFSET: {
            audio_registers.next_buffer_size = data;
            break;
        }
        case REGISTER_READ_HEAD_OFFSET: {
            audio_registers.read_head = data;
            break;
        }
    }
}

uint32_t c_soft_audio::read32(uint64_t address)
{
    switch(address)
    {
        case REGISTER_GENERAL_STATUS_OFFSET: {
            return this->play_general;
        }
        case REGISTER_SAMPLE_STATUS_OFFSET: {
            return SET_BIT(STATUS_CONTROL_OFFSET_ENABLE_PLAYING, this->context.sample_playback_enabled) |
                   SET_BIT(STATUS_CONTROL_OFFSET_ENABLE_STEREO, this->context.stereo) |
                   SET_BIT(STATUS_CONTROL_OFFSET_ENABLE_BUFFER_SWITCH_INTERRUPT, this->context.interrupt_enables.buffer_switch) |
                   SET_BIT(STATUS_CONTROL_OFFSET_INTERRUPT_FIRED, this->context.interrupt_flags.buffer_switch);
        }
        case REGISTER_PRESCALER_OFFSET: {
            return this->audio_registers.cycles_per_sample;
        }
        case REGISTER_FORMAT_OFFSET: {
            return this->audio_registers.format;
        }
        case REGISTER_SUPPORT_FORMATS_OFFSET: {
            /*
            From QT:
            enum SampleFormat : quint16 {
                Unknown,
                UInt8,
                Int16,
                Int32,
                Float,
                NSampleFormats
            };
            */
            return 0b1101001;
        }
        case REGISTER_NEXT_BUFFER_OFFSET: {
            return audio_registers.next_buffer_address;
        }
        case REGISTER_NEXT_BUFFER_SIZE_OFFSET: {
            return audio_registers.next_buffer_size;
        }
        case REGISTER_READ_HEAD_OFFSET: {
            return audio_registers.read_head;
        }
    }
    return 0;
}

bool c_soft_audio::is_addressed(uint64_t adr)
{
    return adr >= base_address && adr < size + base_address;
}

void c_soft_audio::showControlsWindow()
{
    this->audio->show();
}

void c_soft_audio::on_rising_edge_clock()
{
    if(!interrupt_requested)
    {
        return;
    }

    if(interrupt_cycles == 0)
    {
        // set interrupt line
        this->interrupt_callback(true);
        interrupt_cycles++;
        return;
    }

    interrupt_cycles++;

    if(interrupt_cycles > 10)
    {
        // reset interrupt line
        this->interrupt_callback(false);
        interrupt_requested = false;
        interrupt_cycles = 0;
        return;
    }
}
