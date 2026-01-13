/**
 * @file c_soft_audio.h
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

/**
 * @addtogroup c_soft_audio
 * @brief soft peripheral implementation for audio for simulation
 * @author Daniel Dakhno
 *
 * The module has a 32-bit memory interface and can be accessed by the
 * soft peripheral interface.
 *
 * This module exposes registers that allow to configure audio parameters like bitrate or format.
 * Then, a pointer and size to memory containing audio data need to be set, after which
 * audio playback can be started.
 *
 * The peripheral backs up the pointer and size to it's interrupt, after which an interrupt is fired,
 * allowing the application to prepare a new buffer and point this peripheral to the new buffer.
 *
 * Once the new buffer is addressed by this peripheral, the first buffer can be re-used, also called "double-buffering".
 *
 */

#ifndef __SOFT_AUDIO_H__
#define __SOFT_AUDIO_H__

#include "c_soft_peripheral.h"
#include <piconut.h>

#include <vector>
#include <cstring> // For memcpy, if needed
#include <stdio.h> // For snprintf
#include <cstdint> // For fixed width integer types

#include <fstream>
#include <iomanip>
#include <iostream>

#include <membrana_soft.h>

#include "audiooutput.h"

#include <QApplication>

#define REGISTER_GENERAL_CONTROL_OFFSET 0
#define REGISTER_GENERAL_STATUS_OFFSET 4
#define REGISTER_SAMPLE_CONTROL_OFFSET 8
#define REGISTER_SAMPLE_STATUS_OFFSET 12
#define REGISTER_PRESCALER_OFFSET 16
#define REGISTER_FORMAT_OFFSET 20
#define REGISTER_SUPPORT_FORMATS_OFFSET 24
#define REGISTER_NEXT_BUFFER_OFFSET 28
#define REGISTER_NEXT_BUFFER_SIZE_OFFSET 32
#define REGISTER_READ_HEAD_OFFSET 36

#define CHECK_BIT(register, offset) ((register & (1 << offset)) == (1 << offset))
#define SET_BIT(offset, condition) ((condition) ? (1 << offset) : (0))

typedef enum
{
    STATUS_CONTROL_OFFSET_ENABLE_PLAYING = 0x00,
    STATUS_CONTROL_OFFSET_ENABLE_REPEAT,
    STATUS_CONTROL_OFFSET_ENABLE_STEREO,
    STATUS_CONTROL_OFFSET_ENABLE_BUFFER_SWITCH_INTERRUPT,
    STATUS_CONTROL_OFFSET_INTERRUPT_FIRED,
} StatusControlBitOffset;

// struct of all registers of the audio
typedef struct
{
    uint32_t general_control;
    uint32_t general_status;

    uint32_t sample_control;
    uint32_t sample_status;

    uint32_t cycles_per_sample;

    uint32_t format;
    uint32_t supported_formats;

    uint32_t next_buffer_address;
    uint32_t next_buffer_size;

    uint32_t read_head;
} audio_regs_t;

typedef struct
{
    audio_regs_t* regs;
    m_membrana_soft* membana_soft;
    bool repeat, stereo;
    struct
    {
        bool buffer_switch;
    } interrupt_enables;
    struct
    {
        bool buffer_switch;
    } interrupt_flags;

    bool sample_playback_enabled;

    uint32_t current_buffer_address;
    uint32_t current_buffer_size;

    volatile bool* interrupt_requested_flag;

    AudioTest* audio;
} audio_context;

class c_soft_audio : public c_soft_peripheral
{
public:
    char name[32] = "Audio";

    audio_regs_t audio_registers;

    m_membrana_soft* membrana_soft;

    uint64_t size;
    uint64_t base_address; // Starting address of memory

    AudioTest* audio;

    audio_context context;

    bool play_general = false;

    uint32_t bits_per_sample = 0;
    bool float_samples = false;
    bool signed_samples = false;

    uint32_t sample_rate = 44100;

    volatile bool interrupt_requested = false;
    uint32_t interrupt_cycles = 0;

    void (*interrupt_callback)(bool);

    c_soft_audio(uint64_t size, uint64_t base_address, m_membrana_soft* membrana_soft, void (*interrupt_callback)(bool));
    ~c_soft_audio();

    const char* get_info() override;

    void write32(uint64_t adr, uint32_t data) override;

    uint32_t read32(uint64_t address) override;

    bool is_addressed(uint64_t adr) override;

    void showControlsWindow();

    void on_rising_edge_clock() override;

    void check_playback();
};

#endif
