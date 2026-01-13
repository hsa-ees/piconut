/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Daniel Dakhno <daniel.dakhno1@tha.de>
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

#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include "notes.h"

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

#define SAMPLE_SIZE 10

#define BUFFER_SIZE 1000

audio_regs_t* audio_peripheral = (audio_regs_t*)0x50000000U;

uint8_t melody[] = {5, 5, 2, 3, 4, 4, 3, 2, 1, 1, 1, 3, 5, 5, 4, 3, 2, 2, 2, 3, 4, 4, 5, 5, 3, 3, 1, 1, 1, 1, 1, 4, 6, 8, 8, 7, 6, 5, 5, 5, 3, 5, 5, 4, 3, 2, 2, 2, 3, 4, 4, 5, 5, 3, 3, 1, 1, 1, 9, 9, 9};
uint32_t melody_pointer = 0;

void audio_buffer_changed_ISR()
{
    audio_peripheral->next_buffer_address = (uint32_t)notes[melody[melody_pointer] - 1];
    melody_pointer++;
    melody_pointer %= sizeof(melody) / sizeof(melody[0]);
}

// Main interrupt handler - Called by hardware when any interrupt occurs
void __attribute__((interrupt)) handle_interrupt()
{
    // Get cause of the interrupt
    uint32_t mcause;
    __asm__ volatile("csrr %0, mcause"
                     : "=r"(mcause));

    if((mcause & 0x80000000) && ((mcause & 0x7FFFFFFF) == 11))
    {
        audio_buffer_changed_ISR();
    }
    else
    {
        printf("Unexpected interrupt: mcause=0x%08x\n", mcause);
    }
}

// Function to start periodic timer interrupts
void enableInterrupts()
{
    void* trap_vector = handle_interrupt;
    printf("Setting up interrupt handler at address: %p\n", trap_vector);
    __asm__ volatile("csrw mtvec, %0"
                     :
                     : "r"(trap_vector));

    uint32_t mstatus;
    __asm__ volatile("csrr %0, mstatus"
                     : "=r"(mstatus));
    mstatus |= (1 << 3); // Set MIE bit
    __asm__ volatile("csrw mstatus, %0"
                     :
                     : "r"(mstatus));

    uint32_t mie;
    __asm__ volatile("csrr %0, mie"
                     : "=r"(mie));
    mie |= (1 << 7);  // Set MTIE bit for timer interrupts
    mie |= (1 << 3);  // Set MSIE bit for software interrupts
    mie |= (1 << 11); // Set MEIE bit for external interrupts
    __asm__ volatile("csrw mie, %0"
                     :
                     : "r"(mie));
}

int main()
{
    setvbuf(stdout, NULL, _IONBF, 0); // <- important for printf to work correctly with interrupts
    printf("Audio example\n");

    printf("Starting playback...\n");
    // enable audio peripheral
    audio_peripheral->general_control = 1;

    // set to 10 khz
    audio_peripheral->cycles_per_sample = 1200;

    // int_16
    audio_peripheral->format = 0b1000;

    audio_peripheral->next_buffer_address = (uint32_t)notes[melody[melody_pointer] - 1];
    audio_peripheral->next_buffer_size = 5000;
    audio_peripheral->read_head = 0;
    melody_pointer++;

    enableInterrupts();

    // enable interrupts and run
    audio_peripheral->sample_control = 0b1001;

    for(;;)
        ;

    return 0;
}
