/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)     2025 Tristan Kundrat <tristan.kundrat@tha.de>
                         Niklas Sirch <niklas.sirch1@tha.de>
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
#include <stdint.h>
#include <unistd.h>

#include <clint_defs.h>
#include <audio_defs.h>

#define CLK_FREQ_HZ (25 * 1000 * 1000)

#define NUM_AUDIO (1U << (PN_CFG_AUDIO_EXP + 1))

#define C(OCT) (16.35 * (1 << OCT))
#define Cis(OCT) (17.32 * (1 << OCT))
#define D(OCT) (18.35 * (1 << OCT))
#define Dis(OCT) (19.45 * (1 << OCT))
#define E(OCT) (20.6 * (1 << OCT))
#define F(OCT) (21.83 * (1 << OCT))
#define Fis(OCT) (23.12 * (1 << OCT))
#define G(OCT) (24.5 * (1 << OCT))
#define Gis(OCT) (25.96 * (1 << OCT))
#define A(OCT) (27.5 * (1 << OCT))
#define B(OCT) (29.14 * (1 << OCT))
#define H(OCT) (30.87 * (1 << OCT))

typedef struct
{
    unsigned int enable : 1;
    unsigned int resv : 7;
    unsigned int waveform_type : 8;
    unsigned int step_height : 16;
    unsigned int frequency_divisor;
} synth_regs_t;

typedef enum
{
    WF_SQUARE = 0,
    WF_TRIANGLE,
    WF_SAWTOOTH,
    WF_SINE,
} e_waveform_t;

void set_trianglewave(double frequency, unsigned int amplitude, volatile synth_regs_t* synth_regs)
{
    volatile unsigned int* uint_ptr = (volatile unsigned int*)synth_regs;
    if(frequency < 0)
        return;
    if(frequency == 0)
    {
        uint_ptr[0] = 0;
        uint_ptr[1] = 0;
        return;
    }
    unsigned int freqdiv = (unsigned int)(CLK_FREQ_HZ / frequency);
    unsigned int step_height = (unsigned int)(amplitude * 2 / freqdiv);
    uint_ptr[1] = freqdiv;
    uint_ptr[0] = (step_height << 16) | (WF_TRIANGLE << 8) | 1;
}

void set_squarewave(double frequency, unsigned int amplitude, volatile synth_regs_t* synth_regs)
{
    volatile unsigned int* uint_ptr = (volatile unsigned int*)synth_regs;
    if(frequency < 0)
        return;
    if(frequency == 0)
    {
        uint_ptr[0] = 0;
        uint_ptr[1] = 0;
        return;
    }
    unsigned int freqdiv = (unsigned int)(CLK_FREQ_HZ / frequency);
    uint_ptr[1] = freqdiv;
    uint_ptr[0] = (amplitude << 16) | (WF_SQUARE << 8) | 1;
}

void set_sawtoothwave(double frequency, unsigned int amplitude, volatile synth_regs_t* synth_regs)
{
    volatile unsigned int* uint_ptr = (volatile unsigned int*)synth_regs;
    if(frequency < 0)
        return;
    if(frequency == 0)
    {
        uint_ptr[0] = 0;
        uint_ptr[1] = 0;
        return;
    }
    unsigned int freqdiv = (unsigned int)(CLK_FREQ_HZ / frequency);
    unsigned int step_height = (unsigned int)(amplitude / freqdiv);
    uint_ptr[1] = freqdiv;
    uint_ptr[0] = (step_height << 16) | (WF_SAWTOOTH << 8) | 1;
}

void print_regs(volatile synth_regs_t* reg)
{
    volatile unsigned int* ptr = (volatile unsigned int*)(reg);
    printf("ADR: %p\n", ptr);
    printf("RAW:\n0x00\t%08x\n0x04\t%08x\n", ptr[0], ptr[1]);
    printf("STRUCT:\nEnable\t%i\nType\t%i\nStep\t%i\nDiv\t%i\n", reg->enable, reg->waveform_type, reg->step_height, reg->step_height);
}

// clang-format off
const double notes_treble_0[194] = {
    E(5), -1,
    E(5), -1, H(4), C(5), D(5), E(5), C(5), H(4),
    A(4), -1, A(4), C(5), E(5), -1, D(5), C(5),
    H(4), -1, H(4), C(5), D(5), -1, E(5), -1,
    C(5), -1, A(4), -1, A(4), -1, 0, -1,
    0, D(5), -1, F(5), A(5), -1, G(5), F(5),
    E(5), -1, -1, C(5), E(5), -1, D(5), C(5),
    H(4), -1, H(4), C(5), D(5), -1, E(5), -1,
    C(5), -1, A(4), -1, A(4), -1, 0, -1,
    E(5), -1, H(4), C(5), D(5), -1, C(5), H(4),
    A(4), -1, A(4), C(5), E(5), -1, D(5), C(5),
    H(4), -1, H(4), C(5), D(5), -1, E(5), -1,
    C(5), -1, A(4), -1, A(4), -1, 0, -1,
    0, D(5), -1, F(5), A(5), -1, G(5), F(5),
    E(5), -1, -1, C(5), E(5), -1, D(5), C(5),
    H(4), -1, H(4), C(5), D(5), -1, E(5), -1,
    C(5), -1, A(4), -1, A(4), -1, 0, -1,
    E(4), -1, -1, -1, C(4), -1, -1, -1,
    D(4), -1, -1, -1, H(3), -1, -1, -1,
    C(4), -1, -1, -1, A(3), -1, -1, -1,
    Gis(3), -1, -1, -1, H(3), -1, -1, -1,
    E(4), -1, -1, -1, C(4), -1, -1, -1,
    D(4), -1, -1, -1, H(3), -1, -1, -1,
    A(3), -1, C(4), -1, E(4), -1, E(4), -1,
    E(4), -1, -1, -1, -1, -1, -1, -1
};
const double notes_treble_1[194] = {
    H(4), -1,
    0, -1, Gis(4), -1, 0, -1, A(4), Gis(4),
    E(4), -1, 0, -1, -1, -1, -1, -1,
    Gis(4), -1, Gis(4), A(4), H(4), -1, 0, -1,
    0, -1, -1, -1, -1, -1, -1, -1,
    0, F(4), -1, A(4), C(5), -1, H(4), A(4),
    G(4), -1, -1, E(4), G(4), -1, F(4), E(4),
    Gis(4), -1, Gis(4), A(4), H(4), -1, 0, -1,
    0, -1, -1, -1, -1, -1, -1, -1,
    0, -1, Gis(4), 0, -1, -1, A(4), Gis(4),
    E(4), -1, 0, -1, -1, -1, -1, -1,
    Gis(4), -1, Gis(4), A(4), H(4), -1, 0, -1,
    0, -1, -1, -1, -1, -1, -1, -1,
    0, F(4), -1, A(4), C(5), -1, H(4), A(4),
    G(4), -1, -1, E(4), G(4), -1, F(4), E(4),
    Gis(4), -1, Gis(4), A(4), H(4), -1, 0, -1,
    0, -1, -1, -1, -1, -1, -1, -1,
    C(4), -1, -1, -1, A(3), -1, -1, -1,
    H(3), -1, -1, -1, B(3), -1, -1, -1,
    A(3), -1, -1, -1, F(3), -1, -1, -1,
    E(3), -1, -1, -1, G(3), -1, -1, -1,
    C(4), -1, -1, -1, A(3), -1, -1, -1,
    H(3), -1, -1, -1, Gis(3), -1, -1, -1,
    C(4), -1, E(4), -1, A(4), -1, A(4), -1,
    Gis(4), -1, -1, -1, -1, -1, -1, -1
};
const double notes_bass_0[194] = {
    0, -1,
    E(1), E(2), E(1), E(2), E(1), E(2), E(1), E(2),
    A(0), A(1), A(0), A(1), A(0), A(1), A(0), A(1),
    Gis(0), Gis(1), Gis(0), Gis(1), Gis(0), Gis(1), Gis(0), Gis(1),
    A(0), A(1), A(0), A(1), A(0), A(1), H(1), C(2),
    D(2), D(2), D(1), D(2), D(1), D(2), D(1), D(2),
    C(1), C(2), C(1), C(2), C(1), C(2), C(1), C(2),
    H(0), H(1), H(0), H(1), H(0), H(1), H(0), H(1),
    A(0), A(1), A(0), A(1), A(0), A(1), A(0), A(1),
    E(1), E(2), E(1), E(2), E(1), E(2), E(1), E(2),
    A(0), A(1), A(0), A(1), A(0), A(1), A(0), A(1),
    Gis(0), Gis(1), Gis(0), Gis(1), Gis(0), Gis(1), Gis(0), Gis(1),
    A(0), A(1), A(0), A(1), A(0), A(1), H(1), C(2),
    D(2), D(2), D(1), D(2), D(1), D(2), D(1), D(2),
    C(1), C(2), C(1), C(2), C(1), C(2), C(1), C(2),
    E(1), E(2), E(1), E(2), E(1), E(2), E(1), E(2),
    A(1), A(2), A(1), A(2), A(1), A(2), A(1), A(2),
    A(1), E(1), A(1), E(1), A(1), E(1), A(1), E(1),
    Gis(1), E(2), Gis(1), E(2), Gis(1), E(2), Gis(1), E(2),
    A(1), E(2), A(1), E(2), A(1), E(2), A(1), E(2),
    Gis(1), E(2), Gis(1), E(2), Gis(1), E(2), Gis(1), E(2),
    A(1), E(2), A(1), E(2), A(1), E(2), A(1), E(2),
    Gis(1), E(2), Gis(1), E(2), Gis(1), E(2), Gis(1), E(2),
    A(1), E(2), A(1), E(2), A(1), E(2), A(1), E(2),
    E(1), E(2), E(1), E(2), E(1), E(2), E(1), E(2)
};
// clang-format on

volatile synth_regs_t* reg = (volatile synth_regs_t*)PN_CFG_AUDIO_BASE_ADDRESS;

void puzzle_melody_tick(int i)
{
    set_trianglewave(notes_treble_0[i], 0xFFFFFF, &reg[0]);
    set_squarewave(notes_treble_1[i], 0xFFFFFF, &reg[1]);
    set_trianglewave(notes_treble_1[i], 0xFFFFFF, &reg[2]);
    set_squarewave(notes_treble_0[i], 0xFFFFFF, &reg[3]);
    set_squarewave(notes_bass_0[i] * 2, 0xFFFFFF, &reg[4]);
    set_trianglewave(notes_bass_0[i] * 2, 0xFFFFFF, &reg[5]);
    set_squarewave(0, 0, &reg[7]);

    if(i == 185)
        set_squarewave(H(3), 0xFFFFFF, &reg[6]);
    if(i == 193)
        set_squarewave(0, 0, &reg[6]);
    if(i == 34 || i == 98)
        set_squarewave(D(1), 0xFFFFFF, &reg[7]);
}

#define PUZZLE_MEL_TICKS 194
#define PUZZLE_MEL_DELAY_MS 200

const uint32_t CLK_FRQ = 25000000; // ULX3S 25 MHz clock frequency
const uint32_t DELAY_MS = PUZZLE_MEL_DELAY_MS;
const uint32_t TIMER_INTERVAL_TICKS = DELAY_MS * (CLK_FRQ / 1000);

void restart_melody_interrupts()
{
    volatile uint32_t* mtime_lo = (volatile uint32_t*)CLINT_REG_MTIME_LO;
    volatile uint32_t* mtime_hi = (volatile uint32_t*)CLINT_REG_MTIME_HI;
    uint32_t current_time_lo = *mtime_lo;
    uint32_t current_time_hi = *mtime_hi;

    uint64_t current_time = ((uint64_t)current_time_hi << 32) | current_time_lo;
    uint64_t next_time = current_time + TIMER_INTERVAL_TICKS;

    volatile uint32_t* mtimecmp_lo = (volatile uint32_t*)CLINT_REG_MTIMECMP_LO;
    volatile uint32_t* mtimecmp_hi = (volatile uint32_t*)CLINT_REG_MTIMECMP_HI;

    *mtimecmp_lo = -1;
    *mtimecmp_hi = (uint32_t)(next_time >> 32);
    *mtimecmp_lo = (uint32_t)(next_time & 0xFFFFFFFF);
}

uint32_t melody_tick_counter = 0;

void timer_isr(void)
{
    puzzle_melody_tick(melody_tick_counter);
    melody_tick_counter++;

    if(melody_tick_counter >= PUZZLE_MEL_TICKS)
    {
        melody_tick_counter = 0;
    }

    restart_melody_interrupts();
}

void __attribute__((interrupt)) handle_interrupt()
{
    uint32_t mcause;
    __asm__ volatile("csrr %0, mcause"
        : "=r"(mcause));

    if((mcause & 0x80000000) && ((mcause & 0x7FFFFFFF) == 7))
    {
        timer_isr();
    }
    else
    {
        printf("Unhandled interrupt: mcause=0x%08x\n", mcause);
    }
}

void enableInterrupts()
{
    void* trap_vector = handle_interrupt;
    __asm__ volatile("csrw mtvec, %0"
        :
        : "r"(trap_vector));

    uint32_t mstatus;
    __asm__ volatile("csrr %0, mstatus"
        : "=r"(mstatus));
    mstatus |= (1 << 3);
    __asm__ volatile("csrw mstatus, %0"
        :
        : "r"(mstatus));

    uint32_t mie;
    __asm__ volatile("csrr %0, mie"
        : "=r"(mie));
    mie |= (1 << 7);
    __asm__ volatile("csrw mie, %0"
        :
        : "r"(mie));
}

void puzzle_melody_syn_background()
{
    printf("Initializing puzzle melody...\n");
    enableInterrupts();

    restart_melody_interrupts();
}

int puzzle_melody_syn()
{
    while(1)
    {
        printf("start playing puzzle...\n");
        for(int i = 0; i < PUZZLE_MEL_TICKS; i++)
        {
            printf("note %i : %f : %f : %f\n", i, notes_treble_0[i], notes_treble_1[i], notes_bass_0[i]);

            puzzle_melody_tick(i);

            print_regs(&reg[0]);
            print_regs(&reg[1]);
            print_regs(&reg[2]);
            print_regs(&reg[3]);
            print_regs(&reg[4]);
            print_regs(&reg[5]);
            if(i <= 185 && i >= 193)
                print_regs(&reg[6]);
            if(i == 34 || i == 98)
                print_regs(&reg[7]);

            for(int j = 0; j < 400000; j++)
                ;
        }
    }
}