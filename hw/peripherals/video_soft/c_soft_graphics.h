/**
 * @file c_soft_graphics.h
 * @brief Soft peripheral implementation of the graphics peripheral for simulation
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Konrad Armbrecht <konrad.armbrecht@tha.de>
      Technische Hochschule Augsburg, University of Applied Sciences

  Description: This file contains the implementation of the c_soft_graphics for simulation ONLY

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
 * @addtogroup c_soft_graphics
 * @author Konrad Armbrecht
 *
 * This module is used to simulate the graphics peripheral.
 * The module is implemented as a soft peripheral and can be used in the
 * simulation environment.
 *
 * The module has a register for storing the output-images, called framebuffer.
 * The framebuffer is set up as a single large array where all output-image lines
 * are stored successively. The size is calculated by width and height given from
 * the constructor. With a 10 × 10 px resolution, you have 100 array elements, and the
 * framebuffer[10] element is the first pixel of the second output-image line.
 * The module has a 32-bit memory interface and can be accessed by the
 * soft peripheral interface.
 *
 * Note: The module is not synthesizable and is only used for simulation purposes.
 */

#ifndef __C_SOFT_GRAPHICS_H__
#define __C_SOFT_GRAPHICS_H__

#include "c_soft_peripheral.h"
#include <piconut.h>
#include <cstdint>   // For fixed width integer types
#include <cinttypes> //for properly formatted output
#include <functional>
#include <piconut-config.h>
#include <membrana_soft.h>

class c_soft_graphics : public c_soft_peripheral
{
public:
public:
    /**
     * @brief Registers of the c_soft_graphics module
     */
    struct regs_t
    {
        uint32_t control;                 ///< Control register, not in use in this implementation.
        uint32_t width;                   ///< Graphics output resolution width.
        uint32_t height;                  ///< Graphics output resolution height.
        uint32_t resolution_mode;         ///< Available resolution modes.
        uint32_t resolution_mode_support; ///< Number of supported modes, to check if the requested mode exists. Read only.
        uint32_t color_mode;              ///< Available color mode in bit.
        uint32_t color_mode_support;      ///< Number of color modes, to check if the requested mode exists. Read only.
        uint32_t* framebuffer;            ///< Pixel color information. Pointer for variable size.
    };

    uint32_t framebuffer_size_in_px;
    uint64_t base_address;
    uint64_t size;

    /**
     * @brief Register address offsets
     */
    enum e_regs
    {
        CONTROL = 0x0,
        WIDTH = 0x4,
        HEIGHT = 0x8,
        RESOLUTION_MODE = 0xC,
        RESOLUTION_MODE_SUPPORT = 0x10,
        COLOR_MODE = 0x14,
        COLOR_MODE_SUPPORT = 0x18,
        FRAMEBUFFER = 0x1C
    };

    /// Supported resolution modes, for readability
    enum class ResolutionMode : uint32_t
    {
        Mode_3x2 = 0,
        Mode_80x60 = 1,
    };

    /// Supported color modes, for readability
    enum class ColorMode : uint32_t
    {
        Mode_RGB888 = 0,
    };

    /**
     * @brief Constructor
     *
     * @param base_address base address of the peripheral in the address space of the simulation
     * @param resolution_mode resolution mode (recommended for performance reasons: "1" -> 60x80 px)
     * @param color_mode color depth in bits per pixel (here only "4" is implemented -> 32 bit)
     * @param clock_frequency maximum value for interrupt counter
     * @param flush_func Callback function to flush the framebuffer to GUI.
     * @param callback_signal_graphics_interrupt Callback function to signal graphics interrupts.
     */
    c_soft_graphics(
        uint64_t base_address,
        ResolutionMode resolution_mode,
        ColorMode color_mode,
        uint32_t clock_frequency,
        void (*flush_func)(uint32_t* framebuffer, uint32_t framebuffer_size_in_px),
        std::function<void(bool)> callback_signal_graphics_interrupt = nullptr);

    /**
     * @brief Destructor
     */
    ~c_soft_graphics();

    /**
     * @brief Flushes the graphics framebuffer using the provided callback function.
     *
     * @param flush_to_gui Callback function to send framebuffer data to the GUI.
     * @param framebuffer_size Size of the framebuffer in pixels.
     */

    void flush_graphics(void (*flush_to_gui)(uint32_t*, uint32_t), uint32_t framebuffer_size);

    /**
     * @brief Register a callback function for graphics interrupt changes.
     *
     * @param callback The callback function
     */
    void register_meip_callback(std::function<void(bool)> callback);

    /**
     * @brief Triggers interrupts when max_counter value is reached.
     * This function is automatically called on every rising edge of the clock
     */
    void on_rising_edge_clock() override;

    // c_soft_peripheral
    const char* get_info() override;
    bool is_addressed(uint64_t adr) override;

    uint32_t read32(uint64_t adr) override;
    void write32(uint64_t adr, uint32_t data) override;

private:
    void clear_framebuffer();
    void create_framebuffer(uint32_t new_size);

    // c_soft_peripheral
    char name[32] = "Graphics";
    regs_t registers;
    void (*flush_callback)(uint32_t* framebuffer, uint32_t framebuffer_size_in_px);

    // interrupt registers
    uint32_t meip;             // graphics interrupt external signal register
    uint64_t counter;          // interrupt counter
    uint32_t clock_frequency;  // piconut system clock frequency
    uint32_t v_sync_frequency; // image refresh rate
    uint32_t max_counter;      // maximum value for interrupt counter

    // Interrupt callback function
    std::function<void(bool)> callback_signal_graphics_interrupt;
};

#endif
