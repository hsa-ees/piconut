/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2026 Niklas Sirch <niklas.sirch1@tha.de>
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

#ifndef __VIDEO_DRIVER_H__
#define __VIDEO_DRIVER_H__

#include <stdint.h>
#include <stdbool.h>

#include "video_defs.h"

/**
 * @brief Video Driver Instance Structure
 */
typedef struct
{
    uintptr_t base_addr;        /**< Readonly base address of the video module registers */
    uint32_t fb_width;          /**< Readonly framebuffer width from hardware */
    uint32_t fb_height;         /**< Readonly framebuffer height from hardware */
    e_resolution_mode res_mode; /**< Readonly resolution mode from hardware */
    e_color_mode color_mode;    /**< Readonly color mode from hardware */
} video_t;

/**
 * @brief Status codes for video operations
 */
typedef enum
{
    VIDEO_OK = 0,
    VIDEO_ERR_INVALID = -1,
    VIDEO_ERR_UNSUPPORTED = -2
} video_res_t;

/**
 * @brief Initializes the video module, sets modes, and caches dimensions.
 * @param self Pointer to driver instance.
 * @param base_addr Optional physical base address of the hardware registers. If 0, default address is used.
 * @param res_mode Initial resolution mode.
 * @param color_mode Initial color mode.
 * @return VIDEO_OK on success.
 */
video_res_t video_init(video_t* self, uintptr_t base_addr, e_resolution_mode res_mode, e_color_mode color_mode);

/**
 * @brief Enables or disables the video output.
 * @param self Pointer to driver instance.
 * @param enable True to enable, false to disable.
 */
void video_set_enable(video_t* self, bool enable);

/**
 * @brief Reads the current enable state from VIDEO_REG_CONTROL.
 * @param self Pointer to driver instance.
 * @return True if enabled.
 */
bool video_get_enabled(video_t* self);

/**
 * @brief Enables or disables video interrupts.
 * @param self Pointer to driver instance.
 * @param enable True to enable interrupts, false to disable.
 */
void video_set_interrupts(video_t* self, bool enable);

/**
 * @brief Reads the current interrupt enable state from VIDEO_REG_INTERRUPT.
 * @param self Pointer to driver instance.
 * @return True if interrupts are enabled.
 */
bool video_get_interrupts(video_t* self);

/**
 * @brief Reads the hardware status register.
 * @param self Pointer to driver instance.
 * @return 32-bit status register value.
 */
uint32_t video_get_status(video_t* self);

/**
 * @brief Sets resolution and color mode if supported by the hardware.
 * @note Updates internal width/height cache upon success.
 */
video_res_t video_set_mode(video_t* self, e_resolution_mode res_mode, e_color_mode color_mode);

/**
 * @brief Force update of the cached framebuffer size
 */
void video_update_fb_size(video_t* self);

/**
 * @brief Force update of the cached resolution and color mode from hardware
 */
void video_update_modes(video_t* self);

/**
 * @brief Reads the currently rendering scanline.
 * @param self Pointer to driver instance.
 * @return Current scanline index.
 */
uint32_t video_get_current_scanline(video_t* self);

/**
 * @brief Returns the bitmask of supported resolution modes.
 */
uint32_t video_get_supported_res_modes(video_t* self);

/**
 * @brief Returns the bitmask of supported color modes.
 */
uint32_t video_get_supported_color_modes(video_t* self);

/**
 * @brief Reads a 32-bit color value from the hardware color map.
 * @param self Pointer to driver instance.
 * @param index Color index (0-255).
 * @return 32-bit color value.
 */
uint32_t video_read_color_map(video_t* self, uint8_t index);

/**
 * @brief Returns a volatile pointer to the start of the framebuffer memory.
 */
volatile uint32_t* video_get_framebuffer(video_t* self);

/**
 * @brief Safe pixel write with bounds checking.
 * @param self Pointer to driver instance.
 * @param x X-coordinate.
 * @param y Y-coordinate.
 * @param color Color value to write.
 * @return VIDEO_OK if within bounds, VIDEO_ERR_INVALID otherwise.
 */
video_res_t video_write_pixel(video_t* self, uint32_t x, uint32_t y, uint32_t color);

/**
 * @brief Reads a pixel value from the framebuffer with bounds checking.
 * @param self Pointer to driver instance.
 * @param x X-coordinate.
 * @param y Y-coordinate.
 * @return Color value, or 0 if out of bounds.
 */
uint32_t video_read_pixel(video_t* self, uint32_t x, uint32_t y);

#endif // __VIDEO_DRIVER_H__