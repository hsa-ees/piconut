/*******************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Niklas Sirch <niklas.sirch1@tha.de>
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

 ******************************************************************************/

#ifndef __VIDEO_DEFS_H__
#define __VIDEO_DEFS_H__

/**
 * @defgroup m_video_config Video Module Configuration
 * @brief Configuration macros for the Video peripheral module.
 * @{
 */
#ifndef PN_CFG_VIDEO_BASE_ADDRESS
/** @brief Base address of the Video instance in the system
 */
#define PN_CFG_VIDEO_BASE_ADDRESS 0x60000000U
#endif

#ifndef PN_CFG_VIDEO_FB_SIZE
/** @brief Size of the framebuffer in bytes for the video module
 */
#define PN_CFG_VIDEO_FB_SIZE 2048U
#endif

#ifndef PN_CFG_VIDEO_FB_SCALING_DIVISOR
/** @brief Scaling divisor for smaller framebuffers to bigger resolutions
 *  Only works for one resolution currently!
 */
#define PN_CFG_VIDEO_FB_SCALING_DIVISOR 16
#endif
/** @} */ // End of video_config group

/**
 * @brief Video register address offsets
 */
typedef enum
{
    // TBD: Unify register with c_soft_graphics
    VIDEO_REG_CONTROL = 0x0U,
    VIDEO_REG_STATUS = 0x4U,
    VIDEO_REG_RESOLUTION_MODE = 0x8U,
    VIDEO_REG_RESOLUTION_MODE_SUPPORT = 0xCU,
    VIDEO_REG_COLOR_MODE = 0x10U,
    VIDEO_REG_COLOR_MODE_SUPPORT = 0x14U,
    VIDEO_REG_SCANLINE = 0x18U,
    VIDEO_REG_FRAMEBUFFER_WIDTH = 0x1CU,
    VIDEO_REG_FRAMEBUFFER_HEIGHT = 0x20U,
    VIDEO_REG_COLOR_MAP = 0x24U, // 256x32-bits
    VIDEO_REG_FRAMEBUFFER = 0x424U,
} e_video_reg_offset;

#ifndef PN_VIDEO_REG_SIZE_BYTES
/** @brief Size of the video register space in bytes
 */
#define PN_VIDEO_REG_SIZE_BYTES VIDEO_REG_FRAMEBUFFER + ((uint32_t)PN_CFG_VIDEO_FB_SIZE) * 4 // * 4 for bytes to 32-bit words conversion
#endif

/**
 * @brief Supported resolution modes bitmask (hardware implementation)
 */
#define RESOLUTION_MODE_SUPPORTED_MASK_HW 0b01U
/**
 * @brief Supported color modes bitmask (hardware implementation)
 */
#define COLOR_MODE_SUPPORTED_MASK_HW 0b00111111111111U // 0-11 inclusive (only <= 8-bit color modes)

typedef enum
{
    COLOR_MODE_1_MONO = 0,
    COLOR_MODE_2_MAP = 1,
    COLOR_MODE_2_GRAY = 2,
    COLOR_MODE_3_MAP = 3,
    COLOR_MODE_3_RGB = 4,
    COLOR_MODE_3_GRAY = 5,
    COLOR_MODE_4_MAP = 6,
    COLOR_MODE_4_RGBI = 7,
    COLOR_MODE_4_GRAY = 8,
    COLOR_MODE_8_MAP = 9,
    COLOR_MODE_8_RGB332 = 10,
    COLOR_MODE_8_GRAY = 11,
    COLOR_MODE_16_RGB565 = 12,
    COLOR_MODE_32_RGB = 13,
} e_color_mode;

typedef enum
{
    RESOLUTION_MODE_640x480 = 0,
    RESOLUTION_MODE_800x600 = 1,
    RESOLUTION_MODE_1024x768 = 2,
    RESOLUTION_MODE_1280x720 = 3,
    RESOLUTION_MODE_1920x1080 = 4,
    RESOLUTION_MODE_3x2 = 5,
    RESOLUTION_MODE_80x60 = 6,
    RESOLUTION_MODE_320x240 = 7,
} e_resolution_mode;

#endif // __VIDEO_DEFS_H__
