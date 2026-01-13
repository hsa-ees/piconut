/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Martin Erichsen <martin.erichsen@tha.de>
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

#ifndef __WB_GRAPHICS_CONFIG_H__
#define __WB_GRAPHICS_CONFIG_H__

#define FB_RAM_ADDR_WIDTH 32
#define FB_RAM_DATA_WIDTH 8

#define H_COUNTER_WIDTH 16
#define V_COUNTER_WIDTH 16

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

#endif //__WB_GRAPHICS_CONFIG_H__
