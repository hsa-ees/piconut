/*************************************************************************

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

 *************************************************************************/

#ifndef __COLOR_PALETTES_H__
#define __COLOR_PALETTES_H__
#include <stdint.h>

/**
 * @file color_palettes.h
 * @brief Default color palette definitions
 * @details These palettes are hardcoded color arrays used by the video peripheral.
 *          They provide a set of predefined colors for different color modes.
 *          If needed exchange these palettes with custom ones e.g. from https://lospec.com/palette-list
 *          Later a selection could be implemented for the palettes.
 *          These arrays account for ~1300 LUTs
 */

/**
 * @brief 2-Bit Color Palette
 * @details By default a 4-color palette featuring cosmic magentas and yellows.
 *          Source: https://lospec.com/palette-list/galactic-pizza
 */
extern const uint32_t palette_2_bit[4];

/**
 * @brief 3-Bit Color Palette
 * @details By default a balanced 8-color palette with cute cotton candy colors.
 *          Source: https://lospec.com/palette-list/pollen8
 */
extern const uint32_t palette_3_bit[8];

/**
 * @brief 4-Bit Color Palette
 * @details By default a versatile 16-color palette including a wide range of hues and neutrals.
 *          Source: https://lospec.com/palette-list/go-line
 */
extern const uint32_t palette_4_bit[16];

/**
 * @brief Standard 8-Bit RGB Palette
 * @details By default color palette "Doctor Robotnik's Ring Racers Palette Palette"
 *          Source: https://lospec.com/palette-list/doctor-robotniks-ring-racers-palette
 */
extern const uint32_t palette_8_bit[256];

#endif // __COLOR_PALETTES_H__