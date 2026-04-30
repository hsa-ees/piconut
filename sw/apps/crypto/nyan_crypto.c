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

/**
 * @brief Nyan Cat animation demo with encryption modes
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <rv_crypto.h>

#include <video_driver.h>

#include "nyan_cat.h"

#include <stdint.h>

#define FRAME_HEIGHT_FOR_SCREEN 60

// A basic 5x7 bitmap font for characters ' ' (32) through 'z' (122)
// Each byte is a column; 5 bytes per character.
static const uint8_t font5x7[][5] = {
    ['r' - 32] = {0x7C, 0x08, 0x04, 0x04, 0x08},
    ['e' - 32] = {0x38, 0x54, 0x54, 0x54, 0x18},
    ['a' - 32] = {0x20, 0x54, 0x54, 0x54, 0x78},
    ['l' - 32] = {0x00, 0x41, 0x7F, 0x40, 0x00},
    ['-' - 32] = {0x08, 0x08, 0x08, 0x08, 0x08},
    ['t' - 32] = {0x04, 0x3F, 0x44, 0x40, 0x20},
    ['i' - 32] = {0x00, 0x44, 0x7D, 0x40, 0x00},
    ['m' - 32] = {0x7C, 0x04, 0x18, 0x04, 0x78},
    ['c' - 32] = {0x38, 0x44, 0x44, 0x44, 0x20},
    ['y' - 32] = {0x0C, 0x50, 0x50, 0x50, 0x3C},
    ['p' - 32] = {0x7C, 0x14, 0x14, 0x14, 0x08},
    ['o' - 32] = {0x38, 0x44, 0x44, 0x44, 0x38},
    ['d' - 32] = {0x30, 0x48, 0x48, 0x48, 0x7F},
    ['A' - 32] = {0x7E, 0x11, 0x11, 0x11, 0x7E},
    ['E' - 32] = {0x7F, 0x49, 0x49, 0x49, 0x41},
    ['S' - 32] = {0x46, 0x49, 0x49, 0x49, 0x31},
    ['B' - 32] = {0x7F, 0x49, 0x49, 0x49, 0x36},
    ['C' - 32] = {0x3E, 0x41, 0x41, 0x41, 0x22},
    ['T' - 32] = {0x01, 0x01, 0x7F, 0x01, 0x01},
    ['R' - 32] = {0x7F, 0x09, 0x19, 0x29, 0x46},
    ['v' - 32] = {0x1C, 0x20, 0x40, 0x20, 0x1C},
    ['s' - 32] = {0x48, 0x54, 0x54, 0x54, 0x20},
    [' ' - 32] = {0x00, 0x00, 0x00, 0x00, 0x00}};

/**
 * Draws a single character.
 * x, y: top-left coordinate
 */
void draw_char(video_t* video, int x, int y, char c, uint8_t color)
{
    if(c < 32 || c > 122)
        return;

    // Get the actual buffer as 8-bit (since you are in 8-bit mode)
    volatile uint32_t* fb = video_get_framebuffer(video);
    int stride = video->fb_width; // Likely 160 or 640

    const uint8_t* glyph = font5x7[c - 32];

    for(int col = 0; col < 5; col++)
    {
        for(int row = 0; row < 7; row++)
        {
            if(glyph[col] & (1 << row))
            {
                int px = x + col;
                int py = y + row;

                // Use actual video dimensions instead of 64x64
                if(px >= 0 && px < video->fb_width && py >= 0 && py < video->fb_height)
                {
                    fb[py * stride + px] = color;
                }
            }
        }
    }
}

/**
 * Writes the specific demo text to the framebuffer.
 */
void write_crypto_labels(video_t* video, int startX, int startY, uint8_t color)
{
    const char* lines[] = {"real-time", "crypto-demo", "AES ECB", "vs CTR"};

    for(int i = 0; i < 4; i++)
    {
        int curX = startX;
        int curY = startY + (i * 9);
        const char* ptr = lines[i];

        while(*ptr)
        {
            draw_char(video, curX, curY, *ptr++, color);
            curX += 6;
        }
    }
}

// Track the current frame index globally or within your loop
static int current_frame_idx = 0;

// Define encryption keys and buffers
static uint8_t ecb_key[AES_128_KEY_BYTES] = {
    0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};
static uint8_t ctr_key[AES_128_KEY_BYTES] = {
    0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};
static uint8_t ctr_iv[AES_BLOCK_BYTES];
static uint8_t encrypted_frame[FRAME_WIDTH * FRAME_HEIGHT]; // Buffer for encrypted data

void draw_nyan(video_t* video, int32_t start_x, int32_t start_y, int encryption_mode)
{
    volatile uint32_t* framebuffer = video_get_framebuffer(video);
    int32_t fb_width = video->fb_width;
    int32_t fb_height = video->fb_height;

    const uint8_t* src_pixels = (const uint8_t*)frames[current_frame_idx];

    const uint8_t* draw_ptr = src_pixels;

    if(encryption_mode == 1) // ECB mode
    {
        rv_aes_ecb(ecb_key, 128, src_pixels, encrypted_frame, FRAME_HEIGHT * FRAME_WIDTH, 1);
        draw_ptr = encrypted_frame;
    }
    else if(encryption_mode == 2) // CTR mode
    {
        unsecure_initialization_vector(ctr_iv);
        rv_aes_ctr(ctr_key, 128, ctr_iv, src_pixels, encrypted_frame, FRAME_HEIGHT * FRAME_WIDTH);
        draw_ptr = encrypted_frame;
    }

    int32_t frame_width = FRAME_WIDTH;
    int32_t frame_height = FRAME_HEIGHT_FOR_SCREEN; // cropped to 640x480/160x120

    for(int y = 0; y < frame_height; ++y)
    {
        for(int x = 0; x < frame_width; ++x)
        {
            // Calculate the 1D index.
            // If using the raw frames, we must account for the 65th char (null terminator)
            // If using the encrypted buffer, it's usually packed (64).
            int index = y * FRAME_WIDTH + x;

            char pixel_char = draw_ptr[index];
            uint32_t final_color;

            // Nyan Cat Color Map (Indexed to custom palette_8_bit)
            switch(pixel_char)
            {
                case ',':
                    final_color = 1; // Deep Blue (Background)
                    break;
                case '>':
                    final_color = 2; // Red (Rainbow)
                    break;
                case '&':
                    final_color = 3; // Orange (Rainbow)
                    break;
                case '+':
                    final_color = 4; // Yellow (Rainbow)
                    break;
                case '#':
                    final_color = 5; // Green (Rainbow)
                    break;
                case '=':
                    final_color = 6; // Light Blue (Rainbow)
                    break;
                case ';':
                    final_color = 7; // Purple (Rainbow)
                    break;
                case '$':
                    final_color = 8; // Pink (Pop-tart)
                    break;
                case '-':
                    final_color = 9; // Dark Pink (Pop-tart dots)
                    break;
                case '@':
                    final_color = 10; // Beige (Pop-tart crust)
                    break;
                case '\'':
                    final_color = 11; // Black (Outline)
                    break;
                case '%':
                    final_color = 12; // White (Eyes/Stars)
                    break;
                case '*':
                    final_color = 13; // Grey (fur)
                    break;
                case '.':
                    final_color = 12; // White (Stars)
                    break;
                default:
                    final_color = pixel_char;
                    break;
            }

            int32_t screen_x = start_x + x;
            int32_t screen_y = start_y + y;

            if(screen_x >= 0 && screen_x < fb_width && screen_y >= 0 && screen_y < fb_height)
            {
                framebuffer[(screen_y * fb_width) + screen_x] = final_color;
            }
        }
    }

    // Advance to next frame, loop back to 0 when we hit the null terminator in frames[]
    current_frame_idx++;
    if(frames[current_frame_idx] == 0)
    {
        current_frame_idx = 0;
    }
}

void busy_wait(int32_t cycles)
{
    for(int i = 0; i < cycles; i++)
    {
        asm volatile("nop");
    }
}

void nyan_crypto()
{
    video_t video;

    // Initialize the driver instance with 640x480 resolution
    if(video_init(&video, 0, RESOLUTION_MODE_640x480, COLOR_MODE_8_MAP) != VIDEO_OK)
    {
        return;
    }
    if (video.fb_width < FRAME_WIDTH || video.fb_height < FRAME_HEIGHT_FOR_SCREEN)
    {
        return; // Ensure the framebuffer is large enough no printf because ram space is limited
    }

    while(1)
    {
        // Draw Nyan Cat in all four corners
        draw_nyan(&video, 0, 0, 0);                                         // Top-left corner (no encryption)
        draw_nyan(&video, video.fb_width - FRAME_WIDTH, 0, 1);              // Top-right corner (ECB encryption)
        draw_nyan(&video, 0, video.fb_height - FRAME_HEIGHT_FOR_SCREEN, 2); // Bottom-left corner (CTR encryption)
        write_crypto_labels(&video, video.fb_width - FRAME_WIDTH - 10, video.fb_height - FRAME_HEIGHT_FOR_SCREEN + 10, 4);

        busy_wait(50000); // Adjust delay for animation speed
    }
}