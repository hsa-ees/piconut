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
 * @brief Video Spectrum Demo Application switch between different video modes and the drawing
 *  PicoNut sprite and spectrum
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include <video_driver.h>

#include "piconut_sprite.h"

#ifndef ONLY_COLOR_MODE
#define ONLY_COLOR_MODE COLOR_MODE_8_MAP
#undef ONLY_COLOR_MODE
#endif

#ifndef DO_LOOP
#define DO_LOOP 1
#endif

void draw_spectrum(video_t* self)
{
    const int margin = 3;
    volatile uint32_t* framebuffer = video_get_framebuffer(self);
    int32_t width = self->fb_width;
    int32_t height = self->fb_height;

    int max_x = width - margin;
    int max_y = height - margin;

    if(max_x < margin || max_y < margin)
        return;

    uint32_t color_val = 0;
    for(int y = margin; y < max_y; ++y)
    {
        for(int x = margin; x < max_x; ++x)
        {
            framebuffer[(y * width) + x] = color_val++;
        }
    }
}

void draw_black(video_t* self)
{
    volatile uint32_t* framebuffer = video_get_framebuffer(self);
    int32_t width = self->fb_width;
    int32_t height = self->fb_height;

    for(int y = 0; y < height; ++y)
    {
        for(int x = 0; x < width; ++x)
        {
            framebuffer[(y * width) + x] = 0;
        }
    }
}

void draw_border(video_t* self)
{
    const int thickness = 1;
    volatile uint32_t* framebuffer = video_get_framebuffer(self);
    int32_t width = self->fb_width;
    int32_t height = self->fb_height;

    for(int y = 0; y < height; ++y)
    {
        for(int x = 0; x < width; ++x)
        {
            bool is_border = (x < thickness) || (x >= width - thickness) ||
                             (y < thickness) || (y >= height - thickness);
            if(is_border)
            {
                framebuffer[(y * width) + x] = 0xFFFFFFFF;
            }
        }
    }
}

void draw_sprite(video_t* self)
{
    volatile uint32_t* framebuffer = video_get_framebuffer(self);
    int32_t fb_width = self->fb_width;
    int32_t fb_height = self->fb_height;

    const int sprite_size = sizeof(piconut_sprite) / sizeof(piconut_sprite[0]);

    int32_t start_x = (fb_width - sprite_size) / 2;
    int32_t start_y = (fb_height - sprite_size) / 2;

    for(int y = 0; y < sprite_size; ++y)
    {
        for(int x = 0; x < sprite_size; ++x)
        {
            uint32_t raw_pixel = piconut_sprite[y][x];

            int32_t screen_x = start_x + x;
            int32_t screen_y = start_y + y;

            // Simple safety check
            if(screen_x >= 0 && screen_x < fb_width && screen_y >= 0 && screen_y < fb_height)
            {
                uint32_t final_color;

                switch(raw_pixel)
                {
                    case 0:
                        final_color = 0x00000000;
                        break;
                    case 1:
                        final_color = 0x05;
                        break;
                    case 2:
                        final_color = 0xD6;
                        break;
                    case 3:
                        final_color = 0x88;
                        break;
                    case 4:
                    default:
                        final_color = 0x00000000;
                        break;
                }

                framebuffer[(screen_y * fb_width) + screen_x] = final_color;
            }
        }
    }
}

void busy_wait(int32_t cycles)
{
    for(int i = 0; i < cycles; i++)
    {
        asm volatile("nop");
    }
}

int main()
{
    printf("######### Starting Video Driver Demo #########\n\n");
    srand(6758);

    video_t video;

    // Initialize the driver instance
    if(video_init(&video, 0, RESOLUTION_MODE_640x480, COLOR_MODE_1_MONO) != VIDEO_OK)
    {
        printf("Error: Could not initialize video driver.\n");
        return -1;
    }

    uint32_t supported_res = video_get_supported_res_modes(&video);
    uint32_t supported_colors = video_get_supported_color_modes(&video);

    printf("Targeting video module at: 0x%lx\n", video.base_addr);
    printf("Supported Resolutions (bitmask): 0x%08X\n", supported_res);
    printf("Supported Color Modes (bitmask): 0x%08X\n\n", supported_colors);

    busy_wait(6000000); // Wait for display to start up

    do
    {
        for(int res_bit = 0; res_bit < 32; res_bit++)
        {
            if(supported_res & (1 << res_bit))
            {
                for(int color_bit = 0; color_bit < 32; color_bit++)
                {
                    if(supported_colors & (1 << color_bit))
                    {
                        // Set the hardware mode (this also updates driver internal cache)
#ifdef ONLY_COLOR_MODE
                        video_set_mode(&video, (e_resolution_mode)res_bit, ONLY_COLOR_MODE);
#else
                        video_set_mode(&video, (e_resolution_mode)res_bit, (e_color_mode)color_bit);
#endif

                        printf("Testing Mode - Res: %d, Color: %d [%d x %d]\n",
                            res_bit,
                            color_bit,
                            video.fb_width,
                            video.fb_height);

                        draw_spectrum(&video);
                        draw_border(&video);

                        busy_wait(10000000);

                        draw_black(&video);
                        draw_sprite(&video);
                        draw_border(&video);

                        busy_wait(3000000);
                        printf("Cycle complete.\n");
                    }
                }
            }
        }
    } while(DO_LOOP);

    printf("\n######### Finished #########\n");

    return 0;
}