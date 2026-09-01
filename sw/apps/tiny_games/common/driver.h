/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Daniel Sommerfeldt <daniel.sommerfeldt1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
        This file contains piconut driver functions,
        used by the tiny_games of Daniel C (electro_l.i.b@tinyjoypad.com)

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

#pragma once

#include <stdint.h>
#include <stdio.h>
#include <gpio_driver.h>
#include <i2c_driver.h>
#include <video_driver.h>

static video_t video_driver = {};

/**
 * ROTATE_CONTROLS:
 * If defined, the controls are rotated 90 degrees counter-clockwise.
 * This is useful for games that are designed to be played in landscape mode with
 * a non-rotated controller.
 */

#ifdef __cplusplus
extern "C" {
#endif

#define SSD1306_ADDR 0x3C

static i2c_t global_i2c_instance;
static gpio_t global_gpio_instance;
static uint8_t last_frame_buffer[8][128];

void JOY_OLED_init(void);
uint16_t JOY_random(void);

// Helper ...
static inline uint8_t read_button(const uint8_t button)
{
    bool pressed = 0;
    if(gpio_read_input_pin(&global_gpio_instance, button, &pressed) != GPIO_OK)
    {
        return 0;
    }
    return (uint8_t)pressed;
}

static inline void JOY_init(void)
{
    gpio_init(&global_gpio_instance, 0x40000000);

    video_init(&video_driver, 0, RESOLUTION_MODE_640x480, COLOR_MODE_1_MONO);
    if(video_driver.fb_width < 128 || video_driver.fb_height < 64)
    {
        // Framebuffer too small (no printf because of resource constraints)
        while(1)
            ;
        // TODO GPIO alarm
    }
    JOY_OLED_init();
}

i2c_status_t clear_page(i2c_t* i2c, uint8_t page)
{
    i2c_status_t st = I2C_OK;

    if(page > 7)
        page = 7;

    uint8_t data[7] = {0x00, 0x21, 0, 127, 0x22, page, page};

    for(bool running = true; running; running = false)
    {

        st = i2c_master_send(i2c, SSD1306_ADDR, data, sizeof(data));
        if(st != I2C_OK)
            break;

        st = i2c_start(i2c);
        if(st != I2C_OK)
            break;

        st = i2c_send_addr(i2c, SSD1306_ADDR, false);
        if(st != I2C_OK)
            break;

        st = i2c_send_data(i2c, 0x40);
        if(st != I2C_OK)
            break;

        for(int i = 0; i < 128; i++)
        {
            st = i2c_send_data(i2c, 0x00);
            if(st != I2C_OK)
                break;
        }
    }
    i2c_stop(i2c);
    return st;
}

void JOY_OLED_send_range(uint8_t page, uint8_t start, uint8_t end, uint8_t* data)
{
    if(page > 7 || start > 127 || end > 127 || start > end)
        return;
    uint8_t cmds[] = {0x00, 0x21, start, end, 0x22, page, page};
    i2c_master_send(&global_i2c_instance, SSD1306_ADDR, cmds, sizeof(cmds));

    // Send data
    i2c_start(&global_i2c_instance);
    i2c_send_addr(&global_i2c_instance, SSD1306_ADDR, false);
    i2c_send_data(&global_i2c_instance, 0x40);

    for(int i = 0; i <= (end - start); i++)
    {
        i2c_send_data(&global_i2c_instance, data[i]);
    }
    i2c_stop(&global_i2c_instance);
}

void write_video_framebuffer(uint8_t current_buffer[8][128])
{
    int32_t fb_width = video_driver.fb_width;
    for(uint8_t page = 0; page < 8; page++)
    {
        volatile uint32_t* fb_ptr = video_get_framebuffer(&video_driver);
        for(uint8_t col = 0; col < 128; col++)
        {
            for(uint8_t bit = 0; bit < 8; bit++)
            {
                if(current_buffer[page][col] & (1 << bit))
                {
                    fb_ptr[(col) + ((page * 8 + bit) * fb_width)] = 0xFFFFFFFF;
                }
                else
                {
                    fb_ptr[(col) + ((page * 8 + bit) * fb_width)] = 0x00000000;
                }
            }
        }
    }
}

void JOY_OLED_update(uint8_t current_buffer[8][128])
{
    write_video_framebuffer(current_buffer);
    for(int page = 0; page < 8; page++)
    {
        int start_col = -1;
        for(int col = 0; col < 128; col++)
        {
            uint8_t val = current_buffer[page][col];
            if(val != last_frame_buffer[page][col])
            {
                if(start_col == -1)
                    start_col = col;
                last_frame_buffer[page][col] = val;
            }
            else
            {
                if(start_col != -1)
                {
                    JOY_OLED_send_range(page, start_col, col - 1, &current_buffer[page][start_col]);
                    start_col = -1;
                }
            }
        }
        if(start_col != -1)
        {
            JOY_OLED_send_range(page, start_col, 127, &current_buffer[page][start_col]);
        }
    }
}

void JOY_OLED_send_command(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    i2c_master_send(&global_i2c_instance, SSD1306_ADDR, buf, 2);
}

void JOY_OLED_init()
{
    global_i2c_instance.base_addr = 0x70000000;
    global_i2c_instance.scl_speed_hz = 100000;

    i2c_init(&global_i2c_instance, 25E6);

    uint8_t wake_up[] = {0x00, 0x8D, 0x14, 0xA0, 0xC0, 0xAF};
    i2c_master_send(&global_i2c_instance, SSD1306_ADDR, wake_up, sizeof(wake_up));

    // Clear the screen so we start fresh
    for(uint8_t i = 0; i < 8; i++)
    {
        clear_page(&global_i2c_instance, i);
        for(uint8_t j = 0; j < 128; j++)
            last_frame_buffer[i][j] = 0;
    }
}

void JOY_OLED_end()
{
    i2c_stop(&global_i2c_instance);
}

void JOY_OLED_send(uint8_t data)
{
    i2c_send_data(&global_i2c_instance, data);
}

void JOY_OLED_data_start(uint8_t y)
{
    uint8_t setup_cmds[] = {0x00, 0x21, 0, 127, 0x22, y, y};
    i2c_master_send(&global_i2c_instance, SSD1306_ADDR, setup_cmds, sizeof(setup_cmds));

    i2c_start(&global_i2c_instance);
    i2c_send_addr(&global_i2c_instance, SSD1306_ADDR, false);
    i2c_send_data(&global_i2c_instance, 0x40);
}

static inline uint8_t JOY_act_pressed()
{
    return read_button(4);
}

static inline uint8_t JOY_act_released()
{
    return !JOY_act_pressed();
}

static inline uint8_t JOY_up_pressed()
{
#ifdef ROTATE_CONTROLS
    return read_button(2);
#else
    return read_button(0);
#endif
}

static inline uint8_t JOY_down_pressed()
{
#ifdef ROTATE_CONTROLS
    return read_button(3);
#else
    return read_button(1);
#endif
}

static inline uint8_t JOY_left_pressed()
{
#ifdef ROTATE_CONTROLS
    return read_button(1);
#else
    return read_button(2);
#endif
}

static inline uint8_t JOY_right_pressed()
{
#ifdef ROTATE_CONTROLS
    return read_button(0);
#else
    return read_button(3);
#endif
}

void JOY_sound(uint8_t freq, uint8_t dur)
{
    // produce sound
}

uint16_t rnval = 0xACE1;
uint16_t JOY_random(void)
{
    rnval = (rnval >> 0x01) ^ (-(rnval & 0x01) & 0xB400);
    return rnval;
}

#define DLY_ms JOY_DLY_ms
#define DLY_us JOY_DLY_us

void JOY_DLY_ms(uint32_t n)
{
    for(uint32_t i = 0; i < n; i++)
    {
        for(volatile int j = 0; j < 2500; j++)
            ;
    }
}

void JOY_DLY_us(uint32_t n)
{
    for(volatile int j = 0; j < (n * 2); j++)
        ;
}

// #define JOY_SLOWDOWN() JOY_DLY_us(600)

#define abs(n) ((n >= 0) ? (n) : (-(n)))

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#ifdef __cplusplus
};
#endif
