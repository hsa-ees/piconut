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

#include "video_driver.h"

// Internal helper for register access
#define REG_WRITE(base, offset, val) (*(volatile uint32_t*)((base) + (offset)) = (val))
#define REG_READ(base, offset) (*(volatile uint32_t*)((base) + (offset)))

video_res_t video_init(video_t* self, uintptr_t base_addr, e_resolution_mode res_mode, e_color_mode color_mode)
{
    if(!base_addr)
    {
        base_addr = PN_CFG_VIDEO_BASE_ADDRESS;
    }
    self->base_addr = base_addr;

    uint32_t status = video_get_status(self);
    if(status)
    {
        return VIDEO_ERR_INVALID;
    }

    video_set_enable(self, true);

    video_res_t res = video_set_mode(self, res_mode, color_mode);
    if(res != VIDEO_OK)
    {
        return res;
    }

    return VIDEO_OK;
}

void video_set_enable(video_t* self, bool enable)
{
    uint32_t ctrl = REG_READ(self->base_addr, VIDEO_REG_CONTROL);
    if(enable)
    {
        ctrl |= 0x1;
    }
    else
    {
        ctrl &= ~0x1;
    }
    REG_WRITE(self->base_addr, VIDEO_REG_CONTROL, ctrl);
}

bool video_get_enabled(video_t* self)
{
    return (REG_READ(self->base_addr, VIDEO_REG_CONTROL) & 0x1);
}

void video_set_interrupts(video_t* self, bool enable)
{
    uint32_t ctrl = REG_READ(self->base_addr, VIDEO_REG_CONTROL);
    if(enable)
    {
        ctrl |= 0x2;
    }
    else
    {
        ctrl &= ~0x2;
    }
    REG_WRITE(self->base_addr, VIDEO_REG_CONTROL, ctrl);
}

bool video_get_interrupts(video_t* self)
{
    return (REG_READ(self->base_addr, VIDEO_REG_CONTROL) & 0x2);
}

uint32_t video_get_status(video_t* self)
{
    return REG_READ(self->base_addr, VIDEO_REG_STATUS);
}

video_res_t video_set_mode(video_t* self, e_resolution_mode res_mode, e_color_mode color_mode)
{
    // Check hardware support bitmasks
    uint32_t res_supp = video_get_supported_res_modes(self);
    uint32_t col_supp = video_get_supported_color_modes(self);

    if(!(res_supp & (1 << res_mode)) || !(col_supp & (1 << color_mode)))
    {
        return VIDEO_ERR_UNSUPPORTED;
    }

    // Write modes to hardware
    REG_WRITE(self->base_addr, VIDEO_REG_RESOLUTION_MODE, (uint32_t)res_mode);
    REG_WRITE(self->base_addr, VIDEO_REG_COLOR_MODE, (uint32_t)color_mode);

    /* Update cached mode values and framebuffer size */
    self->res_mode = res_mode;
    self->color_mode = color_mode;
    video_update_fb_size(self);

    return VIDEO_OK;
}

void video_update_modes(video_t* self)
{
    self->res_mode = (e_resolution_mode)REG_READ(self->base_addr, VIDEO_REG_RESOLUTION_MODE);
    self->color_mode = (e_color_mode)REG_READ(self->base_addr, VIDEO_REG_COLOR_MODE);
}

void video_update_fb_size(video_t* self)
{
    self->fb_width = REG_READ(self->base_addr, VIDEO_REG_FRAMEBUFFER_WIDTH);
    self->fb_height = REG_READ(self->base_addr, VIDEO_REG_FRAMEBUFFER_HEIGHT);
}

uint32_t video_get_current_scanline(video_t* self)
{
    return REG_READ(self->base_addr, VIDEO_REG_SCANLINE);
}

uint32_t video_get_supported_res_modes(video_t* self)
{
    return REG_READ(self->base_addr, VIDEO_REG_RESOLUTION_MODE_SUPPORT);
}

uint32_t video_get_supported_color_modes(video_t* self)
{
    return REG_READ(self->base_addr, VIDEO_REG_COLOR_MODE_SUPPORT);
}

uint32_t video_read_color_map(video_t* self, uint8_t index)
{
    return REG_READ(self->base_addr, VIDEO_REG_COLOR_MAP + (index * sizeof(uint32_t)));
}

volatile uint32_t* video_get_framebuffer(video_t* self)
{
    return (volatile uint32_t*)(self->base_addr + VIDEO_REG_FRAMEBUFFER);
}

video_res_t video_write_pixel(video_t* self, uint32_t x, uint32_t y, uint32_t color)
{
    if(x >= self->fb_width || y >= self->fb_height)
        return VIDEO_ERR_INVALID;

    volatile uint32_t* fb = video_get_framebuffer(self);
    fb[(y * self->fb_width) + x] = color;
    return VIDEO_OK;
}

uint32_t video_read_pixel(video_t* self, uint32_t x, uint32_t y)
{
    if(x >= self->fb_width || y >= self->fb_height)
        return 0;

    volatile uint32_t* fb = video_get_framebuffer(self);
    return fb[(y * self->fb_width) + x];
}