/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2026 Johannes Hofmann <johannes.hofmann1@tha.de>
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
#include "gpio_driver.h"
#include "../gpio_defs.h"

#include <stdio.h>

// Internal helper for register access
#define REG_WRITE(base, offset, val) (*(volatile uint32_t*)((base) + (offset)) = (val))
#define REG_READ(base, offset) (*(volatile uint32_t*)((base) + (offset)))

gpio_status_t gpio_init(gpio_t* self, uintptr_t base_addr)
{
    if(!self)
    {
        return GPIO_ERR_INVALID;
    }
    if(!base_addr)
    {
        base_addr = PN_CFG_GPIO_BASE_ADDRESS;
    }

    self->base_addr = base_addr;

    REG_WRITE(self->base_addr, GPIO_REG_INPUT_EN, 0xFFFFFFFF);
    REG_WRITE(self->base_addr, GPIO_REG_OUTPUT_EN, 0xFFFFFFFF);

    return GPIO_OK;
}

gpio_status_t gpio_de_init(gpio_t* self)
{
    if(!self)
    {
        return GPIO_ERR_INVALID;
    }

    REG_WRITE(self->base_addr, GPIO_REG_INPUT_EN, 0);
    REG_WRITE(self->base_addr, GPIO_REG_OUTPUT_EN, 0);

    return GPIO_OK;
}

gpio_status_t gpio_read_input(gpio_t* self, uint32_t* value)
{
    if(!self)
    {
        return GPIO_ERR_INVALID;
    }
    if(!value)
    {
        return GPIO_ERR_INVALID;
    }

    *value = REG_READ(self->base_addr, GPIO_REG_INPUT_VAL);

    return GPIO_OK;
}

gpio_status_t gpio_read_input_pin(gpio_t* self, uint8_t pin, bool* value)
{
    if(!self)
    {
        return GPIO_ERR_INVALID;
    }
    if(!value)
    {
        return GPIO_ERR_INVALID;
    }

    uint32_t value_bank = 0;
    gpio_status_t status = gpio_read_input(self, &value_bank);
    if(status != GPIO_OK)
    {
        return status;
    }

    *value = (bool)((value_bank >> pin) & 1);

    return GPIO_OK;
}

gpio_status_t gpio_write_output(gpio_t* self, uint32_t value)
{
    if(!self)
    {
        return GPIO_ERR_INVALID;
    }

    REG_WRITE(self->base_addr, GPIO_REG_OUTPUT_VAL, value);

    return GPIO_OK;
}

gpio_status_t gpio_write_output_pin(gpio_t* self, uint8_t pin, bool value)
{
    if(!self)
    {
        return GPIO_ERR_INVALID;
    }

    uint32_t mask = (uint32_t)(1 << pin);
    REG_WRITE(self->base_addr, GPIO_REG_OUTPUT_VAL, //
        (REG_READ(self->base_addr, GPIO_REG_OUTPUT_VAL) & ~mask) | (value << pin));

    return GPIO_OK;
}
