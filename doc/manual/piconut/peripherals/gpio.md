# GPIO
**Author: Daniel Sommerfeldt 2026, Johannes Hofmann 2026**

## Module

```{doxygenclass} m_gpio
:project: m_gpio
```


## Configuration and Registers

```{doxygengroup} gpio_defs
:project: m_gpio
:content-only:
```


## Driver

```{doxygengroup} gpio_driver
:project: m_gpio
:content-only:
:members:
```


## Examples

### Adding the GPIO Module to a toplevel design

```c++
// === top.h ===
#include <gpio.h>
// ...
SC_MODULE(m_top)
{
public:
    sc_in<GPIO_MAX_PINS> PN_NAME(gpio_input);
    sc_out<GPIO_MAX_PINS> PN_NAME(gpio_output);
// ...
m_gpio* gpio;
// ...
};
```
```c++
# === top.cpp
// ...
void m_top::init_submodules()
{
// ...
    // GPIO module with 8 input pins and 8 output pins
    gpio = sc_new<m_gpio>("i_gpio", PN_CFG_GPIO_BASE_ADDRESS, 8, 8);

    gpio->reset(reset);
    gpio->clk(clk);

    gpio->input(gpio_input);
    gpio->output(gpio_output);

    pn_interconnect->add_module(gpio);
// ...
    pn_interconnect->elaborate();
}
// ...
```


### Turn LED on

```c++
#include <gpio_driver.h>

#define LED0 0

gpio_t gpio;
gpio_init(&gpio, PN_CFG_GPIO_BASE_ADDRESS);

gpio_write_output_pin(&gpio, LED0, 1);
```

