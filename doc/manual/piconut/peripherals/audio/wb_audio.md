# Audio hardware peripheral

**Author: Tristan Kundrat 2025**

## Overview

The wishbone audio hardware module allows audio signals to be generated (audio synthesizer).
The module is configured using simple [registers](sec:audio_hardware_peripheral:interface_specification:register-table).

The **[wb_audio](sec:audio_hardware_peripheral:modules:wb_audio)** top module consists of the following components:

- **[wb_audio_single](sec:audio_hardware_peripheral:modules:wb_audio_single)**: Module generating one audio voice.
- **[squarewave](sec:audio_hardware_peripheral:modules:squarewave)**: Module generating a squarewave audio signal.
- **[trianglewave](sec:audio_hardware_peripheral:modules:trianglewave)**: Module generating a trianglewave audio signal.
- **[sawtoothwave](sec:audio_hardware_peripheral:modules:sawtoothwave)**: Module generating a sawtoothwave audio signal.
- **[sinewave](sec:audio_hardware_peripheral:modules:sinewave)**: Module generating a sinewave audio signal.*
- **[counter](sec:audio_hardware_peripheral:modules:counter)**: Configurable counter module used by all wave generating modules.

*) **Hint:** Sinewave generation is currently not implemented.
   The module was kept to have a starting point for adding new waveform types.

## Interface specification

(sec:audio_hardware_peripheral:interface_specification:register-table)=

### Register table

This table describes the registers and their function:

| Byte Offset | Name | Type | Bit Offset (in Word) |
|---|---|---|---|
| 0x00 | [enable](sec:audio_hardware_peripheral:interface_specification:enable-register) | bool | 0 |
| | [waveform_type](sec:audio_hardware_peripheral:interface_specification:waveform-type-register) | uint8_t | 8 |
| | [step_height](sec:audio_hardware_peripheral:interface_specification:step-height-register) | uint16_t | 16 |
| 0x04 | [frequency_divisor](sec:audio_hardware_peripheral:interface_specification:frequency-divisor-register) | uint32_t | 0 |

### Register description

(sec:audio_hardware_peripheral:interface_specification:enable-register)=

### Enable register

**bool**

| Value | Function |
|---|---|
| 0 | Disable generation of audio signal (Audio output is 0) and reset internal counter to 0 |
| 1 | Enable generation of audio signal using register configuration |

(sec:audio_hardware_peripheral:interface_specification:waveform-type-register)=

### Waveform type register

**uint8_t**

The SystemC module uses the following struct for enumerating the waveform types:

```cpp
typedef enum
{
    WF_SQUARE = 0,
    WF_TRIANGLE,
    WF_SAWTOOTH,
    WF_SINE,
} e_waveform_t;
```

| Enum value | Integer value | Function |
|---|---|---|
| WF_SQUARE | 0 | Generate squarewave audio signal |
| WF_TRIANGLE | 1 | Generate trianglewave audio signal |
| WF_SAWTOOTH | 2 | Generate sawtoothwave audio signal |
| WF_SINE | 3 | Generate sinewave audio signal* |

*) **Hint:** Sinewave generation is currently not implemented.
   The module was kept to have a starting point for adding new waveform types.

(sec:audio_hardware_peripheral:interface_specification:step-height-register)=

### Step height register

**uint16_t**

The step height is calculated differently based on the waveform type.
The parameters mentioned are:

| Parameter | Mathematical symbol | Type |
|---|---|---|
| Amplitude* | {math}`A` | uint24_t |
| Audio signal frequency | {math}`f` | double |
| Audio signal period length | {math}`T` | double |
| System clock frequency | {math}`f_{clk}` | double |
| System clock period length | {math}`T_{clk}` | double |
| Frequency divisor | {math}`d` | uint32_t |
| Step height | {math}`s` | uint16_t |

*) **Hint:** The amplitude {math}`A` is a value between `0x000000` (minimum)
and `0xFFFFFF` (maximum)

#### Squarewave step height

```{figure} ./figures/audio/calc_square.svg
:name: calc_square
:width: 50%
:align: center
Diagramm of a squarewave showing parameters for calculating the step height.
```

{math}`d = \frac{T}{T_{clk}} = \frac{f_{clk}}{f}`

{math}`s = A`

#### Trianglewave step height

```{figure} ./figures/audio/calc_triangle.svg
:name: calc_triangle
:width: 50%
:align: center
Diagramm of a trianglewave showing parameters for calculating the step height.
```

{math}`d = \frac{T}{T_{clk}} = \frac{f_{clk}}{f}`

{math}`s = \frac{A \cdot 2}{d}`

#### Sawtoothwave step height

```{figure} ./figures/audio/calc_sawtooth.svg
:name: calc_sawtooth
:width: 50%
:align: center
Diagramm of a sawtoothwave showing parameters for calculating the step height.
```

{math}`d = \frac{T}{T_{clk}} = \frac{f_{clk}}{f}`

{math}`s = \frac{A}{d}`

#### Sinewave step height

Sinewave generation is currently not implemented.

(sec:audio_hardware_peripheral:interface_specification:frequency-divisor-register)=

### Frequency divisor register

**uint32_t**

This value is calculated using the following formula:
{math}`d = \frac{T}{T_{clk}} = \frac{f_{clk}}{f}`

Also see the chapter [Step height register](sec:audio_hardware_peripheral:interface_specification:step-height-register).

## Modules

(sec:audio_hardware_peripheral:modules:wb_audio)=

### wb_audio

```{figure} ./figures/audio/wb_audio.svg
:name: wb_audio
:width: 80%
:align: center
Diagramm of the `wb_audio` top module containing {math}`n` voices.
```

```{doxygenfunction} SC_MODULE(m_audio)
:project: wb_audio
```

(sec:audio_hardware_peripheral:modules:wb_audio_single)=

### wb_audio_single

```{doxygenfunction} SC_MODULE(m_audio_single)
:project: wb_audio
```

(sec:audio_hardware_peripheral:modules:squarewave)=

### squarewave

```{doxygenfunction} SC_MODULE(m_squarewave)
:project: wb_audio
```

(sec:audio_hardware_peripheral:modules:trianglewave)=

### trianglewave

```{doxygenfunction} SC_MODULE(m_trianglewave)
:project: wb_audio
```

(sec:audio_hardware_peripheral:modules:sawtoothwave)=

### sawtoothwave

```{doxygenfunction} SC_MODULE(m_sawtoothwave)
:project: wb_audio
```

(sec:audio_hardware_peripheral:modules:sinewave)=

### sinewave

```{doxygenfunction} SC_MODULE(m_sinewave)
:project: wb_audio
```

(sec:audio_hardware_peripheral:modules:counter)=

### counter

```{doxygenfunction} SC_MODULE(m_counter)
:project: wb_audio
```

## Example code

These functions can be used to calculate and write values to the registers,
where CLK_FREQ_HZ is the systems clock frequency in Hz and `regs` is the pointer
(address) of an audio voice `v`, where
`unsigned int *regs = (unsigned int *)(CFG_WB_AUDIO_BASE_ADDRESS + 0x08 * v)`:

```c
void set_trianglewave(double frequency, unsigned int amplitude, volatile unsigned int *regs)
{
    if (frequency < 0) return;
    if (frequency == 0) {
      regs[0] = 0;
      regs[1] = 0;
      return;
    }
    unsigned int freqdiv = (unsigned int)(CLK_FREQ_HZ / frequency);
    unsigned int step_height = (unsigned int)(amplitude * 2 / freqdiv);
    regs[1] = freqdiv;
    regs[0] = (step_height << 16) | (WF_TRIANGLE << 8) | 1;
}

void set_squarewave(double frequency, unsigned int amplitude, volatile unsigned int *regs)
{
    if (frequency < 0) return;
    if (frequency == 0) {
      regs[0] = 0;
      regs[1] = 0;
      return;
    }
    unsigned int freqdiv = (unsigned int)(CLK_FREQ_HZ / frequency);
    regs[1] = freqdiv;
    regs[0] = (amplitude << 16) | (WF_SQUARE << 8) | 1;
}

void set_sawtoothwave(double frequency, unsigned int amplitude, volatile unsigned int *regs)
{
    if (frequency < 0) return;
    if (frequency == 0) {
      regs[0] = 0;
      regs[1] = 0;
      return;
    }
    unsigned int freqdiv = (unsigned int)(CLK_FREQ_HZ / frequency);
    unsigned int step_height = (unsigned int)(amplitude / freqdiv);
    regs[1] = freqdiv;
    regs[0] = (step_height << 16) | (WF_SAWTOOTH << 8) | 1;
}
```

The following code is an example playing a 200 Hz Squarewave, 500 Hz Trianglewave
and 1500 Hz Sawtoothwave at full amplitude, then turning off:

```c
#define VOICE(NUM) ((unsigned int *)(CFG_WB_AUDIO_BASE_ADDRESS + 0x08 * NUM))
#define MAX_AMPLITUDE 0xFFFFFFU

// set_...wave functions here

int main(void)
{
   set_squarewave(200.0, MAX_AMPLITUDE, VOICE(0));
   set_trianglewave(500.0, MAX_AMPLITUDE, VOICE(1));
   set_sawtoothwave(1500.0, MAX_AMPLITUDE, VOICE(2));

   for (int i = 0; i < 100000; i++);

   // When disabling, any set_ method can be used with frequency 0
   set_squarewave(0.0, 0, VOICE(0));
   set_squarewave(0.0, 0, VOICE(1));
   set_squarewave(0.0, 0, VOICE(2));
   return 0;
}
