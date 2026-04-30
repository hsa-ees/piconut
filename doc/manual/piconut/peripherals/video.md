# Video
**Author: Konrad Armbrecht 2025, Martin Erichsen 2025, Beaurel Ingride Ngaleu 2025,\
Niklas Sirch 2026**

## Overview

The video module is split in two parts: a hardware and a software implementation. The hardware
implementation provides a wishbone peripheral and outputs a VGA signal to an external monitor. The
software implementation renders graphics in a framebuffer in main memory, which can be displayed in
the GUI on the host machine.

## Interface Definition

### Overview

The Video module serves as the display output for the Piconut. This module provides a set of registers that facilitate the control and management of graphic rendering within the system. The Video adapter contains several registers, which are described below.

While the implementation of hardware or software video does not necessarily require the use of every feature of each register, it is essential that all registers are defined. This ensures that developers have a clear understanding of the available components and their functionalities, providing security and flexibility when developing video applications for the Piconut system.


### Register offsets

The memory map for the Video control registers is shown in the table below. The Video memory map has been designed to require only 32-bit memory accesses.

| Offset | Name                    | Description                                                                                  |
| ------ | ----------------------- | -------------------------------------------------------------------------------------------- |
| 0x000  | CONTROL                 | Control register for the video interface [RW].                                               |
| 0x004  | STATUS                  | General status register. [RW implementation dependent]                                   |
| 0x008  | RESOLUTION_MODE         | Set value to choose resolution mode (bits select mode).  [RW]                                     |
| 0x00C  | RESOLUTION_MODE_SUPPORT | Bitmask of supported resolution modes (1 = supported, 0 = unsupported). [R]           |
| 0x010  | COLOR_MODE              | Set bits to choose color mode (bits-per-pixel). [RW]                                             |
| 0x014  | COLOR_MODE_SUPPORT      | Bitmask of supported color modes (1 = supported, 0 = unsupported). [R]               |
| 0x018  | SCANLINE                | Current scanline being rendered. [R] |
| 0x01C  | FRAMEBUFFER_WIDTH       | Framebuffer width in pixels. [R] | 
| 0x020  | FRAMEBUFFER_HEIGHT      | Framebuffer height in pixels. [R] | 
| 0x024  | COLOR_MAP               | 256 entries of 32-bit color values (mapped palette). [R]                   |
| 0x424  | FRAMEBUFFER             | base address to framebuffer memory in the Piconut system (framebuffer area follows). [RW] |

### Detailed register description
Note:
- Attr. RW = Read and write
- Attr. R = Read only

#### CONTROL Register
**Register offset**: 0x000
Can be used to control the graphic interface.

| Bit | Field name | Attr. | Description                  |
| ---- | ---------- | ----- | ---------------------------- |
| 0    | Interrupt  | RW    | Enable/disable Interrupt     |
| 1    | Output     | RW    | Enable/disable image output. |

#### STATUS Register
**Register offset**: 0x004
General status register for the graphic interface.

| Bit | Field name | Attr. | Description                                                           |
| ---- | ---------- | ----- | --------------------------------------------------------------------- |
| 0    | Ready      | R     | Indicates if the video interface is ready (0: not ready, 1: ready). |
| 1    | Error      | R     | Indicates if there is an error (0: no error, 1: error occurred).      |
| 2    | Busy       | R     | Indicates if the video interface is busy (0: idle, 1: busy).        |

#### RESOLUTION_MODE Register
**Register offset**: 0x008
Set register bit to choose respective resolution mode in pixel.

| Bit | Field name | Attr. | Description                     |
| ---- | ---------- | ----- | ------------------------------- |
| 0    | 640x480    | RW    | Resolution mode 640x480 pixel   |
| 1    | 800x600    | RW    | Resolution mode 800x600 pixel   |
| 2    | 1024x768   | RW    | Resolution mode 1024x768 pixel  |
| 3    | 1280x720   | RW    | Resolution mode 1280x720 pixel  |
| 4    | 1920x1080  | RW    | Resolution mode 1920x1080 pixel |
| 5    | 3x2        | RW    | Resolution mode 3x2 pixel       |
| 6    | 80x60      | RW    | Resolution mode 80x60 pixel     |
| 7    | 320x240    | RW    | Resolution mode 320x240 pixel   |

#### RESOLUTION_MODE_SUPPORT Register
**Register offset**: 0x00C
Read only. Stores supported resolution modes as a bitmask. Each bit represents a supported mode (1 = supported, 0 = unsupported).
Before requesting to set a certain resolution mode, check, if it is supported. E.g. if mode 0x10 is requested while only modes 0x0 (640x480) and 0x1 (800x600) are supported, an error will occur.

#### COLOR_MODE Register
**Register offset**: 0x010
Set register bit to choose respective color mode. For undefined color modes, it is the implementation’s responsibility to select appropriate entries from the COLOR_MAP.
COLOR_MAP: Address range 0x100 - 0x4FF contains 256 colors, with 4 bytes per entry.

| Bit | Field name       | Attr. | Description                                                                                                                                                                                                                                                |
| --- | ---------------- | ----- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0   | 1 Bit Monochrome | RW    | Black and White only                                                                                                                                                                                                                                       |
| 1   | 2-Bit Mapped     | RW    | 4 colors from the COLOR_MAP                                                                                                                                                                                                                                |
| 2   | 2-Bit Grayscale  | RW    | 4 levels of gray from the COLOR_MAP                                                                                                                                                                                                                        |
| 3   | 3-Bit Mapped     | RW    | 8 colors from the COLOR_MAP                                                                                                                                                                                                                                |
| 4   | 3-Bit RGB        | RW    | 1 bit per RGB channel: black, red, green, blue, cyan, magenta, yellow and white                                                                                                                                                                            |
| 5   | 3-Bit Grayscale  | RW    | 8 levels of gray from the COLOR_MAP                                                                                                                                                                                                                        |
| 6   | 4-Bit Mapped     | RW    | 16 colors from the COLOR_MAP                                                                                                                                                                                                                               |
| 7   | 4-Bit RGBI       | RW    | 4-bit RGBI is similar to the 3-bit RGB but adds one bit for dark/bright intensity, see CGA-standard                                                                                                                                                        |
| 8   | 4-Bit Grayscale  | RW    | 16 levels of gray from the COLOR_MAP                                                                                                                                                                                                                       |
| 9   | 8-Bit Mapped     | RW    | All 256 colors from COLOR_MAP                                                                                                                                                                                                                              |
| 10  | 8-Bit RGB332     | RW    | 3 bits red, 3 bits green, 2 bits blue                                                                                                                                                                                                                      |
| 11  | 8-Bit Grayscale  | RW    | 256 levels of gray: RGB(0, 0, 0) to RGB (255, 255 , 255)                                                                                                                                                                                                   |
| 12  | 16-Bit RGB565    | RW    | RGB565 Color mode with 16 bits per pixel: 5 bits red, 6 bits green, 5 bits blue.                                                                                                                                                                           |
| 13  | 32-Bit RGB888    | RW    | Color mode with 32 bits per pixel, uses 24-bit truecolor (RGB888) packed into a 32-bit format for memory alignment.<br>The upper 8 bits (typically alpha in RGBA) are not being used,<br>as transparency is not relevant in the context of direct image display. |


#### COLOR_MODE_SUPPORT Register
**Register offset**: 0x014
Read only. Stores supported color modes as a bitmask. Each bit represents a supported mode (1 = supported, 0 = unsupported).
Before requesting to set a certain color mode, check, if it is supported. E.g. when trying to set resolution mode 0x10, but only 0x0 (640x480) and 0x1(800x600) is defined, an error will occur.

#### SCANLINE Register
**Register offset**: 0x018
Read only. Provides the current scanline index being rendered.

#### COLOR_MAP Register
**Register offset**: 0x024
Read only. The COLOR_MAP register range (0x100 - 0x4FF) consists of 256 registers for color values. Each entry is a 32-bit value representing a color, with the bits allocated for the RGB color channels. The layout is as follows:

| 0xUURRGGBB | Description |
| ---------- | ----------- |
| UU         | Unused      |
| RR         | Red         |
| GG         | Green       |
| BB         | Blue        |

These registers are optional and depend on the supported color modes of the hardware. They are necessary, when using a mapped color mode.
Each entry in the COLOR_MAP is structured as a 32-bit value, where the bits are allocated for the RGB color channels. Specifically, the layout is as follows:

Example use:

| Hex (ARGB) | RGB Decimal | Color |
| ---------- | ----------- | ----- |
| 0x00FF0000 | (255, 0, 0) | Red   |
| 0x0000FF00 | (0, 255, 0) | Green |
| 0x000000FF | (0, 0, 255) | Blue  |

**Practical Access**  
The COLOR_MAP is used by setting the corresponding color index in color modes that map directly to the color table. For example, in a 2-bit color mode, you could access the first four colors with COLOR_MAP[index] as follows:

- `0 = Black` (0x00000000)
- `1 = White` (0x00FFFFFF)
- `2 = Red` (0x00FF0000)
- `3 = Green` (0x0000FF00)

In higher bit modes (e.g., 8-bit), you can access up to 256 colors from the COLOR_MAP. A specific color is referenced by its index, and the respective color value is retrieved from the corresponding COLOR_MAP entry.


#### FRAMEBUFFER Register
**Register offset**: 0x424
Pointer to framebuffer memory in the piconut system.

### Color Palettes

In color_palettes.h, several predefined color palettes are provided for different color modes. These
palettes can be used to initialize the COLOR_MAP register in the video peripheral.  Here is some
documentation for the available color palettes:

```{doxygenfile} color_palettes.h
:project: m_video
```

## Video Driver

There is a video driver available to interface with the video peripheral from software.

### Usage

For using the video modules in your Piconut project, look at the reference implementation in `sw/apps/video_spectrum/video_spectrum.c`.
Here is a brief overview of the steps needed to set up and use the video modules:

1. Build a PicoNut-System with a video modules (soft or hardware).
2. Use the video_driver to initialize the video module.
  ```c
  video_t video;

  // Initialize the driver instance
  if(video_init(&video, 0, RESOLUTION_MODE_640x480, COLOR_MODE_1_MONO) != VIDEO_OK)
  {
      printf("Error: Could not initialize video driver.\n");
      return -1;
  }
  ```
3. Use the video driver functions to manipulate the framebuffer and control the video output.
  ```c
  video_write_pixel(&video, x, y, color_index);
  ```

### Reference

```{doxygenfile} video_driver.h
:project: m_video
```


## Soft Graphics

```{doxygengroup} c_soft_graphics
:project: graphics_soft
```

### Registers

```{doxygenstruct} c_soft_graphics::regs_t
:project: graphics_soft
:members: true
```

```{doxygenenum} c_soft_graphics::e_regs
:project: graphics_soft
```

### Functions

```{doxygenfunction} c_soft_graphics
:project: graphics_soft
```

```{doxygenfunction} ~c_soft_graphics
:project: graphics_soft
```

```{doxygenfunction} flush_graphics
:project: graphics_soft
```

```{doxygenfunction} register_meip_callback
:project: graphics_soft
```

```{doxygenfunction} on_rising_edge_clock
:project: graphics_soft
```

```{doxygenfunction} clear_framebuffer
:project: graphics_soft
```

### Global config

Graphics-related settings in global config file `/hw/piconut/config.mk`

`CFG_GRAPHICS_BASE_ADDRESS = 0x90000000`

Defines the memory address of the graphics peripheral in the simulator address space.

## Hardware Video

The m_video peripheral offers an interface to external monitors.
Currently, output via a VGA compatible interface at an resolution of 640x480 is
supported. A wishbone interface gives access to control and status registers,
as well as access to the internal framebuffer. Following figure shows the
structure of this peripheral. It is designed around a streaming pipeline.
Video signals flow unidirectional from source (e.g.
[internal framebuffer](sec:m_video:modules:framebuffer_source)
) to sink modules (e.g.
[VGA signal generator](sec:m_video:modules:vga_color_generator)).
Controlled by the line/column counter, it outputs a continuous stream of pixels.

```{figure} ./figures/video/m_video_structure.svg
:name: m_video_structure
:width: 100%
:align: center
Structure of the m_video peripheral.
```

### Color Modes

This peripheral supports several color modes shown in the table below. The
[color translator](sec:m_video:modules:color_translator) module interprets
colors according to this table.

```{figure} ./figures/video/color_definition.svg
:name: color_definition
:width: 100%
:align: center
Table of color mode definitions.
```

The 4 bit RGBI mode is similar to the 3 bit RGB mode, with an additional bit
to select between two color intensities. Its 16 colors are shown in the figure
below.

```{figure} ./figures/video/rgbi_palette.svg
:name: rgbi_palette
:width: 30%
:align: center
4 bit RGBI color palette
```

### Unimplemented Features

- Currently, only a resolution of 640x480 is supported. Other resolution need different video-clock.
  We use the only resolution possible with 25MHz pixel clock. (although 640x480 is defined with 25.175 MHz it works)
- Interrupts are not implemented yet.
- Framebuffer memory is limited.
- Scaling for multiple resolutions is not implemented yet.
- Statuses other than "ready".


### Configuration

```{doxygengroup} m_video_config
:project: m_video
```

### Modules

(sec:m_video:modules:framebuffer_source)=

#### Framebuffer

```{doxygenfunction} SC_MODULE(m_framebuffer_source)
:project: m_video
```

```{doxygenfunction} SC_MODULE(m_framebuffer_ram)
:project: m_video
```

(sec:m_video:modules:color_translator)=

#### Color Translator

```{doxygenfunction} SC_MODULE(m_color_translator)
:project: m_video
```

(sec:m_video:modules:vga_color_generator)=

#### VGA Signal Generator

```{doxygenfunction} SC_MODULE(m_vga_color_generator)
:project: m_video
```

```{doxygenfunction} SC_MODULE(m_vga_sync_generator)
:project: m_video
```

#### VGA Timing Table

```{doxygenfunction} SC_MODULE(m_vga_timings)
:project: m_video
```

#### Wishbone Slave Interface

```{doxygenfunction} SC_MODULE(m_video_wb)
:project: m_video
```

#### m_video

```{doxygenfunction} SC_MODULE(m_video)
:project: m_video
```

(sec:video_hardware_peripheral:modules:wb_slave)=



