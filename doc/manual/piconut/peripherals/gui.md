# GUI
**Author: Konrad Armbrecht 2025**

The GUI adds a user-friendly interface to the PicoNut simulator. While previously only the output of text via the UART interface was possible, the GUI now also allows the display of graphics and the output of audio signals. To achieve this, the GUI reads directly from the peripheral registers and converts the information into a Qt-compatible format in order to output it.

Thanks to its modular architecture, further functions can be added to the GUI, for example, to capture user input. In the long term, this opens up the possibility of running interactive applications such as computer games directly on the PicoNut.

```{figure} ./figures/gui/GUI_screenshot.png
:name: GUI Screenshot
:width: 75%
:align: center
GUI window that displays the running graphics_fancy_random_squares program.
```

## GUI class

```{doxygenclass} gui
:project: gui
```

### Functions

```{doxygenfunction} set_image
:project: gui
```

```{doxygenfunction} is_gui_window_open
:project: gui
```

### Call sequence and data flow of a new image frame

Every write access to the framebuffer triggers a flush operation. This means that the `flush_to_gui` callback function in the graphics class is called automatically upon each write.

This initiates a chain of function calls:  
`top_tb::flush_framebuffer -> gui::set_image -> mainwindow::image_to_gui -> framebuffer_to_image::output_image`.

The class `framebuffer_to_image` is responsible for converting the framebuffer data into a Qt-compatible image format. For this purpose, the color information of each pixel is read from the framebuffer in hex format and converted into the `QRgb` format. The converted values are stored in a two-dimensional `QVector<QVector<QRgb>>`, which enables efficient processing and subsequent image assembly. The pixel values are then assembled into a `QImage`, which is returned after the `output_image` method is executed.

The output image, returned by the `framebuffer_to_image` class, is converted by the `mainwindow` class to a `QPixmap` and sent to the label that is part of the GUI and displays the graphics framebuffer content for the user.

## Framebuffer to image class

```{doxygenclass} framebuffer_to_image
:project: gui
```

### Functions

```{doxygenfunction} output_image
:project: gui
```

## Mainwindow class

```{doxygenclass} mainwindow
:project: gui
```

### Functions

```{doxygenfunction} MainWindow
:project: gui
```

```{doxygenfunction} image_to_gui
:project: gui
```

```{doxygenfunction} is_mainwindow_open
:project: gui
```

## Audio playback
TODO