/**
 * @file gui.h
 * @brief Provides a Qt-based GUI window for the piconut simulator.
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Konrad Armbrecht <konrad.armbrecht@tha.de>
      Technische Hochschule Augsburg, University of Applied Sciences

  Description: This file contains the implementation of the gui for simulation ONLY

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
 * @class gui
 * @author Konrad Armbrecht
 *
 * This module provides a Qt-based GUI that is available to the piconut simulator.
 * Currently implemented are functions for the image output of the graphics peripheral.
 * The simulator terminates when the GUI window is closed.
 */

#ifndef GUI_H
#define GUI_H

#include "mainwindow.h"
#include <QApplication>

class gui
{

public:
    /**
     * @brief Constructor. Initializes the GUI and creates a MainWindow instance.
     */
    gui(int& argc, char** argv);


    /**
     * @brief Destructor.
     */
    virtual ~gui();

    /**
     * @brief Displays the image from the framebuffer in the GUI by instructing the
     * mainwindow class accordingly
     * @param framebuffer Pointer to the framebuffer data.
     * @param framebuffer_size Size of the framebuffer in pixel.
     *
     */
    void set_image(uint32_t* framebuffer, uint32_t framebuffer_size);

    /**
     * @brief Checks if the main window is currently open. Used for terminating
     * the simulation when closing the GUI.
     * @return True if the main window is open, otherwise false.
     */
    bool is_gui_window_open() const;

    void setActionCallback(const std::function<void(GUI_ACTION)> triggerCallback);

    void processEvents();

    void quit();

private:
    int argc = 0;
    char** argv = nullptr;
    QApplication* app;
    MainWindow* mwindow; ///< Pointer to the main window instance.
    void *callbackContext = nullptr;
};
#endif // GUI_H
