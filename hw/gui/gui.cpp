/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Konrad Armbrecht <konrad.armbrecht@tha.de>
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

#include <QApplication>
#include "gui.h"
#include "framebuffer_to_image.h"

gui::gui(int& argc, char** argv)
    : app{new QApplication(argc, argv)}
{
    app->setApplicationName("PicoNut Simulator");

    mwindow = new MainWindow();
    mwindow->show();
}

gui::~gui()
{
}

// Gets called from top_tb for framebuffer flush
void gui::set_image(uint32_t* framebuffer, uint32_t framebuffer_size)
{
    if(mwindow == nullptr)
    {
        return;
    }

    mwindow->image_to_gui(framebuffer, framebuffer_size);
}

void gui::setActionCallback(const std::function<void(GUI_ACTION)> triggerCallback) {
    mwindow->triggerCallback = triggerCallback;
}

void gui::processEvents()
{
    app->processEvents();
}

void gui::quit()
{
    mwindow->close();
    app->quit();
}

bool gui::is_gui_window_open() const
{

    if(mwindow == nullptr)
    {
        return false;
    }

    return mwindow->is_mainwindow_open();
}