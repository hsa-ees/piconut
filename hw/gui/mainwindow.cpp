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

#include <piconut-config.h>
#include <iostream>

#include "mainwindow.h"
#include "mainwindow_ui.h"
#include "framebuffer_to_image.h"

#include <QLabel>
#include <QImage>
#include <QVBoxLayout>
#include <QWidget>
#include <QDir>

bool initial_resized = false;

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , output_image{1000, 1000, QImage::Format_RGB32}
{
    ui->setupUi(this);

    QMenu *settingsMenu = menuBar()->addMenu(tr("&Settings"));
    QAction *audioAction = settingsMenu->addAction("&Audio");

    connect(audioAction, &QAction::triggered, [this](){
        if(!this->triggerCallback) {
            return;
        }
        this->triggerCallback(SETTINGS_AUDIO); 
    });
}

MainWindow::~MainWindow()
{
}

void MainWindow::image_to_gui(uint32_t* framebuffer, uint32_t framebuffer_size)
{
    // Image scales with the size of the QLabel (when the window size is changed)
    ui->image_label->setScaledContents(true);

    if(framebuffer == nullptr)
    {
        qWarning() << "Framebuffer is null!";
        return;
    }

    // Convert framebuffer to output image
    framebuffer_to_image fb_to_image(framebuffer, framebuffer_size);
    output_image = fb_to_image.output_image();
    
    ui->image_label->setMinimumSize(output_image.width(), output_image.height());

    if(output_image.isNull())
    {
        qWarning() << "Output image is null!";
        return;
    }

    // Convert image to pixmap in order to display image
    ui->image_label->setPixmap(QPixmap::fromImage(output_image));

    //adapt MainWindow size to 800 x 573 as "default" size
    if(!initial_resized)
    {
        ui->image_label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        ui->image_label->resize(900, 500);
        //resize(ui->image_label->sizeHint());
        initial_resized = true;
    }

    setWindowTitle(QString("Image resolution: %1x%2 px | Label size: %3x%4 px")
                       .arg(output_image.width())
                       .arg(output_image.height())
                       .arg(ui->image_label->width())
                       .arg(ui->image_label->height()));
}

// Override resize event handler to maintain the aspect ratio of the window and its contents
void MainWindow::resizeEvent(QResizeEvent* event)
{
    QSize label_size = ui->image_label->size();
    int label_pixel_count = label_size.width() * label_size.height();
    QSize newSize = event->size();

    // Force the aspect ratio depending on the resolution
    int newWidth = newSize.width();
    int newHeight = static_cast<int>(newWidth * output_image.height() / static_cast<float>(output_image.width()));
    resize(newWidth, newHeight);

    QMainWindow::resizeEvent(event);

    // setWindowTitle(QString("Image resolution: %1x%2 px | Label size: %3x%4 px")
    //                    .arg(output_image.width())
    //                    .arg(output_image.height())
    //                    .arg(ui->image_label->width())
    //                    .arg(ui->image_label->height()));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    mainwindow_open = false;
    event->accept();
}


bool MainWindow::is_mainwindow_open() const
{
    return mainwindow_open;
}
