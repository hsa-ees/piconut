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

#include "framebuffer_to_image.h"
#include <QDebug>
#include <QRgb>
#include <cstdint>
#include <piconut-config.h>
#include <iostream>

framebuffer_to_image::framebuffer_to_image(uint32_t *framebuffer, uint32_t framebuffer_size_in_px)
{
    // Values hardcoded, because current implementation of graphics only supports these resolution modes
    framebufferptr = framebuffer;
    if(framebuffer_size_in_px == 6)
    {
        framebuffer_width = 3;
        framebuffer_height = 2;
    }
    else if (framebuffer_size_in_px == 4800)
    {
        framebuffer_width = 80;
        framebuffer_height = 60;
    }
    else
    {
        qWarning() << "framebuffer_to_image: framebuffer size not supported. Supported sizes are 4800 and 6";
    }
    
    output_qimage = QImage(framebuffer_width, framebuffer_height, QImage::Format_RGB32);
    this->create_pixel_array(framebuffer_width, framebuffer_height);
}

void framebuffer_to_image::create_pixel_array(uint16_t width, uint16_t height)
{
    //Colordepth will have no effect for now. Colordepth is always 24 bit, pixel colors can be all possible hex values
    //Create 2D-array with default color-value 1 (white) for all pixel
    qrgb_array = QVector<QVector<QRgb>>(height, QVector<QRgb>(width, 1));
}

void framebuffer_to_image::framebuffer_to_vector()
{

    if(framebufferptr == nullptr)
    {
        qDebug() << "framebuffer_to_image.cpp: framebuffer is empty";
        return;
    }
    // Iterate through the array and convert the values
    for (int y = 0; y < framebuffer_height; ++y) {
        for (int x = 0; x < framebuffer_width; ++x) {
            // Get the current pixel value from the one-dimensional framebuffer array
            uint32_t hexValue = framebufferptr[y * framebuffer_width + x];

            // Split hex value into red, green and blue components
            int red = (hexValue >> 16) & 0xFF;  // Upper 8 Bits
            int green = (hexValue >> 8) & 0xFF; // Middle 8 Bits
            int blue = hexValue & 0xFF;         // Lower 8 Bits

            // Convert hex value to QRgb
            QRgb qRgbValue = qRgb(red, green, blue);

            // Insert into the two-dimensional QVector
            qrgb_array[y][x] = qRgbValue;
        }
    }
}

void framebuffer_to_image::array_to_image()
{
    for (int y = 0; y < framebuffer_height; ++y) {
        for (int x = 0; x < framebuffer_width; ++x) {

            // Set pixel color from given array position
            output_qimage.setPixel(x, y, qrgb_array[y][x]);
        }
    }
}

QImage framebuffer_to_image::output_image()
{
    framebuffer_to_vector();
    array_to_image();
    return output_qimage;
}