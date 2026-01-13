/**
 * @file framebuffer_to_image.h
 * @brief Converts the framebuffer content to a displayable Qimage
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Konrad Armbrecht <konrad.armbrecht@tha.de>
      Technische Hochschule Augsburg, University of Applied Sciences

  Description: This file contains the implementation of the framebuffer_to_image for simulation ONLY

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


#ifndef FRAMEBUFFERTOIMAGE_H
#define FRAMEBUFFERTOIMAGE_H
#include <QImage>
#include <QVector>

/**
 * @class framebuffer_to_image
 * @brief A class for converting framebuffer data into a Qt-compatible QImage.
 * @author Konrad Armbrecht
 *
 * This class is responsible for converting the frame buffer content into a Qt
 * displayable format. The class can be used in the simulation environment.
 * 
 * Within that class an empty QVector "qrgb_array" is created that will be filled
 * with Qt readable pixel information. To fill the QVector, the framebuffer content
 * is read address by address. The containing hex values are converted to QRgb values
 * and copied into the QVector. The QRgb values are then used to assemble the output
 * image, which is finally being returned.
 */
class framebuffer_to_image {
public:
    
    /**
     * @brief Constructor
     *
     * @param framebuffer Pointer to the framebuffer containing pixel data.
     * @param framebuffer_size_in_px Size of the framebuffer in bytes.
     */
    framebuffer_to_image(uint32_t *framebuffer, uint32_t framebuffer_size_in_px);

    /**
     * @brief Converts the raw framebuffer data into QRgb format and creates a QImage from it.
     *
     * @return QImage containing the processed framebuffer data.
     */
    QImage output_image();
    
private:
    
    /**
     * @brief Initializes a 2D pixel array.
     *
     * @param width Width of the framebuffer in pixels.
     * @param height Height of the framebuffer in pixels.
     */
    void create_pixel_array(uint16_t width, uint16_t height);

    /**
     * @brief Converts framebuffer data into a QRgb array.
     */
    void framebuffer_to_vector();

    /**
     * @brief Converts the QRgb array into a QImage.
     */
    void array_to_image();

    uint32_t* framebufferptr;           ///< Pointer to the framebuffer data.
    QVector<QVector<QRgb>> qrgb_array;  ///< 2D array storing converted QRgb values.
    QImage output_qimage;               ///< The final output image.
    uint32_t framebuffer_width;
    uint32_t framebuffer_height;
};

#endif // FRAMEBUFFERTOIMAGE_H
