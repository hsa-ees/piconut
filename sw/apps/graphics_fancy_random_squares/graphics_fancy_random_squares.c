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

// Execute this file with 'PN_SYSTEM=refdesign_c_soft_graphics make sim'

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>

// Function to create a randomly sized and colored square for the framebuffer
uint32_t* create_square(int width, int height, int total_pixels)
{
    uint32_t* pixel_array = (uint32_t*)malloc(total_pixels * sizeof(uint32_t));
    if (!pixel_array) {
        perror("Failed to allocate memory for framebuffer");
        return NULL;
    }

    // Initialize the array with black (set all pixels to black)
    for (int i = 0; i < total_pixels; ++i) {
        pixel_array[i] = 0x00000000;  // Black
    }

    // Calculate a random square size between 2x2 and heightxheight
    int square_size = rand() % (height - 1) + 2;  // Random size between 2 and height

    // Calculate a random position for the square
    int start_x = rand() % (width - square_size);  // Random x-coordinate
    int start_y = rand() % (height - square_size); // Random y-coordinate

    // Generate a random color value (32-bit ARGB)
    uint32_t random_color = (rand() % 256) << 16 |    // Red value (R)
                            (rand() % 256) << 8  |    // Green value (G)
                            (rand() % 256);           // Blue value (B)

    // Set the pixels of the square to the random color
    for (int y = start_y; y < start_y + square_size; ++y) {
        for (int x = start_x; x < start_x + square_size; ++x) {
            if (x < width && y < height) {  // Ensure the square is not written outside the array
                pixel_array[y * width + x] = random_color; // Set the pixel to the random color
            }
        }
    }

    return pixel_array;
}

int main() {

    printf("######### Starting graphics_fancy_random_squares #########\n\n");

    // Initialize the random number generator
    srand(6758);

    // Pointer to the graphics module
    volatile uint32_t* graphics = (uint32_t*)0x40000000;

    // Read the framebuffer size
    int width = *(graphics + 0x1);
    int height = *(graphics + 0x2);
    int framebuffersize = width * height;

    // Is intended as a while loop, but implemented as a for loop for Jenkins test
    for (int i = 0; i < 10; ++i) 
    {
        // Create a randomly sized and colored square for the framebuffer
        uint32_t* square = create_square(width, height, framebuffersize);

        // Write array, filled with the square, to the graphics framebuffer
        for (int i = 0; i < framebuffersize; i++) {
            *(graphics + 0x7 + i) = square[i];
        }

        // Free the memory
        free(square);
    }

    printf("\n######### Terminating graphics_fancy_random_squares #########\n\n");

    return 0;
}

