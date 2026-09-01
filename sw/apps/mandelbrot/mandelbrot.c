/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2026 Daniel Sommerfeldt <daniel.sommerfeldt1@tha.de>
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

#include <stdio.h>

int main() {
    const int WIDTH = 382;
    const int HEIGHT = 100;
    const float WIDTH_F = 382.0f;
    const float HEIGHT_F = 100.0f;
    const int MAX_ITER = 200;
    const int TOTAL_FRAMES = 1; 

    // Target coordinates (Seahorse Valley area)
    const float TARGET_X =  -0.5f;
    const float TARGET_Y =  0.0f;

    // Terminal font aspect ratio tweak (Height / Width of a single character slot).
    // 2.0 works perfectly for most standard terminal fonts. 
    // If it still looks slightly tall or wide, tweak this to 1.8 or 2.2.
    const float FONT_ASPECT = 2.0f; 

    // 1. Set your desired vertical view span
    float y_span = 2.4f; 
    
    // 2. Calculate the X span automatically using the screen dimensions and font aspect ratio
    float x_span = y_span * (WIDTH_F / HEIGHT_F) / FONT_ASPECT; 

    // 3. Center the viewport boundaries perfectly around the target coordinates
    float x_min = TARGET_X - (x_span / 2.0f);
    float x_max = TARGET_X + (x_span / 2.0f);
    float y_min = TARGET_Y - (y_span / 2.0f);
    float y_max = TARGET_Y + (y_span / 2.0f);

    const float ZOOM_FACTOR = 0.93f;

    for (int frame = 0; frame < TOTAL_FRAMES; frame++) {
        float dx = (x_max - x_min) / WIDTH_F;
        float dy = (y_max - y_min) / HEIGHT_F;

        float ci = y_min;
        for (int y = 0; y < HEIGHT; y++) {
            float cr = x_min;
            for (int x = 0; x < WIDTH; x++) {
                float zr = 0.0f, zi = 0.0f;
                int iter = 0;

                while (zr * zr + zi * zi <= 4.0f && iter < MAX_ITER) {
                    float zr_sq = zr * zr;
                    float zi_sq = zi * zi;
                    zi = (zr + zr) * zi + ci;
                    zr = zr_sq - zi_sq + cr;
                    iter++;
                }

                if (iter == MAX_ITER) {
                    printf("\x1b[48;5;52m "); // Single space
                } else {
                    int color = 196 + (iter % 6); 
                    printf("\x1b[48;5;%dm ", color); // Single space
                }
                cr += dx;
            }
            printf("\x1b[0m\n");
            ci += dy;
        }

        // --- Zoom Logic ---
        x_span *= ZOOM_FACTOR;
        y_span *= ZOOM_FACTOR;

        x_min = TARGET_X - (x_span / 2.0f);
        x_max = TARGET_X + (x_span / 2.0f);
        y_min = TARGET_Y - (y_span / 2.0f);
        y_max = TARGET_Y + (y_span / 2.0f);
    }

    return 0;
}