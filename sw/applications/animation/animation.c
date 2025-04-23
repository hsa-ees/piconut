/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)   2023 Lukas Bauer <lukas.bauer@hs-augsburg.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This is a appliacation demo to execute a never ending app.

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
#include <stdlib.h> // for rand(), srand()
#include <string.h> // for memset()

#define WIDTH 50
#define HEIGHT 5

// star demo
int star_density = 75; // Control the density of asteroids (1 = max, higher = less)

// person demo
int position = 0;
int direction = 1; // 1 for right, -1 for left
int frame = 0;
int facingRight = 1; // 1 for facing right, 0 for left

// Variables for probabilities
int waveProbability = 20; // Probability of waving (1 in 20)
int turnProbability = 15; // Probability of turning (1 in 15)

// Selection
int selection = 1; // 0 for star, 1 for person

void galaxy()
{
    for(int i = 0; i < HEIGHT; ++i)
    {
        for(int j = 0; j < WIDTH; ++j)
        {
            // Print an asteroid or space based on random chance
            if(rand() % star_density == 0)
            {
                printf("*");
            }
            else
            {
                printf(" ");
            }
        }
        printf("\n");

        // Wait to make the animation visible
        // pn_usleep(100000); // 100000 microseconds = 0.1 second
    }
}

char buffer[HEIGHT][WIDTH + 1]; // Buffer with an extra space for null-termination

// Function to clear the buffer
void clearBuffer()
{
    for(int i = 0; i < HEIGHT; ++i)
    {
        memset(buffer[i], ' ', WIDTH);
        buffer[i][WIDTH] = '\0'; // Null-terminate each string
    }
}

// Function to print the buffer
void printBuffer()
{
    for(int i = 0; i < HEIGHT; ++i)
    {
        printf("%s\n", buffer[i]);
    }
}

// Function to add a frame of the walking person to the buffer
void addPerson(int position, int frame, int facingRight)
{
    const char* rightFrames[2][3] = {
        {" O ", "/|\\", "/ \\"},
        {" O ", "/|\\", " | "}};
    const char* leftFrames[2][3] = {
        {" O ", "/|\\", "/ \\"},
        {" O ", "/|\\", " | "}};
    const char* waveFrame[3] = {" O/", "/| ", "/ \\"};

    for(int i = 0; i < 3; ++i)
    { // The person is 3 lines tall
        for(int j = 0; j < 3; ++j)
        { // The person is 3 characters wide
            int bufferIndex = position + j;
            if(bufferIndex >= 0 && bufferIndex < WIDTH)
            {
                if(frame == 2)
                { // Waving frame
                    buffer[i][bufferIndex] = waveFrame[i][j];
                }
                else
                { // Walking frames
                    buffer[i][bufferIndex] = facingRight ? rightFrames[frame][i][j] : leftFrames[frame][i][j];
                }
            }
        }
    }
}

void addGround()
{
    for(int i = 0; i < WIDTH; ++i)
    {
        buffer[HEIGHT - 2][i] = '=';
    }
}

void person_animation()
{
    clearBuffer();

    // Randomly decide to wave or walk
    if(rand() % waveProbability == 0)
    {
        frame = 2; // Waving frame
    }
    else
    {
        frame = (frame + 1) % 2; // Walking frames
    }

    // Randomly decide to turn
    if(rand() % turnProbability == 0)
    {
        direction *= -1;
        facingRight = !facingRight;
    }

    addPerson(position, frame, facingRight);
    addGround();
    printBuffer();

    // Update position only if not waving and ensure the figure stays within the screen bounds
    if(frame != 2)
    {
        position += direction;
        if(position < 0)
        {
            position = 0;
            direction = 1;
            facingRight = 1;
        }
        else if(position > WIDTH - 4)
        { // Account for the width of the person
            position = WIDTH - 4;
            direction = -1;
            facingRight = 0;
        }
    }
}

int main()
{
    srand(146587);

    while(1)
    {

        if(selection == 0)
        {
            galaxy();
        }
        else
        {
            person_animation();
        }
    }

    return 0;
}