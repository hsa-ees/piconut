/*************************************************************************

  This file is part of the PicoNut project.
  Copied from https://github.com/Gregwar/ASCII-Tetris

  Copyright (C) <2010-2015> Grégoire Passault
                2025 Niklas Sirch <niklas.sirch1@tha.de>
            Technische Hochschule Augsburg, University of Applied Sciences

  MIT License
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is furnished
  to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
 *************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "puzzle.h"

#include <uart_defs.h>

#define DISPLAY_BLANKS 22
#define SLEEP_FACTOR 2
#define INIT_SLEEP 10000
#define PRINT_INTERVAL 50
#define GRAVITY_INTERVAL 350

struct puzzle_level
{
    int score;
    int musec;
};

struct puzzle_block
{
    char data[5][5];
    int w;
    int h;
};

struct puzzle
{
    char** game;
    int w;
    int h;
    int level;
    int gameover;
    int score;
    struct puzzle_block current;
    int x;
    int y;
};

struct puzzle_block blocks[] =
    {
        {{"##",
             "##"},
            2,
            2},
        {{" X ",
             "XXX"},
            3,
            2},
        {{"@@@@"},
            4,
            1},
        {{"OO",
             "O ",
             "O "},
            2,
            3},
        {{"&&",
             " &",
             " &"},
            2,
            3},
        {{"ZZ ",
             " ZZ"},
            3,
            2}};

struct puzzle_level levels[] =
    {
        {0,
            1200 * SLEEP_FACTOR},
        {1500,
            900 * SLEEP_FACTOR},
        {8000,
            700 * SLEEP_FACTOR},
        {20000,
            500 * SLEEP_FACTOR},
        {40000,
            400 * SLEEP_FACTOR},
        {75000,
            300 * SLEEP_FACTOR},
        {100000,
            200 * SLEEP_FACTOR}};

#define PUZZLE_PIECES (sizeof(blocks) / sizeof(struct puzzle_block))
#define PUZZLE_LEVELS (sizeof(levels) / sizeof(struct puzzle_level))

void puzzle_init(struct puzzle* t, int w, int h)
{
    int x, y;
    t->level = 1;
    t->score = 0;
    t->gameover = 0;
    t->w = w;
    t->h = h;
    t->game = malloc(sizeof(char*) * w);
    for(x = 0; x < w; x++)
    {
        t->game[x] = malloc(sizeof(char) * h);
        for(y = 0; y < h; y++)
            t->game[x][y] = ' ';
    }
}

void puzzle_clean(struct puzzle* t)
{
    int x;
    for(x = 0; x < t->w; x++)
    {
        free(t->game[x]);
    }
    free(t->game);
}

void puzzle_print(struct puzzle* t)
{
    int x, y;
    for(x = 0; x < 30; x++)
        printf("\n");
    printf("[LEVEL: %d | SCORE: %d]\n", t->level, t->score);
    for(x = 0; x < 2 * t->w + 2; x++)
        printf("~");
    printf("\n");
    for(y = 0; y < t->h; y++)
    {
        printf("!");
        for(x = 0; x < t->w; x++)
        {
            if(x >= t->x && y >= t->y && x < (t->x + t->current.w) && y < (t->y + t->current.h) && t->current.data[y - t->y][x - t->x] != ' ')
                printf("%c ", t->current.data[y - t->y][x - t->x]);
            else
                printf("%c ", t->game[x][y]);
        }
        printf("!\n");
    }
    for(x = 0; x < 2 * t->w + 2; x++)
        printf("~");
    printf("\n");
    fflush(stdout);
}

int puzzle_hittest(struct puzzle* t)
{
    int x, y, X, Y;
    struct puzzle_block b = t->current;
    for(x = 0; x < b.w; x++)
        for(y = 0; y < b.h; y++)
        {
            X = t->x + x;
            Y = t->y + y;
            if(X < 0 || X >= t->w)
                return 1;
            if(b.data[y][x] != ' ')
            {
                if((Y >= t->h) ||
                    (X >= 0 && X < t->w && Y >= 0 && t->game[X][Y] != ' '))
                {
                    return 1;
                }
            }
        }
    return 0;
}

void puzzle_new_block(struct puzzle* t)
{
    t->current = blocks[random() % PUZZLE_PIECES];
    t->x = (t->w / 2) - (t->current.w / 2);
    t->y = 0;
    if(puzzle_hittest(t))
    {
        t->gameover = 1;
    }
}

void puzzle_print_block(struct puzzle* t)
{
    int x, y;
    struct puzzle_block b = t->current;
    for(x = 0; x < b.w; x++)
        for(y = 0; y < b.h; y++)
        {
            if(b.data[y][x] != ' ')
                t->game[t->x + x][t->y + y] = b.data[y][x];
        }
}

void puzzle_rotate(struct puzzle* t)
{
    struct puzzle_block b = t->current;
    struct puzzle_block s = b;
    int x, y;
    b.w = s.h;
    b.h = s.w;
    for(x = 0; x < s.w; x++)
        for(y = 0; y < s.h; y++)
        {
            b.data[x][y] = s.data[s.h - y - 1][x];
        }
    x = t->x;
    y = t->y;
    t->x -= (b.w - s.w) / 2;
    t->y -= (b.h - s.h) / 2;
    t->current = b;
    if(puzzle_hittest(t))
    {
        t->current = s;
        t->x = x;
        t->y = y;
    }
}

void puzzle_gravity(struct puzzle* t)
{
    t->y++;
    if(puzzle_hittest(t))
    {
        t->y--;
        puzzle_print_block(t);
        puzzle_new_block(t);
    }
}

void puzzle_fall(struct puzzle* t, int l)
{
    int x, y;
    for(y = l; y > 0; y--)
    {
        for(x = 0; x < t->w; x++)
            t->game[x][y] = t->game[x][y - 1];
    }
    for(x = 0; x < t->w; x++)
        t->game[x][0] = ' ';
}

void puzzle_check_lines(struct puzzle* t)
{
    int x, y, l;
    int p = 100;
    for(y = t->h - 1; y >= 0; y--)
    {
        l = 1;
        for(x = 0; x < t->w && l; x++)
        {
            if(t->game[x][y] == ' ')
            {
                l = 0;
            }
        }
        if(l)
        {
            t->score += p;
            p *= 2;
            puzzle_fall(t, y);
            y++;
        }
    }
}

int puzzle_level(struct puzzle* t)
{
    int i;
    for(i = 0; i < PUZZLE_LEVELS; i++)
    {
        if(t->score >= levels[i].score)
        {
            t->level = i + 1;
        }
        else
            break;
    }
    return levels[t->level - 1].musec;
}

void nanosleep(int32_t sleep_time_mus)
{
    for(int i = 0; i < sleep_time_mus; i++)
    {
        asm volatile("nop"); // Busy-wait loop for nanosleep
    }
}

void puzzle_run(int w, int h, int seed)
{
    setvbuf(stdout, NULL, _IOFBF, 0);
    srand(seed);

    struct puzzle t;
    puzzle_init(&t, w, h);
    int sleep_time_mus = INIT_SLEEP;
    int count = 0;
    puzzle_new_block(&t);
    while(!t.gameover)
    {
        count++;
        if(count % PRINT_INTERVAL == 0)
        {
            puzzle_print(&t);
        }
        if(count % GRAVITY_INTERVAL == 0)
        {
            puzzle_gravity(&t);
            puzzle_check_lines(&t);
        }

        volatile uint32_t* uart_rxdata = (uint32_t*)(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_RXDATA);
        uint32_t last_uart_rxdata;
        while((((last_uart_rxdata = *uart_rxdata) >> 31) & 0x1) == 0)
        {
            char cmd = last_uart_rxdata & 0xFF;
            switch(cmd)
            {
                case 'a':
                    t.x--;
                    if(puzzle_hittest(&t))
                        t.x++;
                    break;
                case 'd':
                    t.x++;
                    if(puzzle_hittest(&t))
                        t.x--;
                    break;
                case 's':
                    puzzle_gravity(&t);
                    break;
                case ' ':
                    puzzle_rotate(&t);
                    break;
            }
        }
        sleep_time_mus = puzzle_level(&t);
        nanosleep(sleep_time_mus);
    }

    puzzle_print(&t);
    printf("*** GAME OVER ***\n");

    puzzle_clean(&t);
    setvbuf(stdout, NULL, _IOLBF, 0);
}