/*************************************************************************

  This file is part of the PicoNut project.
  Copyright (C)     2025 Eikya Lagisetti <eikya.lagisetti1@tha.de>
                    2025 Sebastian Ebenhöh <sebastian.moritz.ebenhoeh@tha.de>
                    2025 Johannes Fleiner <Johannes.Fleiner1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This application is a demo. It shows "PicoNut" in colorised ascii art.

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
 * @file demo_i2c.c
 * @brief I2C demo application using SSD1306 display and BME280 sensor.
 *
 * This demo application initializes the I2C peripheral and demonstrates
 * communication with an SSD1306 OLED display and a BME280 temperature sensor.
 * The current temperature is periodically read from the sensor and displayed
 * on the OLED using a simple 5x8 pixel font.
 *
 * The application is intended as a functional demonstration of the PicoNut
 * I2C driver and peripheral integration.
 */

 #include <stdio.h>
 #include <stdint.h>
 #include <i2c_driver.h>
 
 #define BME_ADDR 0x76
 #define SSD1306_ADDR 0x3C
 #define TEMP_INVALID 0x80000
 
 int16_t dig_T1 = 28463;
 int16_t dig_T2 = 26700;
 int16_t dig_T3 = 50;
 int32_t t_fine;
 
 /**
  * @brief Returns the 5x8 pixel columns for a given ASCII character. Unsupported 
  * characters are returned as blank.
  *
  * @param[in]  c    ASCII character to render.
  * @param[out] buf  Pointer to a 5-byte buffer receiving the glyph columns.
  */
 void get_char_pixels(char c, uint8_t* buf)
 {
     switch(c)
     {
         case 'P':
             buf[0] = 0x7F;
             buf[1] = 0x09;
             buf[2] = 0x09;
             buf[3] = 0x09;
             buf[4] = 0x06;
             break;
         case 'i':
             buf[0] = 0x00;
             buf[1] = 0x44;
             buf[2] = 0x7D;
             buf[3] = 0x40;
             buf[4] = 0x00;
             break;
 
         case 'c':
             buf[0] = 0x38;
             buf[1] = 0x44;
             buf[2] = 0x44;
             buf[3] = 0x44;
             buf[4] = 0x20;
             break;
 
         case 'C':
             buf[0] = 0x3E;
             buf[1] = 0x41;
             buf[2] = 0x41;
             buf[3] = 0x41;
             buf[4] = 0x22;
             break;
 
         case 'o':
             buf[0] = 0x38;
             buf[1] = 0x44;
             buf[2] = 0x44;
             buf[3] = 0x44;
             buf[4] = 0x38;
             break;
 
         case 'N':
             buf[0] = 0x7F;
             buf[1] = 0x04;
             buf[2] = 0x08;
             buf[3] = 0x10;
             buf[4] = 0x7F;
             break;
 
         case 'u':
             buf[0] = 0x3C;
             buf[1] = 0x40;
             buf[2] = 0x40;
             buf[3] = 0x20;
             buf[4] = 0x7C;
             break;
 
         case 't':
             buf[0] = 0x04;
             buf[1] = 0x3F;
             buf[2] = 0x44;
             buf[3] = 0x40;
             buf[4] = 0x20;
             break;
 
         case 'T':
             buf[0] = 0x01;
             buf[1] = 0x01;
             buf[2] = 0x7F;
             buf[3] = 0x01;
             buf[4] = 0x01;
             break;
 
         case 'e':
             buf[0] = 0x38;
             buf[1] = 0x54;
             buf[2] = 0x54;
             buf[3] = 0x54;
             buf[4] = 0x18;
             break;
 
         case 'm':
             buf[0] = 0x7C;
             buf[1] = 0x04;
             buf[2] = 0x18;
             buf[3] = 0x04;
             buf[4] = 0x78;
             break;
 
         case 'p':
             buf[0] = 0x7C;
             buf[1] = 0x14;
             buf[2] = 0x14;
             buf[3] = 0x14;
             buf[4] = 0x08;
             break;
 
         case 'r':
             buf[0] = 0x7C;
             buf[1] = 0x08;
             buf[2] = 0x04;
             buf[3] = 0x04;
             buf[4] = 0x08;
             break;
 
         case 'a':
             buf[0] = 0x20;
             buf[1] = 0x54;
             buf[2] = 0x54;
             buf[3] = 0x54;
             buf[4] = 0x78;
             break;
 
         case ':':
             buf[0] = 0x00;
             buf[1] = 0x36;
             buf[2] = 0x36;
             buf[3] = 0x00;
             buf[4] = 0x00;
             break;
 
         case '.':
             buf[0] = 0x00;
             buf[1] = 0x60;
             buf[2] = 0x60;
             buf[3] = 0x00;
             buf[4] = 0x00;
             break;
 
         case '^':   
             buf[0] = 0x06;
             buf[1] = 0x09;
             buf[2] = 0x09;
             buf[3] = 0x06;
             buf[4] = 0x00;
             break;
 
         case '-':
             buf[0] = 0x08;
             buf[1] = 0x08;
             buf[2] = 0x08;
             buf[3] = 0x08;
             buf[4] = 0x08;
             break;
 
         case '0':
             buf[0] = 0x3E;
             buf[1] = 0x51;
             buf[2] = 0x49;
             buf[3] = 0x45;
             buf[4] = 0x3E;
             break;
 
         case '1':
             buf[0] = 0x00;
             buf[1] = 0x42;
             buf[2] = 0x7F;
             buf[3] = 0x40;
             buf[4] = 0x00;
             break;
 
         case '2':
             buf[0] = 0x42;
             buf[1] = 0x61;
             buf[2] = 0x51;
             buf[3] = 0x49;
             buf[4] = 0x46;
             break;
 
         case '3':
             buf[0] = 0x21;
             buf[1] = 0x41;
             buf[2] = 0x45;
             buf[3] = 0x4B;
             buf[4] = 0x31;
             break;
 
         case '4':
             buf[0] = 0x18;
             buf[1] = 0x14;
             buf[2] = 0x12;
             buf[3] = 0x7F;
             buf[4] = 0x10;
             break;
 
         case '5':
             buf[0] = 0x27;
             buf[1] = 0x45;
             buf[2] = 0x45;
             buf[3] = 0x45;
             buf[4] = 0x39;
             break;
 
         case '6':
             buf[0] = 0x3C;
             buf[1] = 0x4A;
             buf[2] = 0x49;
             buf[3] = 0x49;
             buf[4] = 0x30;
             break;
 
         case '7':
             buf[0] = 0x01;
             buf[1] = 0x71;
             buf[2] = 0x09;
             buf[3] = 0x05;
             buf[4] = 0x03;
             break;
 
         case '8':
             buf[0] = 0x36;
             buf[1] = 0x49;
             buf[2] = 0x49;
             buf[3] = 0x49;
             buf[4] = 0x36;
             break;
 
         case '9':
             buf[0] = 0x06;
             buf[1] = 0x49;
             buf[2] = 0x49;
             buf[3] = 0x29;
             buf[4] = 0x1E;
             break;
 
         default: 
             buf[0] = 0x00;
             buf[1] = 0x00;
             buf[2] = 0x00;
             buf[3] = 0x00;
             buf[4] = 0x00;
             break;
     }
 }
 
 /**
  * @brief Converts raw BME280 temperature to 0.01 °C.
  *
  * @param[in] adc_T  Raw temperature ADC value.
  *
  * @return Temperature in 0.01 °C.
  */
 int32_t BME280_compensate_T_int32(int32_t adc_T)
 {
     int32_t var1, var2, T;
     var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
     var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
                ((int32_t)dig_T3)) >>
            14;
     t_fine = var1 + var2;
     T = (t_fine * 5 + 128) >> 8;
     return T;
 }
 
 /**
  * @brief Busy-wait delay using NOP instructions.
  *
  * @param[in] loops  Number of delay iterations.
  */
 void delay_cycles(uint32_t loops)
 {
     while(loops--)
     {
         __asm__ volatile(
             "nop\n"
             "nop\n"
             "nop\n"
             "nop\n");
     }
 }
 
 /**
  * @brief Writes a text string to a specific page of an SSD1306 display.
  *
  * Renders the given ASCII string using a 5x8 font and writes it to the
  * specified display page via I2C.
  *
  * @param[in,out] i2c   I2C driver instance.
  * @param[in]     text  Null-terminated string to display.
  * @param[in]     page  Target display page (0–7).
  *
  * @return I2C_OK on success, otherwise an error code.
  */
 i2c_status_t write_string_on_display(i2c_t* i2c, const char* text, uint8_t page)
 {
     i2c_status_t st = I2C_OK;
 
     if(page > 7){
         page = 7;
     }
         
     for(bool running = true; running; running = false)
     {
         uint8_t data_1[6] = {0x00, 0x8D, 0x14, 0x20, 0x00, 0xAF};
         st = i2c_master_send(i2c, SSD1306_ADDR, data_1, sizeof(data_1));
         if(st != I2C_OK)
             break;
 
         uint8_t data_2[7] = {0x00, 0x21, 10, 127, 0x22, page, page};
         st = i2c_master_send(i2c, SSD1306_ADDR, data_2, sizeof(data_2));
         if(st != I2C_OK)
             break;
 
         uint8_t letter_pixels[5];
 
         st = i2c_start(i2c);
         if(st != I2C_OK)
             break;
         
         st = i2c_send_addr(i2c, SSD1306_ADDR, false);
         if(st != I2C_OK)
             break;
         
         st = i2c_send_data(i2c, 0x40);
         if(st != I2C_OK)
             break;
 
         for(int i = 0; text[i] != '\0'; i++)
         {
             get_char_pixels(text[i], letter_pixels);
 
             for(int col = 0; col < 5; col++)
             {
                 st = i2c_send_data(i2c, letter_pixels[col]);
                 if(st != I2C_OK)
                     break;
             }
 
             st = i2c_send_data(i2c, 0x00);
             if(st != I2C_OK)
                 break;
         }
     }
     i2c_stop(i2c);
     return st;
 }
 
 /**
  * @brief Clears a single page of the SSD1306 display.
  *
  * Sets the column range to the full display width and writes zeros
  * to all columns of the specified page.
  *
  * @param[in,out] i2c   I2C driver instance.
  * @param[in]     page  Display page to clear (0–7).
  *
  * @return I2C_OK on success, otherwise an error code.
  */
 i2c_status_t clear_page(i2c_t* i2c, uint8_t page)
 {
     i2c_status_t st = I2C_OK;
     
     if(page > 7)
         page = 7;
 
     uint8_t data[7] = {0x00, 0x21, 0, 127, 0x22, page, page};
 
     for(bool running = true; running; running = false)
     {
 
         st = i2c_master_send(i2c, SSD1306_ADDR, data, sizeof(data));
         if(st != I2C_OK)
             break;
         
         st = i2c_start(i2c);
         if(st != I2C_OK)
             break;
         
         st = i2c_send_addr(i2c, SSD1306_ADDR, false);
         if(st != I2C_OK)
             break;
         
         st = i2c_send_data(i2c, 0x40);
         if(st != I2C_OK)
             break;
 
         for(int i = 0; i < 128; i++){
             st = i2c_send_data(i2c, 0x00);
             if(st != I2C_OK)
                 break;
         }
     }
     i2c_stop(i2c);
     return st;
 }
 
 /**
  * @brief Reads and returns the current temperature from the BME280 sensor.
  *
  * Configures the sensor, reads the raw temperature value via I2C and
  * applies the BME280 compensation formula.
  *
  * @param[in,out] i2c  I2C driver instance.
  *
  * @return Temperature in °C on success, or TEMP_INVALID on error.
  */
 float read_sensor(i2c_t* i2c)
 {
 
     i2c_status_t st = I2C_OK;
     float temp_float = 0;
     uint8_t setup_data[2] = {0xF4, 0x27};
     uint8_t data[1] = {0xFA};
     uint8_t raw_data[3];
 
     st = i2c_master_send(i2c, BME_ADDR, setup_data, 2);
     if(st != I2C_OK)
     {
         return TEMP_INVALID;
     }
 
     i2c_master_send(i2c, BME_ADDR, data, 1);
     if(st != I2C_OK)
     {
         return TEMP_INVALID;
     }
 
     i2c_master_recv(i2c, BME_ADDR, raw_data, 3);
     if(st != I2C_OK)
     {
         return TEMP_INVALID;
     }
 
     int32_t adc_T = ((int32_t)raw_data[0] << 12) | ((int32_t)raw_data[1] << 4) | ((int32_t)raw_data[2] >> 4);
 
     if(adc_T == TEMP_INVALID)
     {
         return TEMP_INVALID;
     }
     else
     {
         int32_t temp_int = BME280_compensate_T_int32(adc_T);
         temp_float = temp_int / 100.0f;
     }
     return temp_float;
 }
 
 int main()
 {
     printf("Start...\n");
 
     i2c_t i2c;
     i2c.base_addr = 0x70000000;
     i2c.scl_speed_hz = 100000;
     i2c_status_t st = I2C_OK;
 
     float temp = 0;
     char temp_string[16];
     i2c_init(&i2c, 25E6);
 
     clear_page(&i2c, 0);
     clear_page(&i2c, 1);
     clear_page(&i2c, 2);
     clear_page(&i2c, 3);
     clear_page(&i2c, 4);
     clear_page(&i2c, 5);
     clear_page(&i2c, 6);
     clear_page(&i2c, 7);
 
     write_string_on_display(&i2c, "PicoNut", 0);
     write_string_on_display(&i2c, "Temperature:", 2);
 
     while(1)
     {
 
         temp = read_sensor(&i2c);
         if(temp == TEMP_INVALID)
         {
             snprintf(temp_string, sizeof(temp_string), " error ");
         }
         else
         {
             snprintf(temp_string, sizeof(temp_string), "%.2f^C", temp);
         }
         delay_cycles(250000);
     
         write_string_on_display(&i2c, "PicoNut", 0);
         write_string_on_display(&i2c, "Temperature:", 2);
         write_string_on_display(&i2c, temp_string, 4);
     }
 
     i2c_de_init(&i2c);
 
     return 0;
 }
 