/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2023  Lukas Bauer <lukas.bauer1@hs-augsburg.de>
        Augsburg, University of Applied Sciences

  Description:
     code of the bootloader used for loading programs onto the hardware of the paranut

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
 *
 * Command definitions
 * This is a list of all commands that can be used to communicate with the bootloader
 * The Words in () are the command bytes that are sent over the UART.
 * With Words that are written like this e.g. CMD_BEGIN are the control sequences
 * A number [] indicates the length of the command in bytes
 * A Label in <> indicates data that is sent that has no control sequence
 * For example <destination address>[4]  means that the destination address should be sent
 * here and has the length of 4 bytes
 * The CHECKSUM is calculated by XORing all bytes of the command together
 * and adding the checksum to the end of the command
 * If a command is received with an invalid checksum the bootloader will respond with a ERR_INVALID_CHK.
 * The definition of the error codes can be found after the command definitions
 *
 * CMD_HELLO:
 *      (CMD_BEGIN, CMD_HELLO, CHECKSUM)
 *      This command is used to get the version of the bootloader
 *      The command has no additional data and the bootloader will respond with:
 *     (CMD_ACK, CMD_HELLO, VERSION_MAJOR, VERSION_MINOR, VERSION_REVISION[2], CHECKSUM)
 *
 * CMD_MEM_WRITE:
 *      (CMD_BEGIN, CMD_MEM_WRITE, <address>[4], <data>[4], CHECKSUM)
 *      This command is used to write a 4 byte word to the specified address in the memory of the PicoNut
 *      The bootloader will respond with:
 *      (CMD_ACK, CMD_MEM_WRITE, <address>[4], <data>[4], CHECKSUM)
 *      If the address is not in the range of the memory of the PicoNut the bootloader will respond with ERR_INVALID_ADR.
 *
 * CMD_MEM_READ:
 *     (CMD_BEGIN, CMD_MEM_READ, <address>[4], CHECKSUM)
 *     This command is used to read a 4 byte word from the specified address in the memory of the PicoNut
 *     The bootloader will respond with:
 *     (CMD_ACK, CMD_MEM_READ, <address>[4], <data>[4], CHECKSUM)
 *     If the address is not in the range of the memory of the PicoNut the bootloader will respond with ERR_INVALID_ADR.
 *
 * CMD_SETUP_MEM_BLOCK_WRITE:
 *     (CMD_BEGIN, CMD_SETUP_MEM_BLOCK_WRITE, <address>[4], <size>[4], CHECKSUM)
 *     This command sets up the bootloader to receive a block of data and write it to the memory of the PicoNut
 *     The command includes the start address where the data should be written and the size of the data
 *     The bootloader will respond with:
 *     (CMD_ACK, CMD_SETUP_MEM_BLOCK_WRITE, <address>[4], <size>[4], CHECKSUM)
 *     If the address is not in the range of the memory or the last 4 byte of the data is outside of the memory of the PicoNut
 *     the bootloader will respond with ERR_INVALID_ADR.
 *
 * CMD_MEM_BLOCK_WRITE:
 *    Before running this command the CMD_SETUP_MEM_BLOCK_WRITE command has to be run successfully
 *    (CMD_BEGIN, CMD_MEM_BLOCK_WRITE, <data>[size of the block], CHECKSUM)
 *    This command is used to write a block of data to the memory of the PicoNut
 *    After the command CMD_SETUP_MEM_BLOCK_WRITE has been sent the bootloader will wait the amount of bytes specified in the
 *    CMD_SETUP_MEM_BLOCK_WRITE command and write them to the memory of the PicoNut
 *    The bootloader will respond with:
 *    (CMD_ACK, CMD_MEM_BLOCK_WRITE, <address>[4], <size>[4], CHECKSUM)
 *    If the CMD_SETUP_MEM_BLOCK_WRITE command has not been run successfully the bootloader will respond with ERR_BLOCK_ERROR.
 *
 * CMD_RUN:
 *    (CMD_BEGIN, CMD_RUN, CHECKSUM)
 *    This command is used to jump to the beginning of the memory of the PicoNut and run it.
 *    There is no additional data and the bootloader will respond with:
 *    (CMD_ACK, CMD_RUN, CHECKSUM)
 *
 * CMD_DECOMP:
 *    (CMD_BEGIN, CMD_DECOMP, <source address>[4], <destination address>[4], <size>[4], CHECKSUM)
 *    This command is used to decompress a program in the memory of the PicoNut
 *    The command includes the source address of the compressed data, the destination address
 *    of the decompressed data and the size of the compressed data
 *    The bootloader will respond with:
 *    (CMD_ACK, CMD_DECOMP, CHECKSUM)
 *    If there is an error during the decompression the bootloader will respond with ERR_DECOMP_ERROR.
 *
 *
 * Error message definitions:
 *
 * All error messages have the following format:
 *    (CMD_ERROR, <command id>, <error code>, CHECKSUM)
 *
 * ERR_INVALID_CMD:
 *     This error is sent if the command id is not valid
 *
 * ERR_INVALID_ADR:
 *     This error is sent if the address is not in the range of the memory of the PicoNut
 *
 * ERR_INVALID_CHK:
 *     This error is sent if the checksum of the command is not correct
 *
 * ERR_BLOCK_ERROR:
 *     This error is sent if the CMD_SETUP_MEM_BLOCK_WRITE command has not been run successfully
 *
 * ERR_DECOMP_ERROR:
 *     This error is sent if there is an error during the decompression of a program
 *
 *
 */

/*-------------------------------Includes-------------------------------*/

#include "uzlib.h"
#include <stdint.h>

/*------------------------------Definitions-----------------------------*/


// size and address of the memory of the PicoNut
#define PARANUT_MEM_ADDR 0x10000000
#define MAX_MEM_SIZE (256 * 1024 * 1024) // 256 MB

// control sequences
#define CMD_BEGIN (0b11110000) // 0xF0
#define CMD_ERROR (0b11110001) // 0xF1
#define CMD_ACK (0b11110010)   // 0xF2

// error codes
#define ERR_INVALID_CMD (0b11100000)      // 0xE0
#define ERR_INVALID_ADR (0b11100001)      // 0xE1
#define ERR_INVALID_CHK (0b11100010)      // 0xE2
#define ERR_BLOCK_ERROR (0b11100011)      // 0xE3
#define ERR_DECOMP_ERROR (0b11100100)     // 0xE4

// commands
#define CMD_HELLO (0b00000001)                 // 0x01
#define CMD_MEM_WRITE (0b00000010)             // 0x02
#define CMD_SETUP_MEM_BLOCK_WRITE (0b00000011) // 0x03
#define CMD_MEM_BLOCK_WRITE (0b00000100)       // 0x04
#define CMD_MEM_READ (0b00000101)              // 0x05
#define CMD_RUN (0b00000110)                   // 0x06
#define CMD_DECOMP (0b00000111)                 // 0x07

// command lengths
#define CMD_LEN_HELLO 3
#define CMD_LEN_MEM_WRITE 11
#define CMD_LEN_SETUP_MEM_BLOCK_WRITE 11
#define CMD_LEN_MEM_BLOCK_WRITE 2
#define CMD_LEN_MEM_READ 7
#define CMD_LEN_RUN 3
#define CMD_LEN_DECOMP 15

// decompression
#define SMALLEST_DECOMP_CHUNK_SIZE 1

extern unsigned char uart_begin;

// initial address and size of the block write
uint32_t blk_write_adr = 0;
uint32_t blk_write_size = 0;

/*------------------------------Helper Functions-------------------------------*/

// receive a character from the UART
uint8_t uart_rx()
{

    uint8_t rcv_data;

    // use the receive code for the external UART if the external UART is enabled
#ifdef UART_EXTERNAL
    volatile uint32_t *uart_lsr = (uint32_t *)(0x60000000 + 0x14);

    while ((*uart_lsr & 0x01) == 0)
        ; // Wait until UART has received a character

    rcv_data = uart_begin;
#else
    volatile uint32_t *uart_sr = (uint32_t *)(0xE0001000 + 0x2C);
    volatile uint32_t *uart_fifo = (uint32_t *)(0xE0001000 + 0x30);
    while ((*uart_sr & 0x02) == 0x02)
        ; // Wait until UART has received a character

    rcv_data = *uart_fifo;

#endif
    return rcv_data;
}


// transmit a character over the UART
void uart_tx(uint8_t tx_data)
{

        // use the transmit code for the external UART if the external UART is enabled
#ifdef UART_EXTERNAL
    volatile uint32_t *uart_lsr = (uint32_t *)(0x60000000 + 0x14);

    while ((*uart_lsr & 0x20) == 1)
        ; // Wait until UART is ready to transmit
    uart_begin = tx_data;
#else

    volatile uint32_t *uart_sr = (uint32_t *)(0xE0001000 + 0x2C);
    volatile uint32_t *uart_fifo = (uint32_t *)(0xE0001000 + 0x30);
    while ((*uart_sr & 0x10) == 0x10)
        ; // Wait when transmitter full

    *uart_fifo = tx_data;

#endif
}

// transmit a 32 bit value over the UART
void uart_tx32(uint32_t tx_data)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        uart_tx((uint8_t)(tx_data >> (i * 8)));
    }
}

// transmit a char over the UART
void uart_putc(char c)
{

    uart_tx((uint8_t)c);

    // replace \n with \r
    if (c == '\n')
    {
        uart_tx(0x0D);
    }
}

// send a string over the UART
void uart_print(char *tx_str, uint32_t str_length)
{

    for (uint32_t i = 0; i < str_length; i++)
    {

        uart_putc(tx_str[i]);
    }
}

// calculate the checksum
uint8_t calc_checksum(const uint8_t *data, uint8_t data_length)
{
    uint8_t checksum = 0;

    for (uint8_t i = 0; i < data_length; i++)
    {
        checksum ^= data[i];
    }

    return checksum;
}

// calculate the checksum by adding a byte to the checksum
uint8_t calc_checksum_cont(uint8_t data, uint8_t checksum)
{
    return checksum ^ data;
}

// get 32 bit arg out of cmd buffer
uint32_t get_arg32(const uint8_t *cmd, uint8_t offset)
{
    uint32_t arg = 0;

    for (uint8_t i = offset + 3; i >= offset; i--)
    {
        arg |= (cmd[i]);
        if (i != offset)
        {
            arg <<= 8;
        }
    }

    return arg;
}

// calculate the decompressed size of a compressed file
uint32_t get_decomp_size(uint32_t src_adr, uint32_t size)
{

    uint8_t *src_adr_ptr = (uint8_t*)src_adr;
    uint32_t decomp_size = 0;

    // gotten the formular from the uzlib exapmle code
    // https://github.com/pfalcon/uzlib/blob/master/examples/tgunzip/tgunzip.c
    decomp_size = src_adr_ptr[size - 1];
    decomp_size = 256 * decomp_size + src_adr_ptr[size - 2];
    decomp_size = 256 * decomp_size + src_adr_ptr[size - 3];
    decomp_size = 256 * decomp_size + src_adr_ptr[size - 4];

    // add one to ensure that the size is correct
    decomp_size++;

    return decomp_size;
}

// decompress a file by giving the source address, destination address and size of
// the compressed file
int8_t decompress(uint32_t src_adr, uint32_t dst_adr, uint32_t size)
{
    TINF_DATA tinf;
    int ret_value;
    uint8_t *src_adr_ptr = (uint8_t *)src_adr;
    uint8_t *dst_adr_ptr = (uint8_t *)dst_adr;

    // calculate the decompressed size of the file
    uint32_t decomp_size = get_decomp_size(src_adr, size);

    // check if the decompressed data overlaps with the compressed data chunk
    if (dst_adr + decomp_size > src_adr)
    {
        return -1;
    }

    // init the uzlib library
    uzlib_uncompress_init(&tinf, NULL, 0);

    // set the source address for the compressed data
    tinf.source = src_adr_ptr;
    tinf.source_limit = src_adr_ptr + size - 4;
    tinf.source_read_cb = NULL;

    // parse the gzip header
    ret_value = uzlib_gzip_parse_header(&tinf);
    if (ret_value != TINF_OK)
    {
        return -1;
    }

    // set the destination address for the decompressed data
    tinf.dest_start = dst_adr_ptr;
    tinf.dest = dst_adr_ptr;

    // decompress the data chunk by chunk until all data is decompressed
    while(decomp_size != 0){

        uint32_t chunk_size = decomp_size < SMALLEST_DECOMP_CHUNK_SIZE ? decomp_size : SMALLEST_DECOMP_CHUNK_SIZE;
        tinf.dest_limit = tinf.dest + chunk_size;
        ret_value = uzlib_uncompress_chksum(&tinf);
        decomp_size -= chunk_size;
        if(ret_value != TINF_OK){
            break;
        }

    }

    // if the decompression was not successful return -1 else return 0
    if (ret_value != TINF_DONE){
        return -1;
    }else{
        return 0;
    }
}

/*---------------------------------Commands------------------------------------*/

// cmd to get the version of the hardware
void command_hello(const uint8_t *cmd)
{
    uint8_t rcv_ver_mjr, rcv_ver_mnr;
    uint16_t rcv_ver_rev;

    // get checksum from the command one byte long after data
    uint8_t checksum = cmd[CMD_LEN_HELLO - 1];

    // check if the checksum is correct
    if (checksum != calc_checksum(cmd, CMD_LEN_HELLO - 1))
    {
        uart_tx(CMD_ERROR);
        uart_tx(cmd[1]);
        uart_tx(ERR_INVALID_CHK);
        uint8_t chk_data[] = {CMD_ERROR, cmd[1], ERR_INVALID_CHK};
        uart_tx(calc_checksum(chk_data, sizeof(chk_data)));
        return;
    }


    // get the version of the hardware
    uint32_t version;
    asm volatile("csrr %0, 0xf13" : "=r"(version));

    // decode the csr with the version number
    uint8_t ver_mjr = (uint8_t)(version >> 24);
    uint8_t ver_mnr = (uint8_t)(version >> 16);
    uint16_t ver_rev = (uint16_t)((version >> 1) & 0x7FFF);


    // send bootloader version
    uart_tx(CMD_ACK);
    uart_tx(CMD_HELLO);
    uart_tx(ver_mjr);
    uart_tx(ver_mnr);
    uart_tx((uint8_t)(ver_rev & 0xFF));
    uart_tx((uint8_t)(ver_rev >> 8));
    uint8_t chk_data[] = {CMD_ACK,
                          CMD_HELLO,
                          ver_mjr,
                          ver_mnr,
                          (uint8_t)(ver_rev & 0xFF),
                          (uint8_t)(ver_rev >> 8)};
    uart_tx(calc_checksum(chk_data, sizeof(chk_data)));

    return;
}

// cmt to write a 32 bit word to the memory of the PicoNut
void command_mem_write(const uint8_t *cmd)
{
    // extract the address and data from the received command
    uint32_t adr, data;
    adr = get_arg32(cmd, 2);
    data = get_arg32(cmd, 6);

    // get checksum from the command one byte long after data
    uint8_t checksum = cmd[CMD_LEN_MEM_WRITE - 1];

    // check if the checksum is correct
    if (checksum != calc_checksum(cmd, CMD_LEN_MEM_WRITE - 1))
    {
        uart_tx(CMD_ERROR);
        uart_tx(cmd[1]);
        uart_tx(ERR_INVALID_CHK);
        uint8_t chk_data[] = {CMD_ERROR, *cmd, ERR_INVALID_CHK};
        uart_tx(calc_checksum(chk_data, sizeof(chk_data)));
        return;
    }

    // check if the address is in the range of the memory of the PicoNut
    if (adr < PARANUT_MEM_ADDR || adr > PARANUT_MEM_ADDR + MAX_MEM_SIZE)
    {
        uart_tx(CMD_ERROR);
        uart_tx(cmd[1]);
        uart_tx(ERR_INVALID_ADR);
        uint8_t chk_data[] = {CMD_ERROR, cmd[1], ERR_INVALID_ADR};
        uart_tx(calc_checksum(chk_data, sizeof(chk_data)));
        return;
    }

    // save the data to the memory at the given address
    volatile uint32_t *adr_snd = (uint32_t *)adr;

    *adr_snd = data;

    // send back an acknowledge
    uart_tx(CMD_ACK);
    uart_tx(cmd[1]);
    uart_tx32(adr);
    uart_tx32(data);
    // create checksum of the acknowledge get every byte of adr and data with and 0xFF
    uint8_t chk_data[] = {CMD_ACK,
                          cmd[1],
                          (uint8_t)(adr & 0xFF),
                          (uint8_t)(adr >> 8 & 0xFF),
                          (uint8_t)(adr >> 16 & 0xFF),
                          (uint8_t)(adr >> 24 & 0xFF),
                          (uint8_t)(data & 0xFF),
                          (uint8_t)(data >> 8 & 0xFF),
                          (uint8_t)(data >> 16 & 0xFF),
                          (uint8_t)(data >> 24 & 0xFF)};
    uart_tx(calc_checksum(chk_data, sizeof(chk_data)));

    return;
}

// cmd to read a 32 bit word from the memory of the PicoNut
void command_mem_read(const uint8_t *cmd)
{
    // extract the address from the received command
    uint32_t adr;
    adr = get_arg32(cmd, 2);

    // get checksum from the command one byte long after data
    uint8_t checksum = cmd[CMD_LEN_MEM_READ - 1];

    // check if the checksum is correct
    if (checksum != calc_checksum(cmd, CMD_LEN_MEM_READ - 1))
    {
        uart_tx(CMD_ERROR);
        uart_tx(cmd[1]);
        uart_tx(ERR_INVALID_CHK);
        uint8_t chk_data[] = {CMD_ERROR, cmd[1], ERR_INVALID_CHK};
        uart_tx(calc_checksum(chk_data, sizeof(chk_data)));
        return;
    }

    // check if the address is in the range of the memory of the PicoNut
    if (adr < PARANUT_MEM_ADDR || adr > PARANUT_MEM_ADDR + MAX_MEM_SIZE)
    {
        uart_tx(CMD_ERROR);
        uart_tx(cmd[1]);
        uart_tx(ERR_INVALID_ADR);
        uint8_t chk_data[] = {CMD_ERROR, cmd[1], ERR_INVALID_ADR};
        uart_tx(calc_checksum(chk_data, sizeof(chk_data)));
        return;
    }

    // read the data from the memory at the given address
    volatile uint32_t *adr_snd = (uint32_t *)adr;
    uint32_t data_out = *adr_snd;

    // send back data and ack
    uart_tx(CMD_ACK);
    uart_tx(cmd[1]);
    uart_tx32(adr);
    uart_tx32(data_out);
    uint8_t chk_data[] = {CMD_ACK,
                          cmd[1],
                          (uint8_t)(adr & 0xFF),
                          (uint8_t)(adr >> 8 & 0xFF),
                          (uint8_t)(adr >> 16 & 0xFF),
                          (uint8_t)(adr >> 24 & 0xFF),
                          (uint8_t)(data_out & 0xFF),
                          (uint8_t)(data_out >> 8 & 0xFF),
                          (uint8_t)(data_out >> 16 & 0xFF),
                          (uint8_t)(data_out >> 24 & 0xFF)};
    uart_tx(calc_checksum(chk_data, sizeof(chk_data)));

    return;
}

// cmd to write a block of data to the memory of the PicoNut
void command_setup_blockwrite(const uint8_t *cmd)
{

    // get the arguemts from the command
    blk_write_adr = get_arg32(cmd, 2);
    blk_write_size = get_arg32(cmd, 6);

    uint8_t checksum = cmd[CMD_LEN_SETUP_MEM_BLOCK_WRITE - 1];

    // check if the checksum is correct
    if (checksum != calc_checksum(cmd, CMD_LEN_SETUP_MEM_BLOCK_WRITE - 1))
    {
        uart_tx(CMD_ERROR);
        uart_tx(cmd[1]);
        uart_tx(ERR_INVALID_CHK);
        uint8_t chk_data[] = {CMD_ERROR, cmd[1], ERR_INVALID_CHK};
        uart_tx(calc_checksum(chk_data, sizeof(chk_data)));
        return;
    }

    // check if the address is in the range of the memory of the PicoNut and check
    // if the data size is within the range of the memory
    if (blk_write_adr < PARANUT_MEM_ADDR || blk_write_adr > PARANUT_MEM_ADDR + MAX_MEM_SIZE
    | blk_write_adr + blk_write_size > PARANUT_MEM_ADDR + MAX_MEM_SIZE)
    {
        uart_tx(CMD_ERROR);
        uart_tx(cmd[1]);
        uart_tx(ERR_INVALID_ADR);
        uint8_t chk_data[] = {CMD_ERROR, cmd[1], ERR_INVALID_ADR};
        uart_tx(calc_checksum(chk_data, sizeof(chk_data)));
        return;
    }

    // send back an acknowledge of the received command
    uart_tx(CMD_ACK);
    uart_tx(cmd[1]);
    uart_tx32(blk_write_adr);
    uart_tx32(blk_write_size);
    uint8_t chk_data[] = {CMD_ACK,
                          cmd[1],
                          (uint8_t)(blk_write_adr & 0xFF),
                          (uint8_t)(blk_write_adr >> 8 & 0xFF),
                          (uint8_t)(blk_write_adr >> 16 & 0xFF),
                          (uint8_t)(blk_write_adr >> 24 & 0xFF),
                          (uint8_t)(blk_write_size & 0xFF),
                          (uint8_t)(blk_write_size >> 8 & 0xFF),
                          (uint8_t)(blk_write_size >> 16 & 0xFF),
                          (uint8_t)(blk_write_size >> 24 & 0xFF)};
    uart_tx(calc_checksum(chk_data, sizeof(chk_data)));

    return;
}

// cmd to write a block of data to the memory of the PicoNut
void command_blockwrite(const uint8_t *cmd)
{

    // check if the address and size are set and compressed is not set
    if (blk_write_adr == 0 || blk_write_size == 0)
    {
        uart_tx(CMD_ERROR);
        uart_tx(cmd[1]);
        uart_tx(ERR_BLOCK_ERROR);
        uint8_t chk_data[] = {CMD_ERROR, cmd[1], ERR_BLOCK_ERROR};
        uart_tx(calc_checksum(chk_data, sizeof(chk_data)));
        return;
    }

    // add the received cmd to the checksum
    uint8_t checksum = 0;
    uint8_t check_data[] = {CMD_BEGIN, cmd[1]};
    checksum = calc_checksum(check_data, sizeof(check_data));

    // receive the data and write it to the memory at the given address specifed in the setup command
    for (uint32_t end_adr = blk_write_adr + blk_write_size; blk_write_adr < end_adr; blk_write_adr++)
    {

        // get data from the UART
        uint8_t rcv_data = uart_rx();
        checksum = calc_checksum_cont(rcv_data, checksum);

        // write the data to the memory at the given address
        volatile uint8_t *adr_snd = (uint8_t *)blk_write_adr;
        *adr_snd = rcv_data;
    }

    // send an error if the checksum is not correct
    if (checksum != uart_rx())
    {
        uart_tx(CMD_ERROR);
        uart_tx(cmd[1]);
        uart_tx(ERR_INVALID_CHK);
        uint8_t chk_data[] = {CMD_ERROR, cmd[1], ERR_INVALID_CHK};
        uart_tx(calc_checksum(chk_data, sizeof(chk_data)));
        return;
    }

    // send back an acknowledge of the received data
    uart_tx(CMD_ACK);
    uart_tx(cmd[1]);
    uart_tx32(blk_write_adr);
    uart_tx32(blk_write_size);
    uint8_t chk_data[] = {CMD_ACK,
                          cmd[1],
                          (uint8_t)(blk_write_adr & 0xFF),
                          (uint8_t)(blk_write_adr >> 8 & 0xFF),
                          (uint8_t)(blk_write_adr >> 16 & 0xFF),
                          (uint8_t)(blk_write_adr >> 24 & 0xFF),
                          (uint8_t)(blk_write_size & 0xFF),
                          (uint8_t)(blk_write_size >> 8 & 0xFF),
                          (uint8_t)(blk_write_size >> 16 & 0xFF),
                          (uint8_t)(blk_write_size >> 24 & 0xFF)};
    uart_tx(calc_checksum(chk_data, sizeof(chk_data)));

    // reset the address and size
    blk_write_adr = 0;
    blk_write_size = 0;

    return;
}

// cmd to jump to the beginning of the memory of the PicoNut and run it
void command_run(const uint8_t *cmd)
{

    // check the checksum

    uint8_t checksum = cmd[CMD_LEN_RUN - 1];

    // check if the checksum is correct
    if (checksum != calc_checksum(cmd, CMD_LEN_RUN - 1))
    {
        uart_tx(CMD_ERROR);
        uart_tx(cmd[1]);
        uart_tx(ERR_INVALID_CHK);
        uint8_t chk_data[] = {CMD_ERROR, cmd[1], ERR_INVALID_CHK};
        uart_tx(calc_checksum(chk_data, sizeof(chk_data)));
        return;
    }

    // send back an acknowledge of the received command
    uart_tx(CMD_ACK);
    uart_tx(cmd[1]);
    uint8_t chk_data[] = {CMD_ACK, cmd[1]};
    uart_tx(calc_checksum(chk_data, sizeof(chk_data)));

    // jump to the beginning of paranuts memory
    void (*prg_main)(void) = (void *)PARANUT_MEM_ADDR;

    prg_main();
}

// cmd to decompress a program in the memory of the PicoNut
void command_decompress(const uint8_t *cmd)
{

    uint32_t src_adr = get_arg32(cmd, 2);
    uint32_t dst_adr = get_arg32(cmd, 6);
    uint32_t size = get_arg32(cmd, 10);
    int ret_value;


    uint8_t checksum = cmd[CMD_LEN_DECOMP - 1];

    if (checksum != calc_checksum(cmd, CMD_LEN_DECOMP - 1))
    {
        uart_tx(CMD_ERROR);
        uart_tx(cmd[1]);
        uart_tx(ERR_INVALID_CHK);
        uint8_t chk_data[] = {CMD_ERROR, cmd[1], ERR_INVALID_CHK};
        uart_tx(calc_checksum(chk_data, sizeof(chk_data)));
        return;
    }

    ret_value = decompress(src_adr, dst_adr, size);

    if (ret_value != 0)
    {
        uart_tx(CMD_ERROR);
        uart_tx(cmd[1]);
        uart_tx(ERR_DECOMP_ERROR);
        uint8_t chk_data[] = {CMD_ERROR, cmd[1], ERR_DECOMP_ERROR};
        uart_tx(calc_checksum(chk_data, sizeof(chk_data)));
        return;
    }

    // send back an acknowledge of the received data
    uart_tx(CMD_ACK);
    uart_tx(cmd[1]);
    uint8_t chk_data[] = {CMD_ACK,
                          cmd[1]};
    uart_tx(calc_checksum(chk_data, sizeof(chk_data)));

    return;
}

/*---------------------------------Main------------------------------------*/

int main()
{

    // send a welcome message
    uart_print("(S1LOADER) PicoNut fist stage bootloader ready.\n", 49);

    // disable the cache
    asm volatile("csrrw x0, 0x7c0, x0");





    uint8_t data = 0;
    uint8_t cmd_buffer[32] = {0};
    int8_t cmd_idx = 0;

    // main command loop
    while (1)
    {

        // get a character from the UART
        data = uart_rx();

        // read the command until a CMD_END is received
        cmd_buffer[cmd_idx] = data;
        cmd_idx++;

        // check if the command is complete and valid if not reset the command index
        // and send an error message
        // else execute the command and reset command index
        if (((cmd_buffer[1] != CMD_HELLO) &&
             (cmd_buffer[1] != CMD_MEM_WRITE) &&
             (cmd_buffer[1] != CMD_MEM_READ) &&
             (cmd_buffer[1] != CMD_SETUP_MEM_BLOCK_WRITE) &&
             (cmd_buffer[1] != CMD_MEM_BLOCK_WRITE) &&
             (cmd_buffer[1] != CMD_RUN) &&
             (cmd_buffer[1] != CMD_DECOMP)) &&
            cmd_idx > 1 && cmd_idx < CMD_LEN_DECOMP + 1)
        {
            cmd_idx = 0;
            uart_tx(CMD_ERROR);
            uart_tx(cmd_buffer[1]);
            uart_tx(ERR_INVALID_CMD);
            uint8_t chk_data[] = {CMD_ERROR, cmd_buffer[1], ERR_INVALID_CMD};
            uart_tx(calc_checksum(chk_data, sizeof(chk_data)));
        }
        else if (cmd_buffer[1] == CMD_HELLO && cmd_idx == CMD_LEN_HELLO)
        {
            command_hello(cmd_buffer);
            cmd_idx = 0;
        }
        else if (cmd_buffer[1] == CMD_MEM_WRITE && cmd_idx == CMD_LEN_MEM_WRITE)
        {
            command_mem_write(cmd_buffer);
            cmd_idx = 0;
        }
        else if (cmd_buffer[1] == CMD_MEM_READ && cmd_idx == CMD_LEN_MEM_READ)
        {
            command_mem_read(cmd_buffer);
            cmd_idx = 0;
        }
        else if (cmd_buffer[1] == CMD_SETUP_MEM_BLOCK_WRITE && cmd_idx == CMD_LEN_SETUP_MEM_BLOCK_WRITE)
        {
            command_setup_blockwrite(cmd_buffer);
            cmd_idx = 0;
        }
        else if (cmd_buffer[1] == CMD_MEM_BLOCK_WRITE && cmd_idx == CMD_LEN_MEM_BLOCK_WRITE)
        {
            command_blockwrite(cmd_buffer);
            cmd_idx = 0;
        }
        else if (cmd_buffer[1] == CMD_RUN && cmd_idx == CMD_LEN_RUN)
        {
            command_run(cmd_buffer);
            cmd_idx = 0;
        }
        else if (cmd_buffer[1] == CMD_DECOMP && cmd_idx == CMD_LEN_DECOMP)
        {
            command_decompress(cmd_buffer);
            cmd_idx = 0;
        }
    }

    return -1;

}