
/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2010-2022 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
                          Christian H. Meyer <christian.meyer@hs-augsburg.de>
                2023-2024 Lukas Bauer <lukas.bauer1@hs-augsburg.de
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    This file contains genereal implementations of simple block rams.
    As synthesising these for different verndors may necessitate different
    implementations, these implementations will be held in seperate files and
    included here.

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

module m_dual_port_byte_enable_block_ram (clka, clkb,ena,enb,wea,web,addra,addrb,dia,dib,doa,dob);

parameter   RAM_SIZE   =   {RAM_SIZE},
            NUM_COL    =   {NUM_COL},
            COL_WIDTH  =   {COL_WIDTH},
            ADDR_WIDTH =   {ADDR_WIDTH},
            DATA_WIDTH =   {DATA_WIDTH};

input clka,clkb,ena,enb;
input [NUM_COL-1:0] wea,web;
input [ADDR_WIDTH-1:0] addra,addrb;
input [DATA_WIDTH-1:0] dia,dib;
output [DATA_WIDTH-1:0] doa,dob;

reg [DATA_WIDTH-1:0] ram [RAM_SIZE-1:0];
reg [DATA_WIDTH-1:0] doa,dob;

integer                i;

always @(posedge clka) begin
    if(ena) begin
        for(i=0;i<NUM_COL;i=i+1) begin
            if(wea[i]) begin
                ram[addra][i*COL_WIDTH +: COL_WIDTH] <= dia[i*COL_WIDTH +: COL_WIDTH];
            end
        end
        doa <= ram[addra];
    end
end

always @(posedge clkb) begin
    if(enb) begin
        for(i=0;i<NUM_COL;i=i+1) begin
            if(web[i]) begin
                ram[addrb][i*COL_WIDTH +: COL_WIDTH] <= dib[i*COL_WIDTH +: COL_WIDTH];
            end
        end
        dob <= ram[addrb];
    end
end


initial begin
    $readmemb("text.mem", ram, 32'h0);
    $readmemb("data.mem", ram, 32'h6000);
end

endmodule