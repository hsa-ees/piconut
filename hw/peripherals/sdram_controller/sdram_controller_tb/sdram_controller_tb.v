/*******************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2026 Hermann Zoha <hermann.zoha@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    Testbench for the SDRAM-Controller Module.

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

 ******************************************************************************/

/*
*This testbench verifies the functionality of the 'top_sdram_module' using
*Verification Phases:
*      - Phase 1 (First 50 Bytes/Cells): Writes incremental values to addresses starting 
*        at 0x40000000, then reads them back after a delay to verify correctness.
*      - Phase 2 (Middle Cells): Repeated write/read loop targeting addresses starting 
*        at 0x41000000.
*      - Phase 3 (Last Cells): Repeated write/read loop targeting the upper memory boundary 
*        addresses around 0x41FFFFCE to 0x42000000.
*/


`timescale 1ns/1ns


module sdram_controller_tb();
    parameter ADDR_WIDHT = 64;
    parameter DATA_WIDHT = 64;
    
    
    reg clk_i                   = 0;
    reg reset                   = 0;
    reg [ADDR_WIDHT-1: 0]addr_i = 0;
    reg [DATA_WIDHT-1 :0]dat_i  = 0;
    reg [7:0] sel_i             = 0;
    reg stb_i                   = 0;
    reg cyc_i                   = 0;
    reg we_i                    = 0;
    wire [DATA_WIDHT-1:0]dat_o;
    wire ack_o;
    integer i;
    integer comp;
    integer cont       = 0;
    integer data;

    top_sdram_module top(
    .clk(clk_i),
    .reset(reset),
    .adr_i(addr_i),
    .dat_i(dat_i),
    .sel_i(sel_i),
    .stb_i(stb_i),
    .cyc_i(cyc_i),
    .we_i(we_i),
    .dat_o(dat_o),
    .ack_o(ack_o) );

    always #42 clk_i =~clk_i;
    task assert;
        input [31:0] data;
        input [31:0] comp;
        inout integer cont;
        begin
            if(data != comp) begin
                $error("Wrong! The input %h dose NOT match %h", comp, data);
                cont = cont + 1;
            end
        end
    endtask 

    initial begin
        
        $dumpfile("sdram_controllertb.vcd");
        $dumpvars(0, sdram_controller_tb);
        
        // --- System Reset Sequence ---

        reset = 1;
        #84
        $display(" Reset is aktived");

        reset = 0;
        $display("Reset is disabled");

        // Wait for SDRAM initialization time
        #2400


        // ========================================================
        // TEST SEGMENT 1: First Memory Region
        // ========================================================

        // WRITE LOOP
        for(i = 32'h0; i<='h32; i= i+4) begin

            addr_i  =  32'h40000000 + i;
            dat_i   =  i;
            stb_i   = 1;
            cyc_i   = 1;
            we_i    = 1;

            @(posedge clk_i);
            while (ack_o !== 1) begin
                @(posedge clk_i);
            end

            addr_i  =  0;
            dat_i   =  0;
            stb_i   = 0;
            cyc_i   = 0;
            we_i    = 0;
            @(posedge clk_i);
        end
        $display("first write loop done");
    
        
        #1600

        // READ & VERIFY LOOP
        for(i = 32'h0; i<='h32; i= i+4) begin
            comp = i;
            addr_i  = 32'h40000000 + i;
            dat_i   = 0;
            stb_i   = 1;
            cyc_i   = 1;
            we_i    = 0;

            @(posedge clk_i);
            while (ack_o !== 1) begin
                @(posedge clk_i);
            end
            
            #1
            data = dat_o;

            assert(data, comp, cont);

            addr_i  = 0;
            dat_i   = 0;
            stb_i   = 0;
            cyc_i   = 0;
            we_i    = 0;
            @(posedge clk_i);

        end 

        $display("the first 50 cells are tested");

        // ========================================================
        // TEST SEGMENT 2: Middle Memory Region
        // ========================================================

        // WRITE LOOP
        for(i = 32'h1000000; i<='h1000032; i= i+4) begin

            addr_i  =  32'h40000000 + i;
            dat_i   =  i;
            stb_i   = 1;
            cyc_i   = 1;
            we_i    = 1;

            @(posedge clk_i);
            while (ack_o !== 1) begin
                @(posedge clk_i);
            end

            addr_i  =  0;
            dat_i   =  0;
            stb_i   = 0;
            cyc_i   = 0;
            we_i    = 0;
            @(posedge clk_i);
        end
        $display("middle write loop done");
    
        
        #1600
        // READ & VERIFY LOOP
        for(i = 32'h1000000; i<='h1000032; i= i+4) begin
            comp = i;
            addr_i  = 32'h40000000 + i;
            dat_i   = 0;
            stb_i   = 1;
            cyc_i   = 1;
            we_i    = 0;

            @(posedge clk_i);
            while (ack_o !== 1) begin
                @(posedge clk_i);
            end
            
            #1
            data = dat_o;

            assert(data, comp, cont); 

            addr_i  = 0;
            dat_i   = 0;
            stb_i   = 0;
            cyc_i   = 0;
            we_i    = 0;
            @(posedge clk_i);

        end 

        $display("the middle 50 cells are tested");


        // ========================================================
        // TEST SEGMENT 3: Last Memory Region
        // ========================================================

        // WRITE LOOP
        for(i = 32'h1ffffce; i<='h2000000; i= i+4) begin

            addr_i  =  32'h40000000 + i;
            dat_i   =  i;
            stb_i   = 1;
            cyc_i   = 1;
            we_i    = 1;

            @(posedge clk_i);
            while (ack_o !== 1) begin
                @(posedge clk_i);
            end

            addr_i  =  0;
            dat_i   =  0;
            stb_i   = 0;
            cyc_i   = 0;
            we_i    = 0;
            @(posedge clk_i);
        end
            $display("last write loop done");
    
        
        #1600
        // READ & VERIFY LOOP
        for(i = 32'h1ffffce; i<='h2000000; i= i+4) begin
            comp = i;
            addr_i  = 32'h40000000 + i;
            dat_i   = 0;
            stb_i   = 1;
            cyc_i   = 1;
            we_i    = 0;

            @(posedge clk_i);
            while (ack_o !== 1) begin
                @(posedge clk_i);
            end
            
            #1
            data = dat_o;

            assert(data, comp, cont); 

            addr_i  = 0;
            dat_i   = 0;
            stb_i   = 0;
            cyc_i   = 0;
            we_i    = 0;
            @(posedge clk_i);

        end 

        $display("the last 50 cells are tested");

        #13104
        $display("refresh aktivieren");

        #500

        // --- Final Testbench Evaluation ---

        if(cont != 0)begin
            $error("you have %d",cont);
        end else begin
            $display("testbench run was successful");
        end
        $display("end of the testbanch");
        
        $finish;


         
    end

endmodule