
/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)     2025 Hermann Zoha <hermann.zoha@tha.de> 
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
 
// module init
module tri_state (dq_s_io, dq_c_i, dq_c_o, c_we);

    inout   wire   [15:0]   dq_s_io;  // SDRAM INOUT-put
    input   wire   [15:0]   dq_c_i;   // SDRAM-Controller data input    
    output  wire   [15:0]   dq_c_o;   // SDRAM-Controller data output
    input   reg             c_we;     // Write enable

    // if c_we = 1, then we write it into dq_c_i.
    // but if c_we = 0, then it becomes high-impedance
    assign dq_s_io  = (c_we == 1'b1) ? dq_c_i : 16'bzzzz_zzzz_zzzz_zzzz;
    // Data is always read whenever ther is a signal on dq_s_io. 
    // However, if c_we = 0, the data from the SDRAM arrives here
    assign dq_c_o   = dq_s_io;

endmodule