module m_blockram_memu (clka, clkb,ena,enb,wea,web,addra,addrb,dia,dib,doa,dob);

parameter   RAM_SIZE   =   {RAM_SIZE},
            NUM_COL    =   4,
            COL_WIDTH  =   8,
            ADDR_WIDTH =   32,
            DATA_WIDTH =   32;

input clka, clkb,ena,enb;
input [NUM_COL-1:0] wea,web;
input [ADDR_WIDTH-1:0] addra,addrb;
input [DATA_WIDTH-1:0] dia,dib;
output [DATA_WIDTH-1:0] doa,dob;

reg [DATA_WIDTH-1:0] ram [RAM_SIZE-1:0];
reg [DATA_WIDTH-1:0] doa,dob;

wire [ADDR_WIDTH - 1:0] addra_internal = addra - (32'h10000000 >> 2);
wire [ADDR_WIDTH - 1:0] addrb_internal = addrb - (32'h10000000 >> 2);

initial begin
    $readmemb("text.mem", ram, 32'h0);
    $readmemb("data.mem", ram, 32'h8000);
end

integer                i;
always @(posedge clka) begin
    if(ena) begin
        for(i=0;i<NUM_COL;i=i+1) begin
            if(wea[i]) begin
                ram[addra_internal][i*COL_WIDTH +: COL_WIDTH] <= dia[i*COL_WIDTH +: COL_WIDTH];
            end
        end

        doa <= ram[addra_internal];
    end
end

always @(posedge clkb) begin
    if(enb) begin
        for(i=0;i<NUM_COL;i=i+1) begin
            if(web[i]) begin
                ram[addrb_internal][i*COL_WIDTH +: COL_WIDTH] <= dib[i*COL_WIDTH +: COL_WIDTH];
            end
        end
        dob <= ram[addrb_internal];
    end
end

endmodule