 /*
  * This file implements a simplified dual-port memory module with optional hex-file initialization.
  *
  * Derived from an AMD/Xilinx memory template and modified for this project.
  *
  */

`include "clog2.v"

`default_nettype none

module single_port_mem #(
    parameter WIDTH = 32,
    parameter DEPTH = 512,
    parameter FILE = "",
    parameter LAST_ENTRY_0 = 0
)(
    input wire                      clk, wea,
    input wire [`CLOG2(DEPTH)-1:0]  addra,
    input wire [WIDTH-1:0]          dia,
    output reg [WIDTH-1:0]          doa
);

    reg [WIDTH-1:0] ram [0:DEPTH-1];
    
    initial begin
      // set last entry to 0 if LAST_ENTRY_0 is true
      if (LAST_ENTRY_0)
          ram[DEPTH-1] = {WIDTH{1'b0}};  
        
      // read file contents if FILE is given
      if (FILE != "")
//        $readmemb(FILE, ram);
        $readmemh(FILE, ram);
   end
    
    always @(posedge clk) begin
        doa <= ram[addra];
        
        if (wea)
            ram[addra] <= dia;
    end
    
endmodule
