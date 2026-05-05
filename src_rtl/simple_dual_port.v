 /*
  * This file implements a simple dual-port memory module.
  *
  * Derived from an AMD/Xilinx memory template and modified for this project.
  *
  */

`include "clog2.v"

`default_nettype none

module simple_dual_port_mem #(
    parameter WIDTH = 32,
    parameter DEPTH = 512,
    parameter FILE = ""
)(
    input wire                      clk, wea,
    input wire [`CLOG2(DEPTH)-1:0]  addra, addrb,
    input wire [WIDTH-1:0]          dia,
    output reg [WIDTH-1:0]          dob
);

    reg [WIDTH-1:0] ram [0:DEPTH-1];
    
    initial begin
      // read file contents if FILE is given
      if (FILE != "")
//        $readmemb(FILE, ram);
        $readmemh(FILE, ram);
   end
    
    always @(posedge clk) begin
            dob <= ram[addrb];
    end
            
    always @(posedge clk) begin
            if (wea)
                ram[addra] <= dia;
    end
endmodule
