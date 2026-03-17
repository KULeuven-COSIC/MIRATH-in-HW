`include "math.vh"

`default_nettype none

module C_mem #(
    parameter M_PARAM_M = 42,
    parameter M_PARAM_N = M_PARAM_M,
    parameter M_PARAM_R = 4,
    parameter M_PARAM_K = 1443,
    parameter WORD_SIZE = 64,
    parameter S_MEM_DEPTH = `GET_BYTES(M_PARAM_M)*M_PARAM_R,
    parameter C_MEM_DEPTH = M_PARAM_N-M_PARAM_R+2
)(
    input wire clk,
    
    input wire wren,
    
    input wire [$clog2(C_MEM_DEPTH)-1:0]  addr,
    input wire [M_PARAM_R-1:0]  din,
    
    output reg [M_PARAM_R-1:0]  dout
);

// We add a zero and an all-1 entry for convenience (below) -> Turned to 2 zero addresses
(* ram_style = "distributed" *) reg [M_PARAM_R-1:0] C_mem [C_MEM_DEPTH-1:0];

localparam FULL_ZEROS_C_ADDR = M_PARAM_N-M_PARAM_R;

initial begin   // r is at most 6 for all parameter sets
    C_mem[FULL_ZEROS_C_ADDR]   = 6'h00;  // 2nd to last
    C_mem[FULL_ZEROS_C_ADDR+1] = 6'h00;  // Last -> Also put to zeros
end

always @(posedge clk) begin
    dout <= C_mem[addr];
    if (wren)
        C_mem[addr] <= din;
end

endmodule
