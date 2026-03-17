`include "math.vh"

`default_nettype none

module S_mem #(
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
    
    input wire [$clog2(S_MEM_DEPTH)-1:0]  addr,
    input wire [7:0]  din,
    
    output reg [7:0]  dout
);

// We add a zero and an all-1 entry for convenience (below)
(* ram_style = "distributed" *) reg [7:0] S_mem [S_MEM_DEPTH-1:0];

always @(posedge clk) begin
    dout <= S_mem[addr];
    if (wren)
        S_mem[addr] <= din;
end

endmodule
