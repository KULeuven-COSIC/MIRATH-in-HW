/*
 * S_mem.sv
 * ---------
 * This file implements a RAM that stores the secret (witness) matrix S
 *
 * Copyright (c) 2026 KU Leuven - COSIC
 * Author: Stelios Manasidis    
 *        
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
 
`include "math.vh"

`default_nettype none

module S_mem #(
    parameter M_PARAM_M = `M_PARAM_M,
    parameter M_PARAM_N = `M_PARAM_N,
    parameter M_PARAM_R = `M_PARAM_R,
    parameter M_PARAM_K = `M_PARAM_K,
    parameter WORD_SIZE = `WORD_SIZE,
    parameter S_MEM_DEPTH = `GET_BYTES(M_PARAM_M)*M_PARAM_R,
    parameter C_MEM_DEPTH = M_PARAM_N-M_PARAM_R+2
)(
    input wire clk,
    
    input wire wren,
    
    input wire [$clog2(S_MEM_DEPTH)-1:0]  addr,
    input wire [7:0]  din,
    
    output reg [7:0]  dout
);

(* ram_style = "distributed" *) reg [7:0] S_mem [S_MEM_DEPTH-1:0];

always @(posedge clk) begin
    dout <= S_mem[addr];
    if (wren)
        S_mem[addr] <= din;
end

endmodule
