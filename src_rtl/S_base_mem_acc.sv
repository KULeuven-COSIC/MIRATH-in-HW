/*
 * S_base_mem_acc.sv
 * -----------
 * This file implements an array of simple Dual-Port Ram instances used to
 * accumulate and store the S_base matrices for all MPCitH protocol iterations.
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
`include "mirath_hw_params.vh"

`default_nettype none

module S_base_mem_acc #(
    parameter M_PARAM_M = `M_PARAM_M,
    parameter M_PARAM_N = `M_PARAM_N,
    parameter M_PARAM_R = `M_PARAM_R,
    parameter M_PARAM_K = `M_PARAM_K,
    parameter WORD_SIZE = `WORD_SIZE,
    parameter S_BASE_MEM_DEPTH = `GET_NIBBLES(M_PARAM_M)*M_PARAM_R+1,
    parameter C_MEM_DEPTH = M_PARAM_N-M_PARAM_R+2,
    parameter TAU = `TAU,
    parameter SAMPLE_COUNT = 10, // TODO: parametrize for all levels
    parameter MEM_WIDTH_BYTES = 4
)(
    input wire rst,
    input wire clk,
    
//    input wire wren,
    input wire [$clog2(TAU)-1:0] mpc_round_S_base,
    input wire [7:0]             phi_i,
    input wire                  shift_en_next,
    input wire                  acc_sample_valid,
    input wire [WORD_SIZE-1:0]  acc_sample,
    
    input wire [$clog2(SAMPLE_COUNT)-1:0] sample_counter,
    
    input wire [$clog2(S_BASE_MEM_DEPTH)-1:0] re_addr_S_base,
    
    output reg [8*MEM_WIDTH_BYTES-1:0]  dout [TAU-1:0]
);

//reg phi_i_pip;
//always_ff @ (posedge clk) phi_i_pip <= phi_i;

localparam GRAB_REG_LEN = (`ROUND_TO_4(M_PARAM_M)*(M_PARAM_R-1)) +M_PARAM_M;
reg [GRAB_REG_LEN-1:0] grab_regs;

reg [31:0] phi_i_times_S;
always_ff @ (posedge clk) begin
    if (!grab_regs[0])
        phi_i_times_S[7:0] <= 8'h0;
    else
        phi_i_times_S[7:0] <= phi_i;
    
    if (!grab_regs[1])
        phi_i_times_S[15:8] <= 8'h0;
    else
        phi_i_times_S[15:8] <= phi_i;
    
    if (!grab_regs[2])
        phi_i_times_S[23:16] <= 8'h0;
    else
        phi_i_times_S[23:16] <= phi_i;
    
    if (!grab_regs[3])
        phi_i_times_S[31:24] <= 8'h0;
    else
        phi_i_times_S[31:24] <= phi_i;
end

reg shift_en; //, shift_en_pip;
always_ff @ (posedge clk) shift_en <= shift_en_next;
//always_ff @ (posedge clk) shift_en <= |{shift_init, shift_counter};
////always_ff @ (posedge clk) shift_en_pip <= shift_en;

always_ff @ (posedge clk) begin // grab_regs update
    if (rst)
       grab_regs <= 1'b0;
    else if (shift_en_next) 
        grab_regs <= (grab_regs >> MEM_WIDTH_BYTES);
    
    else if (acc_sample_valid) begin
        case (sample_counter)
            'h0: begin
                grab_regs[M_PARAM_M-1 : 0] <= acc_sample[M_PARAM_M-1 : 0];
                grab_regs[63-4 : `ROUND_TO_4(M_PARAM_M)] <= acc_sample[63 : `ROUND_TO_8(M_PARAM_M)];
             end
             
             'h1: begin
                grab_regs[M_PARAM_M+`ROUND_TO_4(M_PARAM_M)-1 : 60] <= acc_sample[M_PARAM_M +`ROUND_TO_8(M_PARAM_M)-64-1 : 0];
                grab_regs[127-8 : 2*`ROUND_TO_4(M_PARAM_M)] <= acc_sample[127-64 : 2*`ROUND_TO_8(M_PARAM_M)-64];
             end
             
             'h2: begin
                grab_regs[M_PARAM_M+2*`ROUND_TO_4(M_PARAM_M)-1 : 120] <= acc_sample[M_PARAM_M+2*`ROUND_TO_8(M_PARAM_M)-1-128 : 0];
                grab_regs[M_PARAM_M+3*`ROUND_TO_4(M_PARAM_M)-1 : 3*`ROUND_TO_4(M_PARAM_M)] <= acc_sample[M_PARAM_M+3*`ROUND_TO_8(M_PARAM_M)-1-128 : 3*`ROUND_TO_8(M_PARAM_M)-128];
             end
        endcase
    end
end

localparam FULL_ZEROS_S_BASE_MEM_ADDR = S_BASE_MEM_DEPTH-1;
       
reg [$clog2(S_BASE_MEM_DEPTH)-1:0] wr_addr_S_base;
always_ff @ (posedge clk) begin
    if (rst)
        wr_addr_S_base <= 'h0;
    else if (shift_en)
        wr_addr_S_base <= (wr_addr_S_base==S_BASE_MEM_DEPTH-2) ? 'h0 : (wr_addr_S_base+1'b1);
end   

genvar i;
generate
    for (i=0; i<TAU; i++) begin : TAU_S_base_mems
        localparam int IDX = i;
    
        reg mem_wren;
        always_ff @ (posedge clk) mem_wren <= shift_en_next && mpc_round_S_base==i;
        
        // Instantiate the mem
        (* ram_style = "distributed" *) 
        reg [8*MEM_WIDTH_BYTES-1:0] S_base_mem [S_BASE_MEM_DEPTH-1:0];
        
        // Set last address to zeros 
        initial S_base_mem[FULL_ZEROS_S_BASE_MEM_ADDR] = 'h0;
        
        always_ff @(posedge clk) begin
            dout[i] <= S_base_mem[re_addr_S_base];
            if (mem_wren)
                S_base_mem[wr_addr_S_base] <=  dout[i] ^    // FF_mu acc
                                               phi_i_times_S;
        end
    end
endgenerate

endmodule


//reg first_acc, new_round;
//always_ff @ (posedge clk)
//    if (rst)
//        new_round <= 1'b1;
//    else if (shift_en_pip && !shift_en) // First acc done
//        new_round <= 1'b0;
//    else if (phi_i_pip[7] && !phi_i[7])
