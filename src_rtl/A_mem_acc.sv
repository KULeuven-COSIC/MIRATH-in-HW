/*
 * A_mem_acc.sv
 * -----------
 * This file corresponds to the VecAlpha MAC module shown in Figure 4.
 * Using v_rnd, Gamma, tmp and tmp', it computes vectors alpha_base and
 * alpha_mid.
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

`include "mirath_hw_params.vh"

`default_nettype none

module A_mem_acc #(
    parameter M_PARAM_RHO = `M_PARAM_RHO,
    parameter WORD_SIZE = `WORD_SIZE,
    parameter TAU = `TAU,
    parameter SAMPLE_COUNT = 10,
    parameter V_MEM_DEPTH = M_PARAM_RHO+1
)(
    input wire rst,
    input wire clk,

    input wire [$clog2(TAU)-1:0] mpc_round_V,
    input wire [7:0]             phi_i,
    input wire [7:0]             init_elem_Vb,
    input wire [7:0]             init_elem_V,
    input wire [7:0]             gamma_elem,
    input wire [7:0]             tmp_dout_base [`TAU-1:0],
    input wire [7:0]             tmp_dout_mid [`TAU-1:0],
    
    input wire                  acc_a,
    input wire                  shift_en_next,
    input wire                  shift_en,
    input wire                  acc_sample_valid,
    input wire [WORD_SIZE-1:0]  acc_sample,
    
    input wire [$clog2(SAMPLE_COUNT)-1:0] sample_counter,
        
    input wire [$clog2(V_MEM_DEPTH)-1:0] re_addr_V,
    
    output reg [7:0] dout_A_b [TAU-1:0],
    output reg [7:0] dout_A_m  [TAU-1:0]
);

reg acc_a_pip;
always_ff @ (posedge clk) acc_a_pip <= acc_a;

//reg shift_en_pip;
//always_ff @ (posedge clk) shift_en_pip <= shift_en;

reg [$clog2(V_MEM_DEPTH)-1:0] wr_addr_V;
always_ff @ (posedge clk) begin
    if (rst)
        wr_addr_V <= 'h0;
    else if (shift_en || acc_a_pip)
        wr_addr_V <= (wr_addr_V==V_MEM_DEPTH-2) ? 'h0 : (wr_addr_V+1'b1);
end   

genvar i;
generate
    for (i=0; i<TAU; i++) begin : TAU_A_mems
        localparam int IDX = i;
    
        reg mem_wren;
        always_ff @ (posedge clk) mem_wren <= (shift_en_next && mpc_round_V==i) || acc_a;
        
        // Instantiate the mem
        (* ram_style = "distributed" *) 
        reg [7:0] A_base_mem [V_MEM_DEPTH-1:0];
        reg [7:0] A_mid_mem [V_MEM_DEPTH-1:0];
        
        wire [7:0] gamma_mul_TMP,     gamma_mul_TMP_prime;
        reg  [7:0] gamma_mul_TMP_reg, gamma_mul_TMP_prime_reg;
        
        gf256_mul_mast mul_gamma_by_tmp ( // gf256_mul_mast instance
            .a  (gamma_elem),
            .b  (tmp_dout_base[i]),
            .p  (gamma_mul_TMP)
        );
        always_ff @ (posedge clk) gamma_mul_TMP_reg <= gamma_mul_TMP;
        
        gf256_mul_mast mul_gamma_by_tmp_pr ( // gf256_mul_mast instance
            .a  (gamma_elem),
            .b  (tmp_dout_mid[i]),
            .p  (gamma_mul_TMP_prime)
        );
        always_ff @ (posedge clk) gamma_mul_TMP_prime_reg <= gamma_mul_TMP_prime;
        
        // Set last address to zeros 
        initial begin
            A_base_mem[V_MEM_DEPTH-1] = 8'h0;
            A_mid_mem[V_MEM_DEPTH-1] = 8'h0;
        end
        
        always_ff @(posedge clk) begin
            dout_A_b[i] <= A_base_mem[re_addr_V];
            if (mem_wren) begin
                A_base_mem[wr_addr_V] <=  dout_A_b[i] ^    // FF_mu acc
                                  (acc_a_pip ? gamma_mul_TMP_reg : init_elem_Vb);
            end
                                              
            dout_A_m[i] <= A_mid_mem[re_addr_V];
            if (mem_wren) begin
                A_mid_mem[wr_addr_V] <=  dout_A_m[i] ^    // FF_mu acc
                                  (acc_a_pip ? gamma_mul_TMP_prime_reg : init_elem_V);
            end
                                               
        end
    end
endgenerate

endmodule
