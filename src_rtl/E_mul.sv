/*
 * E_mul.sv
 * ----------
 * This is a multiplier array used to compute E_base and E_mid.
 * It corresponds to the MatE MUL module shown in Figure 4.
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
`include "E_mul_defines.svh"

`default_nettype none

module E_mul(
    input wire              rst, 
    input wire              clk,
     
    input wire              S_elem, 
    input wire              C_elem, 
    input wire [7:0]        S_b_elem [`TAU-1:0], 
    input wire [7:0]        C_b_elem [`TAU-1:0],
    
    input wire E_mul_opcode_t    E_mul_opc_in,
    
    output reg [7:0]        E_b_elem [`TAU-1:0],
    output reg [7:0]        E_m_elem [`TAU-1:0]
);

E_mul_opcode_t  E_mul_opc;
//always_ff @ (posedge clk) E_mul_opc <= E_mul_opc_in;
assign E_mul_opc = E_mul_opc_in;

wire [7:0] E_base_add_terms [`TAU-1:0];
wire [7:0] E_mid_add_terms [`TAU-1:0];

genvar i;
generate
    for (i=0; i<`TAU; i=i+1) begin: gen_muls
        gf256_mul_mast Vb_load_mul_inst ( // gf256_mul_mast instance
            .a  (S_b_elem[i]),
            .b  (C_b_elem [i]),
            .p  (E_base_add_terms[i])
        );
        
        assign E_mid_add_terms[i] = (S_elem ? C_b_elem[i] : 8'h0) ^ (C_elem ? S_b_elem[i] : 8'h0);
    end
endgenerate

always_ff @ (posedge clk) begin
    case (E_mul_opc)
        init_e: begin
            E_b_elem <= E_base_add_terms;
            E_m_elem <= E_mid_add_terms;
        end
        
        acc_e: begin
            for (int j = 0; j < `TAU; j++) begin
                E_b_elem[j] <= E_b_elem[j] ^ E_base_add_terms[j];
                E_m_elem[j] <= E_m_elem[j] ^ E_mid_add_terms[j];
            end
        end
    endcase // E_mul_opc
end

endmodule
