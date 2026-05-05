/*
 * TMP_wrapper.sv
 * -----------
 * This file corresponds to the VecTmp MAC module shown in Figure 4.
 * Using E_base and E_mid, along with H', it computes vectors tmp and
 * tmp'.
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

module TMP_wrapper #(
    parameter M_VAR_E_A = `MIRATH_VAR_E_A,
    parameter TAU       = `TAU,
    parameter WORD_SIZE = `WORD_SIZE,
    parameter TMP_MEM_W = 8 * 8, // 8 elements of 8 bits
    parameter TMP_MEM_D = `GET_BYTES(M_VAR_E_A) + 1 // As many as we need to process H' in real-time plus a full-zero address
)(
    input wire              rst, 
    input wire              clk,
     
    input wire              H_done,
    input wire              E_mul_res_valid_in_1,
    input wire              H_valid,
    input wire [7:0]        H_elements,
    input wire [7:0]        E_base_elem_in [`TAU-1:0],
    input wire [7:0]        E_mid_elem_in [`TAU-1:0],
    input wire [$clog2(`N-1)-1:0]      i_star_elem_in,
    input wire                         i_star_valid_in,
    input wire [`WORD_SIZE-1:0]        y_word,
    input wire                         y_word_valid,
    
    output reg              next_E_ff_mu,
    output reg              next_y_word,
    output reg              tmp_prefill_done,
    output reg [7:0]        tmp_dout_base [`TAU-1:0],
    output reg [7:0]        tmp_dout_mid [`TAU-1:0]
);
reg [8*`TAU-1:0]        i_star_regs;
reg [`WORD_SIZE-1:0]    y_word_reg;

//reg [TMP_MEM_W-1:0]  dout_tmp_base [TAU-1:0]; // TMP_MEM_W = 8 * 8
//reg [TMP_MEM_W-1:0]  dout_tmp_mid  [TAU-1:0];
wire [TMP_MEM_W-1:0]  dout_tmp_base [TAU-1:0]; // TMP_MEM_W = 8 * 8
wire [TMP_MEM_W-1:0]  dout_tmp_mid  [TAU-1:0];
reg [71:0]  dout_tmp_base_merged [TAU*8/9-1:0]; // 32 dout buses - mem_width = max RAMB36 width
reg [71:0]  dout_tmp_mid_merged  [TAU*8/9-1:0];

reg [2:0] H_row_sub_ctr, H_row_sub_ctr_pip;
always_ff @ (posedge clk) begin
    H_row_sub_ctr_pip <= H_row_sub_ctr;
    
    for (int k=0; k<`TAU; k++) begin
        tmp_dout_base[k] <= dout_tmp_base[k][({3'b0, H_row_sub_ctr_pip}<<3) +: 8];
        tmp_dout_mid[k]  <= dout_tmp_mid[k][({3'b0, H_row_sub_ctr_pip}<<3) +: 8];
    end
end

reg next_E_ff_mu_comb, H_done_pip;
always_ff @ (posedge clk) next_E_ff_mu <= next_E_ff_mu_comb;
always_ff @ (posedge clk) H_done_pip <= H_done && state_tmp==ACC_H_eB;

reg E_mul_res_valid, E_mul_res_valid_pip;
always_ff @ (posedge clk) E_mul_res_valid      <= rst ? 1'b0 : E_mul_res_valid_in_1;
always_ff @ (posedge clk) E_mul_res_valid_pip  <= rst ? 1'b0 : E_mul_res_valid;

reg         H_valid_pip, H_valid_pip_2, H_valid_pip_3;
reg [7:0]   H_elements_pip;
always_ff @ (posedge clk) H_valid_pip_3  <= H_valid_pip_2;
always_ff @ (posedge clk) H_valid_pip_2  <= H_valid_pip;
always_ff @ (posedge clk) H_valid_pip    <= H_valid;
always_ff @ (posedge clk) H_elements_pip <= H_valid ? H_elements : 'h0;

reg [7:0]        E_base_elem [`TAU-1:0];
reg [7:0]        E_mid_elem [`TAU-1:0];
always_ff @(posedge clk) begin
    if ((E_mul_res_valid_pip && !first_e_B_loaded)
     || ((wr_addr_TMP==H_ROWS_DIV_8-'h1) && state_tmp==ACC_H_eB && H_valid_pip_2)) begin
        
        for (int j = 0; j < `TAU; j++) begin
            E_base_elem[j] <= E_base_elem_in[j];
            E_mid_elem[j] <= E_mid_elem_in[j];
        end
    end
end

typedef enum logic [1:0] {
    FILL_eA,
    ACC_H_eB,
    COMPUTE_A,
    DONE
} state_tmp_t;

state_tmp_t state_tmp, next_state_tmp;

reg [$clog2(TMP_MEM_D)-1:0] wr_addr_TMP, re_addr_TMP;
reg incr_wr_addr, incr_wr_addr_next, incr_re_addr;
always_ff @ (posedge clk) begin
    if (rst) begin
        incr_wr_addr <= 1'b0;
        wr_addr_TMP <= 'h0;
        re_addr_TMP <= TMP_MEM_D-1'b1;
    end else begin
        incr_wr_addr <= incr_wr_addr_next;
    
        if (incr_wr_addr)
            wr_addr_TMP <= (wr_addr_TMP==TMP_MEM_D-2) ? 'h0 : (wr_addr_TMP+1'b1);
            
        if (H_done_pip)
            re_addr_TMP <= 'h0;
        else if (incr_re_addr)
            re_addr_TMP <= (re_addr_TMP==(TMP_MEM_D-2) || re_addr_TMP==(TMP_MEM_D-1)) ? 'h0 : (re_addr_TMP+1'b1);
    end
end

localparam H_ROWS_DIV_8 = `GET_BYTES(M_VAR_E_A);
reg [$clog2(H_ROWS_DIV_8)-1:0] H_row_div_8_ctr;

localparam MAX_COUNT_RHO_CTR = `M_PARAM_RHO-1;
localparam RHO_CTR_BITS      = $clog2(MAX_COUNT_RHO_CTR);
reg [RHO_CTR_BITS-1:0] rho_ctr;
reg incr_rho_ctr;

always_ff @ (posedge clk) begin
    if (rst)
        rho_ctr <= 'h0;
    else if (incr_rho_ctr)
        rho_ctr <= rho_ctr + 1'b1;
//        rho_ctr <= (rho_ctr==MAX_COUNT_RHO_CTR) ? 'h0 : (rho_ctr + 1'b1); // For L3 only
end

reg incr_H_row_ctr, incr_H_sub_row_ctr;
//reg rst_H_row_ctr;
reg rst_H_sub_row_ctr;
always_ff @ (posedge clk) begin
    if (rst)
        H_row_div_8_ctr <= 'h0;
//    else if (rst_H_row_ctr)
//        H_row_div_8_ctr <= 'h0;
    else if (incr_H_row_ctr)
        H_row_div_8_ctr <= (H_row_div_8_ctr==H_ROWS_DIV_8-'h1) ? 'h0 : (H_row_div_8_ctr + 1'b1);
        
    if (rst)
        H_row_sub_ctr <= 'h0;
    else if (rst_H_sub_row_ctr)
        H_row_sub_ctr <= 'h0;
    else if (incr_H_sub_row_ctr)
        H_row_sub_ctr <= H_row_sub_ctr + 1'b1;
end

//reg eA_first_elements;
//always_ff @ (posedge clk) begin
//    if (rst)
//        eA_first_elements <= 1'b1;
//    else if (H_row_div_8_ctr==(`GET_BYTES(`S_ROWS*`S_COLS)))
//        eA_first_elements <= 1'b0;
//end

reg first_e_B_loaded;
always_ff @ (posedge clk) begin
    if (rst)
        first_e_B_loaded <= 1'b0;
    else if (E_mul_res_valid_pip && state_tmp==ACC_H_eB)
        first_e_B_loaded <= 1'b1;
end

// State logic:
always_comb begin
    // Defaults
    incr_rho_ctr      = 1'b0;
    incr_H_row_ctr      = 1'b0;
//    rst_H_row_ctr       = 1'b0;
    incr_H_sub_row_ctr  = 1'b0;
    rst_H_sub_row_ctr   = 1'b0;
    incr_wr_addr_next   = 1'b0;
    incr_re_addr        = 1'b0;
    
    next_E_ff_mu_comb   = 1'b0;
    
    case (state_tmp)
        FILL_eA: begin
            next_E_ff_mu_comb = 1'b1;
            
            if (E_mul_res_valid_pip) begin
                incr_H_sub_row_ctr = 1'b1;
                
                if (H_row_sub_ctr=='h7 || (H_row_div_8_ctr==(H_ROWS_DIV_8-'h1) && H_row_sub_ctr=='h4)) begin
                    incr_H_row_ctr = 1'b1;
                    incr_wr_addr_next = 1'b1;
                end
                
                if (H_row_div_8_ctr==(H_ROWS_DIV_8-'h1) && H_row_sub_ctr=='h4)
                    incr_re_addr = 1'b1;
            end
        end
        
        ACC_H_eB: begin
            if (H_valid_pip)
                incr_re_addr = 1'b1;
                
            if (H_valid_pip) begin
                incr_wr_addr_next = 1'b1;
                incr_H_row_ctr = 1'b1;
                
                if (H_row_div_8_ctr=='h14)
                    next_E_ff_mu_comb = 1'b1;
            end
            
            if (H_done_pip) begin
//                rst_H_row_ctr       = 1'b1;
                rst_H_sub_row_ctr   = 1'b1;
            end
        end
        
        COMPUTE_A: begin
            if (H_valid_pip) begin // In this case Gamma valid, but it comes from the same wires
                incr_rho_ctr = 1'b1;
                
                if (rho_ctr == MAX_COUNT_RHO_CTR) begin
                    incr_H_sub_row_ctr = 1'b1;
                    
                    if (H_row_sub_ctr=='h7) begin
                        incr_re_addr = 1'b1;
                        rst_H_sub_row_ctr = 1'b1;
                    end
                end
            end
        end
        
    endcase // state_tmp
end


always_ff @ (posedge clk) tmp_prefill_done <= (E_mul_res_valid_pip && H_row_sub_ctr=='h4 && H_row_div_8_ctr==(H_ROWS_DIV_8-'h1));

// ***************************************************
// FSM state update:
always_ff @ (posedge clk) begin
    if (rst)
        state_tmp <= FILL_eA;
    else
        state_tmp <= next_state_tmp;
end

always_comb begin
    next_state_tmp = state_tmp; // Default
    
    case(state_tmp)
        FILL_eA:
            if (E_mul_res_valid_pip && H_row_div_8_ctr==(H_ROWS_DIV_8-'h1) && H_row_sub_ctr=='h4)
                next_state_tmp = ACC_H_eB;
        
        ACC_H_eB:
            if (H_done_pip)
                next_state_tmp = COMPUTE_A;
                   
    endcase // state_tmp
end


// ***************************************************
// Memory instantiations
reg mem_wren[7:0];
always_ff @(posedge clk) begin            
    for (int k=0; k<8; k++) begin             
        mem_wren[k] <= ((state_tmp==FILL_eA)  && (E_mul_res_valid_pip) && (H_row_sub_ctr==k))
            || ((state_tmp==ACC_H_eB) && (H_elements_pip[k]));
    end
end

// Compute y * p^2 for verification:
wire [7:0] p_pow_2_mul_out [0:TAU-1];
reg [7:0]  y_times_p_squared [0:TAU-1];
always_ff @ (posedge clk) begin
    for (int j=0; j<TAU; j++) begin
      y_times_p_squared[j] <= (~y_word_reg[0]) ? 8'h00 : p_pow_2_mul_out[j];
    end
end

genvar i;
genvar l;
generate // Only for TAU=='d36
    for (i=0; i<TAU; i++) begin : TAU_p_pow_2_muls
        gf256_squaring_mul mul_p_pow_2_inst (
            .a  (i_star_regs[8*i +: 8]),
            .p  (p_pow_2_mul_out[i])
        );
    end
    
    for (i=0; i<TAU; i+=9) begin : TAU_TMP_mems_x9 // We take advantage of the maximum width of a BRAM36 being 72 bits
        localparam int I0 = i;
        localparam int I1 = i+1;
        localparam int I2 = i+2;
        localparam int I3 = i+3;
        localparam int I4 = i+4;
        localparam int I5 = i+5;
        localparam int I6 = i+6;
        localparam int I7 = i+7;
        localparam int I8 = i+8;
        
        // Instantiate the mems
        (* ram_style="block" *) reg [71:0] TMP_base_mem_0 [TMP_MEM_D-1:0];
        (* ram_style="block" *) reg [71:0] TMP_base_mem_1 [TMP_MEM_D-1:0];
        (* ram_style="block" *) reg [71:0] TMP_base_mem_2 [TMP_MEM_D-1:0];
        (* ram_style="block" *) reg [71:0] TMP_base_mem_3 [TMP_MEM_D-1:0];
        (* ram_style="block" *) reg [71:0] TMP_base_mem_4 [TMP_MEM_D-1:0];
        (* ram_style="block" *) reg [71:0] TMP_base_mem_5 [TMP_MEM_D-1:0];
        (* ram_style="block" *) reg [71:0] TMP_base_mem_6 [TMP_MEM_D-1:0];
        (* ram_style="block" *) reg [71:0] TMP_base_mem_7 [TMP_MEM_D-1:0];
        
        (* ram_style="block" *) reg [71:0] TMP_mid_mem_0  [TMP_MEM_D-1:0];
        (* ram_style="block" *) reg [71:0] TMP_mid_mem_1  [TMP_MEM_D-1:0];
        (* ram_style="block" *) reg [71:0] TMP_mid_mem_2  [TMP_MEM_D-1:0];
        (* ram_style="block" *) reg [71:0] TMP_mid_mem_3  [TMP_MEM_D-1:0];
        (* ram_style="block" *) reg [71:0] TMP_mid_mem_4  [TMP_MEM_D-1:0];
        (* ram_style="block" *) reg [71:0] TMP_mid_mem_5  [TMP_MEM_D-1:0];
        (* ram_style="block" *) reg [71:0] TMP_mid_mem_6  [TMP_MEM_D-1:0];
        (* ram_style="block" *) reg [71:0] TMP_mid_mem_7  [TMP_MEM_D-1:0];
        
        // Set last address to zeros for initial prefill
        initial TMP_base_mem_0[TMP_MEM_D-1] = 'h0;
        initial TMP_base_mem_1[TMP_MEM_D-1] = 'h0;
        initial TMP_base_mem_2[TMP_MEM_D-1] = 'h0;
        initial TMP_base_mem_3[TMP_MEM_D-1] = 'h0;
        initial TMP_base_mem_4[TMP_MEM_D-1] = 'h0;
        initial TMP_base_mem_5[TMP_MEM_D-1] = 'h0;
        initial TMP_base_mem_6[TMP_MEM_D-1] = 'h0;
        initial TMP_base_mem_7[TMP_MEM_D-1] = 'h0;
        
        initial TMP_mid_mem_0 [TMP_MEM_D-1] = 'h0;
        initial TMP_mid_mem_1 [TMP_MEM_D-1] = 'h0;
        initial TMP_mid_mem_2 [TMP_MEM_D-1] = 'h0;
        initial TMP_mid_mem_3 [TMP_MEM_D-1] = 'h0;
        initial TMP_mid_mem_4 [TMP_MEM_D-1] = 'h0;
        initial TMP_mid_mem_5 [TMP_MEM_D-1] = 'h0;
        initial TMP_mid_mem_6 [TMP_MEM_D-1] = 'h0;
        initial TMP_mid_mem_7 [TMP_MEM_D-1] = 'h0;
        
        localparam integer GROUP = i/9;
        always_ff @(posedge clk) begin            
            // Lane 0 (given)
            dout_tmp_base_merged[8*GROUP+0] <= TMP_base_mem_0[re_addr_TMP];
            dout_tmp_mid_merged [8*GROUP+0] <= TMP_mid_mem_0 [re_addr_TMP];
            if (mem_wren[0]) begin
                TMP_base_mem_0[wr_addr_TMP] <= dout_tmp_base_merged[8*GROUP+0] ^ {
                    (E_base_elem[I8] ^ y_times_p_squared[I8]),
                    (E_base_elem[I7] ^ y_times_p_squared[I7]),
                    (E_base_elem[I6] ^ y_times_p_squared[I6]),
                    (E_base_elem[I5] ^ y_times_p_squared[I5]),
                    (E_base_elem[I4] ^ y_times_p_squared[I4]),
                    (E_base_elem[I3] ^ y_times_p_squared[I3]),
                    (E_base_elem[I2] ^ y_times_p_squared[I2]),
                    (E_base_elem[I1] ^ y_times_p_squared[I1]),
                    (E_base_elem[I0] ^ y_times_p_squared[I0])
                };
            
                TMP_mid_mem_0[wr_addr_TMP] <= dout_tmp_mid_merged[8*GROUP+0] ^ {
                    E_mid_elem[I8], E_mid_elem[I7], E_mid_elem[I6], E_mid_elem[I5],
                    E_mid_elem[I4], E_mid_elem[I3], E_mid_elem[I2], E_mid_elem[I1], E_mid_elem[I0]
                };
            end
            
            // Lane 1
            dout_tmp_base_merged[8*GROUP+1] <= TMP_base_mem_1[re_addr_TMP];
            dout_tmp_mid_merged [8*GROUP+1] <= TMP_mid_mem_1 [re_addr_TMP];
            if (mem_wren[1]) begin
                TMP_base_mem_1[wr_addr_TMP] <= dout_tmp_base_merged[8*GROUP+1] ^ {
                    (E_base_elem[I8] ^ y_times_p_squared[I8]),//
                    (E_base_elem[I7] ^ y_times_p_squared[I7]),
                    (E_base_elem[I6] ^ y_times_p_squared[I6]),
                    (E_base_elem[I5] ^ y_times_p_squared[I5]),
                    (E_base_elem[I4] ^ y_times_p_squared[I4]),
                    (E_base_elem[I3] ^ y_times_p_squared[I3]),
                    (E_base_elem[I2] ^ y_times_p_squared[I2]),
                    (E_base_elem[I1] ^ y_times_p_squared[I1]),
                    (E_base_elem[I0] ^ y_times_p_squared[I0])
                };
            
                TMP_mid_mem_1[wr_addr_TMP] <= dout_tmp_mid_merged[8*GROUP+1] ^ {
                    E_mid_elem[I8], E_mid_elem[I7], E_mid_elem[I6], E_mid_elem[I5],
                    E_mid_elem[I4], E_mid_elem[I3], E_mid_elem[I2], E_mid_elem[I1], E_mid_elem[I0]
                };
            end
            
            // Lane 2
            dout_tmp_base_merged[8*GROUP+2] <= TMP_base_mem_2[re_addr_TMP];
            dout_tmp_mid_merged [8*GROUP+2] <= TMP_mid_mem_2 [re_addr_TMP];
            if (mem_wren[2]) begin
                TMP_base_mem_2[wr_addr_TMP] <= dout_tmp_base_merged[8*GROUP+2] ^ {
                    (E_base_elem[I8] ^ y_times_p_squared[I8]),
                    (E_base_elem[I7] ^ y_times_p_squared[I7]),
                    (E_base_elem[I6] ^ y_times_p_squared[I6]),
                    (E_base_elem[I5] ^ y_times_p_squared[I5]),
                    (E_base_elem[I4] ^ y_times_p_squared[I4]),
                    (E_base_elem[I3] ^ y_times_p_squared[I3]),
                    (E_base_elem[I2] ^ y_times_p_squared[I2]),
                    (E_base_elem[I1] ^ y_times_p_squared[I1]),
                    (E_base_elem[I0] ^ y_times_p_squared[I0])
                };
            
                TMP_mid_mem_2[wr_addr_TMP] <= dout_tmp_mid_merged[8*GROUP+2] ^ {
                    E_mid_elem[I8], E_mid_elem[I7], E_mid_elem[I6], E_mid_elem[I5],
                    E_mid_elem[I4], E_mid_elem[I3], E_mid_elem[I2], E_mid_elem[I1], E_mid_elem[I0]
                };
            end
            
            // Lane 3
            dout_tmp_base_merged[8*GROUP+3] <= TMP_base_mem_3[re_addr_TMP];
            dout_tmp_mid_merged [8*GROUP+3] <= TMP_mid_mem_3 [re_addr_TMP];
            if (mem_wren[3]) begin
                TMP_base_mem_3[wr_addr_TMP] <= dout_tmp_base_merged[8*GROUP+3] ^ {
                    (E_base_elem[I8] ^ y_times_p_squared[I8]),
                    (E_base_elem[I7] ^ y_times_p_squared[I7]),
                    (E_base_elem[I6] ^ y_times_p_squared[I6]),
                    (E_base_elem[I5] ^ y_times_p_squared[I5]),
                    (E_base_elem[I4] ^ y_times_p_squared[I4]),
                    (E_base_elem[I3] ^ y_times_p_squared[I3]),
                    (E_base_elem[I2] ^ y_times_p_squared[I2]),
                    (E_base_elem[I1] ^ y_times_p_squared[I1]),
                    (E_base_elem[I0] ^ y_times_p_squared[I0])
                };
            
                TMP_mid_mem_3[wr_addr_TMP] <= dout_tmp_mid_merged[8*GROUP+3] ^ {
                    E_mid_elem[I8], E_mid_elem[I7], E_mid_elem[I6], E_mid_elem[I5],
                    E_mid_elem[I4], E_mid_elem[I3], E_mid_elem[I2], E_mid_elem[I1], E_mid_elem[I0]
                };
            end
            
            // Lane 4
            dout_tmp_base_merged[8*GROUP+4] <= TMP_base_mem_4[re_addr_TMP];
            dout_tmp_mid_merged [8*GROUP+4] <= TMP_mid_mem_4 [re_addr_TMP];
            if (mem_wren[4]) begin
                TMP_base_mem_4[wr_addr_TMP] <= dout_tmp_base_merged[8*GROUP+4] ^ {
                    (E_base_elem[I8] ^ y_times_p_squared[I8]),
                    (E_base_elem[I7] ^ y_times_p_squared[I7]),
                    (E_base_elem[I6] ^ y_times_p_squared[I6]),
                    (E_base_elem[I5] ^ y_times_p_squared[I5]),
                    (E_base_elem[I4] ^ y_times_p_squared[I4]),
                    (E_base_elem[I3] ^ y_times_p_squared[I3]),
                    (E_base_elem[I2] ^ y_times_p_squared[I2]),
                    (E_base_elem[I1] ^ y_times_p_squared[I1]),
                    (E_base_elem[I0] ^ y_times_p_squared[I0])
                };
            
                TMP_mid_mem_4[wr_addr_TMP] <= dout_tmp_mid_merged[8*GROUP+4] ^ {
                    E_mid_elem[I8], E_mid_elem[I7], E_mid_elem[I6], E_mid_elem[I5],
                    E_mid_elem[I4], E_mid_elem[I3], E_mid_elem[I2], E_mid_elem[I1], E_mid_elem[I0]
                };
            end
            
            // Lane 5
            dout_tmp_base_merged[8*GROUP+5] <= TMP_base_mem_5[re_addr_TMP];
            dout_tmp_mid_merged [8*GROUP+5] <= TMP_mid_mem_5 [re_addr_TMP];
            if (mem_wren[5]) begin
                TMP_base_mem_5[wr_addr_TMP] <= dout_tmp_base_merged[8*GROUP+5] ^ {
                    (E_base_elem[I8] ^ y_times_p_squared[I8]),
                    (E_base_elem[I7] ^ y_times_p_squared[I7]),
                    (E_base_elem[I6] ^ y_times_p_squared[I6]),
                    (E_base_elem[I5] ^ y_times_p_squared[I5]),
                    (E_base_elem[I4] ^ y_times_p_squared[I4]),
                    (E_base_elem[I3] ^ y_times_p_squared[I3]),
                    (E_base_elem[I2] ^ y_times_p_squared[I2]),
                    (E_base_elem[I1] ^ y_times_p_squared[I1]),
                    (E_base_elem[I0] ^ y_times_p_squared[I0])
                };
            
                TMP_mid_mem_5[wr_addr_TMP] <= dout_tmp_mid_merged[8*GROUP+5] ^ {
                    E_mid_elem[I8], E_mid_elem[I7], E_mid_elem[I6], E_mid_elem[I5],
                    E_mid_elem[I4], E_mid_elem[I3], E_mid_elem[I2], E_mid_elem[I1], E_mid_elem[I0]
                };
            end
            
            // Lane 6
            dout_tmp_base_merged[8*GROUP+6] <= TMP_base_mem_6[re_addr_TMP];
            dout_tmp_mid_merged [8*GROUP+6] <= TMP_mid_mem_6 [re_addr_TMP];
            if (mem_wren[6]) begin
                TMP_base_mem_6[wr_addr_TMP] <= dout_tmp_base_merged[8*GROUP+6] ^ {
                    (E_base_elem[I8] ^ y_times_p_squared[I8]),
                    (E_base_elem[I7] ^ y_times_p_squared[I7]),
                    (E_base_elem[I6] ^ y_times_p_squared[I6]),
                    (E_base_elem[I5] ^ y_times_p_squared[I5]),
                    (E_base_elem[I4] ^ y_times_p_squared[I4]),
                    (E_base_elem[I3] ^ y_times_p_squared[I3]),
                    (E_base_elem[I2] ^ y_times_p_squared[I2]),
                    (E_base_elem[I1] ^ y_times_p_squared[I1]),
                    (E_base_elem[I0] ^ y_times_p_squared[I0])
                };
            
                TMP_mid_mem_6[wr_addr_TMP] <= dout_tmp_mid_merged[8*GROUP+6] ^ {
                    E_mid_elem[I8], E_mid_elem[I7], E_mid_elem[I6], E_mid_elem[I5],
                    E_mid_elem[I4], E_mid_elem[I3], E_mid_elem[I2], E_mid_elem[I1], E_mid_elem[I0]
                };
            end
            
            // Lane 7
            dout_tmp_base_merged[8*GROUP+7] <= TMP_base_mem_7[re_addr_TMP];
            dout_tmp_mid_merged [8*GROUP+7] <= TMP_mid_mem_7 [re_addr_TMP];
            if (mem_wren[7]) begin
                TMP_base_mem_7[wr_addr_TMP] <= dout_tmp_base_merged[8*GROUP+7] ^ {
                    (E_base_elem[I8] ^ y_times_p_squared[I8]),
                    (E_base_elem[I7] ^ y_times_p_squared[I7]),
                    (E_base_elem[I6] ^ y_times_p_squared[I6]),
                    (E_base_elem[I5] ^ y_times_p_squared[I5]),
                    (E_base_elem[I4] ^ y_times_p_squared[I4]),
                    (E_base_elem[I3] ^ y_times_p_squared[I3]),
                    (E_base_elem[I2] ^ y_times_p_squared[I2]),
                    (E_base_elem[I1] ^ y_times_p_squared[I1]),
                    (E_base_elem[I0] ^ y_times_p_squared[I0])
                };
            
                TMP_mid_mem_7[wr_addr_TMP] <= dout_tmp_mid_merged[8*GROUP+7] ^ {
                    E_mid_elem[I8], E_mid_elem[I7], E_mid_elem[I6], E_mid_elem[I5],
                    E_mid_elem[I4], E_mid_elem[I3], E_mid_elem[I2], E_mid_elem[I1], E_mid_elem[I0]
                };
            end
            
        end
        
        for (l=0; l<9; l++) begin
            assign dout_tmp_base[i+l] = {
                dout_tmp_base_merged[8*GROUP+7][8*l+:8],
                dout_tmp_base_merged[8*GROUP+6][8*l+:8],
                dout_tmp_base_merged[8*GROUP+5][8*l+:8],
                dout_tmp_base_merged[8*GROUP+4][8*l+:8],
                dout_tmp_base_merged[8*GROUP+3][8*l+:8],
                dout_tmp_base_merged[8*GROUP+2][8*l+:8],
                dout_tmp_base_merged[8*GROUP+1][8*l+:8],
                dout_tmp_base_merged[8*GROUP+0][8*l+:8]
            };
            
            assign dout_tmp_mid[i+l] = {
                dout_tmp_mid_merged[8*GROUP+7][8*l+:8],
                dout_tmp_mid_merged[8*GROUP+6][8*l+:8],
                dout_tmp_mid_merged[8*GROUP+5][8*l+:8],
                dout_tmp_mid_merged[8*GROUP+4][8*l+:8],
                dout_tmp_mid_merged[8*GROUP+3][8*l+:8],
                dout_tmp_mid_merged[8*GROUP+2][8*l+:8],
                dout_tmp_mid_merged[8*GROUP+1][8*l+:8],
                dout_tmp_mid_merged[8*GROUP+0][8*l+:8]
            };
        end
    end
endgenerate

// *******************
// Update i* regs
always_ff @ (posedge clk) begin
    if (rst)
        i_star_regs <= 'h0;
    else if (i_star_valid_in)
        i_star_regs <= {i_star_elem_in, i_star_regs[8*`TAU-1 : $clog2(`N-1)]};
//    else if (next_i_star)
//        i_star_regs <= {i_star_elem_in, i_star_regs[8*`TAU-1 : $clog2(`N-1)]};
//        i_star_regs <= {i_star_regs[$clog2(`N-1)-1 : 0], i_star_regs[8*`TAU-1 : $clog2(`N-1)]};
end

reg next_y_elem, y_elem_upd_disable;
always_ff @ (posedge clk) begin
    y_elem_upd_disable  <= ~E_mul_res_valid && H_row_div_8_ctr!='h0;
//    next_y_elem         <= E_mul_res_valid && !(H_row_div_8_ctr[2:0]=='h7 && H_row_sub_ctr=='h6);
    next_y_word         <= H_row_div_8_ctr[2:0]=='h7 && H_row_sub_ctr=='h2 && incr_H_sub_row_ctr;
    
    if (rst)
        y_word_reg <= 'h0;
    else if (y_elem_upd_disable)
        y_word_reg <= y_word_reg;
//    else if (next_y_elem)
    else if (E_mul_res_valid_pip && !(H_row_div_8_ctr[2:0]=='h7 && incr_H_row_ctr))
        y_word_reg <= y_word_reg>>1;
    else if (y_word_valid)
        y_word_reg <= y_word;
end

endmodule
