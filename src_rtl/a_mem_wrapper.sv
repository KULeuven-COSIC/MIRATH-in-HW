/*
 * a_mem_wrapper.sv
 * -----------
 * This file is an interface between the VecAlpha MAC module (shown in
 * Figure 4) and the rest of the design.
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

module a_mem_wrapper #(
    parameter M_PARAM_RHO = `M_PARAM_RHO,
    parameter WORD_SIZE = 64,
    parameter TAU = `TAU,
    parameter SAMPLE_COUNT = 10,
    parameter V_MEM_DEPTH = M_PARAM_RHO+1
)(
    input wire                  rst,
    input wire                  clk,
    
    input wire [$clog2(TAU)-1:0] mpc_round_V_in,
//    input wire                  load_i_star,
    input wire                  share_split_done_V,
    input wire                  init_acc_round_aes_pip,
    input wire                  keccak_done,
//    input wire [$clog2(`N)-1:0] i_star,
    input wire [$clog2(`N)-1:0] phi_i_in,
    input wire [7:0]            gamma,
    input wire                  gamma_valid,
    input wire                  store_a_start,
    input wire                  rst_v_regs,
    input wire                  acc_sample_valid,
    input wire [WORD_SIZE-1:0]  acc_sample,
    
    input wire [7:0]        tmp_dout_base [`TAU-1:0],
    input wire [7:0]        tmp_dout_mid [`TAU-1:0],
    
    input wire [`WORD_SIZE-1:0]    a_mid_word,
    input wire                     a_mid_word_valid, // for verfication
    
    output reg  [`WORD_SIZE-1:0] a_word,
    output reg                   a_word_valid,
    output reg                   a_word_is_mid_out,
    output reg                   store_a_done
);

reg a_word_is_mid;
always_ff @ (posedge clk) a_word_is_mid_out <= a_word_is_mid;

reg [(`M_PARAM_RHO/8)-1:0] a_mid_in_valid_train;

reg [$clog2(`M_PARAM_RHO)-1:0] rho_ctr;
reg [$clog2(`TAU)-1:0] mpc_round_a;
reg incr_mpc_round, incr_mpc_round_next, flip_a_word_is_mid, flip_a_word_is_mid_next, a_word_valid_next, last_mpc_round, incr_rho_ctr, fill_a_word, fill_a_word_next;
always_ff @ (posedge clk) begin
    flip_a_word_is_mid  <= flip_a_word_is_mid_next;
    incr_mpc_round  <= incr_mpc_round_next;
    fill_a_word     <= fill_a_word_next;
    a_word_valid    <= a_word_valid_next;
    a_word_is_mid   <= rst ? 1'b0 :
                       flip_a_word_is_mid ? !a_word_is_mid : a_word_is_mid;
    last_mpc_round  <= mpc_round_a==`TAU;
    store_a_done    <= last_mpc_round;
    
    if (rst)
        mpc_round_a <= 'h0;
    else if (incr_mpc_round)
        mpc_round_a <=  mpc_round_a + 1;
    
    if (rst)
        rho_ctr <= 'h0;
    else if (incr_rho_ctr)
        rho_ctr <=  rho_ctr + 1;
//        rho_ctr <=  (rho_ctr==`M_PARAM_RHO-1) ? 'h0 : (rho_ctr + 1); // Only needed for L3

    if (fill_a_word) begin
        if (a_word_is_mid)
            a_word <= {dout_A_m[mpc_round_a], a_word[`WORD_SIZE-1:8]};          
        else
            a_word <= {dout_A_b[mpc_round_a], a_word[`WORD_SIZE-1:8]};
    end
end

//wire [7:0] debug_wire_a_m = dout_A_m[mpc_round_a];
//wire [7:0] debug_wire_a_b = dout_A_b[mpc_round_a];

wire [7:0] gamma_delayed;
pipeline_delay #(
    .WIDTH ( $bits(gamma) ),
    .DEPTH   ( 3 )
) delay_gamma (
    .clk  ( clk ),
    .din  ( gamma ),
    .dout ( gamma_delayed )
);

reg keccak_done_pip, keccak_done_pip_2, keccak_done_pip_3;
always_ff @ (posedge clk) keccak_done_pip   <= keccak_done;
always_ff @ (posedge clk) keccak_done_pip_2 <= keccak_done_pip;
always_ff @ (posedge clk) keccak_done_pip_3 <= keccak_done_pip_2;

reg [$clog2(TAU)-1:0] mpc_round_V;
always_ff @ (posedge clk) if (sample_counter=='h1)  mpc_round_V <=  mpc_round_V_in;

localparam GRAB_REG_V_LEN = 8*M_PARAM_RHO;
reg [GRAB_REG_V_LEN-1:0] grab_regs_V;
reg [8*`M_PARAM_RHO-1:0] grab_regs_A_mid;
wire [$clog2(`N)-1:0] shift_regs_out = rst_v_regs ? grab_regs_A_mid[$clog2(`N)-1:0] : grab_regs_V[$clog2(`N)-1:0];

wire  [7:0]  dout_A_b [TAU-1:0];
wire  [7:0]  dout_A_m [TAU-1:0];

//reg [$clog2(`N)-1:0] i_star_regs [TAU-1:0];
//always_ff @ (posedge clk) begin
//    if (rst) begin
//        for (int i=0; i<TAU; i=i+1)
//            i_star_regs[i] <= 8'h0;
            
//    end else if (load_i_star) begin
//        i_star_regs[TAU-1] <= i_star;
        
//        for (int i=0; i<TAU-1; i=i+1)
//            i_star_regs[i] <= i_star_regs[i+1];
//    end        
//end

reg [$clog2(`N)-1:0] phi_i;
always @ (posedge clk) begin
     if (sample_counter=='h1) phi_i <= phi_i_in;
end

reg load_i_star_pip;
wire [$clog2(`N)-1:0] v_base_load_mul_out;
reg [$clog2(`N)-1:0] v_base_acc_in, v_acc_in;//, sq_mul_out_reg;

gf256_mul_mast Vb_load_mul_inst ( // gf256_mul_mast instance
    .a  (shift_regs_out),
    .b  (phi_i),
    .p  (v_base_load_mul_out)
);

//always @ (posedge clk) sq_mul_out_reg <= sq_mul_out;
always @ (posedge clk) v_base_acc_in <= v_base_load_mul_out;
always @ (posedge clk) v_acc_in <= shift_regs_out;

//reg [$clog2(`N)-1:0] p_sq_regs [TAU-1:0];
//always_ff @ (posedge clk) begin
//    load_i_star_pip <= load_i_star;
    
//    if (rst)
//        p_sq_regs <= 'h0;
        
//    else if (load_i_star_pip) begin
//        p_sq_regs[TAU-1] <= sq_mul_out;
        
//        for (int i=0; i<TAU-1; i=i+1)
//            p_sq_regs[i] <= p_sq_regs[i+1];
//    end        
//end


reg [$clog2(SAMPLE_COUNT+1)-1:0] sample_counter, sample_counter_pip;
always_ff @ (posedge clk) begin
    sample_counter_pip <= sample_counter;
    
    if (rst)
        sample_counter <= 'h0;
    else if (acc_sample_valid)
        sample_counter <= (sample_counter==SAMPLE_COUNT-1) 
                        ? 'h0 : (sample_counter + 1'b1);
end

// ****************************
// v_base / v_rnd write count
// ***************************
reg shift_init_next_V;
localparam SAMPLES_V = 10;
always_ff @ (posedge clk) shift_init_next_V <= (sample_counter==SAMPLES_V-1); // was -2 in the S_base module

localparam SHIFT_COUNT_MAX_V = M_PARAM_RHO-1;
localparam SHIFT_COUNT_BITS_S_B = $clog2(SHIFT_COUNT_MAX_V+1);
reg [SHIFT_COUNT_BITS_S_B-1:0] shift_counter_V;

always_ff @ (posedge clk) begin
    if (rst)
        shift_counter_V <= 'h0;
    else if (shift_init_next_V)
        shift_counter_V <= SHIFT_COUNT_MAX_V;
    else if (shift_counter_V!=0)
        shift_counter_V <= shift_counter_V -1'b1;
end

reg read_addr_upd_V; //, shift_en_pip;
always_ff @ (posedge clk) read_addr_upd_V <= |{shift_init_next_V, shift_counter_V};
wire shift_en_next_V = read_addr_upd_V;
reg shift_en_V;
always_ff @ (posedge clk) shift_en_V <= shift_en_next_V;

// **************************************************
// Instantiate an FSM to control reads from the mems

typedef enum logic [2:0] {
    ACC_V,
    WAIT_FOR_TMP,
    COMPUTE_A,
    STORE_A,
    DONE
} state_t;

state_t state_v, next_state_v;

typedef enum logic [1:0] { //C_base mem address opcode
    freeze_V_addr,
    rst_V_addr,
    incr_V_addr,
    goto_zero_V
} V_address_opcode_t;

V_address_opcode_t V_addr_opc;
reg [$clog2(V_MEM_DEPTH)-1:0] re_addr_V;
always_ff @ (posedge clk) begin // update Cb_mem_addr
    if (rst)
        re_addr_V <= V_MEM_DEPTH -1;
    else begin
        case (V_addr_opc)
            freeze_V_addr:     re_addr_V <= re_addr_V;
            
            incr_V_addr:       re_addr_V <= (re_addr_V==M_PARAM_RHO-1) ? 'h0 : (re_addr_V +1'b1);
            
            rst_V_addr:        re_addr_V <= 'h0;
            
            goto_zero_V:       re_addr_V <= V_MEM_DEPTH -1;
        endcase
    end
end

always_comb begin
    // Defaults
    V_addr_opc = freeze_V_addr;
    incr_mpc_round_next = 1'b0;
    flip_a_word_is_mid_next = 1'b0;
    a_word_valid_next = 1'b0;
    fill_a_word_next  = 1'b0;
    incr_rho_ctr = 1'b0;
    
    case (state_v)
        
        ACC_V: begin
            // If initial round, then it is enough to hold read
            if (~init_acc_round_aes_pip) begin // addresses to the all-zero address.
                if (read_addr_upd_V)
                    V_addr_opc = incr_V_addr;
                else
                    V_addr_opc = rst_V_addr;
                
                if (read_addr_upd_V)
                    V_addr_opc = incr_V_addr;
                else
                    V_addr_opc = rst_V_addr;
            end else begin
                V_addr_opc = goto_zero_V;
            end
        end
        
        WAIT_FOR_TMP: begin
        end
        
        COMPUTE_A: begin
            if (acc_a)
                V_addr_opc = incr_V_addr;
        end

        STORE_A: begin
            V_addr_opc = incr_V_addr;
            fill_a_word_next = 1'b1;
            
            if (fill_a_word)
                incr_rho_ctr = 1'b1;
            
            if (rho_ctr[2:0]=='h7)
                a_word_valid_next = 1'b1;
                
            if (rho_ctr==`M_PARAM_RHO-2) begin
                flip_a_word_is_mid_next = 1'b1;
                
                if (a_word_is_mid)
                    incr_mpc_round_next = 1'b1;
            end    
        end
        
    endcase
end

//  State update
always_ff @(posedge clk) begin
    if (rst)
        state_v <= ACC_V;
    else
        state_v <= next_state_v;
end

// Next-state logic
always_comb begin
    next_state_v = state_v;  // default

    case (state_v)
        ACC_V:
            if (share_split_done)
                next_state_v = WAIT_FOR_TMP;
        
        WAIT_FOR_TMP:
            if  ( keccak_done_pip )
                next_state_v = COMPUTE_A;

        COMPUTE_A:
            if  ( store_a_start )
                next_state_v = STORE_A;
        
        STORE_A:
            if  ( last_mpc_round )
                next_state_v = DONE;
    endcase
end

reg /*init_round,*/ share_split_done;
always_ff @ (posedge clk) begin // TODO: update for L3 and L5
//    if (rst)
//        init_round <= 1'b1;
//    else if (sample_counter=='h1)
//        init_round <= phi_i_in=='h0;
    
    if (rst) begin
        share_split_done <= 1'b0;
    end else begin
        share_split_done <= share_split_done_V;
    end
end

// ***************************************************
always_ff @ (posedge clk) begin // grab_regs_V update
//    if (rst)
//       grab_regs_V <= 1'b0;
//    else
//    if (rst_v_regs)
//       grab_regs_V <= 1'b0;
       
//    else
    if (shift_en_next_V) 
        grab_regs_V <= (grab_regs_V >> 8);
    
    else if (acc_sample_valid) begin
        case (sample_counter)
            'h7: begin
                grab_regs_V[0+:16] <= acc_sample[`WORD_SIZE-1 -: 16];
             end
             
             'h8: begin
                grab_regs_V[16+:`WORD_SIZE] <= acc_sample;
             end
             
             'h9: begin
                grab_regs_V[GRAB_REG_V_LEN-48+:48] <= acc_sample;
             end
        endcase
    end
    
    if (shift_en_next_V) begin // rst_v_regs
        grab_regs_A_mid <= (grab_regs_A_mid >> 8);
    end else begin
        if (a_mid_in_valid_train[0])
            grab_regs_A_mid[0 +: 64] <= a_mid_word;
        if (a_mid_in_valid_train[1])
            grab_regs_A_mid[64+: 64] <= a_mid_word;
    end
    
    a_mid_in_valid_train <= {a_mid_in_valid_train, a_mid_word_valid};
end

// ****************************************************
reg acc_a, acc_a_in_1, acc_a_in_2;
always_ff @ (posedge clk) begin
    if (rst) begin
        acc_a_in_2 <= 1'b0;
        acc_a_in_1 <= 1'b0;
        acc_a      <= 1'b0;
    end else begin
        acc_a_in_2 <= gamma_valid && state_v==COMPUTE_A;
        acc_a_in_1 <= acc_a_in_2;
        acc_a      <= acc_a_in_1;  
    end    
end

A_mem_acc A_mem_acc_inst (
    .rst              ( rst ),
    .clk              ( clk ),

    .mpc_round_V      ( mpc_round_V ),
    .phi_i            ( phi_i ),
    .init_elem_Vb     ( v_base_acc_in ),
    .init_elem_V      ( v_acc_in ),
    .gamma_elem       ( gamma_delayed ),
    .tmp_dout_base    ( tmp_dout_base ),
    .tmp_dout_mid     ( tmp_dout_mid ),

    .acc_a            ( acc_a ),
    .shift_en_next    ( shift_en_next_V ),
    .shift_en         ( shift_en_V ),
    .acc_sample_valid ( acc_sample_valid ),
    .acc_sample       ( acc_sample ),

    .sample_counter   ( sample_counter ),

    .re_addr_V        ( re_addr_V ),

    .dout_A_b         ( dout_A_b ),
    .dout_A_m         ( dout_A_m )
);

endmodule
