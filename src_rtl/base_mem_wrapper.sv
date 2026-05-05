/*
 * base_mem_wrapper.sv
 * -------------
 * This file corresponds to the MatBase MAC module shown in Figure 4.
 * It captures the high-througput sample coming from the AES/Rijndael
 * and forwards them to the S_base (Sb_mem_acc) and C'_base (Cb_mem_acc)
 * accumulators instantiated in the same file. A small FSM is used to
 * bookkeep the incoming samples and provide read addresses to the
 * S_base and C'_base accumulators.
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
`include "E_mul_defines.svh"

`default_nettype none

//typedef enum logic [1:0] {
//hold_e,
//acc_e,
//init_e
//} E_mul_opcode_t;

module base_mem_wrapper #(
    parameter M_PARAM_M = `M_PARAM_M,
    parameter M_PARAM_N = `M_PARAM_N,
    parameter M_PARAM_R = `M_PARAM_R,
    parameter M_PARAM_K = `M_PARAM_K,
    parameter WORD_SIZE = `WORD_SIZE,
    parameter S_BASE_MEM_DEPTH = `GET_BYTES(M_PARAM_M)*M_PARAM_R+1,
    parameter C_BASE_MEM_DEPTH = (M_PARAM_N-M_PARAM_R)/2+1,
    parameter TAU = `TAU,
    parameter SAMPLE_COUNT = `SAMPLE_COUNT,
    parameter S_MEM_WIDTH_BYTES = 8,
    parameter C_MEM_WIDTH_BYTES = 2*M_PARAM_R
)(
    input wire rst,
    input wire clk,
    
//    input wire [1:0] start,
    
    input wire next_E_ff_mu,
    
//    input wire wren,
    input wire [$clog2(TAU)-1:0] mpc_round_base_wrap_in,
    input wire [7:0]             phi_i_in,
    input wire                  acc_sample_valid,
    input wire [WORD_SIZE-1:0]  acc_sample,
    
    input wire                  share_split_done_base,
    input wire                  init_acc_round_aes_pip,
    input wire [$clog2(`N-1)-1:0]      i_star_elem_in,
    input wire                         i_star_valid_in,
    
//    input wire [$clog2(S_BASE_MEM_DEPTH)-1:0] re_addr_S_base,
//    input wire [$clog2(C_BASE_MEM_DEPTH)-1:0] re_addr_C_base,
    
//    output reg [8*MEM_WIDTH_BYTES-1:0]  dout [TAU-1:0]
    output E_mul_opcode_t E_mul_opc_base,
    
    output reg [7:0] S_base_elem_out [TAU-1:0],
    output reg [7:0] C_base_elem_out [TAU-1:0],
    output reg       E_mul_res_valid,
    output reg       to_ff_wrap_start_MPC
);

E_mul_opcode_t E_mul_opc_base_int;
pipeline_delay #(
    .WIDTH ( $bits(E_mul_opcode_t) ),
    .DEPTH   ( 2 )
) pipeline_delay_inst (
    .clk  ( clk ),
    .din  ( E_mul_opc_base_int ),
    .dout ( E_mul_opc_base )
);

//reg verify, start_bit, start_bit_pip;
//always_ff @ (posedge clk) begin
//    if (rst) begin
//        verify <= 1'b0;
//        start_bit <= 1'b0;
//        start_bit_pip <= 1'b0;
        
//    end else begin
//        start_bit <= start[`START_BIT];
        
//        if (start_bit)
//            start_bit_pip <= 1'b1;
        
//        if (start_bit && !start_bit_pip)
//            verify <= start[`VERIFY_MODE_BIT];
//    end
//end

//reg [$clog2(SAMPLE_COUNT+1)-1:0] sample_counter;
reg [$clog2(SAMPLE_COUNT)-1:0] sample_counter;
reg init_acc_round_aes_internal;
always_ff @ (posedge clk) if (sample_counter=='h0 && acc_sample_valid) init_acc_round_aes_internal <= init_acc_round_aes_pip;

reg [8*`TAU-1:0] i_star_regs;
reg       E_mul_choose_p;
always_ff @ (posedge clk) E_mul_choose_p <= (state==FILL_EA);

reg [7:0] phi_i, phi_i_pip;
always_ff @ (posedge clk) if (sample_counter=='h1) phi_i <= phi_i_in;
always_ff @ (posedge clk) phi_i_pip <= phi_i;

reg [$clog2(TAU)-1:0] mpc_round_base_wrap;
always_ff @ (posedge clk) if (sample_counter=='h1)  mpc_round_base_wrap <=  mpc_round_base_wrap_in;

reg phi_i_change;
always_ff @ (posedge clk) phi_i_change <= (phi_i != phi_i_pip);

reg [$clog2(S_BASE_MEM_DEPTH)-1:0] Sb_re_addr;
reg [$clog2(C_BASE_MEM_DEPTH)-1:0] Cb_re_addr;

wire  [8*S_MEM_WIDTH_BYTES-1:0]  dout_S_base [TAU-1:0];
wire  [8*C_MEM_WIDTH_BYTES-1:0]  dout_C_base [TAU-1:0];

always_ff @ (posedge clk) begin
    if (rst)
        sample_counter <= 'h0;
    else if (acc_sample_valid)
        sample_counter <= (sample_counter + 1'b1); 
//        sample_counter <= (sample_counter==SAMPLE_COUNT-1) 
//                        ? 'h0 : (sample_counter + 1'b1);
end

reg       E_mul_res_valid_next;
always_ff @ (posedge clk) E_mul_res_valid <= rst ? 1'b0 : E_mul_res_valid_next;

// **************************
// S_base_mem write count
// ***********************
reg shift_init_next_Sb;
localparam SAMPLES_S = 6;
always_ff @ (posedge clk) shift_init_next_Sb <= (sample_counter==SAMPLES_S-2); // was -2 in the S_base module

localparam GRAB_REG_S_LEN = (`ROUND_TO_8(M_PARAM_M)*(M_PARAM_R-1)) +M_PARAM_M;
localparam SHIFT_COUNT_MAX_S_B = `GET_BYTES(GRAB_REG_S_LEN)-1;
localparam SHIFT_COUNT_BITS_S_B = $clog2(SHIFT_COUNT_MAX_S_B+1);
reg [SHIFT_COUNT_BITS_S_B-1:0] shift_counter_S_b;

always_ff @ (posedge clk) begin
    if (rst)
        shift_counter_S_b <= 'h0;
    else if (shift_init_next_Sb)
        shift_counter_S_b <= SHIFT_COUNT_MAX_S_B;
    else if (shift_counter_S_b!=0)
        shift_counter_S_b <= shift_counter_S_b -1'b1;
end

reg read_addr_upd_Sb; //, shift_en_pip;
always_ff @ (posedge clk) read_addr_upd_Sb <= |{shift_init_next_Sb, shift_counter_S_b};
wire shift_en_next_Sb = read_addr_upd_Sb;

// **************************
// C_base_mem write count
// ***********************
reg shift_init_next_Cb;
localparam SAMPLES_C = 12;
always_ff @ (posedge clk) shift_init_next_Cb <= (sample_counter==SAMPLES_C-2);

localparam GRAB_REG_C_LEN = ((M_PARAM_M -M_PARAM_R) * (M_PARAM_R));
localparam SHIFT_COUNT_MAX_C_B = (GRAB_REG_C_LEN/C_MEM_WIDTH_BYTES)-1;
localparam SHIFT_COUNT_BITS = $clog2(SHIFT_COUNT_MAX_C_B+1);
reg [SHIFT_COUNT_BITS-1:0] shift_counter_C_b;

always_ff @ (posedge clk) begin
    if (rst)
        shift_counter_C_b <= 'h0;
    else if (shift_init_next_Cb)
        shift_counter_C_b <= SHIFT_COUNT_MAX_C_B;
    else if (shift_counter_C_b!=0)
        shift_counter_C_b <= shift_counter_C_b -1'b1;
end

reg read_addr_upd_Cb; //, shift_en_pip;
always_ff @ (posedge clk) read_addr_upd_Cb <= |{shift_init_next_Cb, shift_counter_C_b};
//always_ff @ (posedge clk) wren_C_base      <= read_addr_upd_Cb;
wire shift_en_next_Cb = read_addr_upd_Cb;


// ******************************************
// Instantiate S_base + C_base mem-acc pairs

S_base_mem_acc Sb_mem_acc_inst (
    .rst    (rst),
    .clk    (clk),
    
    .mpc_round_S_base   (mpc_round_base_wrap),
    .phi_i  (phi_i),
    
    .sample_counter     (sample_counter),
    
    .acc_sample_valid   (acc_sample_valid),
    .acc_sample         (acc_sample),
    
    .re_addr_S_base     (Sb_re_addr),
    
    .shift_en_next      (shift_en_next_Sb),
    .dout               (dout_S_base)
);

C_base_mem_acc Cb_mem_acc_inst (
    .rst    (rst),
    .clk    (clk),
    
    .mpc_round_C_base   (mpc_round_base_wrap),
    .phi_i  (phi_i),
    
    .sample_counter     (sample_counter),
        
    .acc_sample_valid   (acc_sample_valid),
    .acc_sample         (acc_sample),
    
    .re_addr_C_base     (Cb_re_addr),
    
    .shift_en_next      (shift_en_next_Cb),
    .dout               (dout_C_base)
);


// **************************************************
// Instantiate an FSM to control reads from the mems

typedef enum logic [2:0] {
    ACC_BASE,
    FILL_EA,
    EMULATE_MPC,
    WAIT_NEXT_ELEM,
    DONE
} state_t;

state_t state, next_state;

reg mpc_round_base_MSB_pip, mpc_round_base_MSB_pip_2;

reg/*init_round,*/ share_split_done;
always_ff @ (posedge clk) begin // TODO: update for L3 and L5
//    if (rst)
//        init_round <= 1'b1;
//    else if (sample_counter=='h1)
//        init_round <= phi_i_in=='h0;
    
    if (rst) begin
        share_split_done <= 1'b0;
        mpc_round_base_MSB_pip <= 1'b0;
        mpc_round_base_MSB_pip_2 <= 1'b0;
    end else begin
        mpc_round_base_MSB_pip <= mpc_round_base_wrap[$clog2(TAU)-1];
        mpc_round_base_MSB_pip_2 <= mpc_round_base_MSB_pip;
//        share_split_done <= mpc_round_base_MSB_pip_2 && ~mpc_round_base_MSB_pip;
        share_split_done <= share_split_done_base;
    end
end

typedef enum logic [2:0] { // S_base mem address opcode
    freeze_Sb_addr,
    rst_Sb_addr,
    incr_Sb_addr,
    move_up_Sb_column,
    goto_Sb_row_start,
    start_next_Sb_row,
    goto_zero_Sb
} Sb_address_opcode_t;

Sb_address_opcode_t Sb_addr_opc;
always_ff @ (posedge clk) begin // update Sb_mem_addr
    if (rst)
        Sb_re_addr <=  S_BASE_MEM_DEPTH -1;
    else begin
        case (Sb_addr_opc)
            freeze_Sb_addr:     Sb_re_addr <= Sb_re_addr;
            
            incr_Sb_addr:       Sb_re_addr <= Sb_re_addr +1'b1;
            
            rst_Sb_addr:        Sb_re_addr <= 'h0;
            
            move_up_Sb_column:  Sb_re_addr <= Sb_re_addr +`GET_BYTES(M_PARAM_M);
            
            goto_Sb_row_start:  Sb_re_addr <= Sb_re_addr -((M_PARAM_R-1)*`GET_BYTES(M_PARAM_M));
            
            start_next_Sb_row:  Sb_re_addr <= Sb_re_addr -((M_PARAM_R-1)*`GET_BYTES(M_PARAM_M) -1);
            
            goto_zero_Sb:       Sb_re_addr <= S_BASE_MEM_DEPTH -1;
        endcase
    end
end

typedef enum logic [1:0] { //C_base mem address opcode
    freeze_Cb_addr,
    rst_Cb_addr,
    incr_Cb_addr,
    goto_zero_Cb
} Cb_address_opcode_t;

Cb_address_opcode_t Cb_addr_opc;
always_ff @ (posedge clk) begin // update Cb_mem_addr
    if (rst)
        Cb_re_addr <= C_BASE_MEM_DEPTH -1;
    else begin
        case (Cb_addr_opc)
            freeze_Cb_addr:     Cb_re_addr <= Cb_re_addr;
            
            incr_Cb_addr:       Cb_re_addr <= Cb_re_addr +1'b1;
            
            rst_Cb_addr:        Cb_re_addr <= 'h0;
            
            goto_zero_Cb:       Cb_re_addr <= C_BASE_MEM_DEPTH -1;
        endcase
    end
end

// ********************************************************************
// 2 counters to book-keep where we are at in the multiplications
typedef enum logic [1:0] { // counter opcode
    hold_count,
    rst_count,
    incr_count
} counter_opcode_t;

counter_opcode_t Sb_col_counter_opc;
counter_opcode_t Sb_row_counter_opc; //ROUND_TO_4(x)

reg Cb_col_lsb, change_Cb_col_lsb, change_Cb_col_lsb_next;
always_ff @ (posedge clk) begin
    change_Cb_col_lsb <= rst ? 1'b0 : change_Cb_col_lsb_next;
    
    if (rst)
        Cb_col_lsb <= 1'b0;
    else if (change_Cb_col_lsb)
        Cb_col_lsb <= ~Cb_col_lsb;
end

reg [$clog2(`ROUND_TO_8(M_PARAM_M))-1:0] Sb_row_counter, Sb_row_counter_pip;
reg [$clog2(M_PARAM_R)-1:0] Sb_col_counter;
reg [$clog2(M_PARAM_R)-1:0] Sb_col_counter_pip;

always_ff @ (posedge clk) begin // Sb_row_counter update
    if (rst)
        Sb_row_counter <= 'h0;
    else begin
        case (Sb_row_counter_opc)
            hold_count: Sb_row_counter <= Sb_row_counter;
            
            rst_count: Sb_row_counter <= 'h0;
            
            incr_count: Sb_row_counter <= (Sb_row_counter==M_PARAM_M-1) ? 'h0 : (Sb_row_counter+1'b1);
        endcase
    end
    
    Sb_row_counter_pip <= Sb_row_counter;
end

always_ff @ (posedge clk) begin // S_col_counter update
    Sb_col_counter_pip <= Sb_col_counter;
    if (rst)
        Sb_col_counter <= 'h0;
    else begin
        case (Sb_col_counter_opc)
            hold_count: Sb_col_counter <= Sb_col_counter;
            
            rst_count: Sb_col_counter <= 'h0;
            
            incr_count: Sb_col_counter <= (Sb_col_counter==M_PARAM_R-1) ? 'h0 : (Sb_col_counter+1'b1);
//            incr_count: Sb_col_counter <= (Sb_col_counter+1'b1);
        endcase
    end
end

// ***************************
// Connect to output elements

always_ff @(posedge clk) begin
    for (int i = 0; i < TAU; i++) begin
        S_base_elem_out[i] <= dout_S_base[i][8*Sb_row_counter_pip[2:0] +: 8];
    end
end

always_ff @(posedge clk) begin
    for (int i = 0; i < TAU; i++) begin
//        C_base_elem_out[i] <= E_mul_choose_p ? i_star_regs[8*i +: 8] : dout_C_base[i][8*{S_b_row_counter_pip, Sb_col_counter_pip} +: 8];
        if (E_mul_choose_p)
            C_base_elem_out[i] <= i_star_regs[8*i +: 8];
    
        else
          case ({Cb_col_lsb, Sb_col_counter_pip})
            // Cb_col_lsb = 0  -> offsets 0..5  (0..40 bits)
//            3'd0: C_base_elem_out[i] <= dout_C_base[i][ 0 +: 8];
            3'd1: C_base_elem_out[i] <= dout_C_base[i][ 8 +: 8];
            3'd2: C_base_elem_out[i] <= dout_C_base[i][16 +: 8];
            3'd3: C_base_elem_out[i] <= dout_C_base[i][24 +: 8];
            3'd4: C_base_elem_out[i] <= dout_C_base[i][32 +: 8];
            3'd5: C_base_elem_out[i] <= dout_C_base[i][40 +: 8];
            
            // Cb_col_lsb = 1  -> same selection but +48 bits (i.e., +6 bytes)
            // encoded as {1,0..5} = 8..13
            4'd8:  C_base_elem_out[i] <= dout_C_base[i][ 48 +: 8];
            4'd9:  C_base_elem_out[i] <= dout_C_base[i][ 56 +: 8];
            4'd10: C_base_elem_out[i] <= dout_C_base[i][ 64 +: 8];
            4'd11: C_base_elem_out[i] <= dout_C_base[i][ 72 +: 8];
            4'd12: C_base_elem_out[i] <= dout_C_base[i][ 80 +: 8];
            4'd13: C_base_elem_out[i] <= dout_C_base[i][ 88 +: 8];
            
            default: C_base_elem_out[i] <= dout_C_base[i][ 0 +: 8]; // all-zero case
          endcase
    end
end

// State logic:
always_comb begin
    // Defaults
    Sb_addr_opc = freeze_Sb_addr;
    Cb_addr_opc = freeze_Cb_addr;
    
    E_mul_opc_base_int = hold_e;
    E_mul_res_valid_next = 1'b0;
    
    Sb_row_counter_opc = hold_count;
    Sb_col_counter_opc = hold_count;
    
    change_Cb_col_lsb_next = 1'b0;
    
    case (state)
        
        ACC_BASE: begin
            // If initial round, then it is enough to hold read
            if (~init_acc_round_aes_internal) begin // addresses to the all-zero address.
                if (read_addr_upd_Sb)
                    Sb_addr_opc = incr_Sb_addr;
                else
                    Sb_addr_opc = rst_Sb_addr;
                
                if (read_addr_upd_Cb)
                    Cb_addr_opc = incr_Cb_addr;
                else
                    Cb_addr_opc = rst_Cb_addr;
            end else begin
                Sb_addr_opc = goto_zero_Sb;
                Cb_addr_opc = goto_zero_Cb;
            end
            
            if (share_split_done)
                Cb_addr_opc = goto_zero_Cb;
        end
        
        FILL_EA: begin
            E_mul_res_valid_next = 1'b1;
            E_mul_opc_base_int = init_e;
            Sb_row_counter_opc = incr_count;
            
//            if (Sb_row_counter==M_PARAM_M-1 || Sb_row_counter[2:0]=='h7) begin
            if (Sb_row_counter[2:0]=='h7) begin
                Sb_addr_opc = incr_Sb_addr;
            end 
            
            if (Sb_row_counter==M_PARAM_M-1) begin
                Sb_col_counter_opc = incr_count;
                
                if (Sb_col_counter==M_PARAM_R-1) begin
                    Sb_addr_opc = rst_Sb_addr;
                    Cb_addr_opc = rst_Cb_addr;
                end
            end
        end
        
        EMULATE_MPC: begin
            Sb_col_counter_opc = incr_count;
            if (Sb_col_counter==0)
                E_mul_opc_base_int = init_e;
            else
                E_mul_opc_base_int = acc_e;
            
            if (Sb_col_counter!=M_PARAM_R-1) begin
                Sb_addr_opc = move_up_Sb_column;
            end else begin
                Sb_row_counter_opc = incr_count;
//                if (Sb_row_counter[2:0]!='h7 && Sb_row_counter!=M_PARAM_M-1) begin
                if (Sb_row_counter[2:0]!='h7) begin
                    Sb_addr_opc = goto_Sb_row_start;
                
                end else begin
                    Sb_addr_opc = start_next_Sb_row;
                    
                    if (Sb_row_counter==M_PARAM_M-1) begin
                        change_Cb_col_lsb_next = 1'b1;
                        Sb_addr_opc = rst_Sb_addr;
                        
                        if (Cb_col_lsb)
                            Cb_addr_opc = incr_Cb_addr;
                    end
                end                
            end
            
            if  (Sb_col_counter==M_PARAM_R-1)
                E_mul_res_valid_next = 1'b1;
        end
        
        WAIT_NEXT_ELEM: begin

        end
        
    endcase
end

//  State update
always_ff @(posedge clk) begin
    if (rst)
        state <= ACC_BASE;
    else
        state <= next_state;
end

// Next-state logic
always_comb begin
    next_state = state;  // default

    case (state)
        ACC_BASE:
            if (share_split_done)
                next_state = FILL_EA;
        
        FILL_EA:
            if ((Sb_row_counter==M_PARAM_M-1) && (Sb_col_counter==M_PARAM_R-1))
                next_state = EMULATE_MPC;
        
        EMULATE_MPC:
            if  (Sb_col_counter==M_PARAM_R-1)
                next_state = WAIT_NEXT_ELEM;
        
        WAIT_NEXT_ELEM: if (next_E_ff_mu) next_state = EMULATE_MPC;
        
    endcase
end

// Send start signal to FF_math_wrapper FSM
always_ff @ (posedge clk) to_ff_wrap_start_MPC <= (Sb_row_counter==M_PARAM_M-2) && (Sb_col_counter==M_PARAM_R-1) && (state==FILL_EA);

// *******************
// Update i* regs
always_ff @ (posedge clk) begin
    if (rst)
        i_star_regs <= 'h0;
    else if (i_star_valid_in)
        i_star_regs <= {i_star_elem_in, i_star_regs[8*`TAU-1 : $clog2(`N-1)]};
end

endmodule
