/*
 * mirath_top_lvl_ctrl_v2.sv
 * --------------
 * This file is the top-level Controller FSM of our hardware implementation of Mirath.
 *
 * To keep the complexity of the top-level wrapper (mirath_wrapper_v2) minimal,
 * a few modules that are directly interfaced with the top-level controller are
 * also directly instantiated here. These are the accumulator of the auxiliary
 * value aux (see aux_acc), a small distibuted memory that stores the secret
 * matrices S and C' (S_C_mem), and a second distibuted memory (h_subctx_mem)
 * that is used to store the hash values that are derived by hashing the commitments.
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

// *******************
// State definitions
// *****************
localparam NUM_STATES_TL = 'd12;
typedef enum logic [$clog2(NUM_STATES_TL)-1:0] {
    S_IDLE_TL,
    S_EXP_SEED_SK,
    S_EXP_SEED_PK,
    S_COMMIT_TL,
    S_HASH_COM,
    S_HASH_SH,
    S_PREFILL_TMP,
    S_COMPUTE_TMP,
    S_COMPUTE_A,
    S_HASH_MPC,
    S_EXP_VIEW_CHAL,
    S_SEND_I_STAR,
    S_DONE_TL                  // State to end in for all routines. Includes case where sig is rejected.
} state_tl_t;

typedef enum logic [2:0] {FREEZE_ADDR_D_M_TL, PLUS_1_D_M_TL, GOTO_COMMIT_BUFF_TL} data_mem_addr_opcode_tl_t;

typedef enum logic [3:0] {FREEZE_ADDR_KS_M_TL, PLUS_1_KS_M_TL, GOTO_COMMIT, GOTO_SK_SEED,
                          GOTO_PK_SEED, GOTO_Y, GOTO_AUX, GOTO_SALT,
                          GOTO_HASH_SH, GOTO_MSG, GOTO_ALPHA_BASE, GOTO_H_MPC} key_sig_mem_addr_opcode_tl_t;

localparam S_BYTES = `GET_BYTES(`S_ROWS) * `S_COLS;
localparam C_BYTES = `C_COLS;
localparam AUX_WORDS = `CEIL( 8*(S_BYTES+C_BYTES), `WORD_SIZE) / `WORD_SIZE;

module mirath_top_lvl_ctrl_v2 #(
    parameter SAMPLE_INPUT_ROUNDS   = `EXPAND_LENGTH*`NODE_WORDS,
    parameter CLOG2_WORD_BYTES      = $clog2(`WORD_SIZE>>3),
    parameter K_IN_LEN_H_MPC        = 1 + 1 + 2*`COMMIT_WORDS + `TOTAL_A_WORDS +`PK_WORDS +`GET_WORD_COUNT_FROM_BYTES(`MSG_LEN_BYTES), // In words
    parameter K_IN_LEN_H_SH         = 1 + 1 + 2*`COMMIT_WORDS + (`TAU*`AUX_WORDS), // In words
    parameter K_IN_LEN_SEQ_MAX      = (K_IN_LEN_H_SH > K_IN_LEN_H_MPC) ? K_IN_LEN_H_SH : K_IN_LEN_H_MPC,
    parameter KECCAK_IN_LEN_CTR_BITS= $clog2(K_IN_LEN_SEQ_MAX+1)
)(
    input wire                          clk,
    input wire                          rst,
    input wire [1:0]                    start, // 2'b00: no start, 2'b01: keygen, 2'b10: sign, 2'b11: verify
    
    input wire                          keccak_dout_valid,
//    input wire                          keccak_din_ready,
    input wire                          keccak_data_done,
    input wire                          keccak_input_data_done,
    
    input wire [`WORD_SIZE-1:0]         a_word,
    input wire                          a_word_valid,
    input wire                          a_word_is_mid,
    input wire                          store_a_done,
    
    input wire                          path_accepted,
    input wire                          path_rejected,
    input wire                          aes_is_done,
//    input wire                          aes_waiting_for_i_star,
    input wire                          party_is_i_star_fr_aes,
    
    input wire                          acc_y_init_done,
    input wire [`WORD_SIZE-1:0]         y_out,
   
    input wire                          tmp_prefill_done,
    input wire                          next_y_word_from_tmp,
    
    input wire                          finalize_aux,
    input wire                          next_verify_round,
    input wire                          commit_valid,
    input wire                          sample_valid_tl,
    input wire [`WORD_SIZE-1:0]         sample_tl,
    
    input wire [`WORD_SIZE-1:0]         cmt_fifo_dout,
//    input wire [`WORD_SIZE-1:0]         data_mem_dout,
    input wire [`WORD_SIZE-1:0]         key_sig_mem_dout,
    input wire [`WORD_SIZE-1:0]         keccak_dout,
    
    output reg [CLOG2_WORD_BYTES:0]     keccak_bytelen_din,
    output reg                          keccak_dout_shift_bytes, // 1'b0: output full words,    1'b1: output byte-by-byte
    output reg                          keccak_din_valid,
//    output wire                          keccak_din_valid,
//    output reg                          key_seed_extract_done,
    output reg [`WORD_SIZE-1:0]         keccak_din,
    output reg [`WORD_SIZE-1:0]         keccak_command,
    
//    output reg                          wren_data_mem,
    output reg                          wren_key_sig_mem,
    
    output reg [`CMT_FIFO_ADDR_BITS-1:0]    cmt_fifo_addr_re,
    
    output wire [$clog2(`N-1)-1:0]      i_star_out,
    output reg                          i_star_valid,
    output reg                          y_word_valid_to_tmp,
    
//    output reg [`WORD_SIZE-1:0]         data_mem_din,
    output reg [`WORD_SIZE-1:0]         key_sig_mem_din,
    
//    output reg [`DATA_MEM_ADDR_BITS-1:0] data_mem_addr,
    output reg [`KEY_SIG_MEM_ADDR_BITS-1:0] key_sig_mem_addr,
    
    output reg                           next_E_reg,
    output reg                           squeeze_y,
    output reg                           key_unpack_done,
    output reg                           store_a_start,
    output reg                           done_tl,
    output reg                           bad_sig_out,
    output reg                           share_split_done_tl
);

localparam AUX_WORDS  = `AUX_WORDS;
localparam SIBL_PATH_ADDR  = `SIBL_PATH_ADDR;
localparam TOTAL_A_WORDS  = `TOTAL_A_WORDS;
localparam HASH_SH_ADDR  = `HASH_SH_ADDR;
localparam MSG_ADDR  = `MSG_ADDR;
localparam COMMIT_WORDS  = `COMMIT_WORDS;

localparam NUM_Y_SQUEEZES  = `GET_WORD_COUNT_FROM_BITS(`MIRATH_VAR_FF_Y_BITS);

reg prep_store_hash_subctx, switch_comm_buf, switch_comm_buf_next, rst_store_subctx_flag;

reg [31:0] ctr;
reg incr_ctr, incr_ctr_next, i_star_valid_next;

// hash subctx mem defines: depth always power of 2 for L1 & L5
wire [`WORD_SIZE-1:0] h_subctx_mem_dout;
reg [`H_SUBCTX_MEM_ADDR_BITS-1:0] h_subctx_mem_re_addr, h_subctx_mem_wr_addr;
reg h_subctx_mem_incr_wr_addr, h_subctx_mem_incr_wr_addr_next;
reg h_subctx_mem_incr_re_addr, h_subctx_mem_incr_re_addr_next;

// cmt_fifo defines: depth always power of 2 for L1 & L5
reg cmt_fifo_incr_re_addr, cmt_fifo_incr_re_addr_next;

reg [`COMMIT_SIZE-1:0] h_mpc_storage;
reg h_mpc_storage_en, h_mpc_storage_en_next;
reg last_mpc_round, hold_last_mpc_round;

reg [2:0] S_C_mem_address;
reg [`WORD_SIZE-1:0] S_C_mem_din, S_C_mem_din_next;
wire [`WORD_SIZE-1:0] S_C_mem_dout;
reg incr_S_C_addr, incr_S_C_addr_next, wren_S_C, wren_S_C_next;

state_tl_t                     state_tl, next_state_tl;
//data_mem_addr_opcode_tl_t      data_mem_addr_opc, data_mem_addr_opc_next;
key_sig_mem_addr_opcode_tl_t   key_sig_mem_addr_opc, key_sig_mem_addr_opc_next;

reg [1:0]  four_counter, four_counter_prev; // For the 4 hash sub-contexts
reg [$clog2(`TAU)-1:0] mpc_round_tl;
reg four_counter_incr, four_counter_incr_next;

reg y_word_valid_to_tmp_next, next_y_word_from_tmp_pip;
reg handshake_pip;//, extra_k_lookahead_update;
reg acc_e_last_word_next;
wire [`KEY_SIG_MEM_ADDR_BITS-1:0] check_last_acc_e_word = (key_sig_mem_addr-(`AUX_ADDR-1));

//reg mem_addr_upd_en, mem_addr_upd_en_next;

reg bad_sig, h_mpc_equality_check;

reg k_input_done_wait_for_output;
reg [1+`COMMIT_WORDS:0] keccak_inp_done_train;

reg incr_mpc_round, incr_mpc_round_next;
localparam F_ST_CYCLE_MAX = (4*(`COMMIT_WORDS+1)); // first_state_cycles[F_ST_CYCLE_MAX]
reg [F_ST_CYCLE_MAX:0] first_state_cycles; // Can be optimized for SRLCs
reg        start_en, sign, verify, four_counter_0;

reg [`WORD_SIZE-1:0] keccak_command_next;

reg [KECCAK_IN_LEN_CTR_BITS-1:0] keccak_input_ctr, keccak_input_ctr_rst_val, keccak_input_ctr_rst_val_next;
reg keccak_input_ctr_rst, keccak_input_ctr_rst_next, keccak_dout_shift_bytes_next;
reg  [CLOG2_WORD_BYTES:0] keccak_bytelen_din_next;
reg [`WORD_SIZE-1:0] keccak_din_next;

//reg wren_data_mem_next;
reg wren_key_sig_mem_next;
reg keccak_dout_valid_pip;
reg [63:0] keccak_dout_valid_train;
reg [10:0]  keccak_data_done_train;
reg [2*AUX_WORDS+2:0] finalize_aux_train;
reg [17:0] next_verify_round_train;
reg [10:0] commit_valid_train;

//reg [`WORD_SIZE-1:0]         data_mem_din_next;
reg [`WORD_SIZE-1:0]         key_sig_mem_din_next;

reg reject_v_grinding;
reg next_E, squeeze_y_next;
reg init_aux, init_aux_next;
reg commit_buf_idx;

reg update_state, update_state_next;

wire [`WORD_SIZE-1:0] aux_word;
reg [`WORD_SIZE-1:0]  acc_in;

reg [$clog2(SAMPLE_INPUT_ROUNDS)-1:0] sample_counter;
reg sample_is_Srnd_Crnd;
reg next_party;

reg enable_bytelen_din;
always_ff @ (posedge clk) enable_bytelen_din <= keccak_input_ctr[KECCAK_IN_LEN_CTR_BITS-1:1]!=0;

localparam V_GRINDING_LSB       = (8*`TAU) % `WORD_SIZE;
localparam CHAL_WORDS           = `GET_WORD_COUNT_FROM_BYTES(`TAU+1);
localparam EXP_CHAL_REGS_WIDTH  = CHAL_WORDS*`WORD_SIZE;
reg [EXP_CHAL_REGS_WIDTH-1:0] exp_chal_regs;
assign i_star_out = exp_chal_regs[$clog2(`N-1):0];

  /********************/
 /* Pipeline signals */
/********************/
always_ff @ (posedge clk) begin
    keccak_input_ctr_rst_val    <= keccak_input_ctr_rst_val_next;
    keccak_dout_shift_bytes     <= keccak_dout_shift_bytes_next;
//    keccak_bytelen_din          <= (keccak_input_ctr==0) ? 'h0 : keccak_bytelen_din_next;
    keccak_bytelen_din          <= keccak_bytelen_din_next;
    keccak_command              <= keccak_command_next;
    keccak_din                  <= keccak_din_next;
    keccak_din_valid            <= rst ? 1'b0 : (keccak_input_ctr!=0);
    keccak_input_ctr_rst        <= rst ? 1'b0 : keccak_input_ctr_rst_next;
//    data_mem_addr_opc           <= rst ? GOTO_SUBCTX : data_mem_addr_opc_next;
//    data_mem_addr_opc           <= data_mem_addr_opc_next;
    key_sig_mem_addr_opc        <= rst ? GOTO_SK_SEED : key_sig_mem_addr_opc_next;
//    wren_data_mem               <= wren_data_mem_next;
    wren_key_sig_mem            <= wren_key_sig_mem_next;

    keccak_dout_valid_pip       <= keccak_dout_valid;
    keccak_dout_valid_train     <= rst ? 'h0 : {keccak_dout_valid_train, (keccak_dout_valid && ~keccak_dout_valid_pip)};
    keccak_data_done_train      <= rst ? 'h0 : {keccak_data_done_train, keccak_data_done};
    finalize_aux_train          <= {finalize_aux_train, finalize_aux};
    next_verify_round_train     <= {next_verify_round_train, next_verify_round};
//    data_mem_din                <= data_mem_din_next;
    key_sig_mem_din             <= key_sig_mem_din_next;
    next_E_reg                  <= next_E;
    squeeze_y                   <= squeeze_y_next;
    key_unpack_done             <= (state_tl == S_EXP_SEED_PK) && keccak_data_done_train[NUM_Y_SQUEEZES-1];

    init_aux                    <= init_aux_next;
    acc_in                      <= four_counter_0 ? S_C_mem_dout : key_sig_mem_dout;
    incr_mpc_round              <= rst ? 1'b0 : incr_mpc_round_next;
    commit_valid_train          <= {commit_valid_train, commit_valid};

    handshake_pip               <= keccak_din_valid;
//    extra_k_lookahead_update    <= (state_tl==S_COMMIT_TL) && (~|commit_valid_train[`COMMIT_WORDS +: (`COMMIT_WORDS+2)] && (keccak_din_valid || handshake_pip));

    incr_S_C_addr               <= incr_S_C_addr_next;
    wren_S_C                    <= wren_S_C_next;
    S_C_mem_din                 <= S_C_mem_din_next;
    switch_comm_buf             <= switch_comm_buf_next;
    
    bad_sig_out <= bad_sig;
    h_mpc_equality_check <= h_mpc_storage[`WORD_SIZE-1:0]!==key_sig_mem_dout; 
    if (rst)
        bad_sig <= 1'b0;
    else if (verify && (path_rejected || reject_v_grinding || (state_tl==S_HASH_MPC && |keccak_dout_valid_train[2+:`COMMIT_WORDS] && h_mpc_equality_check)))
        bad_sig <= 1'b1;
    
    update_state                <= update_state_next;
    four_counter_incr           <= four_counter_incr_next;
    acc_e_last_word_next        <= !first_state_cycles[12] && {check_last_acc_e_word[`KEY_SIG_MEM_ADDR_BITS-1], check_last_acc_e_word[2:0]}=='h0; // We're reading from the last word of a given aux[e] value
//    mem_addr_upd_en             <= rst ? 1'b1 : mem_addr_upd_en_next;
    
    share_split_done_tl         <= (state_tl==S_PREFILL_TMP);
    store_a_start               <= (state_tl == S_COMPUTE_A && keccak_data_done_train[0] && ~keccak_data_done_train[1]);
    
    if (keccak_input_data_done)
        k_input_done_wait_for_output <= 1'b1;
    else if (keccak_data_done)
        k_input_done_wait_for_output <= 1'b0;
    
    keccak_inp_done_train <= {keccak_inp_done_train, keccak_input_data_done};
    h_mpc_storage_en      <= h_mpc_storage_en_next;
    i_star_valid          <= i_star_valid_next;
    y_word_valid_to_tmp   <= y_word_valid_to_tmp_next;
    next_y_word_from_tmp_pip <= next_y_word_from_tmp;
    
    if (h_mpc_storage_en) h_mpc_storage <= {key_sig_mem_dout, h_mpc_storage[`COMMIT_SIZE-1:`WORD_SIZE]};
    
    incr_ctr <= rst ? 1'b0 : incr_ctr_next;
    if (rst)
        ctr <= 'h0;
    else if (incr_ctr)
        ctr <= ctr+1'b1;
        
    if (state_tl==S_EXP_VIEW_CHAL && keccak_dout_valid)
        exp_chal_regs <= {keccak_dout, exp_chal_regs[EXP_CHAL_REGS_WIDTH-1:`WORD_SIZE]};
    else if (i_star_valid)
        exp_chal_regs <= (exp_chal_regs >> $clog2(`N));
    
    if (state_tl==S_EXP_VIEW_CHAL && keccak_dout_valid_train[CHAL_WORDS-2])
//        reject_v_grinding <= keccak_dout[V_GRINDING_LSB +: 8]!='h0;
        reject_v_grinding <= {keccak_dout[V_GRINDING_LSB+7+8], keccak_dout[V_GRINDING_LSB +: 8]}!='h0;
    else
        reject_v_grinding <= 1'b0;
    
    if (mpc_round_tl==`TAU-2)
        last_mpc_round <= 1'b1;
    else if (~hold_last_mpc_round)
        last_mpc_round <= 1'b0;
end

//assign keccak_din_valid = keccak_input_ctr!=0;

  /*****************/
 /* Control logic */
/*****************/
always_comb begin
    keccak_command_next     = `SHAKE_CMD_64(`NODE_SIZE, `MIRATH_VAR_FF_H_BITS);
//    keccak_command_next     = `SHAKE_CMD_64(`NODE_SIZE, (`MIRATH_VAR_FF_S_BITS + `MIRATH_VAR_FF_C_BITS));
    keccak_din_next         = key_sig_mem_dout;
//    keccak_bytelen_din_next = 'h8;
    keccak_bytelen_din_next = 'h0;
    keccak_dout_shift_bytes_next    = 1'b1;
    keccak_input_ctr_rst_val_next   = `NODE_WORDS+1;
    keccak_input_ctr_rst_next       = 'h0;
//    data_mem_addr_opc_next  = FREEZE_ADDR_D_M_TL;
    key_sig_mem_addr_opc_next  = FREEZE_ADDR_KS_M_TL;
//    wren_data_mem_next      = 1'b0;
    wren_key_sig_mem_next   = 1'b0;
//    data_mem_din_next       = keccak_dout;
    key_sig_mem_din_next    = keccak_dout;
    next_E                  = 1'b0;
    squeeze_y_next          = 1'b0;
    
    init_aux_next           = 1'b0;
    incr_mpc_round_next     = 1'b0;
    
    incr_S_C_addr_next      = 1'b0;
    wren_S_C_next           = 1'b0;
    S_C_mem_din_next        = keccak_dout;
    
    switch_comm_buf_next    = 1'b0;
    rst_store_subctx_flag   = 1'b0;
    
    update_state_next       = 1'b0;
    four_counter_incr_next  = 1'b0;   
    
//    mem_addr_upd_en_next    = mem_addr_upd_en;
    h_mpc_storage_en_next   = 1'b0;
    incr_ctr_next           = 1'b0;
    i_star_valid_next       = 1'b0;
    y_word_valid_to_tmp_next = 1'b0;
    hold_last_mpc_round     = 1'b0;
    
    
    h_subctx_mem_incr_wr_addr_next = 1'b0;
    h_subctx_mem_incr_re_addr_next = 1'b0;
    cmt_fifo_incr_re_addr_next = 1'b0;
    
    case (state_tl)
        S_EXP_SEED_SK: begin
            keccak_command_next     = `SHAKE_CMD_64(`NODE_SIZE, (`MIRATH_VAR_FF_S_BITS + `MIRATH_VAR_FF_C_BITS));
            S_C_mem_din_next = keccak_dout;
            
            if (first_state_cycles[0])
                keccak_input_ctr_rst_next = 1'b1;
                
            if (|first_state_cycles[3+:(`NODE_WORDS)])
                keccak_bytelen_din_next = 'h8;
             
            if (|first_state_cycles[1+:`NODE_WORDS])    
                key_sig_mem_addr_opc_next = PLUS_1_KS_M_TL;
            
            if (keccak_dout_valid && ~keccak_dout_valid_pip) begin
//                S_C_mem_din_next = keccak_dout;// & `S_C_SET_TO_FF_MASK(0);
                wren_S_C_next      =1'b1;
                incr_S_C_addr_next = 1'b1;
            end
            
            if (keccak_dout_valid_train[8-1]) begin
//                S_C_mem_din_next = keccak_dout;// & `S_C_SET_TO_FF_MASK(1);
                wren_S_C_next      =1'b1;
                incr_S_C_addr_next = 1'b1;
            end
            
            if (keccak_dout_valid_train[2*8-1]) begin
//                S_C_mem_din_next = keccak_dout;// & `S_C_SET_TO_FF_MASK(2);
                wren_S_C_next      =1'b1;
                incr_S_C_addr_next = 1'b1;
            end
            
            if (keccak_dout_valid_train[3*8-1] || keccak_dout_valid_train[4*8-1] || keccak_dout_valid_train[5*8-1] || keccak_dout_valid_train[6*8-1]) begin
//                S_C_mem_din_next = keccak_dout;// & `S_C_SET_TO_FF_MASK(3);
                wren_S_C_next      =1'b1;
                incr_S_C_addr_next = 1'b1;
            end
            
            if (keccak_dout_valid_train[7*8-1]) begin
//                S_C_mem_din_next = keccak_dout;// & `S_C_SET_TO_FF_MASK(7);
                wren_S_C_next      =1'b1;
                incr_S_C_addr_next = 1'b1;
            end    
           
           next_E = 1'b1;     
        end // S_EXP_SEED_SK

// ************************************************************************************
        S_EXP_SEED_PK: begin
            keccak_dout_shift_bytes_next    = 1'b0;
            key_sig_mem_din_next            = y_out;
            
            if (first_state_cycles[0]) begin
                keccak_input_ctr_rst_next = 1'b1;
            end
            
            if (|first_state_cycles[3 +:`NODE_WORDS])
                keccak_bytelen_din_next = 'h8;
            
            if (|first_state_cycles[1+:`NODE_WORDS-1]) begin
                key_sig_mem_addr_opc_next = PLUS_1_KS_M_TL;
            end
            
            squeeze_y_next = keccak_data_done || keccak_data_done_train[NUM_Y_SQUEEZES-2:0];
            
            if (keccak_data_done)
                key_sig_mem_addr_opc_next = GOTO_Y;
            else if (keccak_data_done_train[NUM_Y_SQUEEZES-1:0])
                key_sig_mem_addr_opc_next = PLUS_1_KS_M_TL;
                
            wren_key_sig_mem_next = (|keccak_data_done_train[NUM_Y_SQUEEZES-1:0]);
            
            if (keccak_data_done_train[NUM_Y_SQUEEZES-1])
                key_sig_mem_din_next            = y_out & (('h1<<(`MIRATH_VAR_FF_Y_BITS%`WORD_SIZE))-1);
            
            // At the same time, init the AUX accumulator"
            if (|first_state_cycles[7:0]) begin
                incr_S_C_addr_next = 1'b1;
            end
                
            if (|first_state_cycles[9:2])
                init_aux_next = 1'b1;
            
            // Init the first hash_commits subctx
            if (keccak_data_done_train[0]) begin
                keccak_input_ctr_rst_next = 1'b1;
                keccak_input_ctr_rst_val_next = 2'h2;
            end
                        
            if (keccak_data_done_train[3]) begin
                keccak_din_next[7:0] = `DOM_SEP_COMMIT;
            end
            
            if (keccak_data_done_train[3]) begin
                keccak_bytelen_din_next = 1'b1;
            end
            
            if (keccak_data_done_train[2])
                keccak_command_next = `SHA3_CMD_64(`HASH_COM_SUBCTX_IN_LEN);
            
        end // S_EXP_SEED_PK
        
// ************************************************************************************
        S_COMMIT_TL: begin
            keccak_command_next = `SHA3_CMD_64(`HASH_COM_SUBCTX_IN_LEN);
            if (~verify)
                key_sig_mem_addr_opc_next = GOTO_AUX;
            key_sig_mem_din_next = aux_word;
            keccak_dout_shift_bytes_next    = 1'b0;
                        
            // Store one aux instance
            if (|finalize_aux_train[AUX_WORDS-1:0]) begin
                init_aux_next = 1'b1;
            end
            
            if (|finalize_aux_train[AUX_WORDS:1]) begin
                wren_key_sig_mem_next = 1'b1;
            end //  & `S_C_SET_TO_FF_MASK(7)
            
            if (finalize_aux_train[1])
                key_sig_mem_din_next = aux_word & `S_C_SET_TO_FF_MASK(0);
            else if (finalize_aux_train[2])
                key_sig_mem_din_next = aux_word & `S_C_SET_TO_FF_MASK(1);
            else if (finalize_aux_train[3])
                key_sig_mem_din_next = aux_word & `S_C_SET_TO_FF_MASK(2);
            else if (finalize_aux_train[4])
                key_sig_mem_din_next = aux_word & `S_C_SET_TO_FF_MASK(3);
            else if (finalize_aux_train[5])
                key_sig_mem_din_next = aux_word & `S_C_SET_TO_FF_MASK(4);
            else if (finalize_aux_train[6])
                key_sig_mem_din_next = aux_word & `S_C_SET_TO_FF_MASK(5);
            else if (finalize_aux_train[7])
                key_sig_mem_din_next = aux_word & `S_C_SET_TO_FF_MASK(6);
            else if (finalize_aux_train[8])
                key_sig_mem_din_next = aux_word & `S_C_SET_TO_FF_MASK(7);
            
            if (|finalize_aux_train[AUX_WORDS-1:1])
                key_sig_mem_addr_opc_next = PLUS_1_KS_M_TL;
            
            // Increment mpc_round
            if (finalize_aux_train[0] || next_verify_round_train[17])
                incr_mpc_round_next = 1'b1;
            
            
            if (|finalize_aux_train[AUX_WORDS+1 +: (AUX_WORDS-1)]) begin
                key_sig_mem_addr_opc_next = PLUS_1_KS_M_TL;
            end // 3 cycles pipeline wait for correct input to the ACC unit
            
            if (|finalize_aux_train[AUX_WORDS+1 +: AUX_WORDS])
                incr_S_C_addr_next = 1'b1;
            
            if (|finalize_aux_train[AUX_WORDS+3 +: AUX_WORDS]) begin
                init_aux_next = 1'b1;
            end 
            
            // *********************
            // Hash the commits:
            if (~party_is_i_star_fr_aes)
                keccak_din_next         = cmt_fifo_dout; // If party_idx == i* , we should be reading from the sig instead
            
            if ((commit_valid_train[0])) begin
                if (verify)
                    key_sig_mem_addr_opc_next = GOTO_COMMIT;
            end else if (verify) begin
                    key_sig_mem_addr_opc_next = PLUS_1_KS_M_TL;
            end
           
            if (~prep_store_hash_subctx) begin
//                if (commit_valid_train[0]) begin
//                    if (verify)
//                        key_sig_mem_addr_opc_next = GOTO_COMMIT;
////                    data_mem_addr_opc_next = GOTO_COMMIT_BUFF_TL;
//        //            end else if (|commit_valid_train[1 +: (`COMMIT_WORDS-1)] && ~k_look_ahead_state) begin
//                end else if (|commit_valid_train[1 +: (`COMMIT_WORDS-1)] || keccak_din_valid) begin
//                    if (verify)
//                        key_sig_mem_addr_opc_next = PLUS_1_KS_M_TL;
////                    data_mem_addr_opc_next = PLUS_1_D_M_TL;
//                end
                
                
//                if (|commit_valid_train[1 +: (`COMMIT_WORDS)] || keccak_input_ctr!=0)
                if (|commit_valid_train[1 +: (`COMMIT_WORDS)])
                    cmt_fifo_incr_re_addr_next = 1'b1;
                
                if (commit_valid_train[1]) begin
                   keccak_input_ctr_rst_next = 1'b1;
                   keccak_input_ctr_rst_val_next = `COMMIT_WORDS; 
                end
                
//                if ((commit_valid_train[3+:`COMMIT_WORDS] && ~k_look_ahead_state_train[2]) // TODO: make it pretty. It just says 
//                || (keccak_input_ctr!=0 && ~k_look_ahead_state_train[2] && ~(keccak_input_ctr==1 && ~k_look_ahead_state_train[3])) ) begin // the counter is not abput to become zero.
                
                if (commit_valid_train[3+:`COMMIT_WORDS]) begin // the counter is not abput to become zero.
                   keccak_bytelen_din_next = 'h8;
                end
            
            // ***********************************************
            // Store commit & start next one (if not done)
            end else begin
                
                if (keccak_dout_valid) // Store the sub_ctx hash that finished
                    h_subctx_mem_incr_wr_addr_next = 1'b1;
                
                if (~four_counter_0) begin
                    if (keccak_data_done_train[0]) begin // Start the next hash subctx
                        keccak_input_ctr_rst_next = 1'b1;
                        keccak_input_ctr_rst_val_next = commit_buf_idx ? 3'h2 : 3'h6;
                        
                        //                        data_mem_addr_opc_next = GOTO_COMMIT_BUFF_TL;
                    end
                    
                    if (verify && keccak_data_done_train[1])
                        key_sig_mem_addr_opc_next = GOTO_COMMIT; // Note: little bugfix (i* at the start of MPC round with commit_buf_idx==1'b1)

//                    else if (|keccak_data_done_train[3:1])
//                        data_mem_addr_opc_next = PLUS_1_D_M_TL;
                        
                    if (|keccak_data_done_train[2+:`COMMIT_WORDS] && ~commit_buf_idx)
                        cmt_fifo_incr_re_addr_next = 1'b1;
                    
                    if (keccak_data_done_train[3]) begin
                        keccak_din_next[7:0] = `DOM_SEP_COMMIT;
                    end
                    
                    if (keccak_data_done_train[3]) begin
                        keccak_bytelen_din_next = 1'b1;
                    end
                    
//                    if (keccak_data_done_train[1])
//                        keccak_command_next     = `SHA3_CMD_64(`HASH_COM_SUBCTX_IN_LEN);
                end // if (~four_counter_0)

                if (~commit_buf_idx) begin // Note: doesn't need to be gated by ~four_counter_0 for L1. Check if it stays that way for L3 / L5
                    if (keccak_data_done_train[7:4]) begin
                        keccak_bytelen_din_next = 'h8;
                    end
                end
                
                if (keccak_data_done_train[7]) begin
                    rst_store_subctx_flag   = 1'b1;
                    if (four_counter_0) // Exit time
                        update_state_next = 1'b1;
                end              
            end // if (~prep_store_hash_subctx) - else ...
                
            if (update_state) begin
                four_counter_incr_next = 1'b1;
            end
            
            if (verify) begin // Init the first hash_commits subctx
                if (first_state_cycles[0]) begin
                    keccak_input_ctr_rst_next = 1'b1;
                    keccak_input_ctr_rst_val_next = 2'h2;
                end
                
                if (first_state_cycles[3]) begin
                    keccak_din_next[7:0] = `DOM_SEP_COMMIT;
                end
                
                if (first_state_cycles[3]) begin
                    keccak_bytelen_din_next = 1'b1;
                end
            end      
        end // S_COMMIT_TL

// ************************************************************************************
        S_HASH_COM: begin
            keccak_dout_shift_bytes_next    = 1'b0;
            keccak_command_next     = `SHA3_CMD_64(4*`COMMIT_SIZE);
//            data_mem_addr_opc_next = GOTO_SUBCTX;
//            keccak_din_next         = data_mem_dout;
            keccak_din_next         = h_subctx_mem_dout;
            
            if (first_state_cycles[1]) begin
                keccak_input_ctr_rst_next = 1'b1;
                keccak_input_ctr_rst_val_next = (2'h1 + 4*`COMMIT_WORDS); //1 command + 4 hashes
            end
            
            if (first_state_cycles[4 +: 4*`COMMIT_WORDS])
                keccak_bytelen_din_next = 'h8;
            
//            if (first_state_cycles[2 +: (4*`COMMIT_WORDS-1)])
//                data_mem_addr_opc_next = PLUS_1_D_M_TL;
                
            if (first_state_cycles[2 +: (4*`COMMIT_WORDS)])
                h_subctx_mem_incr_re_addr_next = 1'b1;
            
            if (keccak_dout_valid) begin // Store the sub_ctx hash that finished
//                wren_data_mem_next = 1'b1;
//                data_mem_addr_opc_next = PLUS_1_D_M_TL;
                h_subctx_mem_incr_wr_addr_next = 1'b1;
            end            
        end // S_HASH_COM

// ************************************************************************************
        S_HASH_SH: begin // first_state_cycles
            keccak_dout_shift_bytes_next    = 1'b0;
            keccak_command_next     = `SHA3_CMD_64(8 + 2*`COMMIT_SIZE + (`TAU*'h8*(S_BYTES + C_BYTES))); // 1 dom_sep  + 1 salt + 1 hash + tau*AUX
            key_sig_mem_addr_opc_next = PLUS_1_KS_M_TL;
            
            if (first_state_cycles[1]) begin
                keccak_input_ctr_rst_next = 1'b1;
                keccak_input_ctr_rst_val_next = K_IN_LEN_H_SH;
            end
            
            if (first_state_cycles[2])
                key_sig_mem_addr_opc_next = GOTO_SALT;
            
            if (first_state_cycles[4])
                keccak_din_next[7:0] = `DOM_SEP_HASH1;
            
            if (first_state_cycles[3])
                keccak_bytelen_din_next = 'h0;
            else if (first_state_cycles[4])
                keccak_bytelen_din_next = 'h1;
            else if (acc_e_last_word_next)
                keccak_bytelen_din_next = 'h6;
//            else if (keccak_input_ctr[KECCAK_IN_LEN_CTR_BITS-1:1]!=0)
            else if (enable_bytelen_din)
                keccak_bytelen_din_next = 'h8;
                       
            if (first_state_cycles[9+:`COMMIT_WORDS])
                keccak_din_next = h_subctx_mem_dout;
                
            if (first_state_cycles[7+:`COMMIT_WORDS-1])
                h_subctx_mem_incr_re_addr_next = 1'b1;
               
            if (first_state_cycles[10])
                key_sig_mem_addr_opc_next = GOTO_AUX;          
            
            // *****************************************************************
            if (k_input_done_wait_for_output) begin
                key_sig_mem_addr_opc_next = GOTO_HASH_SH;
            end
            
            if (keccak_dout_valid) begin // Store the sub_ctx hash that finished
                wren_key_sig_mem_next = 1'b1;
                key_sig_mem_addr_opc_next = PLUS_1_KS_M_TL;
            end 
        end // S_HASH_SH

// ********************************************************************
        S_PREFILL_TMP: begin
            if (first_state_cycles[0])
                key_sig_mem_addr_opc_next = GOTO_Y;
            else if (next_y_word_from_tmp_pip)
                key_sig_mem_addr_opc_next = PLUS_1_KS_M_TL;
            
            
            if (verify) begin
                    y_word_valid_to_tmp_next = 1'b1;
            end
        end
        
// ********************************************************************
        S_COMPUTE_TMP: begin
            key_sig_mem_addr_opc_next = GOTO_PK_SEED;
            
            if (first_state_cycles[1]) begin
                keccak_input_ctr_rst_next = 1'b1;
            end
            
            if (|first_state_cycles[4+:`NODE_WORDS])
                keccak_bytelen_din_next = 'h8;
            
            if (|first_state_cycles[2+:`NODE_WORDS]) begin
                key_sig_mem_addr_opc_next = PLUS_1_KS_M_TL;
            end
        end // S_COMPUTE_TMP
        
// ********************************************************************
        S_COMPUTE_A: begin            
            keccak_command_next <= `SHAKE_CMD_64(`COMMIT_SIZE, 8*`MIRATH_VAR_GAMMA);
            key_sig_mem_din_next = a_word;
            
            if (first_state_cycles[1])
                key_sig_mem_addr_opc_next = GOTO_HASH_SH; // Get Gamma from Keccak
            
            if (first_state_cycles[1]) begin
                keccak_input_ctr_rst_next = 1'b1;
            end
            
            if (|first_state_cycles[4+:`COMMIT_WORDS])
                keccak_bytelen_din_next = 'h8;
            
            if (|first_state_cycles[2+:`COMMIT_WORDS])
                key_sig_mem_addr_opc_next = PLUS_1_KS_M_TL;
            
            if (a_word_valid) begin
                if (~a_word_is_mid || ~verify) // During verification, a_mid is not valid. Gatting writes should be enough.
                    wren_key_sig_mem_next = 1'b1;
                key_sig_mem_addr_opc_next = PLUS_1_KS_M_TL;
            end
                
            
        end // S_COMPUTE_A
        
// ********************************************************************
        S_HASH_MPC: begin
            // keccak_din is always the key/sig mem this time
            keccak_dout_shift_bytes_next = 1'b0;
            keccak_command_next  = `SHA3_CMD_64(8 + 2*`COMMIT_SIZE + `PK_BITS + (`MSG_LEN_BYTES<<3) +`TOTAL_A_BITS); // Add alpha_mid & alpha_base
            
            if (key_sig_mem_addr==`SALT_ADDR)
                keccak_bytelen_din_next = `GET_BYTES(`PK_LAST_WORD_BITS);
            else if (key_sig_mem_addr==`HASH_SH_ADDR)
                keccak_bytelen_din_next = `LAST_MSG_WORD_BYTES;
//            else if (keccak_input_ctr[KECCAK_IN_LEN_CTR_BITS-1:1]!=0)
            else if (enable_bytelen_din)
                keccak_bytelen_din_next = 'h8;
            
            if (first_state_cycles[1]) begin
                keccak_input_ctr_rst_next = 1'b1;
                keccak_input_ctr_rst_val_next = K_IN_LEN_H_MPC;
            end
            
            if (first_state_cycles[2])
                key_sig_mem_addr_opc_next = GOTO_PK_SEED;
            else if (first_state_cycles[3] || keccak_din_valid)
                key_sig_mem_addr_opc_next = PLUS_1_KS_M_TL;
            
            if (first_state_cycles[4])
                keccak_din_next[7:0] = `DOM_SEP_HASH2_PART;
            
            if (first_state_cycles[4])
                keccak_bytelen_din_next = 'h1;
            
            // **************************************
            // Store h_mpc
            if (keccak_inp_done_train[0] || keccak_inp_done_train[`COMMIT_WORDS])
                key_sig_mem_addr_opc_next = GOTO_H_MPC;
            else if (|keccak_inp_done_train[1+:`COMMIT_WORDS-1])
                key_sig_mem_addr_opc_next = PLUS_1_KS_M_TL;
            
            if (|keccak_inp_done_train[2+:`COMMIT_WORDS] || |keccak_dout_valid_train[0+:`COMMIT_WORDS])
                h_mpc_storage_en_next = 1'b1;
                
            if (keccak_dout_valid) begin // Store h_mpc
                wren_key_sig_mem_next = 1'b1;
                key_sig_mem_addr_opc_next = PLUS_1_KS_M_TL;
            end
                
                
        end // S_HASH_MPC
        
// *************************************************
    S_EXP_VIEW_CHAL: begin
        keccak_dout_shift_bytes_next    = 1'b0;
        keccak_command_next     = `SHAKE_CMD_64(`COMMIT_SIZE+64, 8*`TAU+8); // input: h_mpc + ctr, output: i* and v_grinding
        key_sig_mem_din_next    = {32'h0, ctr};
        
        if (first_state_cycles[1]) begin
            keccak_input_ctr_rst_next = 1'b1;
            keccak_input_ctr_rst_val_next = 'h6;
        end
        
        if (|first_state_cycles[4 +: `COMMIT_WORDS+1])
            keccak_bytelen_din_next = 'h8;
        
        if (|first_state_cycles[1])
            key_sig_mem_addr_opc_next = GOTO_H_MPC;
        else if (|first_state_cycles[2+:`COMMIT_WORDS])
            key_sig_mem_addr_opc_next = PLUS_1_KS_M_TL;
        
        if (first_state_cycles[4+`COMMIT_WORDS]) begin
            if (~verify) begin
                keccak_din_next = {32'h0, ctr};
                wren_key_sig_mem_next = 1'b1;
            end
            incr_ctr_next   = 1'b1;
        end
    end
    
    S_SEND_I_STAR: begin
        hold_last_mpc_round = 1'b1;
        
        if (~last_mpc_round) begin
            incr_mpc_round_next = 1'b1;
            i_star_valid_next   = 1'b1;
        end
    end
    
    endcase // state_tl
end

  /****************/
 /* State update */
/****************/
always_ff @ (posedge clk) state_tl  <=  rst ?  S_IDLE_TL : next_state_tl;

  /********************/
 /* Next_state logic */
/********************/
always_comb begin
    next_state_tl = state_tl; // Default
    
    case (state_tl)
//        S_IDLE_TL:         next_state_tl = ~start_en ? S_IDLE_TL       :
//                                            verify   ? S_EXP_VIEW_CHAL : S_EXP_SEED_SK;
        S_IDLE_TL: begin
            if (verify) begin
//                if (aes_waiting_for_i_star)
                    next_state_tl = S_EXP_VIEW_CHAL;
            end else begin
                if (start_en)
                    next_state_tl = S_EXP_SEED_SK;
            end
        end
        
        S_EXP_SEED_SK:  next_state_tl = acc_y_init_done ? S_EXP_SEED_PK : S_EXP_SEED_SK;
        
        S_EXP_SEED_PK:  next_state_tl = ~keccak_data_done_train[NUM_Y_SQUEEZES-1] ? S_EXP_SEED_PK :
                                            sign                                  ? S_COMMIT_TL   : S_DONE_TL;
                                            
        S_COMMIT_TL:    next_state_tl = update_state ? S_HASH_COM : S_COMMIT_TL;                       
        
        S_HASH_COM:     next_state_tl = keccak_data_done_train[0] ? S_HASH_SH : S_HASH_COM;                       
        
        S_HASH_SH:     next_state_tl = keccak_data_done_train[0] ? S_PREFILL_TMP : S_HASH_SH;                       
//        S_HASH_SH:     next_state_tl = tmp_prefill_done ? S_COMPUTE_TMP : S_HASH_SH;                       
        
        S_PREFILL_TMP:  next_state_tl = tmp_prefill_done ? S_COMPUTE_TMP : S_PREFILL_TMP;                       
        
        S_COMPUTE_TMP:    next_state_tl = keccak_data_done_train[0] ? S_COMPUTE_A : S_COMPUTE_TMP;                       
        
        S_COMPUTE_A:    next_state_tl = store_a_done ? S_HASH_MPC : S_COMPUTE_A;                       
        
//        S_HASH_MPC:     next_state_tl = keccak_data_done_train[0] ? S_EXP_VIEW_CHAL : S_HASH_MPC;
        S_HASH_MPC: begin
            if (keccak_data_done_train[0]) begin
                if (verify)
                    next_state_tl = S_DONE_TL;
                else
                    next_state_tl = S_EXP_VIEW_CHAL;
            end
        end
        
//        S_EXP_VIEW_CHAL: next_state_tl = (keccak_dout_valid_train[CHAL_WORDS-1] && ~reject_v_grinding) ? S_SEND_I_STAR : S_EXP_VIEW_CHAL;
        S_EXP_VIEW_CHAL: begin
            if (keccak_dout_valid_train[CHAL_WORDS-1]) begin
                if (reject_v_grinding) begin
                    if (verify)
                        next_state_tl = S_DONE_TL;
                    else
                        next_state_tl = S_EXP_VIEW_CHAL;
                end else
                    next_state_tl = S_SEND_I_STAR;
            end
        end
        
//        S_SEND_I_STAR:  next_state_tl = path_rejected ? S_EXP_VIEW_CHAL :
//                                        aes_is_done   ? S_DONE_TL       : S_SEND_I_STAR;   
        S_SEND_I_STAR: begin
            if (verify) begin
                if (path_rejected)
                    next_state_tl = S_DONE_TL;
                else if (path_accepted)
                    next_state_tl = S_COMMIT_TL;
            end else begin
                if (path_rejected)
                    next_state_tl = S_EXP_VIEW_CHAL;
                else if (aes_is_done)
                    next_state_tl = S_DONE_TL;
            end
        end          
    endcase // state_tl
end

  /*********************/
 /* State update flag */
/*********************/
always_ff @ (posedge clk) begin
    first_state_cycles[10:0]   <= rst ? 'h0 : {first_state_cycles[9:0], 1'b0};
    first_state_cycles[F_ST_CYCLE_MAX:11]   <= first_state_cycles[F_ST_CYCLE_MAX-1:10];
    
    case (state_tl)
//        S_IDLE_TL       : first_state_cycles[0]  <= verify ? aes_waiting_for_i_star : start_en;
        S_IDLE_TL       : first_state_cycles[0]  <= start_en;
        S_EXP_SEED_SK   : first_state_cycles[0]  <= acc_y_init_done;
        S_EXP_SEED_PK   : first_state_cycles[0]  <= keccak_data_done_train[NUM_Y_SQUEEZES-1];
        S_COMMIT_TL     : first_state_cycles[0]  <= update_state;
        S_HASH_COM      : first_state_cycles[0]  <= keccak_data_done_train[0];
        S_HASH_SH       : first_state_cycles[0]  <= keccak_data_done_train[0];
        S_PREFILL_TMP   : first_state_cycles[0]  <= tmp_prefill_done;
        S_COMPUTE_TMP   : first_state_cycles[0]  <= keccak_data_done_train[0];
        S_COMPUTE_A     : first_state_cycles[0]  <= store_a_done;
        S_HASH_MPC      : first_state_cycles[0]  <= keccak_data_done_train[0];
        S_EXP_VIEW_CHAL : first_state_cycles[0]  <= keccak_dout_valid_train[CHAL_WORDS-1];
        S_SEND_I_STAR   : first_state_cycles[0]  <= path_rejected || path_accepted;
    endcase // state_tl
end

  /******************/
 /* Start commands */
/******************/
always_ff @ (posedge clk) begin
    if (rst) begin
        start_en <= 1'b0;
        sign     <= 1'b0;
        verify   <= 1'b0;
        
    end else begin
        if (|start)
            start_en <= 1'b1;
        
        if (~start_en) begin
            if (start==`START_SIGN)
                sign    <=1'b1;
                
            if (start==`START_VERIFY)
                verify  <=1'b1;
        end
    end
end

  /********************/
 /* Keccak input ctr */
/********************/
always_ff @ (posedge clk) begin
    if (rst)
        keccak_input_ctr <= 'h0;
    else if (keccak_input_ctr_rst)
        keccak_input_ctr <= keccak_input_ctr_rst_val;
    else if (keccak_input_ctr!=0)
        keccak_input_ctr <= keccak_input_ctr -1'b1;
end

  /*******************/
 /* Update adresses */
/*******************/
always_ff @ (posedge clk) begin
//    case (data_mem_addr_opc)
//        FREEZE_ADDR_D_M_TL:     data_mem_addr <= data_mem_addr;
//        PLUS_1_D_M_TL:          data_mem_addr <= data_mem_addr + 1'b1;
//        GOTO_COMMIT_BUFF_TL:    data_mem_addr <= {(`COMMIT_ADDR_OFFSET + (commit_buf_idx ? `NODE_WORDS : 'h0)), {$clog2(`NODE_WORDS){1'b0}}};
////        GOTO_SUBCTX:            data_mem_addr <= `SUBCTX_0_ADDR + {four_counter_prev, 1'b0, {$clog2(`NODE_WORDS){1'b0}}};
//    endcase // data_mem_addr_opc
    
    case (key_sig_mem_addr_opc)
        FREEZE_ADDR_KS_M_TL:    key_sig_mem_addr <= key_sig_mem_addr;
        PLUS_1_KS_M_TL:         key_sig_mem_addr <= key_sig_mem_addr+1'b1;
//        GOTO_COMMIT:            key_sig_mem_addr <= (`KEY_SIG_MEM_COMMIT_ADDR + {mpc_round_tl, 2'b0});
        GOTO_SK_SEED:           key_sig_mem_addr <= `SK_SEED_ADDR;
        GOTO_PK_SEED:           key_sig_mem_addr <= `PK_SEED_ADDR;
        GOTO_Y:                 key_sig_mem_addr <= `Y_ADDR;
        GOTO_AUX:               key_sig_mem_addr <= `AUX_ADDR + (mpc_round_tl<<3);
        GOTO_SALT:              key_sig_mem_addr <= `SALT_ADDR;
        GOTO_HASH_SH:           key_sig_mem_addr <= `HASH_SH_ADDR;
        GOTO_MSG:               key_sig_mem_addr <= `MSG_ADDR;
        GOTO_ALPHA_BASE:        key_sig_mem_addr <= `ALPHA_BASE_0_ADDR;
//        GOTO_ALPHA_MID:         key_sig_mem_addr <= `ALPHA_MID_0_ADDR;
        GOTO_H_MPC:             key_sig_mem_addr <= `H_MPC_ADDR;
        GOTO_COMMIT:            key_sig_mem_addr <= `KEY_SIG_MEM_COMMIT_ADDR + {mpc_round_tl, {$clog2(`COMMIT_WORDS){1'b0}}};
    endcase // key_sig_mem_addr_opc
end

  /***********************/
 /* bookkeeping updates */
/***********************/
always_ff @ (posedge clk) begin
    four_counter_0  <= (four_counter == 2'b0);
    four_counter_prev <= four_counter-1'b1;
    
    if (rst)
        four_counter <= 2'h0;
    else if (four_counter_incr)
        four_counter <= four_counter + 2'h1;
    else if (incr_mpc_round && mpc_round_tl==`TAU-1 && state_tl!=S_SEND_I_STAR)
        four_counter <= four_counter + 2'h1;
    
    if (rst)
        mpc_round_tl <=  'h0;
    else if (incr_mpc_round)
        mpc_round_tl <= (mpc_round_tl==`TAU-1) ? 'h0 : (mpc_round_tl+1'b1);

    if (rst)
        commit_buf_idx <= 1'b1;
    else if (switch_comm_buf)
        commit_buf_idx <= ~commit_buf_idx;
    else if (commit_valid)// && ~prep_store_hash_subctx)
        commit_buf_idx <= ~commit_buf_idx;
end

  /******************************/
 /* aux[e] accumlator instance */
/******************************/
aux_acc u_aux_acc (
    .clk        ( clk ),

    .init_aux   ( init_aux ),
    .acc_in     ( acc_in ),

    .acc_aux    ( sample_valid_tl && sample_is_Srnd_Crnd ),
    .sample_in  ( sample_tl ),

    .acc_out    ( aux_word )
);

  /******************/
 /* sample_counter */
/******************/
always_ff @ (posedge clk) begin
    sample_is_Srnd_Crnd <= (sample_counter < AUX_WORDS);
    next_party  <= (sample_counter==SAMPLE_INPUT_ROUNDS-1);
    
    if (rst)
        sample_counter <= 'h0;
    else if (sample_valid_tl)
        sample_counter <= (sample_counter!=SAMPLE_INPUT_ROUNDS-1) ? (sample_counter+1'b1) : 'h0;
end

  /***********/
 /* S_C_mem */
/***********/
//reg [2:0] S_C_mem_address;
//reg incr_S_C_addr, incr_S_C_addr_next, wren_S_C, wren_S_C_next;
always_ff @ (posedge clk) begin
    if (rst)
        S_C_mem_address <= 1'b0;
    else if (incr_S_C_addr)
        S_C_mem_address <= S_C_mem_address + 1'b1;
end

simple_dual_port_mem_distr #(
    .WIDTH  (`WORD_SIZE),
    .DEPTH  (8)
) S_C_mem (
    .clk    (clk),
    .wea    (wren_S_C),
    .addra  (S_C_mem_address),
    .addrb  (S_C_mem_address),
    
    .dia     (S_C_mem_din),
    .dob     (S_C_mem_dout)
);

  /*******************************/
 /* prep_store_hash_subctx flag */
/*******************************/
always_ff @ (posedge clk) begin
    if (rst || rst_store_subctx_flag)
        prep_store_hash_subctx <= 1'b0;
    else if (keccak_input_data_done && state_tl==S_COMMIT_TL)
        prep_store_hash_subctx <= 1'b1;
end

always_ff @ (posedge clk) done_tl <= rst ? 1'b0 : (state_tl==S_DONE_TL);

// *************************************************************
// hash subctx mem updates: depth always power of 2 for L1 & L5
reg [`WORD_SIZE-1:0] keccak_dout_reg;
simple_dual_port_mem_distr #(
    .WIDTH  (`WORD_SIZE),
    .DEPTH  (`H_SUBCTX_MEM_DEPTH)
) h_subctx_mem (
    .clk    (clk),
    .wea    (h_subctx_mem_incr_wr_addr), // Can be used as wren
    .addra  (h_subctx_mem_wr_addr),
    .addrb  (h_subctx_mem_re_addr),
    
    .dia     (keccak_dout_reg),
    .dob     (h_subctx_mem_dout)
);

always_ff @ (posedge clk) begin
    if (rst) begin
        h_subctx_mem_wr_addr <= 'h0;
        h_subctx_mem_re_addr <= 'h0;
        h_subctx_mem_incr_wr_addr <= 1'b0;
        h_subctx_mem_incr_re_addr <= 1'b0;
    end else begin
        if (h_subctx_mem_incr_wr_addr)
            h_subctx_mem_wr_addr <= h_subctx_mem_wr_addr + 1'b1;
        if (h_subctx_mem_incr_re_addr)
            h_subctx_mem_re_addr <= h_subctx_mem_re_addr + 1'b1;
        
        h_subctx_mem_incr_wr_addr <= h_subctx_mem_incr_wr_addr_next;
        h_subctx_mem_incr_re_addr <= h_subctx_mem_incr_re_addr_next;
        keccak_dout_reg <= keccak_dout;
    end
end

// *******************************************
// cmt_fifo defines: depth always power of 2 for L1 & L5
always_ff @ (posedge clk) begin
    if (rst) begin
        cmt_fifo_incr_re_addr <= 1'b0;
        cmt_fifo_addr_re <= 1'b0;
    end else begin
        cmt_fifo_incr_re_addr <= cmt_fifo_incr_re_addr_next;
        
        if (cmt_fifo_incr_re_addr)
            cmt_fifo_addr_re <= cmt_fifo_addr_re + 1'b1;
    end
end

endmodule
