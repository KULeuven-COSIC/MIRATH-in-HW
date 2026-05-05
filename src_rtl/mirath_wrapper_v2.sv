/*
 * mirath_wrapper_v2.sv
 * ----------------------
 * This file is the main top-level wrapper of our design.
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

module mirath_wrapper_v2 # (
    parameter CLOG2_WORD_BYTES = $clog2(`WORD_SIZE>>3)
)(
    input wire          rst,
    input wire          clk,

    input wire [1:0]    start,
    
    output wire         done,
    output wire         bad_sig
);

localparam MSG_LEN_WORDS = `MSG_LEN_WORDS;

wire wren_data_mem_0 = 1'b0;
wire wren_data_mem_1;
wire wren_key_sig_mem_0, wren_key_sig_mem_1;

wire [`DATA_MEM_ADDR_BITS-1:0] wr_addr_data_mem, addr_data_mem_1;
wire [`KEY_SIG_MEM_ADDR_BITS-1:0] addr_key_sig_mem_0, addr_key_sig_mem_1;
wire [`WORD_SIZE-1:0] result_slice_aes;
reg  [`WORD_SIZE-1:0] key_sig_mem_din_0, key_sig_mem_din_1, data_mem_din_0, data_mem_din_1;
wire [`WORD_SIZE-1:0] data_mem_dout_0, data_mem_dout_1, key_sig_mem_dout_0, key_sig_mem_dout_1;

wire [`WORD_SIZE-1:0] keccak_command, keccak_din, keccak_dout;
wire mem_2_mem_mux_sel_aes, key_extract_done, keccak_done, c2k_din_valid, k2c_dout_valid, c2k_din_ready, k2c_shift_bytes;
wire keccak_input_data_done;

wire [CLOG2_WORD_BYTES:0] keccak_bytelen_din;

wire [$clog2(`N-1)-1:0] i_star_ctrl_2_aes;
wire                    i_star_valid_ctrl_2_aes;
wire                    party_is_i_star_aes_to_ctrl;
wire                    rst_v_regs;
wire                    y_word_valid_to_tmp, next_y_word_tmp_to_tl;
wire                    rst_y_mul;

wire a_mid_valid_to_a_wrap;

wire [7:0] E_byte;
wire       E_byte_valid;
wire       share_split_done_aes, next_E_ff_mu;
wire       share_split_done_tl;
wire       base_to_ff_wrap_start_MPC;
wire       init_acc_round_aes_pip;

wire [7:0]        E_b_elem [`TAU-1:0];
wire [7:0]        E_m_elem [`TAU-1:0];

E_mul_opcode_t    E_mul_opc_wrap;
wire [7:0] S_base_elem [`TAU-1:0];
wire [7:0] C_base_elem [`TAU-1:0];

wire S_elem, C_elem;
wire E_mul_res_valid;
wire tmp_prefill_done;

wire                    next_E, acc_y_init_done, squeeze_y, key_unpack_done;
wire [`WORD_SIZE-1:0]   y_out, sample_aes;
wire                    sample_valid_aes, commit_valid_aes, party_is_i_star_aes, finalize_aux, next_verify_round;

wire [$clog2(`N)-1:0]   phi_i_aes_2_base;
wire [$clog2(`TAU)-1:0] mpc_round_aes_2_base;

wire [7:0]        tmp_dout_base [`TAU-1:0];
wire [7:0]        tmp_dout_mid [`TAU-1:0];

wire [`WORD_SIZE-1:0] a_word;
wire                  a_word_valid;
wire                  a_word_is_mid;
wire                  store_a_start;
wire                  store_a_done;

wire                accept_path_aes; 
wire                reject_path_aes; 
wire                aes_is_done;
//wire                aes_waiting_for_i_star;

// Commit fifo regs & wires
wire [`CMT_FIFO_ADDR_BITS-1:0] cmt_fifo_addr_wr, cmt_fifo_addr_re;
wire cmt_fifo_wren;
wire [`WORD_SIZE-1:0] cmt_fifo_dout;

always @ (posedge clk) key_sig_mem_din_1 <= mem_2_mem_mux_sel_aes ? data_mem_dout_1 : result_slice_aes;
always @ (posedge clk) data_mem_din_1    <= mem_2_mem_mux_sel_aes ? key_sig_mem_dout_1 : result_slice_aes;

mirath_top_lvl_ctrl_v2 u_mirath_top_lvl_ctrl_v2 (
    .clk                     ( clk ),
    .rst                     ( rst ),
    .start                   ( start ),

    // Keccak inputs
    .keccak_dout_valid       ( k2c_dout_valid ),
//    .keccak_din_ready        ( c2k_din_ready ),
    .keccak_data_done        ( keccak_done ),
    .keccak_dout             ( keccak_dout ),
    .keccak_input_data_done  ( keccak_input_data_done ),

    // AES status and data
    .finalize_aux            ( finalize_aux ),
    .next_verify_round       ( next_verify_round ),
    .commit_valid            ( commit_valid_aes ),
    .sample_valid_tl         ( sample_valid_aes ),
    .sample_tl               ( sample_aes ),
    .path_accepted           ( accept_path_aes),
    .path_rejected           ( reject_path_aes),
    .aes_is_done             ( aes_is_done),
//    .aes_waiting_for_i_star  ( aes_waiting_for_i_star),
    .party_is_i_star_fr_aes  ( party_is_i_star_aes_to_ctrl),
    
    // Inputs from a_wrapper
    .a_word                  ( a_word ),
    .a_word_valid            ( a_word_valid ),
    .a_word_is_mid           ( a_word_is_mid ),
    .store_a_done            ( store_a_done ),
    
    // Memory inputs
//    .data_mem_dout           ( data_mem_dout_0 ),
    .key_sig_mem_dout        ( key_sig_mem_dout_0 ),
    .cmt_fifo_dout           ( cmt_fifo_dout ),

    // Keccak configuration outputs
    .keccak_bytelen_din      ( keccak_bytelen_din ),
    .keccak_dout_shift_bytes ( k2c_shift_bytes ),
    .keccak_din_valid        ( c2k_din_valid ),
    
    // Inputs from acc_y
    .acc_y_init_done         ( acc_y_init_done ),
    .y_out                   ( y_out ),
    
    // Signal to end AES wait state
    .key_unpack_done         ( key_unpack_done ),

    // Keccak data outputs
    .keccak_din              ( keccak_din ),
    .keccak_command          ( keccak_command ),
    
    // TMP acc status
    .tmp_prefill_done        ( tmp_prefill_done ),
    .next_y_word_from_tmp    ( next_y_word_tmp_to_tl ),
    
    // y_acc controls
    .squeeze_y               ( squeeze_y ),
    
    // Memory controls
//    .wren_data_mem           ( wren_data_mem_0 ),
    .wren_key_sig_mem        ( wren_key_sig_mem_0 ),
//    .data_mem_addr           ( addr_data_mem_0 ),
    .key_sig_mem_addr        ( addr_key_sig_mem_0 ),
//    .data_mem_din            ( data_mem_din_0 ),
    .key_sig_mem_din         ( key_sig_mem_din_0 ),
    .cmt_fifo_addr_re        ( cmt_fifo_addr_re ),
    
    // Tell a_wrapper to start the store procedure
    .store_a_start           ( store_a_start ),
    
    // Send i_star to aes
    .i_star_out              ( i_star_ctrl_2_aes ),
    .i_star_valid            ( i_star_valid_ctrl_2_aes ),
    
    // Send y to tmp_wrapper
    .y_word_valid_to_tmp     ( y_word_valid_to_tmp ),
    
    // Notify arithmetic modules
    .share_split_done_tl     (share_split_done_tl),
    
    // Done signal
    .done_tl                 ( done ),
    .bad_sig_out             ( bad_sig )
);

aes_wrapper_v2 aes_prg_inst (
    .rst        (rst),
    .clk        (clk),
    
    .start                  (start),
    
    .data_mem_dout          (data_mem_dout_1),
    .key_sig_mem_dout       (key_sig_mem_dout_1),
    
    .i_star                 (i_star_ctrl_2_aes),
    .i_star_valid           (i_star_valid_ctrl_2_aes),
    .key_unpack_done        (key_unpack_done),
    
    .data_mem_addr_aes      (addr_data_mem_1),
    .data_mem_wr_addr_aes   (wr_addr_data_mem),
    .key_sig_mem_addr_aes   (addr_key_sig_mem_1),
    
    .wren_key_sig_mem       (wren_key_sig_mem_1),
    .wren_data_mem          (wren_data_mem_1),
    .finalize_aux           (finalize_aux),
    .next_verify_round      (next_verify_round),
    
    .mem_2_mem_mux_sel      (mem_2_mem_mux_sel_aes),
    .result_slice_aes       (result_slice_aes),
    
    .sample                 (sample_aes),
    .sample_valid_aes       (sample_valid_aes),
    .commit_valid_aes       (commit_valid_aes),
    .rst_v_regs             (rst_v_regs),
    .party_is_i_star_aes    (party_is_i_star_aes),
    .phi_i_out              (phi_i_aes_2_base),
    .mpc_round_aes_out      (mpc_round_aes_2_base),
    .share_split_done_out   (share_split_done_aes),
    .a_mid_valid_to_a_wrap  (a_mid_valid_to_a_wrap),
    .init_acc_round_aes_pip (init_acc_round_aes_pip),
    
    .accept_path            (accept_path_aes),
    .reject_path            (reject_path_aes),
    .done_to_ctrl           (aes_is_done),
//    .aes_waiting_for_i_star ( aes_waiting_for_i_star),
    .party_is_i_star_to_ctrl( party_is_i_star_aes_to_ctrl),
    
    .cmt_fifo_wren          (cmt_fifo_wren),
    .cmt_fifo_addr_wr       (cmt_fifo_addr_wr)
);

//mem_dual #( // data memory instance (True Dual Port)
//    .WIDTH  (`WORD_SIZE),
//    .DEPTH  (`DATA_MEM_DEPTH),
//    .FILE   ("data_mem_init.mem")
//) data_mem_inst (
//    .clock      (clk),
//    .data_0     (data_mem_din_0),
//    .data_1     (data_mem_din_1),
//    .address_0  (addr_data_mem_0),
//    .address_1  (addr_data_mem_1), //(addr_out_data_mem_aes),
//    .wren_0     (wren_data_mem_0),
//    .wren_1     (wren_data_mem_1),//(wren_data_mem_1),
//    .q_0        (data_mem_dout_0),
//    .q_1        (data_mem_dout_1) 
//);

//single_port_mem #(
//    .WIDTH  (`WORD_SIZE),
//    .DEPTH  (`DATA_MEM_DEPTH),
//    .FILE   ("data_mem_init.mem")
//) data_mem_inst (
//    .clk    (clk),
//    .wea    (wren_data_mem_1),
//    .addra  (addr_data_mem_1),
//    .dia    (data_mem_din_1),
//    .doa    (data_mem_dout_1)
//);
simple_dual_port_mem #(
    .WIDTH  (`WORD_SIZE),
    .DEPTH  (`DATA_MEM_DEPTH),
    .FILE   ("data_mem_init.mem")
) data_mem_inst (
    .clk    (clk),
    .wea    (wren_data_mem_1),
    .addra  (wr_addr_data_mem),
    .addrb  (addr_data_mem_1),
    .dia    (data_mem_din_1),
    .dob    (data_mem_dout_1)
);

(* DONT_TOUCH = "TRUE" *) mem_dual #( // key/signature memory instance (True Dual Port)
    .WIDTH  (`WORD_SIZE),
    .DEPTH  (`KEY_SIG_MEM_DEPTH),
    .FILE   ("key_sig_mem_init.mem")
) key_sign_mem_inst (
    .clock      (clk),
    .data_0     (key_sig_mem_din_0),
    .data_1     (key_sig_mem_din_1),
    .address_0  (addr_key_sig_mem_0),
    .address_1  (addr_key_sig_mem_1),
    .wren_0     (wren_key_sig_mem_0),
    .wren_1     (wren_key_sig_mem_1), //(wren_key_sig_mem_1),
    .q_0        (key_sig_mem_dout_0),
    .q_1        (key_sig_mem_dout_1) 
);

simple_dual_port_mem_distr #( // Commit fifo: AES writes commits at one side, TL_CTRL reads at the other
    .WIDTH  (`WORD_SIZE),
    .DEPTH  (`CMT_FIFO_DEPTH)
) cmt_fifo (
    .clk    (clk),
    .wea    (cmt_fifo_wren),
    .addra  (cmt_fifo_addr_wr),
    .addrb  (cmt_fifo_addr_re),
    .dia     (sample_aes),
    .dob     (cmt_fifo_dout)
);


keccak_wrapper keccak_inst (
    .rst              ( rst ),
    .clk              ( clk ),

    // Inputs
    .k2c_shift_bytes  ( k2c_shift_bytes ),
    .c2k_din_valid    ( c2k_din_valid ),
    .bytelen_din      ( keccak_bytelen_din ),
    .c2k_din          ( keccak_din ),
    .com_in           ( keccak_command ),

    // Outputs
    .k2c_dout         ( keccak_dout ),
    .k2c_dout_valid   ( k2c_dout_valid ),
    .c2k_din_ready    ( c2k_din_ready ),
    .data_done        ( keccak_done ),
    .rst_y_mul        ( rst_y_mul ),
    .input_done  ( keccak_input_data_done )
);

base_mem_wrapper base_mem_wrapper_inst (
    .rst                    (rst),
    .clk                    (clk),

    .next_E_ff_mu           (next_E_ff_mu),   // input

    .share_split_done_base  (share_split_done_tl),
    .init_acc_round_aes_pip (init_acc_round_aes_pip),
    .mpc_round_base_wrap_in (mpc_round_aes_2_base),
    .phi_i_in               (phi_i_aes_2_base),
    .acc_sample_valid       (sample_valid_aes),
    .acc_sample             (sample_aes),
    .i_star_valid_in        ( i_star_valid_ctrl_2_aes ),
    .i_star_elem_in         ( i_star_ctrl_2_aes ),

    .E_mul_opc_base         (E_mul_opc_wrap),
    .E_mul_res_valid        (E_mul_res_valid),
    .to_ff_wrap_start_MPC   (base_to_ff_wrap_start_MPC),

    .S_base_elem_out        (S_base_elem),   
    .C_base_elem_out        (C_base_elem) 
);

a_mem_wrapper a_mem_wrapper_inst (
    .rst                ( rst ),
    .clk                ( clk ),
    
    // Inputs
    .mpc_round_V_in     ( mpc_round_aes_2_base ),
//    .load_i_star        ( 1'b0 ),
//    .i_star             ( 8'h0 ),
    .phi_i_in           ( phi_i_aes_2_base ),
    .gamma              ( keccak_dout[7:0] ),
    .gamma_valid        ( k2c_dout_valid ),
    .keccak_done        ( keccak_done ),
    .share_split_done_V ( share_split_done_tl ),
    .acc_sample_valid   ( sample_valid_aes ),
    .acc_sample         ( sample_aes ),
    .store_a_start      ( store_a_start ),
    .rst_v_regs         ( rst_v_regs ),
    .init_acc_round_aes_pip (init_acc_round_aes_pip),
    .a_mid_word_valid   (a_mid_valid_to_a_wrap),
    .a_mid_word         (key_sig_mem_dout_1),
    
    // Outputs
    .tmp_dout_base    ( tmp_dout_base ),
    .tmp_dout_mid     ( tmp_dout_mid ),
    .a_word           ( a_word ),
    .a_word_valid     ( a_word_valid ),
    .a_word_is_mid_out( a_word_is_mid ),
    .store_a_done     ( store_a_done )
);

TMP_wrapper TMP_wrapper_inst (
    .rst                    ( rst ),
    .clk                    ( clk ),
    
    // Inputs
    .E_mul_res_valid_in_1   ( E_mul_res_valid ),
    .H_valid                ( k2c_dout_valid ),
    .H_elements             ( keccak_dout[7:0] ),
    .E_base_elem_in         ( E_b_elem ),
    .E_mid_elem_in          ( E_m_elem ),
    .H_done                 ( keccak_done ),
    .i_star_valid_in        ( i_star_valid_ctrl_2_aes ),
    .i_star_elem_in         ( i_star_ctrl_2_aes ),
    .y_word_valid           ( y_word_valid_to_tmp ),
    .y_word                 ( key_sig_mem_dout_0 ),
    
    // Outputs
    .next_E_ff_mu           ( next_E_ff_mu ),
    .next_y_word            ( next_y_word_tmp_to_tl ),
    .tmp_prefill_done       ( tmp_prefill_done ),
    .tmp_dout_base          ( tmp_dout_base ),
    .tmp_dout_mid           ( tmp_dout_mid )
);

mirath_ff_math_wrap u_mirath_ff_math_wrap (
    .rst                     ( rst ),
    .clk                     ( clk ),
    
    .next                    ( next_E ),
    .next_from_TMP           ( next_E_ff_mu ),

    .start                   ( start ),
    .start_MPC               ( base_to_ff_wrap_start_MPC ),

    .keccak_dout_valid       ( k2c_dout_valid ),
    .keccak_dout_byte        ( keccak_dout[7:0] ),

    .E_byte_to_y_acc         ( E_byte ),
    .E_byte_valid_to_y_acc   ( E_byte_valid ),
    
    .S_elem_out              ( S_elem ),
    .C_elem_out              ( C_elem )
);

E_mul E_mul_inst (
    .rst            ( rst ),
    .clk            ( clk ),

    .S_elem         ( S_elem ),
    .C_elem         ( C_elem ),
    .S_b_elem       ( S_base_elem ),
    .C_b_elem       ( C_base_elem ),

    .E_mul_opc_in   ( E_mul_opc_wrap ),

    .E_b_elem       ( E_b_elem ),
    .E_m_elem       ( E_m_elem )
);

y_acc y_acc_inst (
    .clk                ( clk ),
    .rst                ( rst ),
    
    // Inputs
    .keccak_dout        ( keccak_dout ),
    .keccak_dout_valid  ( k2c_dout_valid ),
    .E_byte             ( E_byte ),
    .E_byte_valid       ( E_byte_valid ),
    .keccak_done        ( keccak_done ),
    .squeeze_y          ( squeeze_y ),
    .rst_y_mul          ( rst_y_mul ),
    
    // Outputs
    .init_load_done      ( acc_y_init_done ),
    .next_E              ( next_E ),
    .y_out               ( y_out )
);

endmodule
