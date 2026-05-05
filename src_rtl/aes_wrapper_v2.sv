/*
 * aes_wrapper_v2.sv
 * --------------
 * This file instantiates the GGM Tree FSM (aes_FSM_v2) and the high-throughput
 * dual AES engine, along with additional wiring and pipeline stages used to
 * interface them to the rest of the design.
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
 
`default_nettype none

`include "aes_parameters.vh"

//typedef enum logic [1:0] { MODE_GGM, MODE_EXP_LEAF, MODE_COMMIT } aes_mode_t;

 // Note: L1 specific
localparam NUM_EXPANDS = `NUM_EXPANDS;
localparam CTR_BITS    = `CLOG2(NUM_EXPANDS)-1; // Left result (res0): always 0 LSB. Right result (res1): always 1 LSB.

module aes_wrapper_v2(
    input wire          rst,
    input wire          clk,
    
    input wire [1:0]    start,
    input wire [`WORD_SIZE-1:0]    data_mem_dout,
    input wire [`WORD_SIZE-1:0]    key_sig_mem_dout,
    
    input wire [7:0]               i_star,
    input wire                     i_star_valid,  
    
    input wire                     key_unpack_done,    

//    output wire [`DATA_MEM_ADDR_BITS-1:0] address_out,
//    output reg [`DATA_MEM_ADDR_BITS-1:0] addr_out_data_mem_aes_pip,
    output wire [`DATA_MEM_ADDR_BITS-1:0] data_mem_addr_aes,
    output wire [`DATA_MEM_ADDR_BITS-1:0] data_mem_wr_addr_aes,
//    output reg [`DATA_MEM_ADDR_BITS-1:0] addr_out_key_sig_mem_aes_pip,
    output wire [`KEY_SIG_MEM_ADDR_BITS-1:0] key_sig_mem_addr_aes,
    
    output wire wren_key_sig_mem,
    output wire wren_data_mem,
    output wire finalize_aux,
    output wire next_verify_round,
    
    output wire mem_2_mem_mux_sel,
    output wire [`WORD_SIZE-1:0] result_slice_aes, // has to be endian-reversed before sent out
    
    output wire [`CMT_FIFO_ADDR_BITS-1:0] cmt_fifo_addr_wr,
    output wire cmt_fifo_wren,
    
    output reg [`WORD_SIZE-1:0] sample,
    output reg                  sample_valid_aes,
    output reg                  share_split_done_out,
    output reg                  commit_valid_aes,
    output reg                  init_acc_round_aes_pip,
    output reg                  party_is_i_star_aes,
    output wire                 party_is_i_star_to_ctrl,
    output wire                 rst_v_regs,
    output reg [$clog2(`N)-1:0] phi_i_out,
    output reg [$clog2(`TAU)-1:0] mpc_round_aes_out,
    
    output wire done_to_ctrl,
    output wire accept_path,
    output wire reject_path,
//    output wire aes_waiting_for_i_star,
    output wire a_mid_valid_to_a_wrap
);

wire [$clog2(`N)-1:0] phi_i;
wire [$clog2(`TAU)-1:0] mpc_round_aes;
always @ (posedge clk) phi_i_out         <= phi_i;
always @ (posedge clk) mpc_round_aes_out <= mpc_round_aes;

wire init_acc_round_aes;
always_ff @ (posedge clk) init_acc_round_aes_pip <= rst ? 1'b1 : init_acc_round_aes;

wire share_split_done;
always_ff @ (posedge clk) share_split_done_out <= share_split_done;

wire sample_valid, sample_source, commit_valid, party_is_i_star_out;

genvar i;
wire [`WORD_SIZE-1:0] data_mem_dout_byte_rev;
wire [`WORD_SIZE-1:0] key_sig_mem_dout_byte_rev;

generate
  for (i = 0; i < `WORD_SIZE/8; i = i + 1) begin : BYTE_REV
    assign data_mem_dout_byte_rev[(i*8)+:8] = data_mem_dout[`WORD_SIZE - (i+1)*8 +: 8];
    assign key_sig_mem_dout_byte_rev[(i*8)+:8] = key_sig_mem_dout[`WORD_SIZE - (i+1)*8 +: 8];
  end
endgenerate

aes_mode_t mode_aes;

wire [`CLOG2(`TREE_NODES)-1:0] node_index_aes;
wire load_salt, load_seed_from_mem, read_source;
wire aes_comm_init, aes_comm_next;
wire aes_core_res_valid, load_key_from_res_0, load_key_from_res_1, store_res;
wire aes_core_ready_in_5;

assign mem_2_mem_mux_sel = read_source;

wire [CTR_BITS-1:0] ctr;

// **********************************************************
aes_FSM_v2 #( // FSM controller
    .NUM_EXPANDS    (NUM_EXPANDS)
) aes_FSM_inst (
    .rst                    (rst),
    .clk                    (clk),

    .start                  (start),       // [1:0]

    .aes_core_ready_in_5    (aes_core_ready_in_5),

    .i_star_in              (i_star),       // [7:0]
    .i_star_valid           (i_star_valid),
    .key_unpack_done        (key_unpack_done),

    .data_mem_addr       (data_mem_addr_aes),       // [`DATA_MEM_ADDR_BITS-1:0]
    .data_mem_wr_addr    (data_mem_wr_addr_aes),       // [`DATA_MEM_ADDR_BITS-1:0]
    .key_sig_mem_addr    (key_sig_mem_addr_aes),       // [`KEY_SIGN_MEM_ADDR_BITS-1:0]

    .node_index_aes     (node_index_aes),       // [`CLOG2(TREE_LEAVES)-1:0]

    .wren_data_mem          (wren_data_mem),
    .wren_key_sig_mem       (wren_key_sig_mem),
    .read_source            (read_source),

    .ctr                    (ctr),
    .load_salt              (load_salt),
    .load_seed              (load_seed_from_mem),
    .mode_aes               (mode_aes),       // aes_mode_t
    .aes_comm_init          (aes_comm_init),
    .aes_comm_next          (aes_comm_next),
//    .rst_sample_regs        (rst_sample_regs),

    .load_key_from_res_0    (load_key_from_res_0),
    .load_key_from_res_1    (load_key_from_res_1),
    .store_res              (store_res),
    .finalize_aux           (finalize_aux),
    .next_verify_round      (next_verify_round),
    .init_acc_round_aes     (init_acc_round_aes),
    .share_split_done_out   (share_split_done),
    .a_mid_valid_to_a_wrap  (a_mid_valid_to_a_wrap),
    
    .party_is_i_star_out    (party_is_i_star_out),
    .commit_valid           (commit_valid),
    .rst_v_regs             (rst_v_regs),
    .sample_valid           (sample_valid),
    .sample_source          (sample_source),
    .phi_i                  (phi_i),
    .mpc_round_aes          (mpc_round_aes),
    .done_to_ctrl           (done_to_ctrl),
    .accept_path            (accept_path),
    .reject_path            (reject_path),
//    .waiting_for_i_star     (aes_waiting_for_i_star),
    .party_is_i_star_to_ctrl(party_is_i_star_to_ctrl),
    
    .cmt_fifo_addr_wr       (cmt_fifo_addr_wr),
    .cmt_fifo_wren          (cmt_fifo_wren)
);


reg [`NODE_SIZE-1:0] salt_reg_aes; // Todo: fix
always @ (posedge clk) begin // always loads from key_sig_mem_dout_byte_rev
    if (load_salt) begin
        salt_reg_aes [`WORD_SIZE-1:0] <= key_sig_mem_dout_byte_rev; // load first bits
        salt_reg_aes [`WORD_SIZE +: (`NODE_SIZE-`WORD_SIZE)] <= salt_reg_aes [0 +: (`NODE_SIZE-`WORD_SIZE)]; // shift up
    end
end

reg [`NODE_SIZE-1:0] input_seed;
always @ (posedge clk) begin
    if (load_key_from_res_0)
        input_seed <= res_aes_0;
    else if (load_key_from_res_1)
        input_seed <= res_aes_1;
    else if (load_seed_from_mem) begin
        input_seed [`WORD_SIZE +: (`NODE_SIZE-`WORD_SIZE)] <= input_seed [0 +: (`NODE_SIZE-`WORD_SIZE)]; // shift up
        if (read_source) begin
            input_seed [`WORD_SIZE-1:0] <= key_sig_mem_dout_byte_rev; // load first bits from key/sign mem
        end else begin
            input_seed [`WORD_SIZE-1:0] <= data_mem_dout_byte_rev; // load first bits from data mem
        end
    end
end

localparam ZEROS_TO_PAD = ('d32 - `CLOG2(`TREE_NODES)); // indices are 32 bits in the software version
wire [31:0] node_index_aes_zero_padded = {{ZEROS_TO_PAD{1'b0}}, node_index_aes};
wire [31:0] node_index_aes_zero_padded_rev;
generate
    for (i = 0; i < 4; i = i + 1) begin : BYTE_REVERSE
        assign node_index_aes_zero_padded_rev[i*8 +: 8] = node_index_aes_zero_padded[(3 - i)*8 +: 8];
    end
endgenerate

// Process input block by XORing with the proper mask
reg [255:0] block_0;// = salt_reg_aes ^ (mode_aes ? {ctr, 1'b0, 120'b0}  : {8'b0, node_index_aes_zero_padded_rev, (`DOMAIN_SEPARATOR_PRG), {'d80{1'b0}}}); // 128 - (8+32+8)
reg [255:0] block_1;// = salt_reg_aes ^ (mode_aes ? {ctr, 1'b1, 120'b0}  : {8'b1, node_index_aes_zero_padded_rev, (`DOMAIN_SEPARATOR_PRG), {'d80{1'b0}}});

// Note: Could indeed try to pipeline if it helps raise fmax
always_comb begin
    case (mode_aes)
        MODE_GGM: begin
            block_0 = salt_reg_aes ^ {8'b0, node_index_aes_zero_padded_rev, (`DOMAIN_SEPARATOR_PRG), {'d208{1'b0}}};
            block_1 = salt_reg_aes ^ {8'b1, node_index_aes_zero_padded_rev, (`DOMAIN_SEPARATOR_PRG), {'d208{1'b0}}};
        end
       
        MODE_EXP_LEAF: begin
            block_0 = salt_reg_aes ^ {ctr, 1'b0, 248'b0};
            block_1 = salt_reg_aes ^ {ctr, 1'b1, 248'b0};
        end
       
        MODE_COMMIT: begin
            block_0 = salt_reg_aes ^ {8'b0, node_index_aes_zero_padded_rev, (`DOMAIN_SEPARATOR_CMT), {'d208{1'b0}}};
            block_1 = salt_reg_aes ^ {8'b1, node_index_aes_zero_padded_rev, (`DOMAIN_SEPARATOR_CMT), {'d208{1'b0}}};
        end
    endcase
end

wire [255:0] res_aes_0, res_aes_1;

rijndael256_core_dual_fast rijndael256_core_dual_inst ( // AES / Rijndael datapath instance
    .reset_n    (~rst),
    .clk        (clk),
    
    .init       (aes_comm_init),
    .next       (aes_comm_next),
    
    .key        (input_seed),
    
    .block_0    (block_0),
    .block_1    (block_1),
    
    .result_0   (res_aes_0),
    .result_1   (res_aes_1),
    
    .result_valid   (aes_core_res_valid),
    .ready_in_5     (aes_core_ready_in_5)
);

// Grab result & send it outside word-by-word. Todo: Investigate if using different shifts when expanding
// leaf can allow to reduce the size of some arithmetic modules.
reg [`NODE_SIZE*2-1:0] result_reg_aes;
always @ (posedge clk) begin
    if (store_res)
//    if (aes_core_ready)
        result_reg_aes <= {res_aes_0, res_aes_1};
    else
        result_reg_aes <= (result_reg_aes<<`WORD_SIZE);
end

// the below localparam is for getting the 64 MSBs of the result regs
localparam BYTES_SHIFT = (`NODE_SIZE*2)/8;
generate
    for (i = 0; i < 8; i = i + 1) begin : BYTE_REV_2
        assign result_slice_aes[i*8 +: 8] = result_reg_aes[(BYTES_SHIFT-1-i)*8 +: 8];
    end
endgenerate

// Send sample: 
always_ff @ (posedge clk) begin
    sample <= sample_source   ? key_sig_mem_dout : result_slice_aes;
    
    if (rst)
        sample_valid_aes <= 1'b0;
    else
        sample_valid_aes <= sample_valid;

    party_is_i_star_aes <= party_is_i_star_out;
    commit_valid_aes    <= commit_valid;
end

endmodule