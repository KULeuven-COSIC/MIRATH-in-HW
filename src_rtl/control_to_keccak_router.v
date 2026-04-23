/*
 * control_to_keccak_router.v
 * -----------
 * This file contains a module used to route data from the top-level
 * contoller FSM to the SHA3 / Keccak engine. A Simple Dual-Port memory
 * is used as a FIFO, decoupling the absorption of incoming data from the
 * internal state of the SHA3 / Keccak engine.
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

`include "clog2.v"
`include "keccak_interc_defs.vh"

`default_nettype none

module control_to_keccak_router #(
    parameter KECCAK_WIN = 64,
    parameter KECCAK_WIN_BYTES = (KECCAK_WIN>>3),
    parameter CLOG2_WIN = `CLOG2(KECCAK_WIN), // should be 6
    parameter CLOG2_WIN_BYTES = `CLOG2(KECCAK_WIN_BYTES), // should be 3
    parameter CAPACITY_BYTES = 15
)(
    input wire rst,
    input wire clk,
    
    input wire c2k_din_valid,
    input wire consume_buffer,
    input wire [CLOG2_WIN_BYTES:0] bytelen_din, // Can go up to 8 -> Make sure its zero when c2k_din_valid==1'b0
    input wire [KECCAK_WIN-1:0] c2k_din,
    input wire [KECCAK_WIN-1:0] com_in,
    input wire squeeze_done,
    
//    output wire c2k_din_ready, // Can remove it, since there is no possibility of stalling if the FIFO is large enoguh (which it will be)
    output reg  c2k_dout_valid, // Used as FIFO_wren_now
    output wire [KECCAK_WIN-1:0] dout,
//    output reg [KECCAK_WIN-1:0] dout,
    output reg  [KECCAK_WIN-1:0] com_out,
    output reg  input_done,
    output reg  input_round_valid,
    output reg  [`K_FIFO_ADDR_BITS-1:0] k_din_fifo_addr_wr
);  

    reg wait_com;
    reg [`CLOG2(`K_FIFO_ROUND_CTR_MAX+1)-1:0] valid_input_rounds_ctr;
    reg [`CLOG2(`RATE_WORDS_SHAKE)-1:0] round_words, round_word_ctr;
    reg [(KECCAK_WIN/2)-5:0] input_bytes;
//    reg input_done;
    
    always @ (posedge clk) begin
        if (wait_com) begin
            round_words     <= com_in[KECCAK_WIN/2-1] ? (`RATE_WORDS_SHAKE-1) : (`RATE_WORDS_SHA3-1);
            k_din_fifo_addr_wr <= 'h0;
            round_word_ctr  <= 'h0;
        end else if (c2k_dout_valid) begin
            k_din_fifo_addr_wr <= k_din_fifo_addr_wr + 1'b1;
            round_word_ctr  <= (round_word_ctr==round_words) ? 'h0 : (round_word_ctr+1'b1);
        end
    end
    
    always @ (posedge clk) begin
        if (rst)
            input_round_valid <= 1'b0;
        else
            input_round_valid <= valid_input_rounds_ctr!=0;
        
        if (wait_com)
            valid_input_rounds_ctr <= 'h0;
        
        else if (consume_buffer) begin
//            if (round_word_ctr!=round_words && !input_done)
//                valid_input_rounds_ctr <= valid_input_rounds_ctr -1'b1;
            if (!((round_word_ctr==round_words && c2k_dout_valid) || input_done))
                valid_input_rounds_ctr <= valid_input_rounds_ctr -1'b1;
            
        end else if ((round_word_ctr==round_words && c2k_dout_valid) || input_done)
            valid_input_rounds_ctr <= valid_input_rounds_ctr +1'b1;
    end
    
    reg [CAPACITY_BYTES*8-1:0] dout_internal;
    reg [`CLOG2(CAPACITY_BYTES):0]   valid_bytes;
    
    wire c2k_dout_valid_next = (((valid_bytes+bytelen_din) >= KECCAK_WIN_BYTES) || (input_bytes==valid_bytes+bytelen_din));
    
    always @ (posedge clk) input_done <= input_done ? 1'b0 : ((input_bytes <= 'h8) && c2k_dout_valid);
    
    // Simple command pipeline
    always @ (posedge clk) begin
        if (c2k_din_valid && wait_com)
            com_out <= com_in;
    end
    
    assign dout = dout_internal[KECCAK_WIN-1:0];
//    always @ (posedge clk) dout <= dout_internal[KECCAK_WIN-1:0];
    
    always @ (posedge clk) begin
        if (rst || input_done) begin
            c2k_dout_valid <= 1'b0;
            valid_bytes <= 1'b0;
            dout_internal <= 1'b0;
            
        end else begin
            c2k_dout_valid <= c2k_dout_valid_next;     
            if (c2k_dout_valid)
                dout_internal <= dout_internal >> KECCAK_WIN; // Shift right one full word
                
            if ((valid_bytes+bytelen_din) >= KECCAK_WIN_BYTES) begin
                valid_bytes <= (bytelen_din + valid_bytes)-KECCAK_WIN_BYTES;
                
                dout_internal[valid_bytes*8+:KECCAK_WIN] <= c2k_din; // Fill starting from the first invalid byte

            end else if (c2k_din_valid) begin
                valid_bytes <= (bytelen_din + valid_bytes);
                dout_internal[valid_bytes*8 +: KECCAK_WIN] <= c2k_din;
            end
        end
        
        if (input_bytes=='h0)
            dout_internal <= 'h0;
    end
    
    always @ (posedge clk) begin // waiting for command flag din[`WIN-2:`HALF_WIN]
        if (squeeze_done || rst)
            wait_com <= 1'b1;
        else if (c2k_din_valid)
            wait_com <= 1'b0;
         
    end
    
    always @ (posedge clk) begin // waiting for command flag din[`WIN-2:`HALF_WIN]
        if (rst || input_done)
            input_bytes <= 'ha;
        else if (c2k_din_valid && wait_com)
            input_bytes <= com_in[KECCAK_WIN-2 : (KECCAK_WIN/2)+3]; // We support full bytes only
        else if (c2k_dout_valid) begin
            if(input_bytes > 'h8)
                input_bytes <= input_bytes - 'h8;
        end
    end
    
endmodule // control_to_keccak_router

//module control_to_keccak_router #(
//    parameter KECCAK_WIN = 64,
//    parameter KECCAK_WIN_BYTES = (KECCAK_WIN>>3),
//    parameter CLOG2_WIN = `CLOG2(KECCAK_WIN), // should be 6
//    parameter CLOG2_WIN_BYTES = `CLOG2(KECCAK_WIN_BYTES), // should be 3
//    parameter CAPACITY_BYTES = 15
//)(
//    input wire rst,
//    input wire clk,
    
//    input wire c2k_din_valid,
//    input wire keccak_din_ready,
//    input wire [CLOG2_WIN_BYTES:0] bytelen_din, // Can go up to 8 -> Make sure its zero when c2k_din_valid==1'b0
//    input wire [KECCAK_WIN-1:0] c2k_din,
//    input wire [KECCAK_WIN-1:0] com_in,
//    input wire squeeze_done,
    
//    output wire c2k_din_ready,
//    output reg  c2k_dout_valid,
//    output wire c2k_dout_valid_next,
//    output wire [KECCAK_WIN-1:0] dout,
//    output reg  [KECCAK_WIN-1:0] com_out,
//    output reg  input_done
//);  

//    reg wait_com;
//    reg [(KECCAK_WIN/2)-5:0] input_bytes;
////    reg input_done;
    
//    reg [CAPACITY_BYTES*8-1:0] dout_internal;
//    reg [`CLOG2(CAPACITY_BYTES):0]   valid_bytes; // 1 byte + 4 bytes + 8 bytes should be enough
    
////    assign c2k_din_ready = ((valid_bytes+bytelen_din) <= (CAPACITY_BYTES)) || keccak_din_ready;
//    assign c2k_din_ready = ((valid_bytes) <= (CAPACITY_BYTES-'h8)) || keccak_din_ready;
    
//    assign c2k_dout_valid_next = ( (c2k_dout_valid && keccak_din_ready)
//                                ? (((valid_bytes+bytelen_din) >= (KECCAK_WIN_BYTES<<1)) || (input_bytes==valid_bytes+bytelen_din-KECCAK_WIN_BYTES))
//                                : ((((valid_bytes+bytelen_din) >= KECCAK_WIN_BYTES)) || (input_bytes==valid_bytes+bytelen_din)) )
//                                ;
    
//    always @ (posedge clk) input_done <= (input_bytes <= 'h8) && c2k_dout_valid && keccak_din_ready;
    
//    // Simple command pipeline
////    reg  [KECCAK_WIN-1:0] com_internal;
//    always @ (posedge clk) begin
//        if (c2k_din_valid && wait_com)
//            com_out <= com_in;
//    end
    
//    assign dout = dout_internal[KECCAK_WIN-1:0];
    
//    always @ (posedge clk) begin
//        if (rst || input_done) begin
//            c2k_dout_valid <= 1'b0;
//            valid_bytes <= 1'b0;
//            dout_internal <= 1'b0;
            
//        end else begin
//            c2k_dout_valid <= c2k_dout_valid_next;     
//            if (c2k_dout_valid && keccak_din_ready) begin
//                dout_internal <= dout_internal >> KECCAK_WIN; // Shift right one full word
                
//                valid_bytes <= (bytelen_din + valid_bytes)-KECCAK_WIN_BYTES;
                
//                dout_internal[(valid_bytes - KECCAK_WIN_BYTES)*8+:KECCAK_WIN] <= c2k_din; // Fill starting from the first current invalid byte
////                c2k_dout_valid <= (((bytelen_din + valid_bytes)) >= (KECCAK_WIN_BYTES<<1)) || (input_bytes==valid_bytes+bytelen_din-KECCAK_WIN_BYTES);
////            end else if (valid_bytes <= (CAPACITY_BYTES-bytelen_din)) begin
//            end else if (c2k_din_valid && (valid_bytes <= (CAPACITY_BYTES-bytelen_din))) begin
//                valid_bytes <= (bytelen_din + valid_bytes);
//                dout_internal[valid_bytes*8 +: KECCAK_WIN] <= c2k_din;
////                c2k_dout_valid <= ((bytelen_din + valid_bytes) >= KECCAK_WIN_BYTES) || (input_bytes==valid_bytes+bytelen_din);
//            end
//        end
        
//        if (input_bytes=='h0)
//            dout_internal <= 'h0;
//    end
    
//    always @ (posedge clk) begin // waiting for command flag din[`WIN-2:`HALF_WIN]
//        if (squeeze_done || rst)
//            wait_com <= 1'b1;
//        else if (c2k_din_valid)
//            wait_com <= 1'b0;
         
//    end
    
//    always @ (posedge clk) begin // waiting for command flag din[`WIN-2:`HALF_WIN]
//        if (rst || input_done)
//            input_bytes <= 'ha;
//        else if (c2k_din_valid && wait_com)
//            input_bytes <= com_in[KECCAK_WIN-2 : (KECCAK_WIN/2)+3] +'h8; // We support full bytes only
//        else if (c2k_dout_valid && keccak_din_ready) begin
//            if(input_bytes > 'h8)
//                input_bytes <= input_bytes - 'h8;
//        end
//    end
    
//endmodule // control_to_keccak_router
