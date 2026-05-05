/*
 * keccak_datapath_64.v
 * ----------------------
 * This file implements the 64-bit Keccak datapath.
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
`include "keccak_pkg_64.v"

module keccak_datapath_64(
    input wire                           rst_keccak,
    input wire                           clk,
    input wire                           computation_en,
    input wire [`ROUND_COUNT_WIDTH-1:0]  round,
//    input wire                           absorb_en,
//    input wire                           squeeze_en,
    input wire                           state_update_en,
    input wire [`WIN-1:0]                absorb_in,
    input wire                           shake_mode,
    
    output wire [`WOUT-1:0]              squeeze_out
);

localparam ROUND_COUNT_WIDTH = `ROUND_COUNT_WIDTH;

localparam SHA3_LANES  = `LANES_SHA3;
localparam SHAKE_LANES = `LANES_SHAKE;
localparam SHA3_RATE  =  `RATE_SHA3;
localparam SHAKE_RATE = `RATE_SHAKE;

reg  [1599:0] state_regs;
wire [1599:0] state_regs_next;
wire [63:0]   rcout;
wire [63:0] state_sliced [24:0];
genvar j;
generate
    for(j=0; j<25; j=j+1) begin: debug_wires
        assign state_sliced[j] = state_regs[j*64+:64];
    end
endgenerate
    
assign squeeze_out = state_regs[`WOUT-1:0];
    
permute_64 permute(
    .din(state_regs), // lane-major
    .rcin(rcout),
    .dout(state_regs_next) //also lane-major
);
    
    rc_64 round_constants (
    .clk (clk),
    .round (round),
    .rcout (rcout)
);

always @ (posedge clk) begin
    if(rst_keccak)
        state_regs <= 'h0;
            
    else if (state_update_en) begin
        if (shake_mode) begin
            if (computation_en)
                state_regs <= state_regs_next;
            else //if (absorb_en)
                state_regs[`RATE_SHAKE-1:0] <= {state_regs[`WOUT-1:0] ^ absorb_in, state_regs[`RATE_SHAKE-1:`WOUT]};
//            else // squeezing
//                state_regs[`RATE_SHAKE-1:0] <= { state_regs[`WOUT-1:0]            , state_regs[`RATE_SHAKE-1:`WOUT]};
        end else begin
            if (computation_en)
                state_regs <= state_regs_next;
            else //if (absorb_en)
                state_regs[`RATE_SHA3-1:0] <= {state_regs[`WOUT-1:0] ^ absorb_in, state_regs[`RATE_SHA3-1:`WOUT]};
//            else // squeezing
//                state_regs[`RATE_SHA3-1:0] <= { state_regs[`WOUT-1:0]            , state_regs[`RATE_SHA3-1:`WOUT]};
        end            
    end           
end
    
endmodule
    
//always @ (posedge clk) begin // state_update_en
//    if(rst_keccak)
//        state_regs <= 1'b0;
            
//    else if(absorb_en) begin
//            if (shake_mode)
//                state_regs[`RATE_SHAKE-1:0] <= { state_regs[`WOUT-1:0] ^ absorb_in, state_regs[`RATE_SHAKE-1:`WOUT]};
//            else
//                state_regs[`RATE_SHA3-1:0] <= { state_regs[`WOUT-1:0] ^ absorb_in, state_regs[`RATE_SHA3-1:`WOUT]};
                
//    end else if(computation_en)
//                state_regs <= state_regs_next;
            
//    else if(squeeze_en) begin
//        if (shake_mode)
//            state_regs[`RATE_SHAKE-1:0] <= { state_regs[`WOUT-1:0], state_regs[`RATE_SHAKE-1:`WOUT]};
//        else
//            state_regs[`RATE_SHA3-1:0] <= { state_regs[`WOUT-1:0], state_regs[`RATE_SHA3-1:`WOUT]};
//    end             
//end
    
