/*
 * keccak_to_control_router.v
 * ------------
 * This file contains a module used to route data from the SHA3 / Keccak
 * engine to the contoller FSM.
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

`default_nettype none

module keccak_to_control_router #(
    parameter KECCAK_WIN = 64,
    parameter CLOG2_WIN = `CLOG2(KECCAK_WIN) // should be 6
)(
    input wire rst,
    input wire clk,
    
    input wire din_valid,
    input wire keccak_squeeze_done, 
    input wire [KECCAK_WIN-1:0] din,
    
    // input wire dout_ready, // Consider unconditionally ready receiver for now
    
    input wire shift_bytes, // 1: just send out bytes
    
    output reg din_ready_next, // Keccak needs it one cycle earlier
    
    output reg dout_valid,
    output reg [KECCAK_WIN-1:0] dout,
    output reg keccak_data_done
);
    
    reg [CLOG2_WIN-3:0] valid_count; // In bytes
    
    reg din_ready;
    always @ (posedge clk) begin
        din_ready <= din_ready_next;
    end    
    
    wire dout_ready = 1'b1; // No place in the rest of the design where we need to stall. Can be hard-coded valid and ignired for now.
    
    always @* begin
        if (shift_bytes) // If we're running out of bytes (2 bytes or less left, update next cycle)
            din_ready_next = (valid_count=='h2) // Unconditionally raise ready if valid_count is 2
                           || (valid_count<'h2 && !din_valid); // Raise ready if valid_count is <2 and we do not have a handshake now
        else
            din_ready_next = 1'b1;
    end
    
    reg last_data;
    always @ (posedge clk) begin
        if (rst) begin
            last_data <= 1'b0;
        end else if (keccak_squeeze_done) begin
            last_data <= 1'b1;
        end else if (~dout_valid) begin
            last_data <= 1'b0;
        end
    end
    
    always @ (posedge clk) begin
       if ((valid_count<=1 || !shift_bytes))
            keccak_data_done <= last_data;
        else
            keccak_data_done <= 1'b0;
    end
    
    always @ (posedge clk) begin
        if (rst) begin // reset
            dout_valid <= 1'b0;
            dout <= 'h0;
            
        end else if (shift_bytes) begin
                if (valid_count <= 1'b1) begin
                    dout_valid <= din_valid;
                end
                
                if (din_valid && din_ready)
                    dout <= din;
                else
                    dout <= (dout >> 'h8);
        end else begin
                dout_valid <= din_valid;
                dout <= din;
        end
    end
    
    always @ (posedge clk) begin
        if (rst)
            valid_count <= 1'b0;
        else if (din_valid && din_ready)
            valid_count <= 'h8;
        else if (valid_count!=0)
            valid_count <= valid_count - 'h1;
    end
    
endmodule
