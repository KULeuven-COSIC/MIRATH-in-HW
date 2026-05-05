/*
 * pipeline_delay.sv
 * -----------
 * This file is used to quickly generate a shift register array of
 * parametrized width and depth.
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

module pipeline_delay #(
    parameter int WIDTH = 8,
    parameter int DEPTH = 1       // DEPTH = 1 => one cycle of delay
)(
    input  logic                 clk,
    input  logic [WIDTH-1:0]     din,
    output logic [WIDTH-1:0]     dout
);

    // Pipeline storage
    logic [WIDTH-1:0] stage [DEPTH-1:0];

    genvar i;
    generate
        for (i = 0; i < DEPTH; i++) begin : gen_pipe
            if (i == 0) begin
                // First stage takes the input
                always_ff @(posedge clk) begin
                    stage[0] <= din;
                end
            end else begin
                // Each stage takes the previous stage output
                always_ff @(posedge clk) begin
                    stage[i] <= stage[i-1];
                end
            end
        end
    endgenerate

    // Output of the last stage
    assign dout = stage[DEPTH-1];

endmodule
