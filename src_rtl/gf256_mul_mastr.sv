/*
 * gf256_mul_mastr.sv
 * ------------
 * This file is a gate-level Mastrovito GF(2^8) multiplier with
 * reduction modulo 0x11B.
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

module gf256_mul_mast (
    input wire  [7:0] a,  // operand 0
    input wire  [7:0] b,  // operand 1
    output wire [7:0] p   // product (mod 0x11B)
);

    // Partial product matrix (a_i AND b_j)
    wire [7:0] pp [7:0];

    genvar i;
    generate
        for (i = 0; i < 8; i = i + 1) begin : gen_pp
            assign pp[i][0] = a[i] & b[0];
            assign pp[i][1] = a[i] & b[1];
            assign pp[i][2] = a[i] & b[2];
            assign pp[i][3] = a[i] & b[3];
            assign pp[i][4] = a[i] & b[4];
            assign pp[i][5] = a[i] & b[5];
            assign pp[i][6] = a[i] & b[6];
            assign pp[i][7] = a[i] & b[7];
        end
    endgenerate

    // Pre-reduction product (up to x^14), collect sums
    wire [14:0] prod;

    assign prod[0]  = pp[0][0];

    assign prod[1]  = pp[0][1] ^ pp[1][0];

    assign prod[2]  = pp[0][2] ^ pp[1][1] ^ pp[2][0];

    assign prod[3]  = pp[0][3] ^ pp[1][2] ^ pp[2][1] ^ pp[3][0];

    assign prod[4]  = pp[0][4] ^ pp[1][3] ^ pp[2][2] ^ pp[3][1] ^ pp[4][0];

    assign prod[5]  = pp[0][5] ^ pp[1][4] ^ pp[2][3] ^ pp[3][2] ^ pp[4][1] ^ pp[5][0];

    assign prod[6]  = pp[0][6] ^ pp[1][5] ^ pp[2][4] ^ pp[3][3] ^ pp[4][2] ^ pp[5][1] ^ pp[6][0];

    assign prod[7]  = pp[0][7] ^ pp[1][6] ^ pp[2][5] ^ pp[3][4] ^ pp[4][3] ^ pp[5][2] ^ pp[6][1] ^ pp[7][0];

    assign prod[8]  = pp[1][7] ^ pp[2][6] ^ pp[3][5] ^ pp[4][4] ^ pp[5][3] ^ pp[6][2] ^ pp[7][1];

    assign prod[9]  = pp[2][7] ^ pp[3][6] ^ pp[4][5] ^ pp[5][4] ^ pp[6][3] ^ pp[7][2];

    assign prod[10] = pp[3][7] ^ pp[4][6] ^ pp[5][5] ^ pp[6][4] ^ pp[7][3];

    assign prod[11] = pp[4][7] ^ pp[5][6] ^ pp[6][5] ^ pp[7][4];

    assign prod[12] = pp[5][7] ^ pp[6][6] ^ pp[7][5];

    assign prod[13] = pp[6][7] ^ pp[7][6];

    assign prod[14] = pp[7][7];


    //Modular reduction by x^8 + x^4 + x^3 + x + 1
    wire r0, r1, r2, r3, r4, r5, r6, r7;
    
    assign r0 = prod[0]  ^ prod[8]  ^ prod[12] ^ prod[13];
    assign r1 = prod[1]  ^ prod[8]  ^ prod[9]  ^ prod[12] ^ prod[14];
    assign r2 = prod[2]  ^ prod[9]  ^ prod[10] ^ prod[13];
    assign r3 = prod[3]  ^ prod[8]  ^ prod[10] ^ prod[11] ^ prod[12] ^ prod[13] ^ prod[14];
    assign r4 = prod[4]  ^ prod[8]  ^ prod[9]  ^ prod[11] ^ prod[14];
    assign r5 = prod[5]  ^ prod[9]  ^ prod[10] ^ prod[12];
    assign r6 = prod[6]  ^ prod[10] ^ prod[11] ^ prod[13];
    assign r7 = prod[7]  ^ prod[11] ^ prod[12] ^ prod[14];

    assign p = {r7, r6, r5, r4, r3, r2, r1, r0};

endmodule

