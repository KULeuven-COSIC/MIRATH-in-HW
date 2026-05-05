/*
 * gf256_squaring_mul.sv
 * -----------
 * This file is a combinational GF(2^8) squarer with reduction modulo
 * x^8 + x^4 + x^3 + x + 1.
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
 
 module gf256_squaring_mul(
    input  wire [7:0] a,  // single operand to be squared
    output wire [7:0] p   // product (mod 0x11B)
);
    
    assign p[0] = a[0]  ^ a[4]  ^ a[6];
    assign p[1] = a[4]  ^ a[6] ^ a[7];
    assign p[2] = a[1]  ^ a[5];
    assign p[3] = a[4]  ^ a[5]  ^ a[6] ^ a[7];
    assign p[4] = a[2]  ^ a[4]  ^ a[7];
    assign p[5] = a[5]  ^ a[6];
    assign p[6] = a[3]  ^ a[5];
    assign p[7] = a[6]  ^ a[7];
endmodule
