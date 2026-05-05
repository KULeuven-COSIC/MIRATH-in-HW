/*
 * keccak_interc_defs.vh
 * -----------
 * This header file declares some parameters specific to the SHA3 / Keccak
 * interface modules.
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
`include "mirath_hw_params.vh"

`ifndef K_INTERC_DEFS
`define K_INTERC_DEFS

`define K_FIFO_ROUND_CTR_MAX 'h3F

`define K_FIFO_DEPTH      512
`define K_FIFO_ADDR_BITS  `CLOG2(`K_FIFO_DEPTH)


`endif