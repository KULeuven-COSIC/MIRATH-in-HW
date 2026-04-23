/*
 * permute_64.v
 * ----------------------
 * This file implements a combinational Keccak-f[1600] round function,
  * including theta, rho, pi, chi, and iota, on a lane-major state.
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

module permute_64 (
    input  wire [`DATAPATH_WIDTH-1:0]     din, // lane-major
    input  wire [`PARALLEL_SLICES-1:0]    rcin,
    input  wire [`PARALLEL_SLICES-1:0]    test,
    output wire [`DATAPATH_WIDTH-1:0]     dout //also lane-major
);

wire [63:0] A_theta [0:24];   // state  A[x+5*y][z]
wire [63:0] C [0:4];    // C[x][z]
wire [63:0] D [0:4];    // D[x][z]

wire [63:0] A_rho [0:24];   // state  A[x+5*y][z]
wire [63:0] A_pi  [0:24];   // state  A[x+5*y][z]
wire [63:0] A_chi [0:24];   // state  A[x+5*y][z]
//wire [63:0] A_iota [0:24];   // state  A[x+5*y][z]
wire [63:0] A_out [0:24];   // state  A[x+5*y][z]


genvar x,y, z;
generate
    //------------------------------------------------------------------
    // 1) unpack the 1600-bit vector into 25 lanes
    //------------------------------------------------------------------
    for(x=0; x<25; x=x+1) begin: unpack_din// lanes
           assign A_theta[x] = din[64*x+:64];
    end 
    
    //------------------------------------------------------------------
    // 2) C[x] = ⨁y A[x,y]
    //------------------------------------------------------------------   
    for(x=0; x<5; x=x+1) begin: get_C_x
        assign C[x] = A_theta[x]^A_theta[x+5]^A_theta[x+10]^A_theta[x+15]^A_theta[x+20];
    end

        
     //------------------------------------------------------------------
     // 3) D[x,z] = C[(x-1)mod5,z] ⨁ C[(x+1)mod5,(z-1)mod64]
     //------------------------------------------------------------------
//    for(x=0; x<5; x=x+1) begin: get_D_x
//        assign D[x] = C[`MOD5(x-1)] ^ {C[`MOD5(x+1)][62:0], C[`MOD5(x+1)][63]}; //[`MOD64(z-1)]
//    end
     
//     //------------------------------------------------------------------
//     // 4) A'[x,y] = A[x,y] ⨁ D[x]
//     //------------------------------------------------------------------
//    for(x=0; x<5; x=x+1) begin: apply_theta_x
//        assign A_rho[x   ] = A_theta[x   ]^D[x];
//        assign A_rho[x+5 ] = A_theta[x+5 ]^D[x];
//        assign A_rho[x+10] = A_theta[x+10]^D[x];
//        assign A_rho[x+15] = A_theta[x+15]^D[x];
//        assign A_rho[x+20] = A_theta[x+20]^D[x];
//    end
    
    //-----------------------------------------------
    // Combine 3 & 4 for sim acceleration
    //-----------------------------------------------
    for(x=0; x<5; x=x+1) begin
        assign A_rho[x   ] = A_theta[x   ]^C[`MOD5(x-1)] ^ {C[`MOD5(x+1)][62:0], C[`MOD5(x+1)][63]};
        assign A_rho[x+5 ] = A_theta[x+5 ]^C[`MOD5(x-1)] ^ {C[`MOD5(x+1)][62:0], C[`MOD5(x+1)][63]};
        assign A_rho[x+10] = A_theta[x+10]^C[`MOD5(x-1)] ^ {C[`MOD5(x+1)][62:0], C[`MOD5(x+1)][63]};
        assign A_rho[x+15] = A_theta[x+15]^C[`MOD5(x-1)] ^ {C[`MOD5(x+1)][62:0], C[`MOD5(x+1)][63]};
        assign A_rho[x+20] = A_theta[x+20]^C[`MOD5(x-1)] ^ {C[`MOD5(x+1)][62:0], C[`MOD5(x+1)][63]};
    end
    
    //------------------------------------------------------------------
    // 5) A'[x+5*y][z] = A[x+5*y][`MOD64(z+`INVERSE_RHO(x+5*y))]
    //------------------------------------------------------------------
//    for(x=0; x<25; x=x+1) begin: apply_rho_x
//        for(z=0; z<64; z=z+1) begin: apply_rho_z
//            assign A_pi[x][z] = A_rho[x][`MOD64(z+`INVERSE_RHO(x))];
//        end
//    end
//    for (x = 0; x < 25; x = x + 1) begin: apply_rho // faster sim version
//        localparam integer R = `INVERSE_RHO(x);
//        assign A_pi[x] = (R == 0) ? A_rho[x] : {A_rho[x][R-1:0], A_rho[x][63:R]};
//    end
    
    //------------------------------------------------------------------
    // 6) A'[x+5*y] = A[`MOD64(`INVERSE_PI(x+5*y)]
    //------------------------------------------------------------------
//    for(x=0; x<25; x=x+1) begin: apply_pi
//        assign A_chi[x] = A_pi[`INVERSE_PI(x)];
//    end
    
    //--------------------------------------------
    // Combine 5 and 6: 
    //--------------------------------------------
    for (x = 0; x < 25; x = x + 1) begin: apply_rho_pi
        localparam integer R = `INVERSE_RHO(x);
        assign A_chi[`PI(x)] = (R == 0) ? A_rho[x] : {A_rho[x][R-1:0], A_rho[x][63:R]};
    end
    
    
    //------------------------------------------------------------------
    // 7) A'[x+5*y] = A[x+5*y]  ⨁ (~A[`MOD5(x+1)+5*y] & A[`MOD5(x+2)+5*y]); + round constants
    //------------------------------------------------------------------
//    for (y = 0; y < 5; y=y+1) begin: apply_chi_y
//        for (x = 0; x < 5; x=x+1) begin: apply_chi_x
//            if(x==0 && y==0) // add iota
//                assign A_out[x+5*y] = (A_chi[x+5*y] ^ (~A_chi[(`MOD5(x+1))+5*y] & A_chi[(`MOD5(x+2))+5*y])) ^ rcin;
//            else
//                assign A_out[x+5*y] = A_chi[x+5*y] ^ (~A_chi[(`MOD5(x+1))+5*y] & A_chi[(`MOD5(x+2))+5*y]);
//        end
//    end

    // Partial unroll
    assign A_out[0] = (A_chi[0] ^ (~A_chi[1] & A_chi[2])) ^ rcin; // add iota
    assign A_out[1] =  A_chi[1] ^ (~A_chi[2] & A_chi[3]);
    assign A_out[2] =  A_chi[2] ^ (~A_chi[3] & A_chi[4]);
    assign A_out[3] =  A_chi[3] ^ (~A_chi[4] & A_chi[0]);
    assign A_out[4] =  A_chi[4] ^ (~A_chi[0] & A_chi[1]);
    
    // Rest of the loop
    for (y = 1; y < 5; y = y + 1) begin: apply_chi_row // faster sim version
        assign A_out[5*y + 0] = (A_chi[5*y + 0] ^ (~A_chi[5*y + 1] & A_chi[5*y + 2]));
        assign A_out[5*y + 1] =  A_chi[5*y + 1] ^ (~A_chi[5*y + 2] & A_chi[5*y + 3]);
        assign A_out[5*y + 2] =  A_chi[5*y + 2] ^ (~A_chi[5*y + 3] & A_chi[5*y + 4]);
        assign A_out[5*y + 3] =  A_chi[5*y + 3] ^ (~A_chi[5*y + 4] & A_chi[5*y + 0]);
        assign A_out[5*y + 4] =  A_chi[5*y + 4] ^ (~A_chi[5*y + 0] & A_chi[5*y + 1]);
    end
    
    
    //------------------------------------------------------------------
    // 8) Add round constants
    //------------------------------------------------------------------
//    assign A_out[0] = A_iota[0] ^ rcin;
//    for (x = 1; x < 25; x=x+1) begin: direct_connect
//        assign A_out[x] = A_iota[x];
//    end
    
    //------------------------------------------------------------------
    // 9) Pack lanes into dout
    //------------------------------------------------------------------
    for (x = 0; x < 25; x=x+1) begin: pack_lanes
        assign dout[x*64 +:64] = A_out[x];
    end
    
endgenerate 

endmodule


   //-------------------------------------------------
    // Combine 2 & 3 & 4 for even faster sim: Works, but the implementation result is worse
    //-------------------------------------------------
//    for(x=0; x<5; x=x+1) begin
//        assign A_rho[x   ] = A_theta[x   ]^A_theta[`MOD5(x-1)]^A_theta[`MOD5(x-1)+5]^A_theta[`MOD5(x-1)+10]^A_theta[`MOD5(x-1)+15]^A_theta[`MOD5(x-1)+20]
//                                          ^ {A_theta[`MOD5(x+1)][62:0]^A_theta[`MOD5(x+1)+5][62:0]^A_theta[`MOD5(x+1)+10][62:0]^A_theta[`MOD5(x+1)+15][62:0]^A_theta[`MOD5(x+1)+20][62:0],
//                                             A_theta[`MOD5(x+1)][63]  ^A_theta[`MOD5(x+1)+5][63]  ^A_theta[`MOD5(x+1)+10][63]  ^A_theta[`MOD5(x+1)+15][63]  ^A_theta[`MOD5(x+1)+20][63]};
        
//        assign A_rho[x+5 ] = A_theta[x+5 ]^A_theta[`MOD5(x-1)]^A_theta[`MOD5(x-1)+5]^A_theta[`MOD5(x-1)+10]^A_theta[`MOD5(x-1)+15]^A_theta[`MOD5(x-1)+20]
//                                          ^ {A_theta[`MOD5(x+1)][62:0]^A_theta[`MOD5(x+1)+5][62:0]^A_theta[`MOD5(x+1)+10][62:0]^A_theta[`MOD5(x+1)+15][62:0]^A_theta[`MOD5(x+1)+20][62:0],
//                                             A_theta[`MOD5(x+1)][63]  ^A_theta[`MOD5(x+1)+5][63]  ^A_theta[`MOD5(x+1)+10][63]  ^A_theta[`MOD5(x+1)+15][63]  ^A_theta[`MOD5(x+1)+20][63]};
        
//        assign A_rho[x+10] = A_theta[x+10]^A_theta[`MOD5(x-1)]^A_theta[`MOD5(x-1)+5]^A_theta[`MOD5(x-1)+10]^A_theta[`MOD5(x-1)+15]^A_theta[`MOD5(x-1)+20]
//                                          ^ {A_theta[`MOD5(x+1)][62:0]^A_theta[`MOD5(x+1)+5][62:0]^A_theta[`MOD5(x+1)+10][62:0]^A_theta[`MOD5(x+1)+15][62:0]^A_theta[`MOD5(x+1)+20][62:0],
//                                             A_theta[`MOD5(x+1)][63]  ^A_theta[`MOD5(x+1)+5][63]  ^A_theta[`MOD5(x+1)+10][63]  ^A_theta[`MOD5(x+1)+15][63]  ^A_theta[`MOD5(x+1)+20][63]};
        
//        assign A_rho[x+15] = A_theta[x+15]^A_theta[`MOD5(x-1)]^A_theta[`MOD5(x-1)+5]^A_theta[`MOD5(x-1)+10]^A_theta[`MOD5(x-1)+15]^A_theta[`MOD5(x-1)+20]
//                                          ^ {A_theta[`MOD5(x+1)][62:0]^A_theta[`MOD5(x+1)+5][62:0]^A_theta[`MOD5(x+1)+10][62:0]^A_theta[`MOD5(x+1)+15][62:0]^A_theta[`MOD5(x+1)+20][62:0],
//                                             A_theta[`MOD5(x+1)][63]  ^A_theta[`MOD5(x+1)+5][63]  ^A_theta[`MOD5(x+1)+10][63]  ^A_theta[`MOD5(x+1)+15][63]  ^A_theta[`MOD5(x+1)+20][63]};
        
//        assign A_rho[x+20] = A_theta[x+20]^A_theta[`MOD5(x-1)]^A_theta[`MOD5(x-1)+5]^A_theta[`MOD5(x-1)+10]^A_theta[`MOD5(x-1)+15]^A_theta[`MOD5(x-1)+20]
//                                          ^ {A_theta[`MOD5(x+1)][62:0]^A_theta[`MOD5(x+1)+5][62:0]^A_theta[`MOD5(x+1)+10][62:0]^A_theta[`MOD5(x+1)+15][62:0]^A_theta[`MOD5(x+1)+20][62:0],
//                                             A_theta[`MOD5(x+1)][63]  ^A_theta[`MOD5(x+1)+5][63]  ^A_theta[`MOD5(x+1)+10][63]  ^A_theta[`MOD5(x+1)+15][63]  ^A_theta[`MOD5(x+1)+20][63]};
//    end
