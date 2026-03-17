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
