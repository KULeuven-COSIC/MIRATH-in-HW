// Simple math operation defines

`define GET_BYTES(x)    ((x+7)/8)

`define GET_NIBBLES(x)    ((x+3)/4)

`define ROUND_TO_8(x)   (8*`GET_BYTES(x))

`define ROUND_TO_4(x)   (4*`GET_NIBBLES(x))