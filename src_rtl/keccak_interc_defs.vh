`include "clog2.v"
`include "mirath_hw_params.vh"

`ifndef K_INTERC_DEFS
`define K_INTERC_DEFS

`define K_FIFO_ROUND_CTR_MAX 'h3F

`define K_FIFO_DEPTH      512
`define K_FIFO_ADDR_BITS  `CLOG2(`K_FIFO_DEPTH)


`endif