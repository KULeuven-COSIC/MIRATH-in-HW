`include "clog2.v"
`include "keccak_pkg_64.v"
`include "keccak_interc_defs.vh"
`default_nettype none

module keccak_wrapper #(
    parameter KECCAK_WIN = 64,
    parameter KECCAK_WIN_BYTES = (KECCAK_WIN>>3),
    parameter CLOG2_WIN = `CLOG2(KECCAK_WIN), // should be 6
    parameter CLOG2_WIN_BYTES = `CLOG2(KECCAK_WIN_BYTES), // should be 3
    parameter CAPACITY_BYTES = 13
)(
    input wire rst,
    input wire clk,
    
    input wire k2c_shift_bytes,
    input wire c2k_din_valid,
    input wire [CLOG2_WIN_BYTES:0] bytelen_din,
    input wire [KECCAK_WIN-1:0] c2k_din,
    input wire [KECCAK_WIN-1:0] com_in,
    
    output wire [KECCAK_WIN-1:0] k2c_dout,
    output wire k2c_dout_valid,
    output wire c2k_din_ready,
    output wire data_done,
    output wire input_done,
    output wire rst_y_mul
);

wire squeeze_done, c2k_dout_valid, c2k_dout_valid_next;
wire [KECCAK_WIN-1:0] keccak_fifo_din, keccak_din, keccak_com_in, keccak_dout;

wire keccak_din_ready, k_dout_ready_next;
wire consume_buffer, input_round_valid;
wire [KECCAK_WIN-1:0] k_fifo_dout;

wire keccak_dout_valid;
wire [`K_FIFO_ADDR_BITS-1:0] k_din_fifo_addr_wr, k_din_fifo_addr_re;

control_to_keccak_router c2k_router (
    .rst                    (rst),
    .clk                    (clk),
    
    .c2k_din_valid          (c2k_din_valid),
//    .keccak_din_ready       (keccak_din_ready),
    .bytelen_din            (bytelen_din), // Can go up to 8 -> Make sure its zero when c2k_din_valid==1'b0
    .c2k_din                (c2k_din),
    .com_in                 (com_in),
    .squeeze_done           (squeeze_done),
    .consume_buffer         (consume_buffer),
    
//    .c2k_din_ready          (c2k_din_ready),
    .c2k_dout_valid         (c2k_dout_valid),
//    .c2k_dout_valid_next    (c2k_dout_valid_next),
    .dout                   (keccak_fifo_din),
    .com_out                (keccak_com_in),
    .input_done             (input_done),
    .input_round_valid      (input_round_valid),
    .k_din_fifo_addr_wr     (k_din_fifo_addr_wr)
);

keccak_top_64 keccak_inst (
    .rst                    (rst),
    .clk                    (clk),
    
//    .din_valid_next         (c2k_dout_valid_next),
    .din                    (k_fifo_dout),
    .com_in                 (keccak_com_in),
    
    .input_round_valid      (input_round_valid),
    .dout_valid             (keccak_dout_valid),
    .dout_ready_next        (k_dout_ready_next),
    .dout                   (keccak_dout),
    .k_din_fifo_addr_re     (k_din_fifo_addr_re),
    
    .squeeze_done           (squeeze_done),
    .consume_buffer         (consume_buffer),
    .rst_y_mul              (rst_y_mul)
);

keccak_to_control_router k2c_router (
    .rst                    (rst),
    .clk                    (clk),
    
    .din_valid              (keccak_dout_valid),
    .keccak_squeeze_done    (squeeze_done), 
    .din                    (keccak_dout),
    
    .shift_bytes            (k2c_shift_bytes),
    
    .din_ready_next         (k_dout_ready_next),
    
    .dout_valid             (k2c_dout_valid),
    .dout                   (k2c_dout),
    .keccak_data_done       (data_done)
);

simple_dual_port_mem #(
    .WIDTH  (KECCAK_WIN),
    .DEPTH  (`K_FIFO_DEPTH)
) k_input_fifo (
    .clk    (clk),
    .wea    (c2k_dout_valid),
    .addra  (k_din_fifo_addr_wr),
    .addrb  (k_din_fifo_addr_re),
    
    .dia     (keccak_fifo_din),
    .dob     (k_fifo_dout)
);

endmodule
