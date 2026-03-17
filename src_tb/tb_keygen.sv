`timescale 1ns / 1ps

`include "mirath_hw_params.vh"

`default_nettype none

module tb_keygen;

localparam clk_period = 10;
localparam half_clk_period = clk_period/2;
    
logic tb_rst = 0;
logic tb_clk = 0;

logic [1:0] tb_start = 0;
logic       tb_done;

mirath_wrapper_v2 DUT (
    .rst    ( tb_rst ),
    .clk    ( tb_clk ),

    .start  ( tb_start ),

    .done   ( tb_done )
);

always #(half_clk_period) tb_clk = ~tb_clk;

initial begin
    @(posedge tb_clk); // one warm-up edge
    // reset
    #(2.1*clk_period);
    tb_rst = 1'b1;
    #(2*clk_period);
    tb_rst = 1'b0;
//    #(2*clk_period);
    
    tb_start = `START_KEYGEN;
    #(1*clk_period);
    tb_start = 'h0;
    
    wait(tb_done);
    //    # (161200*clk_period);
    $finish;
end

endmodule
