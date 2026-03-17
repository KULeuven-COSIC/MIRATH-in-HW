`include "mirath_hw_params.vh"

`default_nettype none

module aux_acc(
    input wire                      clk,
    
    input wire                      init_aux,
    input wire [`WORD_SIZE-1:0]     acc_in,
    
    input wire                      acc_aux,
    input wire [`WORD_SIZE-1:0]     sample_in,
    
    output wire [`WORD_SIZE-1:0]     acc_out
);
    
    localparam S_BYTES = `GET_BYTES(`S_ROWS) * `S_COLS;
    localparam C_BYTES = `C_COLS;      
    
    localparam AUX_WORDS = `CEIL( 8*(S_BYTES+C_BYTES), `WORD_SIZE) / `WORD_SIZE;
    
    reg [`WORD_SIZE-1:0] shift_regs [AUX_WORDS-1:0];
    assign acc_out = shift_regs[AUX_WORDS-1];
    
    always_ff @ (posedge clk) begin
        if (init_aux) begin
            shift_regs[0] <= acc_in;
            
            for (integer i=1; i<AUX_WORDS; i=i+1)
                shift_regs[i] <= shift_regs[i-1];
        end else if (acc_aux) begin
            shift_regs[0] <= shift_regs[AUX_WORDS-1] ^ sample_in;
            
            for (integer i=1; i<AUX_WORDS; i=i+1)
                shift_regs[i] <= shift_regs[i-1];
        end
    end
    
endmodule
