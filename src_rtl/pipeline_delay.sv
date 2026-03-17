module pipeline_delay #(
    parameter int WIDTH = 8,
    parameter int DEPTH = 1       // DEPTH = 1 => one cycle of delay
)(
    input  logic                 clk,
    input  logic [WIDTH-1:0]     din,
    output logic [WIDTH-1:0]     dout
);

    // Pipeline storage
    logic [WIDTH-1:0] stage [DEPTH-1:0];

    genvar i;
    generate
        for (i = 0; i < DEPTH; i++) begin : gen_pipe
            if (i == 0) begin
                // First stage takes the input
                always_ff @(posedge clk) begin
                    stage[0] <= din;
                end
            end else begin
                // Each stage takes the previous stage output
                always_ff @(posedge clk) begin
                    stage[i] <= stage[i-1];
                end
            end
        end
    endgenerate

    // Output of the last stage
    assign dout = stage[DEPTH-1];

endmodule