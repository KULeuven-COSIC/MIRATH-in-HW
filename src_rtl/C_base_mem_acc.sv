`include "math.vh"
`include "mirath_hw_params.vh"

`default_nettype none

module C_base_mem_acc #(
    parameter M_PARAM_M = `M_PARAM_M,
    parameter M_PARAM_N = `M_PARAM_N,
    parameter M_PARAM_R = `M_PARAM_R,
    parameter M_PARAM_K = `M_PARAM_K,
    parameter WORD_SIZE = `WORD_SIZE,
    parameter S_BASE_MEM_DEPTH = `GET_NIBBLES(M_PARAM_M)*M_PARAM_R+1,
    parameter C_BASE_MEM_DEPTH = M_PARAM_N-M_PARAM_R+1,
    parameter TAU = `TAU,
    parameter SAMPLE_COUNT = 10, // TODO: parametrize for all levels
    parameter MEM_WIDTH_BYTES = M_PARAM_R
)(
    input wire rst,
    input wire clk,
    
//    input wire wren,
    input wire [$clog2(TAU)-1:0] mpc_round_C_base,
    input wire [7:0]             phi_i,
    input wire                  shift_en_next,
    input wire                  acc_sample_valid,
    input wire [WORD_SIZE-1:0]  acc_sample,
    
    input wire [$clog2(SAMPLE_COUNT)-1:0] sample_counter,
        
    input wire [$clog2(C_BASE_MEM_DEPTH)-1:0] re_addr_C_base,
    
    output reg [8*MEM_WIDTH_BYTES-1:0]  dout [TAU-1:0]
);

//reg phi_i_pip;
//always_ff @ (posedge clk) phi_i_pip <= phi_i;

localparam GRAB_REG_LEN = ((M_PARAM_M -M_PARAM_R) * (M_PARAM_R));
reg [GRAB_REG_LEN-1:0] grab_regs;

//reg shift_init;
//localparam SAMPLES_C = 8; // 3 for S and then 5 for C
//always_ff @ (posedge clk) shift_init <= (sample_counter==SAMPLES_C-2);

//localparam SHIFT_COUNT_MAX = (GRAB_REG_LEN/M_PARAM_R)-1;
//localparam SHIFT_COUNT_BITS = $clog2(SHIFT_COUNT_MAX+1);
//reg [SHIFT_COUNT_BITS-1:0] shift_counter;

//always_ff @ (posedge clk) begin
//    if (rst)
//        shift_counter <= 'h0;
//    else if (shift_init)
//        shift_counter <= SHIFT_COUNT_MAX;
//    else if (shift_counter!=0)
//        shift_counter <= shift_counter -1'b1;
//end

reg [31:0] phi_i_times_C;
always_ff @ (posedge clk) begin
    if (!grab_regs[0])
        phi_i_times_C[7:0] <= 8'h0;
    else
        phi_i_times_C[7:0] <= phi_i;
    
    if (!grab_regs[1])
        phi_i_times_C[15:8] <= 8'h0;
    else
        phi_i_times_C[15:8] <= phi_i;
    
    if (!grab_regs[2])
        phi_i_times_C[23:16] <= 8'h0;
    else
        phi_i_times_C[23:16] <= phi_i;
    
    if (!grab_regs[3])
        phi_i_times_C[31:24] <= 8'h0;
    else
        phi_i_times_C[31:24] <= phi_i;
end

reg shift_en; //, shift_en_pip;
always_ff @ (posedge clk) shift_en <= shift_en_next;
//always_ff @ (posedge clk) shift_en <= |{shift_init, shift_counter};
////always_ff @ (posedge clk) shift_en_pip <= shift_en;

always_ff @ (posedge clk) begin // grab_regs update
    if (rst)
       grab_regs <= 1'b0;
    else if (shift_en_next) 
        grab_regs <= (grab_regs >> MEM_WIDTH_BYTES);
    
    else if (acc_sample_valid) begin
        case (sample_counter)
            'h3: begin
                for (int i = 0; i < 8; i++) begin
                    grab_regs[M_PARAM_R*i +: M_PARAM_R] <= acc_sample[8*i +: M_PARAM_R];
                end
             end
             
             'h4: begin
                for (int i = 0; i < 8; i++) begin
                    grab_regs[M_PARAM_R*(i+8) +: M_PARAM_R] <= acc_sample[8*i +: M_PARAM_R];
                end
             end
             
             'h5: begin
                for (int i = 0; i < 8; i++) begin
                    grab_regs[M_PARAM_R*(i+16) +: M_PARAM_R] <= acc_sample[8*i +: M_PARAM_R];
                end
             end
             
             'h6: begin
                for (int i = 0; i < 8; i++) begin
                    grab_regs[M_PARAM_R*(i+24) +: M_PARAM_R] <= acc_sample[8*i +: M_PARAM_R];
                end
             end
             
             'h7: begin
                for (int i = 0; i < 6; i++) begin
                    grab_regs[M_PARAM_R*(i+32) +: M_PARAM_R] <= acc_sample[8*i +: M_PARAM_R];
                end
             end
        endcase
    end
end

localparam FULL_ZEROS_C_BASE_MEM_ADDR = C_BASE_MEM_DEPTH-1;

       
reg [$clog2(C_BASE_MEM_DEPTH)-1:0] wr_addr_C_base;
always_ff @ (posedge clk) begin
    if (rst)
        wr_addr_C_base <= 'h0;
    else if (shift_en)
        wr_addr_C_base <= (wr_addr_C_base==C_BASE_MEM_DEPTH-2) ? 'h0 : (wr_addr_C_base+1'b1);
end   

genvar i;
generate
    for (i=0; i<TAU; i++) begin : TAU_C_base_mems
        localparam int IDX = i;
    
        reg mem_wren;
        always_ff @ (posedge clk) mem_wren <= (shift_en_next && mpc_round_C_base==i);
        
        // Instantiate the mem
        (* ram_style = "distributed" *) 
        reg [8*MEM_WIDTH_BYTES-1:0] C_base_mem [C_BASE_MEM_DEPTH-1:0];
        
        // Set last address to zeros 
        initial begin
            C_base_mem[FULL_ZEROS_C_BASE_MEM_ADDR] = 42'h0;
//            C_base_mem[FULL_ZEROS_C_BASE_MEM_ADDR+1] = {8*M_PARAM_R{1'b1}}; // 6*8 = 42 (worst case all sets)
        end
        
        always_ff @(posedge clk) begin
            dout[i] <= C_base_mem[re_addr_C_base];
            if (mem_wren)
                C_base_mem[wr_addr_C_base] <=  dout[i] ^    // FF_mu acc
                                               phi_i_times_C; // TODO: change for L3 and L5!
        end
    end
endgenerate

endmodule


//reg first_acc, new_round;
//always_ff @ (posedge clk)
//    if (rst)
//        new_round <= 1'b1;
//    else if (shift_en_pip && !shift_en) // First acc done
//        new_round <= 1'b0;
//    else if (phi_i_pip[7] && !phi_i[7])


// Old version: less pipeline, extra AND-ing:

/*
module C_base_mem_acc #(
    parameter M_PARAM_M = 42,
    parameter M_PARAM_N = M_PARAM_M,
    parameter M_PARAM_R = 4,
    parameter M_PARAM_K = 1443,
    parameter WORD_SIZE = 64,
    parameter S_BASE_MEM_DEPTH = `GET_NIBBLES(M_PARAM_M)*M_PARAM_R+1,
    parameter C_BASE_MEM_DEPTH = M_PARAM_N-M_PARAM_R+1,
    parameter TAU = 'd17,
    parameter SAMPLE_COUNT = 10, // TODO: parametrize for all levels
    parameter MEM_WIDTH_BYTES = M_PARAM_R
)(
    input wire rst,
    input wire clk,
    
//    input wire wren,
    input wire [$clog2(TAU)-1:0] mpc_round_C_base,
    input wire [7:0]             phi_i,
    input wire                  shift_en_next,
    input wire                  acc_sample_valid,
    input wire [WORD_SIZE-1:0]  acc_sample,
    
    input wire [$clog2(SAMPLE_COUNT)-1:0] sample_counter,
        
    input wire [$clog2(C_BASE_MEM_DEPTH)-1:0] re_addr_C_base,
    
    output reg [8*MEM_WIDTH_BYTES-1:0]  dout [TAU-1:0]
);

//reg phi_i_pip;
//always_ff @ (posedge clk) phi_i_pip <= phi_i;

localparam GRAB_REG_LEN = ((M_PARAM_M -M_PARAM_R) * (M_PARAM_R));
reg [GRAB_REG_LEN-1:0] grab_regs;

//reg shift_init;
//localparam SAMPLES_C = 8; // 3 for S and then 5 for C
//always_ff @ (posedge clk) shift_init <= (sample_counter==SAMPLES_C-2);

//localparam SHIFT_COUNT_MAX = (GRAB_REG_LEN/M_PARAM_R)-1;
//localparam SHIFT_COUNT_BITS = $clog2(SHIFT_COUNT_MAX+1);
//reg [SHIFT_COUNT_BITS-1:0] shift_counter;

//always_ff @ (posedge clk) begin
//    if (rst)
//        shift_counter <= 'h0;
//    else if (shift_init)
//        shift_counter <= SHIFT_COUNT_MAX;
//    else if (shift_counter!=0)
//        shift_counter <= shift_counter -1'b1;
//end

reg shift_en; //, shift_en_pip;
always_ff @ (posedge clk) shift_en <= shift_en_next;
//always_ff @ (posedge clk) shift_en <= |{shift_init, shift_counter};
////always_ff @ (posedge clk) shift_en_pip <= shift_en;

always_ff @ (posedge clk) begin // grab_regs update
    if (rst)
       grab_regs <= 1'b0;
    else if (shift_en) 
        grab_regs <= (grab_regs >> MEM_WIDTH_BYTES);
    
    else if (acc_sample_valid) begin
        case (sample_counter)
            'h3: begin
                for (int i = 0; i < 8; i++) begin
                    grab_regs[M_PARAM_R*i +: M_PARAM_R] <= acc_sample[8*i +: M_PARAM_R];
                end
             end
             
             'h4: begin
                for (int i = 0; i < 8; i++) begin
                    grab_regs[M_PARAM_R*(i+8) +: M_PARAM_R] <= acc_sample[8*i +: M_PARAM_R];
                end
             end
             
             'h5: begin
                for (int i = 0; i < 8; i++) begin
                    grab_regs[M_PARAM_R*(i+16) +: M_PARAM_R] <= acc_sample[8*i +: M_PARAM_R];
                end
             end
             
             'h6: begin
                for (int i = 0; i < 8; i++) begin
                    grab_regs[M_PARAM_R*(i+24) +: M_PARAM_R] <= acc_sample[8*i +: M_PARAM_R];
                end
             end
             
             'h7: begin
                for (int i = 0; i < 6; i++) begin
                    grab_regs[M_PARAM_R*(i+32) +: M_PARAM_R] <= acc_sample[8*i +: M_PARAM_R];
                end
             end
        endcase
    end
end

localparam FULL_ZEROS_C_BASE_MEM_ADDR = C_BASE_MEM_DEPTH-1;

       
reg [$clog2(C_BASE_MEM_DEPTH)-1:0] wr_addr_C_base;
always_ff @ (posedge clk) begin
    if (rst)
        wr_addr_C_base <= 'h0;
    else if (shift_en)
        wr_addr_C_base <= (wr_addr_C_base==C_BASE_MEM_DEPTH-2) ? 'h0 : (wr_addr_C_base+1'b1);
end   

genvar i;
generate
    for (i=0; i<TAU; i++) begin : TAU_C_base_mems
        localparam int IDX = i;
    
        reg mem_wren;
        always_ff @ (posedge clk) mem_wren <= (shift_en_next && mpc_round_C_base==i);
        
        // Instantiate the mem
        (* ram_style = "distributed" *) 
        reg [8*MEM_WIDTH_BYTES-1:0] C_base_mem [C_BASE_MEM_DEPTH-1:0];
        
        // Set last address to zeros 
        initial begin
            C_base_mem[FULL_ZEROS_C_BASE_MEM_ADDR] = 42'h0;
//            C_base_mem[FULL_ZEROS_C_BASE_MEM_ADDR+1] = {8*M_PARAM_R{1'b1}}; // 6*8 = 42 (worst case all sets)
        end
        
        always_ff @(posedge clk) begin
            dout[i] <= C_base_mem[re_addr_C_base];
            if (mem_wren)
                C_base_mem[wr_addr_C_base] <=  dout[i] ^    // FF_mu acc
                                               {(grab_regs[3] ? phi_i : 8'h0), // TODO: change for L3 and L5!
                                                (grab_regs[2] ? phi_i : 8'h0),
                                                (grab_regs[1] ? phi_i : 8'h0),
                                                (grab_regs[0] ? phi_i : 8'h0)};
        end
    end
endgenerate

endmodule
*/