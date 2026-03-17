`include "math.vh"
`include "mirath_hw_params.vh"
`include "E_mul_defines.svh"

`default_nettype none

module mirath_ff_math_wrap #(
    parameter M_PARAM_M = `M_PARAM_M,
    parameter M_PARAM_N = `M_PARAM_N,
    parameter M_PARAM_R = `M_PARAM_R,
    parameter M_PARAM_K = `M_PARAM_K,
    parameter WORD_SIZE = `WORD_SIZE,
    parameter TAU = `TAU
)(  
    input wire                  rst,
    input wire                  clk,
    
    input wire                  next,
    input wire                  next_from_TMP,
    
    input wire [1:0]            start,
    input wire                  start_MPC,
    
    input wire                  keccak_dout_valid,
    input wire [7:0]            keccak_dout_byte,
    
    output wire [7:0]           E_byte_to_y_acc,
    output reg                  E_byte_valid_to_y_acc, // was a reg
    output reg                  S_elem_out,
    output reg                  C_elem_out
);

reg rst_output_elems, rst_output_elems_pip, C_elem_out_next;
always_ff @ (posedge clk) begin
    rst_output_elems_pip <= rst_output_elems;
    if (rst_output_elems_pip) begin
        S_elem_out <= 1'b0;
        C_elem_out <= 1'b1;
        C_elem_out_next <= 1'b1;
    
    end else begin
        S_elem_out <= S_mem_dout[S_row_sel_counter_pip];
        C_elem_out <= C_elem_out_next;
        C_elem_out_next <= C_mem_dout[C_row_sel];
    end
end

reg keccak_dout_valid_pip;//, next_pip;
reg [7:0] keccak_dout_byte_pip;

always_ff @ (posedge clk) begin // Pipeline inputs
//    next_pip              <= next;
    keccak_dout_valid_pip <= keccak_dout_valid;
    keccak_dout_byte_pip  <= keccak_dout_byte;
end

// State encoding
typedef enum logic [2:0] {
    IDLE,
    FILL_S_C_MEMS,
    COMPUTE_E,
    WAIT_NEXT_E,
    WAIT_FOR_NEXT_MPC_EMU,
    EMULATE_MPC,
    WAIT_NEXT_ELEM
} state_t;

state_t state, next_state;

reg ff_math_en;
reg sign;
reg S_addr_incr_reg;
always_ff @ (posedge clk) S_addr_incr_reg <= S_addr_opc == incr_S_addr;

// ***************************************
// Define & work with mems and addresses
// ***************************************

localparam S_MEM_DEPTH = `GET_BYTES(M_PARAM_M)*M_PARAM_R;
localparam S_BASE_MEM_DEPTH = `GET_NIBBLES(M_PARAM_M)*M_PARAM_R+1;
localparam C_MEM_DEPTH = M_PARAM_N-M_PARAM_R+2;
localparam FULL_ZEROS_C_ADDR = C_MEM_DEPTH-'h2;

wire [7:0]           S_mem_dout;
wire [M_PARAM_R-1:0] C_mem_dout;

reg S_stored, S_stored_in_1;
reg C_stored, C_stored_pip, C_stored_in_1;

//wire S_mem_wren = (state == FILL_S_C_MEMS) && ~S_stored;
reg S_mem_wren;
always_ff @ (posedge clk) S_mem_wren <= (state == FILL_S_C_MEMS) && ~S_stored_in_1;

//wire C_mem_wren = (state == FILL_S_C_MEMS) && S_stored && ~C_stored;
reg C_mem_wren;
always_ff @ (posedge clk) C_mem_wren <= (state == FILL_S_C_MEMS) && S_stored_in_1 && ~C_stored_in_1;

always_ff @ (posedge clk) begin
    S_stored <= S_stored_in_1;
    
    if (rst)
        S_stored_in_1 <= 1'b0;
    else if (S_mem_addr==S_MEM_DEPTH-2) // These statements consider purely streaming S and C. i.e. Only one round of SHAKE squeeze for all parameter sets
        S_stored_in_1 <= 1'b1;
end

always_ff @ (posedge clk) begin
    C_stored <= C_stored_in_1;
    C_stored_pip <= C_stored;
    
    if (rst)
        C_stored_in_1 <= 1'b0;
    else if (C_mem_addr==FULL_ZEROS_C_ADDR-2)
        C_stored_in_1 <= 1'b1;
end

reg [$clog2(S_MEM_DEPTH)-1:0]   S_mem_addr;
reg [$clog2(C_MEM_DEPTH)-1:0]   C_mem_addr;

S_mem #(
    .M_PARAM_M(M_PARAM_M),
    .M_PARAM_N(M_PARAM_N),
    .M_PARAM_R(M_PARAM_R)
) S_mem_inst (
    .clk    (clk),
    .wren   (S_mem_wren),
    
    .addr   (S_mem_addr),
    .din    (keccak_dout_byte_pip),
    
    .dout   (S_mem_dout)
);

C_mem #(
    .M_PARAM_M(M_PARAM_M),
    .M_PARAM_N(M_PARAM_N),
    .M_PARAM_R(M_PARAM_R)
) C_mem_inst (
    .clk    (clk),
    .wren   (C_mem_wren),
    
    .addr   (C_mem_addr),
    .din    (keccak_dout_byte_pip),
    
    .dout   (C_mem_dout)
);

typedef enum logic [2:0] { // S mem address opcode
    freeze_S_addr,
    rst_S_addr,
    incr_S_addr,
    move_up_S_column,
    start_next_S_row,
    goto_start_of_S_row
} S_address_opcode_t;

S_address_opcode_t S_addr_opc;

typedef enum logic [1:0] { //C mem address opcode
    freeze_C_addr,
    rst_C_addr,
    incr_C_addr,
//    goto_ones,
    goto_zero
} C_address_opcode_t;

C_address_opcode_t C_addr_opc;

always_ff @ (posedge clk) begin // update S_mem_addr
    if (rst)
        S_mem_addr <= 'h0;
    else begin
        case (S_addr_opc)
            freeze_S_addr: S_mem_addr <= S_mem_addr;
            
            incr_S_addr:   S_mem_addr <= S_mem_addr +1'b1;
            
            rst_S_addr:    S_mem_addr <= 'h0;
            
            move_up_S_column: S_mem_addr <= S_mem_addr +`GET_BYTES(M_PARAM_M);
            
            start_next_S_row: S_mem_addr <= S_mem_addr -(3*`GET_BYTES(M_PARAM_M) -1);
            
            goto_start_of_S_row: S_mem_addr <= S_mem_addr -(3*`GET_BYTES(M_PARAM_M));
        endcase
    end
end

always_ff @ (posedge clk) begin // update C_mem_addr
    if (rst)
        C_mem_addr <= 'h0;
    else begin
        case (C_addr_opc)
            freeze_C_addr: C_mem_addr <= C_mem_addr;
            
            incr_C_addr:   C_mem_addr <= C_mem_addr +1'b1;
            
            rst_C_addr:    C_mem_addr <= 'h0;
            
//            goto_ones:   C_mem_addr <= FULL_ZEROS_C_ADDR+1;
            
            goto_zero:   C_mem_addr <= FULL_ZEROS_C_ADDR;
        endcase
    end
end

// ********************************************************************
// 2 counters to book-keep where we are at in the multiplications

typedef enum logic [1:0] { // counter opcode
    hold_count,
    rst_count,
    incr_count
} counter_opcode_t;

counter_opcode_t S_col_counter_opc;
counter_opcode_t S_row_div8_counter_opc;
counter_opcode_t S_row_sel_counter_opc;

reg [$clog2(`GET_BYTES(M_PARAM_M))-1:0] S_row_div8_counter;
reg [$clog2(`GET_BYTES(M_PARAM_M))-1:0] S_row_sel_counter, S_row_sel_counter_pip;
reg [$clog2(M_PARAM_R)-1:0] S_col_counter;

always_ff @ (posedge clk) begin
    S_row_sel_counter_pip <= S_row_sel_counter;

    case (S_col_counter_opc)
        
        rst_count: S_col_counter <= 'h0;
            
        incr_count: S_col_counter <= S_col_counter + 1'b1;
//        incr_count: S_col_counter <= (S_col_counter==M_PARAM_R-1) ? 'h0 : S_col_counter + 1'b1; // More general and easy to work with: already works for L1 without explicit call
    endcase
end

always_ff @ (posedge clk) begin
    case (S_row_div8_counter_opc)
        
        rst_count: S_row_div8_counter <= 'h0;
            
        incr_count: S_row_div8_counter <= (S_row_div8_counter==`GET_BYTES(M_PARAM_M)-1) // More general and easy to work with:
                                        ? 'h0 : S_row_div8_counter + 1'b1; // already works for L1 without explicit call
    endcase
end

always_ff @ (posedge clk) begin
    case (S_row_sel_counter_opc)
        rst_count: S_row_sel_counter <= 'h0;
            
        incr_count: S_row_sel_counter <= S_row_sel_counter + 1'b1;
    endcase
end
// *************************************************************************************************
// A cheap MAC unit to compute up to 8 elements of the secret matrix E at a time (takes R cycles)

reg E_valid_next;
always_ff @ (posedge clk) E_byte_valid_to_y_acc <= E_valid_next;

reg [7:0] E_acc_regs;
//always_ff @ (posedge clk) E_byte_to_y_acc <= E_acc_regs;
assign E_byte_to_y_acc = E_acc_regs;

reg [$clog2(M_PARAM_R)-1:0] C_row_sel;

E_mul_opcode_t E_mul_opcode;

always_ff @ (posedge clk) begin // update E_acc_regs
    case (E_mul_opcode)
        init_e:
            E_acc_regs <= C_mem_dout[C_row_sel] ? S_mem_dout : 8'h0;
        
        acc_e:
            E_acc_regs <= E_acc_regs ^ (C_mem_dout[C_row_sel] ? S_mem_dout : 8'h0);
    endcase
end

// ***********************************************************
// State logic:     (State transitions written further below)

always_comb begin
    // Defaults:
    S_addr_opc   = freeze_S_addr;
    C_addr_opc   = freeze_C_addr;
    E_mul_opcode = init_e;
    S_row_div8_counter_opc = rst_count;
    S_row_sel_counter_opc = rst_count;
    S_col_counter_opc = rst_count;
    C_row_sel = S_col_counter; // Note: should work for everything?
    
    E_valid_next = 1'b0;
    rst_output_elems = 1'b1;
    
    case (state)
        FILL_S_C_MEMS: begin
            if (keccak_dout_valid_pip) begin
                if (~S_stored) begin
                    S_addr_opc = incr_S_addr;
                end else begin
                    C_addr_opc = incr_C_addr;
                end
            end
            
            if (S_stored)
                    S_addr_opc = rst_S_addr;
            
            if (C_stored)
                C_addr_opc = rst_C_addr;
                
            if (C_stored_pip)
                S_addr_opc = move_up_S_column;
//                S_addr_opc <= move_up_S_column;
        end // FILL_S_C_MEMS
        
        COMPUTE_E: begin
            if (S_col_counter!=M_PARAM_R-1)
                S_addr_opc = move_up_S_column;
            
            S_col_counter_opc = incr_count;
            S_row_div8_counter_opc = hold_count;
            
            if (S_col_counter!='h0)
                E_mul_opcode = acc_e;
//                E_mul_opcode <= acc_e;
            
            if (S_col_counter==M_PARAM_R-2) begin
                S_addr_opc = start_next_S_row;
                
                if (S_row_div8_counter==`GET_BYTES(M_PARAM_M)-1) begin
                    S_addr_opc = rst_S_addr;
                    C_addr_opc = incr_C_addr; 
                end
            end
            
            if (S_col_counter==M_PARAM_R-1) begin
                E_valid_next = 1'b1;
//                E_valid_next <= 1'b1;
                S_row_div8_counter_opc = incr_count;
            end
            
            if (C_mem_addr==FULL_ZEROS_C_ADDR)
                C_addr_opc = rst_C_addr;
        end // COMPUTE_E
        
        WAIT_NEXT_E: begin
            S_row_div8_counter_opc = hold_count;
            
            if (next) begin
                S_addr_opc = move_up_S_column;
            end            
        end // WAIT_NEXT_E
        
        WAIT_FOR_NEXT_MPC_EMU: begin
            if (start_MPC) begin
                rst_output_elems = 1'b0;
//                S_col_counter_opc = incr_count;
//                S_addr_opc = move_up_S_column;
            end
        end
        
        EMULATE_MPC: begin
            rst_output_elems = 1'b0;
            S_col_counter_opc = incr_count;
            
            if (S_col_counter!=M_PARAM_R-1)
                S_addr_opc = move_up_S_column;
            
            S_row_div8_counter_opc = hold_count;
            S_row_sel_counter_opc = hold_count;
            
            if (S_col_counter==M_PARAM_R-1) begin
                S_row_sel_counter_opc = incr_count;
                S_addr_opc = goto_start_of_S_row;
                if (S_row_sel_counter=='h7) begin
                    S_row_div8_counter_opc = incr_count;
                    S_addr_opc = start_next_S_row;
                end
            end
            
            if ({S_row_div8_counter, S_row_sel_counter}==(M_PARAM_M-1) && (S_col_counter==(M_PARAM_R-1))) begin
                S_addr_opc = rst_S_addr;
                C_addr_opc = incr_C_addr;
                S_row_div8_counter_opc = incr_count;
                S_row_sel_counter_opc = rst_count;
            end
        end
        
        WAIT_NEXT_ELEM: begin
            rst_output_elems = 1'b0;
            S_row_div8_counter_opc = hold_count;
            S_row_sel_counter_opc = hold_count;
            S_col_counter_opc = hold_count;
        end
        
    endcase // state
end


// ********************
// Simple FSM

//  State update
always_ff @(posedge clk) begin
    if (rst)
        state <= IDLE;
    else
        state <= next_state;
end

// Next-state logic
always_comb begin
    next_state = state;  // default

    case (state)
        IDLE:
            if (ff_math_en)     next_state = FILL_S_C_MEMS;
        
        FILL_S_C_MEMS:
            if  (C_stored_pip)     next_state = COMPUTE_E;
        
        COMPUTE_E:
            if (C_mem_addr==FULL_ZEROS_C_ADDR)
                next_state = WAIT_FOR_NEXT_MPC_EMU;
            else if((S_col_counter==M_PARAM_R-1))
                next_state = WAIT_NEXT_E;
        
        WAIT_NEXT_E:
            if(next) next_state = COMPUTE_E;
            
        WAIT_FOR_NEXT_MPC_EMU:
            if (start_MPC) next_state = EMULATE_MPC;
        
        EMULATE_MPC:
            if  (S_col_counter==M_PARAM_R-1)
                next_state = WAIT_NEXT_ELEM;
        
        WAIT_NEXT_ELEM:
            if (next_from_TMP)
                next_state = EMULATE_MPC;
            
        
    endcase
end

// Output logic
//always_comb begin
//    done = (state == FINISH);
//end

// ****************
// Start flags

always_ff @ (posedge clk) begin
    if (rst)
        ff_math_en <= 1'b0;
    else if (^start) // 2'b00: no start, 2'b01: keygen, 2'b10: sign, 2'b11: verify -> This module only takes part in keygen & sign
        ff_math_en <= 1'b1;
end

always_ff @ (posedge clk) begin
    if (rst)
        sign <= 1'b0;
    else if (start[1]) // In the verify case, this reg will never be checked anyway
        sign <= 1'b1;
end

wire [5:0] S_row_debug_wire = {S_row_div8_counter, S_row_sel_counter};

// *************************************************************************
// Instantiate the acc[e] module: For every MPC round (during signing only)
// this module starts by asking the FSM in this file for S and C and then 
// accumulates the sampled randomness S_rnd and C_rnd to get acc[e].
// Then it should store it into the key/sig mem, for which it should have
// priority over the top-level control module to make things simple.

//acc_e acc_e_inst (
//    .rst                  ( rst ),
//    .clk                  ( clk ),

//    .acc_sample_valid     ( acc_sample_valid ),
//    .acc_sample           ( acc_sample ),

//    .S_elem               ( S_elem_to_acc ),
//    .C_elem               ( C_elem_to_acc ),

//    .S_elem_valid         ( S_elem_to_acc_valid ),
//    .C_elem_valid         ( C_elem_to_acc_valid ),

////    .S_C_ready            ( /* connect */ )//,
//    .store_round          ( acc_store_round )//,

////    .dout                 ( /* connect */ ),
////    .store_acc_e_mem_addr ( /* connect */ ),
////    .sig_mem_wren         ( /* connect */ )
//);

endmodule
