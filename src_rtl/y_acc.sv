`include "mirath_hw_params.vh"
`include "math.vh"

`default_nettype none


typedef enum logic [1:0] {
    S_Y_IDLE,
    S_Y_ACC,
    S_Y_SQUEEZE                  // State to end in for both routines. Includes case where sig is rejected.
} acc_y_state_t;

module y_acc(
    input wire                      clk,
    input wire                      rst,
    
    input wire [`WORD_SIZE-1:0]     keccak_dout,
    input wire                      keccak_dout_valid,
    input wire                      keccak_done,
    
    input wire [7:0]                E_byte,
    input wire                      E_byte_valid,
    input wire                      squeeze_y,
    input wire                      rst_y_mul,
    
    output wire                     init_load_done,
    output reg                      next_E,
    output wire [`WORD_SIZE-1:0]    y_out
);

acc_y_state_t acc_y_state, acc_y_state_next;
reg acc_en, next_E_comb;
always_ff @ (posedge clk) next_E <= rst ? 'h0 : next_E_comb;

reg rst_y_mul_pip;
always_ff @ (posedge clk) rst_y_mul_pip <= rst_y_mul;

reg E_byte_valid_pip;
always_ff @ (posedge clk) E_byte_valid_pip <= E_byte_valid;

// H_row_div_8_counter
localparam H_ROWS_BYTE_ALIGNED = `GET_BYTES(`MIRATH_VAR_FF_Y_BITS);
//localparam H_COUNT_MAX = `GET_BYTES(`MIRATH_VAR_FF_Y_BITS);
reg [$clog2(H_ROWS_BYTE_ALIGNED)-1:0] H_row_div_8_counter; // H_row_div_8
always_ff @ (posedge clk) begin
    if (rst)
        H_row_div_8_counter <= H_ROWS_BYTE_ALIGNED-1'b1;
    else if (acc_en) begin
        if (H_row_div_8_counter<=('h7))
            H_row_div_8_counter <= H_row_div_8_counter + H_ROWS_BYTE_ALIGNED-'h8;
        else
            H_row_div_8_counter <= H_row_div_8_counter -'h8;
    end
end

localparam MAX_COUNT_S_ROWS = `GET_BYTES(`S_ROWS)-1;
reg [$clog2(MAX_COUNT_S_ROWS+1)-1:0] S_row_counter;
always_ff @ (posedge clk) begin
    if (rst)
        S_row_counter <= 1'b0;
    else if (E_byte_valid_pip)
        S_row_counter <= (S_row_counter=='h0) ? MAX_COUNT_S_ROWS : (S_row_counter-1'b1);
end

reg [$clog2(9)-1:0] valid_E_bits;
always_ff @ (posedge clk) begin
    if (rst)
        valid_E_bits <= 'h5;
    else if (acc_y_state==S_Y_ACC) begin
        if (update_hold_regs) begin
            if (S_row_counter=='h0)
                valid_E_bits <= 'h3;
            else
                valid_E_bits <= 'h9;
        end else if (keccak_dout_valid && (H_row_div_8_counter<8))
            valid_E_bits <= valid_E_bits-1'b1;
    end
end

reg [7:0] keccak_dout_pip;
always_ff @ (posedge clk) keccak_dout_pip <= keccak_dout[7:0];

reg shift_E_byte, update_hold_regs;
reg [7:0] E_byte_pip;
reg [8:0] E_byte_hold_shift_regs;
always_ff @ (posedge clk) begin
    if (E_byte_valid)
        E_byte_pip <= E_byte;
    
    if (shift_E_byte)
        E_byte_hold_shift_regs <= (E_byte_hold_shift_regs>>1);
    if (update_hold_regs)
        E_byte_hold_shift_regs[8:1] <= E_byte_pip;
    if (load_phase_1_en[e_A_2nd_WORDS] && acc_y_state==S_Y_IDLE)
        E_byte_hold_shift_regs <= E_byte_pip>>((`MIRATH_VAR_FF_Y_BITS%`S_ROWS)%8);
    
    if (rst_y_mul_pip)
        E_byte_hold_shift_regs[1] <= 1'b0;
end

reg keccak_dout_valid_hold;
always_ff @ (posedge clk) keccak_dout_valid_hold <= rst ? 1'b0 :
                                      keccak_dout_valid ? 1'b1 : keccak_dout_valid_hold;

localparam S_WORDS = `MIRATH_VAR_FF_S_BYTES;

reg [S_WORDS:0] load_phase_0_en;
always_ff @ (posedge clk) begin
    if (rst)
        load_phase_0_en <= 'h0;
    else
        load_phase_0_en <= {load_phase_0_en, keccak_dout_valid && ~keccak_dout_valid_hold};
end

localparam e_A_2nd_WORDS = `GET_BYTES(`MIRATH_VAR_FF_Y_BITS - `S_ROWS*`S_COLS)*`ROUND_TO_8(`S_ROWS)/`S_ROWS;

localparam ST_UPD_CNT_LEN = 'h2;
reg [e_A_2nd_WORDS:0] load_phase_1_en;
always_ff @ (posedge clk) begin
    if (rst) begin
        load_phase_1_en     <= 'h1;
    end else begin
        if (E_byte_valid_pip) begin
            load_phase_1_en <= {load_phase_1_en, 1'b0};
        end
    end
end

assign init_load_done = load_phase_1_en[e_A_2nd_WORDS];

localparam REG_LEN = `ROUND_TO_8(`MIRATH_VAR_FF_Y_BITS);
reg [REG_LEN-1:0] y_regs;
genvar i, row,col;
generate
    for (row = 0; row < `S_ROWS; row = row + 1) begin : ROW_LOOP
        for (col = 0; col < `S_COLS; col = col + 1) begin : COL_LOOP
            localparam ELEM_IDX  = row + col * `S_ROWS;
            localparam INPUT_IDX = row + col * `CEIL(`S_ROWS, 8);
            localparam INP_WORD_IDX = INPUT_IDX / 8;
            localparam BIT_IDX      = INPUT_IDX % 8;
                
            always_ff @ (posedge clk) begin: first_bits
                if (load_phase_0_en[INP_WORD_IDX])
                    y_regs[ELEM_IDX] <= keccak_dout_pip[BIT_IDX];
                
                else if (acc_en)
                    y_regs[ELEM_IDX] <= y_regs[ELEM_IDX+64];
            end
        end
    end
endgenerate          

localparam SECOND_SET_START = `S_ROWS*`S_COLS;
generate
    for (i = `S_ROWS*`S_COLS; i<REG_LEN; i=i+1) begin: rest_of_bits
        localparam COL = (i / `S_ROWS) -`S_COLS;
        localparam ROW = i % `S_ROWS;
        localparam INPUT_IDX = ROW + COL * `CEIL(`S_ROWS, 8);
        localparam INP_WORD_IDX = INPUT_IDX / 8;
        localparam BIT_IDX      = INPUT_IDX % 8;
        
        localparam LAST_WORD= (i>(REG_LEN-64));
        localparam IN_S = (i+64)%REG_LEN;
        
        always_ff @ (posedge clk) begin
            if (rst && i>=`MIRATH_VAR_FF_Y_BITS)
                    y_regs[i] <= 1'b0;
            else if (load_phase_1_en[INP_WORD_IDX] && i<`MIRATH_VAR_FF_Y_BITS)
                    y_regs[i] <= E_byte_pip[BIT_IDX];
            else if (acc_en) begin
                if (i<(REG_LEN-64))
                    y_regs[i] <= y_regs[i+64];
                else begin
                    if (IN_S < 8) begin
                        y_regs[i] <= y_regs[IN_S] ^ (keccak_dout[IN_S] & E_byte_hold_shift_regs[0]);
                    end else if (IN_S < 16) begin
                        y_regs[i] <= y_regs[IN_S] ^ (keccak_dout[IN_S] & E_byte_hold_shift_regs[(H_row_div_8_counter!= 0) ? 0 : 1]);
                    end else if (IN_S < 24) begin
                        y_regs[i] <= y_regs[IN_S] ^ (keccak_dout[IN_S] & E_byte_hold_shift_regs[(H_row_div_8_counter > 1) ? 0 : 1]);
                    end else if (IN_S < 32) begin
                        y_regs[i] <= y_regs[IN_S] ^ (keccak_dout[IN_S] & E_byte_hold_shift_regs[(H_row_div_8_counter > 2) ? 0 : 1]);
                    end else if (IN_S<40) begin
                        y_regs[i] <= y_regs[IN_S] ^ (keccak_dout[IN_S] & E_byte_hold_shift_regs[(H_row_div_8_counter > 3) ? 0 : 1]);
                    end else if (IN_S < 48) begin
                        y_regs[i] <= y_regs[IN_S] ^ (keccak_dout[IN_S] & E_byte_hold_shift_regs[(H_row_div_8_counter > 4) ? 0 : 1]);
                    end else if (IN_S < 56) begin
                        y_regs[i] <= y_regs[IN_S] ^ (keccak_dout[IN_S] & E_byte_hold_shift_regs[(H_row_div_8_counter > 5) ? 0 : 1]);
                    end else begin
                        y_regs[i] <= y_regs[IN_S] ^ (keccak_dout[IN_S] & E_byte_hold_shift_regs[(H_row_div_8_counter > 6) ? 0 : 1]);
                    end
                end
            end
        end
    end
endgenerate   
       
// *******************************************
// Build small FSM to simplify external comms   
always_ff @ (posedge clk) acc_y_state  <=  rst ?  S_Y_IDLE : acc_y_state_next; // State update

always_comb begin // State logic
    acc_y_state_next = acc_y_state; // Default
    
    case (acc_y_state)
        S_Y_IDLE:   acc_y_state_next = ~load_phase_1_en[e_A_2nd_WORDS] ? S_Y_IDLE : S_Y_ACC;
        
        S_Y_ACC:    acc_y_state_next = keccak_done ? S_Y_SQUEEZE : S_Y_ACC;
    endcase // acc_y_state
end

// *****************************
// Control logic
reg state_is_idle_pip;
always_ff @ (posedge clk) state_is_idle_pip <= acc_y_state==S_Y_IDLE;

always_comb begin
    shift_E_byte = 1'b0;
    update_hold_regs = 1'b0;
    acc_en = 1'b0;
    next_E_comb = 1'b1;
    
    case (acc_y_state)
        S_Y_ACC: begin
            shift_E_byte = (keccak_dout_valid && (H_row_div_8_counter<8));
            update_hold_regs = keccak_dout_valid && (H_row_div_8_counter<8) && valid_E_bits=='h2;
            acc_en = keccak_dout_valid;
            next_E_comb = keccak_dout_valid && (H_row_div_8_counter<8) && valid_E_bits=='h2;
        end
        
        S_Y_SQUEEZE: begin
            acc_en = squeeze_y;
        end
        
    endcase // acc_y_state
end

assign y_out = {y_regs[0+:24], y_regs[REG_LEN-1-:40]}; // TODO: Parametrize

wire [1:0] debug_wire = E_byte_hold_shift_regs;

reg [63:0] debug_input_word;
generate
    for (i=0; i<64; i++) begin
        if (i < 8) begin
            assign debug_input_word[i] = (keccak_dout[i] & E_byte_hold_shift_regs[0]);
        end else if (i < 16) begin
            assign debug_input_word[i] = (keccak_dout[i] & E_byte_hold_shift_regs[(H_row_div_8_counter!= 0) ? 0 : 1]);
        end else if (i < 24) begin
            assign debug_input_word[i] = (keccak_dout[i] & E_byte_hold_shift_regs[(H_row_div_8_counter > 1) ? 0 : 1]);
        end else if (i < 32) begin
            assign debug_input_word[i] = (keccak_dout[i] & E_byte_hold_shift_regs[(H_row_div_8_counter > 2) ? 0 : 1]);
        end else if (i<40) begin
            assign debug_input_word[i] = (keccak_dout[i] & E_byte_hold_shift_regs[(H_row_div_8_counter > 3) ? 0 : 1]);
        end else if (i < 48) begin
            assign debug_input_word[i] = (keccak_dout[i] & E_byte_hold_shift_regs[(H_row_div_8_counter > 4) ? 0 : 1]);
        end else if (i < 56) begin
            assign debug_input_word[i] = (keccak_dout[i] & E_byte_hold_shift_regs[(H_row_div_8_counter > 5) ? 0 : 1]);
        end else begin
            assign debug_input_word[i] = (keccak_dout[i] & E_byte_hold_shift_regs[(H_row_div_8_counter > 6) ? 0 : 1]);
        end
    end
endgenerate

endmodule

