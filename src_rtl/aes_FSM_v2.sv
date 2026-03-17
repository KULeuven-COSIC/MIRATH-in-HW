`include "aes_parameters.vh"

`default_nettype none

// *******************
// State definitions
// *****************
localparam NUM_STATES = 'd24;
typedef enum logic [$clog2(NUM_STATES)-1:0] {
    S_IDLE,
    S_LOAD_SALT, /* and root_seed if relevant*/
    
    S_ARRAY_EXPAND,           // Sign entry
//    S_EXP_STORED_N_BRIDGE,    // used ONCE to bridge between ARRAY_EXPAND and OBTAIN_LEAF routines
    S_EXP_STORED_N,           // Expand last-level stored node
    S_EXP_GREAT_GP,           // Expand first non-stored level (aka great grandparent for K=4)
    S_EXP_GP,                 // Expand grandparent node
    S_EXP_PARENT,            // Expand parent
//    S_EXP_PARENT_ROUND_0,    // Expand parent for round 0 only (simplifies state transition and control complexity)
    S_EXPAND_BRIDGE,         // Store the last leaf/leaves, then start the expand routine
    S_EXPAND,                // Expand a share (leaf), then go to commit
    S_COMMIT,                // Commit the share (leaf) & load the next
    S_COMMIT_PREP_GGM,        // Same as the above, but used once at the end of every MPC share splitting round to prep the next one
    
    S_CLEAN_V_MEM,            // Start of share opening AND verify.
    S_WAIT_I_STAR,            // Wait for top-level controller to send i* list
    S_GET_SIBL_PATH,          // Exit behavior depends on Sign/Verify and ACCEPT/REJECT
    
    S_FILL_SIBL_PATH_0,       // If sibling path is accepted, fill the key/sig mem with the revealed stored nodes
    S_FILL_SIBL_PATH_1_SEARCH,// Continue with generating the non-stored nodes
    S_FILL_SIBL_PATH_1_STORE, // Continue with generating the non-stored nodes
    S_FILL_COMMITS,           // Sign END. -> Go to DONE
    
    S_COPY_NODES_0,           // Copy revealed nodes that belong in the permanent storage to data_mem
    S_COPY_NODES_1,           // Copy revealed nodes to data_mem
    // S_ARRAY_EXPAND,         // Partial GGM tree array expand for all stored nodes, using the same routine as before, but with extra flags 
    S_FAKE_EXPAND,            // Symbolic continuation of partial array tree expand: fill v_bit mem
    
    S_SEARCH_NEXT_NODE,       // During Partial tree expand, cycles between subsequent expansions non-deterministic: Stall for a bit (if needed) and run a search routine
    
    S_DONE                   // State to end in for both routines. Includes case where sig is rejected.
} state_t;

typedef enum logic [1:0] { MODE_GGM, MODE_EXP_LEAF, MODE_COMMIT } aes_mode_t;

typedef enum logic [1:0] { ST_CTR_COUNT_FREEZE, ST_CTR_COUNT_DOWN, ST_CTR_COUNT_RST } state_ctr_opcode_t;
localparam ST_CTR_RST_VAL_TINY = 'h1;
localparam ST_CTR_RST_VAL_SMALL = 'h2;
localparam ST_CTR_RST_VAL_MEDIUM = 'h3;
// FREEZE_ADDR_D_M, GOTO_STORAGE, GOTO_GR_GP, GOTO_GRANDPA, GOTO_PARENT, GOTO_LEAF, PLUS_1_D_M, 
typedef enum logic [2:0] {FREEZE_ADDR_D_M, GOTO_STORAGE, GOTO_GR_GP, GOTO_GRANDPA, GOTO_PARENT, GOTO_LEAF, PLUS_1_D_M } data_mem_addr_opcode_t;
typedef enum logic [3:0] {FREEZE_ADDR_D_M_WR, GOTO_STORAGE_WR, GOTO_GR_GP_WR, GOTO_GRANDPA_WR, GOTO_PARENT_WR, GOTO_LEAF_WR, GOTO_PREV_LEAF_WR, PLUS_1_D_M_WR, GOTO_PREV_NODE_LEFT_CHILD_STORAGE_WR } data_mem_wr_addr_opcode_t;

typedef enum logic [3:0] {FREEZE_ADDR_KS_M, PLUS_1_KS_M, GOTO_COMMIT_STORAGE, GOTO_PATH_POS, GOTO_ACC_STORAGE, GOTO_ALPHA_MID} key_sig_mem_addr_opcode_t;

typedef enum logic [2:0] { FREEZE_PARTY_IDX, RST_PARTY_IDX, PARTY_IDX_ADD_1, PARTY_IDX_ADD_2, PARTY_IDX_ADD_4, PARTY_IDX_ADD_8, PARTY_IDX_SH_STEP, PARTY_IDX_GOTO_I_S } party_idx_opcode_t;

typedef enum logic [2:0] { N_IDX_FREEZE, N_IDX_GOTO_TARG_NODE, N_IDX_ADD_1, N_IDX_ADD_2, N_IDX_SUB_1, N_IDX_SH_LEFT, N_IDX_SH_RIGHT, N_IDX_RST} node_idx_opcode_t;

typedef enum logic [1:0] { PATH_LEN_FREEZE, PATH_LEN_RST, PATH_LEN_ADD_1, PATH_LEN_SUB_1 } path_len_opcode_t;
typedef enum logic [2:0] { PATH_POS_FREEZE, PATH_POS_RST, PATH_POS_ADD_1, PATH_POS_SUB_1, PATH_POS_GOTO_LIST_MIN_1 } path_pos_opcode_t;

typedef enum logic [2:0] { TARG_N_IDX_FREEZE, TARG_N_IDX_GOTO_LEAF, TARG_N_IDX_GOTO_NODE_IDX_P1 } targ_node_idx_opcode_t;

typedef enum logic [1:0] { SEARCH_CTR_FREEZE, SEARCH_CTR_RST, SEARCH_CTR_INCR, SEARCH_CTR_DECR } search_ctr_opcode_t;

localparam SEARCH_CTR_MAX       = 'hf; //f
localparam SEARCH_CTR_bits      = $clog2(SEARCH_CTR_MAX+1);
localparam SEARCH_CTR_INIT_DELAY='h5; //4

module aes_FSM_v2 #(
    parameter TAU = `TAU,
    parameter N   = `N,
    parameter TREE_LEAVES           = (`TREE_LEAVES),
    parameter K_EXIT_IDX            = (`K_EXIT_IDX),
    parameter NODE_BITS             = `CLOG2(2*TAU*N-1), // bits needed to represent all node indices of the GGM tree
    parameter STATE_CTR_bits        = $clog2(10),
    parameter NUM_EXPANDS           = 5,
    parameter CTR_BITS              = `CLOG2(NUM_EXPANDS)-1,
    parameter SHAKE_STEP            = 4,
    parameter NODE_IDX_BITS         = $clog2(`TREE_NODES)
)(
    input wire                      rst,
    input wire                      clk,
    
    input wire [1:0]                start,
    
    input wire                      key_unpack_done,
    input wire                      aes_core_ready_in_4,
    
    input wire [7:0]                i_star_in, // Make an automatic small mem to read these
    input wire                      i_star_valid,
    
//    output wire [`DATA_MEM_ADDR_BITS-1:0] addr_out_data_mem_aes,
    output reg [`DATA_MEM_ADDR_BITS-1:0]    data_mem_addr,
    output reg [`DATA_MEM_ADDR_BITS-1:0]    data_mem_wr_addr,
    output reg [`DATA_MEM_ADDR_BITS-1:0]    data_mem_wr_addr_2,
    output reg [`CMT_FIFO_ADDR_BITS-1:0]    cmt_fifo_addr_wr,
    
//    output reg [`DATA_MEM_ADDR_BITS-1:0]  addr_out_key_sig_mem_aes, // was a wire
    output reg [`KEY_SIG_MEM_ADDR_BITS-1:0] key_sig_mem_addr,

    output reg [NODE_IDX_BITS-1:0]    node_index_aes,
    output reg [CTR_BITS-1:0]               ctr,
    output reg [CTR_BITS-1:0]               ctr_alt,
    
    output reg          wren_data_mem,
    output reg          wren_key_sig_mem,
    output reg          read_source, // 1'b0: data_mem / 1'b1: key_sig_mem
    
    output reg         load_salt,
    output reg         load_seed,
    output aes_mode_t  mode_aes,
    output reg         aes_comm_init,
    output reg         aes_comm_next,
//    output reg         rst_sample_regs,

    output reg         load_key_from_res_0,
    output reg         load_key_from_res_1,
    output reg         store_res,
    output reg         share_split_done_out,
    output reg         a_mid_valid_to_a_wrap,
    
    output reg [$clog2(`N)-1:0]   phi_i,
    output reg [$clog2(`TAU)-1:0] mpc_round_aes,
    
    output reg                  party_is_i_star_out,
    output reg                  commit_valid,
    output reg                  sample_valid,
    output reg                  sample_source, // 1'b0: AES, 1'b1: key/sig mem
    output reg                  finalize_aux,
    output reg                  next_verify_round,
    output reg                  init_acc_round_aes,
    
    output reg                  accept_path, // At the end of a given sibling path evaluation, one of these
    output reg                  reject_path, // 2 goes high to signal the result to the top-level control module
    output reg                  done_to_ctrl,
//    output reg                  waiting_for_i_star,
    output reg                  party_is_i_star_to_ctrl,
    output reg                  rst_v_regs,
    
    output wire                 cmt_fifo_wren
); 

/*******************************************/
/* State update & next/prev state tracking */
/*******************************************/
state_t state_aes, next_state_aes, prev_state_aes, next_state_aes_reg;
always_ff @ (posedge clk) state_aes <= rst ?          S_IDLE :
                                       update_state ? next_state_aes : state_aes;

always_ff @ (posedge clk) begin
    if (rst)
        prev_state_aes <= S_IDLE;
    else if (update_state)
        prev_state_aes <= state_aes; // Keep revious state for post-expansion decision taking
end

always_ff @ (posedge clk)                   next_state_aes_reg <= next_state_aes; // Look-ahead at next state
/*********************************************************************************************************************************/

reg [STATE_CTR_bits-1:0] state_ctr, state_ctr_rst_val, state_ctr_rst_val_reg;
reg       state_ctr_timeout;
reg [5:0] state_ctr_timeout_train;

reg [1:0] four_counter;
reg       four_counter_incr, four_counter_decr, four_counter_rst, four_counter_max;
reg       four_counter_bit_sel, four_counter_bit_sel_pip, four_counter_bit_sel_pip_2, four_counter_bit_sel_pip_3;
reg       direct_move_to_next_expand;
reg       goto_prep_GGM;
reg       leave_expand_bridge;
reg       share_split_done;
reg       hold_aes_core_ready, hold_aes_core_ready_next;
reg       incr_ks_mem_addr_s_expand;
reg       a_mid_valid_to_a_wrap_next;

reg [$clog2(`T_OPEN_LIMIT):0]    path_len, path_pos;
reg [$clog2(`T_OPEN_LIMIT)-1:0]  ordered_list_idx_offset;
reg  [`P_O_I_LIST_AD_BITS-1:0]   partial_list_mem_addr;
wire [NODE_IDX_BITS-1:0]         partial_list_mem_dout;
reg                              part_ord_list_wren;
reg                              part_list_dout_is_node_idx;

reg cmt_fifo_incr_addr_wr, cmt_fifo_incr_addr_wr_next;

reg consec_siblings_hidden_flag, raise_consec_siblings_hidden_flag, raise_consec_siblings_hidden_flag_next, lower_consec_siblings_hidden_flag, lower_consec_siblings_hidden_flag_next;
always_ff @ (posedge clk) begin
    raise_consec_siblings_hidden_flag <= rst ? 1'b0 : raise_consec_siblings_hidden_flag_next;
    lower_consec_siblings_hidden_flag <= rst ? 1'b0 : lower_consec_siblings_hidden_flag_next;
    
    if (rst)
        consec_siblings_hidden_flag <= 1'b0;
    else if (lower_consec_siblings_hidden_flag)
        consec_siblings_hidden_flag <= 1'b0;
    else if (raise_consec_siblings_hidden_flag)
        consec_siblings_hidden_flag <= 1'b1;
end

reg  [3:0] exp_parent_case_sel;
always_ff @ (posedge clk) exp_parent_case_sel <= target_node_idx[3:0] & {4{!sub_round_0}};

always_ff @ (posedge clk) begin
// If invalid address: go to a permanent-zero location to take advantage of the
// fact that we're using 1-based indices. -> part_list_dout == node_index_p1 will then always be false
    if (path_pos<ordered_list_idx_offset || path_pos==path_len)
        partial_list_mem_addr <= (`PART_ORD_IDX_LIST_DEPTH-1'b1); 
    else
        partial_list_mem_addr <= (path_pos - ordered_list_idx_offset);
end
state_ctr_opcode_t state_ctr_opc;
aes_mode_t  mode_aes_next;

always_ff @ (posedge clk) begin
    state_ctr_rst_val_reg <= state_ctr_rst_val;
    state_ctr_timeout <= (state_ctr=='h1);
    
    if (rst)
        state_ctr <= 'h0;
    
    else case (state_ctr_opc)
        ST_CTR_COUNT_FREEZE: state_ctr <= state_ctr;
    
        ST_CTR_COUNT_DOWN:  state_ctr <= (state_ctr!='h0) ? (state_ctr-1'b1) : state_ctr;
        
        ST_CTR_COUNT_RST:   state_ctr <= state_ctr_rst_val_reg;
    endcase        
end

// **************************
// Define wires and regs
// ************************
reg start_asserted, start_en, verify;

reg [$clog2(`N)-1:0] party_idx_aes, party_idx_aes_prev;
wire [$clog2(`N)-1:0] i_star_mem_dout;
reg [$clog2(`TAU)-1:0] mpc_round_aes_prev;

reg buff_idx, party_idx_0, party_idx_old_MSB, mpc_round_0, sub_round_0, party_idx_is_i_star, node_in_storage, new_round, next_sub_phase;

reg party_idx_prev_MSB, party_idx_prev_MSB_1C, init_mpc_round, init_mpc_round_next, read_source_next, update_state, update_state_pip, force_state_update_next;
reg aes_init_next, wren_data_mem_next, wren_key_sig_mem_next, aes_core_ready_in_1, aes_core_ready_in_2, aes_core_ready_in_3, last_party_idx, path_len_0;
reg rst_mpc_round, rst_mpc_round_next, commit_buff_idx, switch_commit_buff, rst_idx_S_CLEAN, node_idx_sub_lvl_2;
reg ances_state_next_data_mem_addr;

reg [12:0] aes_ready_train;
reg [6:0]  first_cycles_train;
reg aes_comm_init_pip, aes_comm_next_pip, aes_comm_next_pip_2;

reg exit_node_reached, exit_fake_expand, node_idx_p1_is_2, store_res_next, load_key_from_res_0_next, load_key_from_res_1_next, load_salt_next, load_seed_next, node_idx_is_target, node_idx_is_target_pip, node_idx_is_target_delayed;

reg [$clog2(`TREE_NODES)-1:0] node_index_p1_aes, target_node_idx, state_exit_node;
reg [$clog2(`TREE_NODES)-2:0] parent_idx_aes;

reg [4:0] done_train;

wire v_bit_mem_dout;
reg v_bit_mem_dout_prev, v_bit_mem_dout_prev_2, v_bit_mem_dout_pip;
reg v_bit_mem_din_0, v_bit_mem_din_next_0, v_bit_mem_din_1, v_bit_mem_din_next_1, v_bit_mem_wren_0, v_bit_mem_wren_1, v_bit_mem_wren_next_0, v_bit_mem_wren_next_1, v_mem_clean_exit;

reg update_forced, accept_path_next, reject_path_next, path_too_big, state_ctr_frozen, path_end_reached, searching_node, node_found, node_found_next, searching_node_next, node_ready, node_ready_next;

reg [$clog2(`TREE_NODES+3)-1:0] v_mem_clean_stop_idx;

reg [SEARCH_CTR_bits-1:0] search_ctr;
//reg [$clog2(`LEVEL_K+1)-1:0] start_lvl;
reg par_rev, gp_rev, gr_gp_rev, gr_2_gp_rev;
reg par_rev_next, gp_rev_next, gr_gp_rev_next, gr_2_gp_rev_next;
reg state_is_not_s_done_reg;

reg node_idx_sh_left_lsb, node_idx_sh_left_lsb_next;

data_mem_addr_opcode_t      data_mem_addr_opc;
data_mem_wr_addr_opcode_t   data_mem_wr_addr_opc;
data_mem_wr_addr_opcode_t   data_mem_wr_addr_opc_reg, data_mem_wr_addr_opc_next;
always_ff @ (posedge clk) data_mem_wr_addr_opc_reg <= data_mem_wr_addr_opc_next;
wire debug_wire = wren_data_mem && (data_mem_wr_addr!=data_mem_wr_addr_2);

key_sig_mem_addr_opcode_t   key_sig_mem_addr_opc;
party_idx_opcode_t          party_idx_opc;
party_idx_opcode_t          party_idx_opc_reg;
always_ff @ (posedge clk) party_idx_opc_reg <= rst ? FREEZE_PARTY_IDX : party_idx_opc;
node_idx_opcode_t           node_idx_opc;
path_len_opcode_t           path_len_opc;
path_pos_opcode_t           path_pos_opc;
targ_node_idx_opcode_t      targ_node_idx_opc;
search_ctr_opcode_t         search_ctr_opc;

// **************************
// Update values for regs
// ***********************
always_ff @ (posedge clk) node_index_aes        <= node_index_p1_aes -1'b1;
always_ff @ (posedge clk) parent_idx_aes        <= (node_index_p1_aes>>1) -1'b1;
always_ff @ (posedge clk) mpc_round_0           <= (mpc_round_aes == 'h0);
always_ff @ (posedge clk) sub_round_0           <= (mpc_round_aes == 'h0) && four_counter=='h0;
always_ff @ (posedge clk) party_idx_0           <= (party_idx_aes == 'h0);
always_ff @ (posedge clk) party_idx_is_i_star   <= (party_idx_aes == i_star_mem_dout);
always_ff @ (posedge clk) share_split_done      <= last_party_idx && mpc_round_aes[$clog2(TAU)-1] && four_counter=='h3;
always_ff @ (posedge clk) state_is_not_s_done_reg <= (state_aes!=S_DONE);
always_ff @ (posedge clk) done_train <= rst ? 'h0 : {done_train, state_aes==S_DONE};
always_ff @ (posedge clk) a_mid_valid_to_a_wrap <= a_mid_valid_to_a_wrap_next;
always_ff @ (posedge clk) v_bit_mem_dout_pip <= v_bit_mem_dout;

always_ff @ (posedge clk) begin
    if (rst)
        init_acc_round_aes <= 'h1;
    else if ((state_aes==S_EXPAND) && (ctr=='h1))
        init_acc_round_aes <= (party_idx_aes=='h0);
end

always_ff @ (posedge clk) share_split_done_out  <= rst ? 1'b0 : (((state_aes==S_WAIT_I_STAR) && ~verify) || done_train[4]);

always_ff @ (posedge clk) part_list_dout_is_node_idx <= (partial_list_mem_dout == node_index_p1_aes);
always_ff @ (posedge clk) ances_state_next_data_mem_addr <= (aes_ready_train[4] || first_cycles_train[4]);

always_ff @ (posedge clk) begin
    party_idx_prev_MSB_1C <= party_idx_aes[$clog2(`N)-1];
    
    if (rst)
        party_idx_prev_MSB <= 1'b0;
    else if (party_idx_opc_reg!=FREEZE_PARTY_IDX)
        party_idx_prev_MSB <= party_idx_aes[$clog2(`N)-1];
end

always_ff @ (posedge clk) par_rev       <= par_rev_next;
always_ff @ (posedge clk) gp_rev        <= gp_rev_next;
always_ff @ (posedge clk) gr_gp_rev     <= gr_gp_rev_next;
always_ff @ (posedge clk) gr_2_gp_rev   <= gr_2_gp_rev_next;

always_ff @ (posedge clk) begin
    case (search_ctr_opc)
        SEARCH_CTR_FREEZE:  search_ctr <= search_ctr;
        
        SEARCH_CTR_RST:     search_ctr <= 'h0;
        
        SEARCH_CTR_INCR:    search_ctr <= search_ctr +1'b1;
        
        SEARCH_CTR_DECR:    search_ctr <= search_ctr -1'b1;
    endcase
end

always_ff @ (posedge clk) begin
//    leave_expand_bridge <= (((four_counter!=0 || ~mpc_round_0) && aes_core_ready_in_2) || key_unpack_done);
    leave_expand_bridge <= ((~sub_round_0 && aes_core_ready_in_2) || key_unpack_done);
end

always_ff @ (posedge clk) if (state_aes==S_COPY_NODES_0) ordered_list_idx_offset <= path_pos;

always_ff @ (posedge clk) init_mpc_round <= init_mpc_round_next;
always_ff @ (posedge clk) if (aes_comm_next_pip_2) v_bit_mem_dout_prev_2 <= v_bit_mem_dout_prev;
always_ff @ (posedge clk) if (aes_comm_next_pip_2) v_bit_mem_dout_prev   <= v_bit_mem_dout;

always_ff @ (posedge clk) node_idx_is_target <= node_index_p1_aes[$clog2(`TREE_NODES)-1:1]==target_node_idx[$clog2(`TREE_NODES)-1:1]
                                             || node_index_p1_aes[$clog2(`TREE_NODES)-1:1]==(target_node_idx[$clog2(`TREE_NODES)-1:1]+1'b1); 
// Note: To cover both MODE_GGM and MODE_COMMIT, we don't care about the lsb

always_ff @ (posedge clk) node_idx_is_target_pip<= node_idx_is_target;

always_ff @ (posedge clk) if (aes_core_ready_in_2) node_idx_is_target_delayed <= node_idx_is_target;

always_ff @ (posedge clk) begin
    four_counter_bit_sel_pip_3 <= four_counter_bit_sel_pip_2;
    four_counter_bit_sel_pip_2 <= four_counter_bit_sel_pip;
    four_counter_bit_sel_pip   <= four_counter_bit_sel;
    four_counter_bit_sel       <= target_node_idx[0];
    
    case(four_counter)
        'h1: four_counter_bit_sel <= target_node_idx[1];
        'h2: four_counter_bit_sel <= target_node_idx[2];
        'h3: four_counter_bit_sel <= target_node_idx[3];
    endcase
end

always_ff @ (posedge clk) rst_mpc_round     <= rst_mpc_round_next;
always_ff @ (posedge clk) searching_node    <= (rst) ? 1'b0 : searching_node_next;
always_ff @ (posedge clk) node_ready        <= (rst) ? 1'b0 : node_ready_next;
always_ff @ (posedge clk) node_found        <= (rst) ? 1'b0 : node_found_next;

always_ff @ (posedge clk) node_idx_sub_lvl_2 <= ( node_index_p1_aes[$clog2(`TREE_NODES)-1:2]=='h0 );
always_ff @ (posedge clk) v_bit_mem_wren_0 <= v_bit_mem_wren_next_0;
always_ff @ (posedge clk) v_bit_mem_wren_1 <= v_bit_mem_wren_next_1;
always_ff @ (posedge clk) v_bit_mem_din_0  <= v_bit_mem_din_next_0;
always_ff @ (posedge clk) v_bit_mem_din_1  <= v_bit_mem_din_next_1;

always_ff @ (posedge clk) path_too_big <= (path_len>`T_OPEN_LIMIT);
always_ff @ (posedge clk) accept_path <= accept_path_next;
always_ff @ (posedge clk) reject_path <= reject_path_next;
always_ff @ (posedge clk) state_ctr_frozen <= (state_ctr_opc==ST_CTR_COUNT_FREEZE);

always_ff @ (posedge clk) begin
    direct_move_to_next_expand <= (four_counter[0]==mpc_round_aes[0]);
    four_counter_max <= (four_counter=='h3);
    
    if (aes_core_ready_in_1)
        goto_prep_GGM <= ~direct_move_to_next_expand && last_party_idx;
    
    if (rst)
        four_counter <= 2'h0;
    else if (four_counter_incr)
        four_counter <= four_counter + 1'b1;
    else if (four_counter_decr)
        four_counter <= four_counter - 1'b1;
    else if (four_counter_rst)
        four_counter <= 2'h0;
end

always_ff @ (posedge clk) sample_source <= (verify && prev_state_aes==S_EXPAND && party_idx_is_i_star); // Always AES for now
always_ff @ (posedge clk) begin
    incr_ks_mem_addr_s_expand <= ((state_aes==S_EXPAND     && aes_core_ready_in_1)
                              || (prev_state_aes==S_EXPAND && |aes_ready_train[2:0]));
    
    if  (rst)
        sample_valid  <= 1'b0;
    else
        sample_valid  <=    (prev_state_aes==S_EXPAND || prev_state_aes==S_EXPAND_BRIDGE)
                         && (|aes_ready_train[1:0] || (|aes_ready_train[3:2] && ctr!=((NUM_EXPANDS+1)/2)));
    
    commit_valid <= (prev_state_aes==S_COMMIT || prev_state_aes==S_COMMIT_PREP_GGM) && aes_ready_train[4];
    party_is_i_star_out <= party_idx_is_i_star;
end

always_ff @ (posedge clk) aes_comm_init <= rst ? 1'b0 : aes_init_next;
always_ff @ (posedge clk) read_source   <= read_source_next;
always_ff @ (posedge clk) mode_aes      <= mode_aes_next;
//always_ff @ (posedge clk) store_res     <= store_res_next;
always_ff @ (posedge clk) store_res     <= aes_core_ready_in_1;

always_ff @ (posedge clk) begin
    hold_aes_core_ready <= hold_aes_core_ready_next;
    
    aes_core_ready_in_3 <= rst ? 1'b0 : aes_core_ready_in_4;
    aes_core_ready_in_2 <= rst ? 1'b0 : aes_core_ready_in_3;
    
    if (aes_core_ready_in_2)
        aes_core_ready_in_1   <= 1'b1;
    else if (~hold_aes_core_ready)
        aes_core_ready_in_1   <= 1'b0;
end

always_ff @ (posedge clk) load_key_from_res_0   <= load_key_from_res_0_next;
always_ff @ (posedge clk) load_key_from_res_1   <= load_key_from_res_1_next;
always_ff @ (posedge clk) load_salt             <= load_salt_next;
always_ff @ (posedge clk) load_seed             <= load_seed_next;
always_ff @ (posedge clk) wren_data_mem         <= wren_data_mem_next;
always_ff @ (posedge clk) wren_key_sig_mem      <= wren_key_sig_mem_next;
always_ff @ (posedge clk) node_idx_p1_is_2      <= (node_index_p1_aes=='h2);
always_ff @ (posedge clk) exit_node_reached     <= (node_index_p1_aes==(`K_EXIT_IDX+1+1));
always_ff @ (posedge clk) exit_fake_expand      <= (node_index_p1_aes==(`TAU*`N-1));
always_ff @ (posedge clk) last_party_idx        <= ({party_idx_aes[$clog2(`N)-1:3], (party_idx_aes[2] || mode_aes==MODE_GGM)}==(N/4-1));
//always_ff @ (posedge clk) rst_idx_S_CLEAN       <= (state_aes!=S_CLEAN_V_MEM || update_state);
always_ff @ (posedge clk) rst_idx_S_CLEAN       <= (state_aes!=S_CLEAN_V_MEM);

always_ff @ (posedge clk) if (update_state) party_idx_old_MSB <= party_idx_aes[$clog2(`N)-1];
always_ff @ (posedge clk) new_round <= party_idx_old_MSB && !party_idx_aes[$clog2(`N)-1]; // When party_idx MSB flips -> round changes

always_ff @ (posedge clk) next_sub_phase <= /*direct_move_to_next_expand &&*/ mpc_round_aes[$clog2(TAU)-1] && last_party_idx;

//always_ff @ (posedge clk) if (aes_core_ready_in_1) party_idx_aes_prev <= party_idx_aes;
always_ff @ (posedge clk) party_idx_aes_prev <= party_idx_aes-'h4;

always_ff @ (posedge clk) aes_ready_train         <= aes_core_ready_in_1 ? 'h1 : {aes_ready_train, 1'b0};
always_ff @ (posedge clk) state_ctr_timeout_train <= {state_ctr_timeout_train, state_ctr_timeout};

always_ff @ (posedge clk) node_in_storage       <= (node_index_p1_aes < (`TREE_LEAVES/(1<<(`LEVEL_K-1)))); // L1 threshold for K==4 -> 'd2176
always_ff @ (posedge clk) phi_i                 <= party_idx_aes ^ ((verify && ~party_idx_is_i_star) ? i_star_mem_dout : 'h0);
always_ff @ (posedge clk) path_end_reached      <= path_pos==path_len;

always_ff @ (posedge clk) update_state        <= rst ? 'h0 : ((aes_core_ready_in_2 && state_aes!=S_SEARCH_NEXT_NODE) || force_state_update_next);
always_ff @ (posedge clk) update_state_pip    <= update_state;
always_ff @ (posedge clk) update_forced       <= force_state_update_next;
//always_ff @ (posedge clk) first_cycles_train  <= {first_cycles_train, update_forced};
always_ff @ (posedge clk) first_cycles_train  <= update_forced ? 'h1: {first_cycles_train, 1'b0};
always_ff @ (posedge clk) path_len_0          <= (path_len==0);

always_ff @ (posedge clk) finalize_aux  <= ~verify && (((state_aes==S_COMMIT || state_aes==S_COMMIT_PREP_GGM) && party_idx_prev_MSB_1C && ~party_idx_aes[$clog2(`N)-1])
                                        || (rst_idx_S_CLEAN && state_aes==S_CLEAN_V_MEM) || (state_is_not_s_done_reg && state_aes==S_DONE));

always_ff @ (posedge clk) next_verify_round  <= verify && (((state_aes==S_COMMIT || state_aes==S_COMMIT_PREP_GGM) && party_idx_prev_MSB_1C && ~party_idx_aes[$clog2(`N)-1])
                                        || (rst_idx_S_CLEAN && state_aes==S_CLEAN_V_MEM) || (state_is_not_s_done_reg && state_aes==S_DONE));

always_ff @ (posedge clk) begin
//    v_mem_clean_exit <= (node_index_p1_aes==v_mem_clean_stop_idx);
    v_mem_clean_exit <= (node_index_p1_aes==`TREE_NODES-1);

//    if (rst)
//        v_mem_clean_stop_idx <= `TREE_NODES-1; // TODO: decide if I want to optimize for cycle count OR simply make it a constant
end

always_ff @ (posedge clk) begin
    if (rst)
        switch_commit_buff <= 1'b0;
    else
        switch_commit_buff <= ((prev_state_aes==S_COMMIT || prev_state_aes==S_COMMIT_PREP_GGM) && aes_ready_train[0]);
    
    if (rst)
        commit_buff_idx <= 1'b0;
    else if (switch_commit_buff)
        commit_buff_idx <= ~commit_buff_idx;
end

always @ (posedge clk) begin
        aes_comm_init_pip     <= aes_comm_init;
        aes_comm_next         <= aes_comm_init_pip;
        aes_comm_next_pip     <= aes_comm_next;
        aes_comm_next_pip_2   <= aes_comm_next_pip;
end

always_ff @ (posedge clk) begin // update path_len
//    if (rst)
//        path_len <= 'h0;
    
//    else 
    case (path_len_opc)
        PATH_LEN_FREEZE:    path_len <= path_len;
        PATH_LEN_RST:       path_len <= 'h0;
        PATH_LEN_ADD_1:     path_len <= path_len+1'b1;
        PATH_LEN_SUB_1:     path_len <= path_len-1'b1;
    endcase // path_len_opc
end

always_ff @ (posedge clk) begin // update path_pos
//    if (rst)
//        path_pos <= 'h0;
    
//    else 
    case (path_pos_opc)
        PATH_POS_FREEZE:    path_pos <= path_pos;
        PATH_POS_RST:       path_pos <= 'h0;
        PATH_POS_ADD_1:     path_pos <= (path_pos==path_len) ? 'h0 : (path_pos+1'b1);
        PATH_POS_SUB_1:     path_pos <= (path_pos=='h0) ? path_len : path_pos-1'b1;
//        PATH_POS_GOTO_LIST: path_pos <= {1'b0, ordered_list_idx_offset};
        PATH_POS_GOTO_LIST_MIN_1: path_pos <= {1'b0, (ordered_list_idx_offset-1'b1)};
    endcase // path_pos_opc
end

always_ff @ (posedge clk) begin // node_idx_p1 update
    if (rst)
        node_index_p1_aes <= 'h1;
    
    else case (node_idx_opc)
        N_IDX_FREEZE:           node_index_p1_aes <= node_index_p1_aes;
        N_IDX_GOTO_TARG_NODE:   node_index_p1_aes <= target_node_idx;
        N_IDX_ADD_1:            node_index_p1_aes <= node_index_p1_aes +1'b1;
        N_IDX_ADD_2:            node_index_p1_aes <= node_index_p1_aes +2'h2;
        N_IDX_SUB_1:            node_index_p1_aes <= node_index_p1_aes -1'b1;
//        N_IDX_ADD_2_TAU:        node_index_p1_aes <= node_index_p1_aes +(TAU<<1); //+TAU/2 + (!target_node_idx[0]); // TODO: check if correct
//        N_IDX_ADD_4_TAU:        node_index_p1_aes <= node_index_p1_aes +(TAU<<2); //+TAU;
        N_IDX_SH_LEFT:          node_index_p1_aes <= {node_index_p1_aes, node_idx_sh_left_lsb};
//        N_IDX_SH_LEFT:          node_index_p1_aes <= node_index_p1_aes<<1;
        N_IDX_SH_RIGHT:         node_index_p1_aes <= node_index_p1_aes>>1;
        N_IDX_RST:              node_index_p1_aes <= 2'b10;
    endcase
    
    node_idx_sh_left_lsb <= node_idx_sh_left_lsb_next;
end

always_ff @ (posedge clk) begin
    case (targ_node_idx_opc) // TARG_N_IDX_FREEZE, TARG_N_IDX_GOTO_LEAF, TARG_N_IDX_GOTO_NODE_IDX_P1
    TARG_N_IDX_FREEZE:      target_node_idx <= target_node_idx;
    
    TARG_N_IDX_GOTO_LEAF:   target_node_idx <= mpc_round_aes + party_idx_aes + {party_idx_aes, 4'b0} + TREE_LEAVES;
    
    TARG_N_IDX_GOTO_NODE_IDX_P1: target_node_idx <= node_index_p1_aes;
    endcase
end

//always_ff @ (posedge clk) begin
//    if (rst)
//        state_exit_node <= `K_EXIT_IDX+1+1;
//end

always_ff @ (posedge clk) begin // Set start_en and verify regs
    if (rst) begin
        start_asserted <= 1'b0;
        start_en <= 1'b0;
        verify <= 1'b0;
        
    end else begin
        if (start[`START_BIT])
            start_asserted <= 1'b1;
            
        if (start_asserted)
            start_en <= 1'b0;
        else
            start_en <= start[`START_BIT];
            
        if (start[`VERIFY_MODE_BIT] && state_aes==S_IDLE)  
            verify <= 1'b1;
    end
end

always @ (posedge clk) begin // mpc_round & buff_idx update
    if (rst) begin
        mpc_round_aes <= 'h0;
        buff_idx <= 1'b0;
    
    end else if (init_mpc_round) begin
        mpc_round_aes <= (mpc_round_aes==TAU-1'b1) ? 'h0 : (mpc_round_aes+1'b1);
        buff_idx <= ~buff_idx;
    end else if (rst_mpc_round)
        mpc_round_aes <= 'h0;
        
    if (aes_ready_train[1])
        mpc_round_aes_prev <= mpc_round_aes;
end

always_ff @ (posedge clk) begin // party_idx update
    if (rst)
        party_idx_aes <= 'h0;
    
    else case (party_idx_opc_reg)
        FREEZE_PARTY_IDX:   party_idx_aes <= party_idx_aes;
        RST_PARTY_IDX:      party_idx_aes <= 'h0;
        PARTY_IDX_ADD_1:    party_idx_aes <= party_idx_aes +1'b1;
        PARTY_IDX_ADD_2:    party_idx_aes <= party_idx_aes +2'h2;
        PARTY_IDX_ADD_4:    party_idx_aes <= party_idx_aes +3'h4;
        PARTY_IDX_ADD_8:    party_idx_aes <= party_idx_aes +4'h8;
        PARTY_IDX_SH_STEP:  party_idx_aes <= party_idx_aes +4'h4 +next_sub_phase;
        PARTY_IDX_GOTO_I_S: party_idx_aes <= i_star_mem_dout;
    endcase
end

always_ff @ (posedge clk) begin // update data_mem_addr
    if (rst)
        data_mem_addr <= `DATA_MEM_ROOT_SEED_ADDR;
    
    else case (data_mem_addr_opc)
        FREEZE_ADDR_D_M: data_mem_addr <= data_mem_addr;
        
        GOTO_STORAGE:   data_mem_addr <= {node_index_aes, {$clog2(`NODE_WORDS){1'b0}}}; // Go to permanent storage for current idx
//        GOTO_STORAGE_FROM_CHILD_IDX:   data_mem_addr <= {parent_idx_aes, {$clog2(`NODE_WORDS){1'b0}}}; // Go to permanent storage for current idx
        GOTO_GR_GP:     data_mem_addr <= ((`K_EXIT_IDX<<2) + 6) + {party_idx_aes[$clog2(`N)-1:2], {$clog2(`NODE_WORDS){1'b0}}}; // Go to the runtime storage for the current party idx
        
        GOTO_GRANDPA:   data_mem_addr <= ((`K_EXIT_IDX<<2) + 6) + {(party_idx_aes[$clog2(`N)-1:2] + N/4), {$clog2(`NODE_WORDS){1'b0}}};
        
        GOTO_PARENT:    data_mem_addr <= ((`K_EXIT_IDX<<2) + 6) + {(party_idx_aes[$clog2(`N)-1:2] + N/2), {$clog2(`NODE_WORDS){1'b0}}};
        
        GOTO_LEAF:      data_mem_addr <= ((`K_EXIT_IDX<<2) + 6) + {((party_idx_aes[$clog2(`N)-1:2]<<1) + (3*N/4) + buff_idx), {$clog2(`NODE_WORDS){1'b0}}}; 
//        GOTO_PREV_LEAF: data_mem_addr <= ((`K_EXIT_IDX<<2) + 6) + {((party_idx_aes_prev[$clog2(`N)-1:2]<<1) + (3*N/4) + buff_idx), {$clog2(`NODE_WORDS){1'b0}}}; // Note: uses party_idx_prev since the current one is a bit ahead
        PLUS_1_D_M:     data_mem_addr <= data_mem_addr +1'b1;
//        NEXT_BUFF:      data_mem_addr <= buff_idx ? (data_mem_addr-2'h3) : (data_mem_addr+1'h1);  // Note: Can merge with the one below?
//        GOTO_PREV_NODE_LEFT_CHILD_STORAGE: data_mem_addr <= {({node_index_p1_aes, 1'b0} -2'h3), {($clog2(`NODE_WORDS)){1'b0}}};
//        GOTO_COMMIT_BUFF:    data_mem_addr <= ((`K_EXIT_IDX<<2) + 6) + {((5*N/4) + {commit_buff_idx, 1'b0}), {$clog2(`NODE_WORDS){1'b0}}};
    endcase
end

always_ff @ (posedge clk) begin // update data_mem_wr_addr
    case (data_mem_wr_addr_opc_reg)
        FREEZE_ADDR_D_M_WR: data_mem_wr_addr <= data_mem_wr_addr;
        
        GOTO_STORAGE_WR:   data_mem_wr_addr <= {node_index_aes, {$clog2(`NODE_WORDS){1'b0}}}; // Go to permanent storage for current idx
//        GOTO_STORAGE_FROM_CHILD_IDX_WR:   data_mem_wr_addr <= {parent_idx_aes, {$clog2(`NODE_WORDS){1'b0}}}; // Go to permanent storage for current idx
        GOTO_GR_GP_WR:     data_mem_wr_addr <= ((`K_EXIT_IDX<<2) + 6) + {party_idx_aes[$clog2(`N)-1:2], {$clog2(`NODE_WORDS){1'b0}}}; // Go to the runtime storage for the current party idx
        
        GOTO_GRANDPA_WR:   data_mem_wr_addr <= ((`K_EXIT_IDX<<2) + 6) + {(party_idx_aes[$clog2(`N)-1:2] + N/4), {$clog2(`NODE_WORDS){1'b0}}};
        
        GOTO_PARENT_WR:    data_mem_wr_addr <= ((`K_EXIT_IDX<<2) + 6) + {(party_idx_aes[$clog2(`N)-1:2] + N/2), {$clog2(`NODE_WORDS){1'b0}}};
        
        GOTO_LEAF_WR:      data_mem_wr_addr <= ((`K_EXIT_IDX<<2) + 6) + {((party_idx_aes[$clog2(`N)-1:2]<<1) + (3*N/4) + buff_idx), {$clog2(`NODE_WORDS){1'b0}}}; 
        
        GOTO_PREV_LEAF_WR: data_mem_wr_addr <= ((`K_EXIT_IDX<<2) + 6) + {((party_idx_aes_prev[$clog2(`N)-1:2]<<1) + (3*N/4) + buff_idx), {$clog2(`NODE_WORDS){1'b0}}}; // Note: uses party_idx_prev since the current one is a bit ahead
        
        PLUS_1_D_M_WR:     data_mem_wr_addr <= data_mem_wr_addr +1'b1;
        
        GOTO_PREV_NODE_LEFT_CHILD_STORAGE_WR: data_mem_wr_addr <= {({node_index_p1_aes, 1'b0} -2'h3), {($clog2(`NODE_WORDS)){1'b0}}};
    endcase
    case (data_mem_wr_addr_opc)
        FREEZE_ADDR_D_M_WR: data_mem_wr_addr_2 <= data_mem_wr_addr_2;
        
        GOTO_STORAGE_WR:   data_mem_wr_addr_2 <= {node_index_aes, {$clog2(`NODE_WORDS){1'b0}}}; // Go to permanent storage for current idx
//        GOTO_STORAGE_FROM_CHILD_IDX_WR:   data_mem_wr_addr <= {parent_idx_aes, {$clog2(`NODE_WORDS){1'b0}}}; // Go to permanent storage for current idx
        GOTO_GR_GP_WR:     data_mem_wr_addr_2 <= ((`K_EXIT_IDX<<2) + 6) + {party_idx_aes[$clog2(`N)-1:2], {$clog2(`NODE_WORDS){1'b0}}}; // Go to the runtime storage for the current party idx
        
        GOTO_GRANDPA_WR:   data_mem_wr_addr_2 <= ((`K_EXIT_IDX<<2) + 6) + {(party_idx_aes[$clog2(`N)-1:2] + N/4), {$clog2(`NODE_WORDS){1'b0}}};
        
        GOTO_PARENT_WR:    data_mem_wr_addr_2 <= ((`K_EXIT_IDX<<2) + 6) + {(party_idx_aes[$clog2(`N)-1:2] + N/2), {$clog2(`NODE_WORDS){1'b0}}};
        
        GOTO_LEAF_WR:      data_mem_wr_addr_2 <= ((`K_EXIT_IDX<<2) + 6) + {((party_idx_aes[$clog2(`N)-1:2]<<1) + (3*N/4) + buff_idx), {$clog2(`NODE_WORDS){1'b0}}}; 
        
        GOTO_PREV_LEAF_WR: data_mem_wr_addr_2 <= ((`K_EXIT_IDX<<2) + 6) + {((party_idx_aes_prev[$clog2(`N)-1:2]<<1) + (3*N/4) + buff_idx), {$clog2(`NODE_WORDS){1'b0}}}; // Note: uses party_idx_prev since the current one is a bit ahead
        
        PLUS_1_D_M_WR:     data_mem_wr_addr_2 <= data_mem_wr_addr_2 +1'b1;
        
        GOTO_PREV_NODE_LEFT_CHILD_STORAGE_WR: data_mem_wr_addr_2 <= {({node_index_p1_aes, 1'b0} -2'h3), {($clog2(`NODE_WORDS)){1'b0}}};
    endcase
end

always_ff @ (posedge clk) begin // update key_sig_mem_addr
    if (rst)
        key_sig_mem_addr <= `KEY_SIG_MEM_SALT_ADDR;
    
    else case (key_sig_mem_addr_opc)
        FREEZE_ADDR_KS_M:       key_sig_mem_addr <= key_sig_mem_addr;
        
        PLUS_1_KS_M:            key_sig_mem_addr <= key_sig_mem_addr +1'b1;
        
        GOTO_PATH_POS:          key_sig_mem_addr <= {path_pos, {$clog2(`NODE_WORDS){1'b0}}} + `SIBL_PATH_ADDR;
        
        GOTO_COMMIT_STORAGE:    key_sig_mem_addr <= (`KEY_SIG_MEM_COMMIT_ADDR+(mpc_round_aes_prev<<2));
        
        GOTO_ACC_STORAGE:       key_sig_mem_addr <= (`KEY_SIG_MEM_ACC_ADDR + (mpc_round_aes<<3));
        
        GOTO_ALPHA_MID:         key_sig_mem_addr <= (`ALPHA_MID_0_ADDR + (mpc_round_aes<<2)); // +2*`M_PARAM_RHO * mpc_round
    endcase
end

// *******************
// Control logic
//******************
always_comb begin
    party_idx_opc       = FREEZE_PARTY_IDX;
    read_source_next    = 1'b0;
    state_ctr_opc       = state_ctr_timeout ? ST_CTR_COUNT_RST : ST_CTR_COUNT_DOWN;
    state_ctr_rst_val   = ST_CTR_RST_VAL_SMALL;
    
    aes_init_next       = 1'b0;
    wren_data_mem_next   = 1'b0;
    wren_key_sig_mem_next = 1'b0;
    
    force_state_update_next = 1'b0;
    
    store_res_next = 1'b0;
    node_idx_opc = N_IDX_FREEZE;
    node_idx_sh_left_lsb_next = 1'b0;
    
    mode_aes_next = MODE_GGM;
    
    load_key_from_res_0_next = 1'b0;
    load_key_from_res_1_next = 1'b0;
    
    load_seed_next = 1'b0;
    load_salt_next = 1'b0;
    
    data_mem_addr_opc       = FREEZE_ADDR_D_M;
    data_mem_wr_addr_opc    = FREEZE_ADDR_D_M_WR;
    data_mem_wr_addr_opc_next    = FREEZE_ADDR_D_M_WR;
    key_sig_mem_addr_opc    = FREEZE_ADDR_KS_M;
    
    init_mpc_round_next = 1'b0;
    rst_mpc_round_next  = 1'b0;
    
    v_bit_mem_wren_next_0 = 1'b0;
    v_bit_mem_wren_next_1 = 1'b0;
    v_bit_mem_din_next_0  = 1'b0;
    v_bit_mem_din_next_1  = 1'b0;
    
    path_len_opc = PATH_LEN_FREEZE;
    path_pos_opc = PATH_POS_FREEZE;
    
    accept_path_next = 1'b0;
    reject_path_next = 1'b0;
    
    targ_node_idx_opc = TARG_N_IDX_GOTO_LEAF;
    
    searching_node_next = searching_node;
    node_found_next     = node_found;
    node_ready_next     = node_ready;
    
    four_counter_incr   = 1'b0;
    four_counter_decr   = 1'b0;
    four_counter_rst    = 1'b0;
    
    part_ord_list_wren  = 1'b0;
    
    search_ctr_opc = SEARCH_CTR_RST;
    
    par_rev_next    = par_rev;
    gp_rev_next     = gp_rev;
    gr_gp_rev_next  = gr_gp_rev;
    gr_2_gp_rev_next= gr_2_gp_rev;
    
    hold_aes_core_ready_next = 1'b0;
    raise_consec_siblings_hidden_flag_next = 1'b0;
    lower_consec_siblings_hidden_flag_next = 1'b0;
    
    a_mid_valid_to_a_wrap_next = 1'b0;
    cmt_fifo_incr_addr_wr_next = 1'b0;
    
// **************************************************************************
    case (state_aes)
        S_IDLE: begin
            force_state_update_next = state_ctr_timeout;
            
            if (start_en) begin
                state_ctr_opc = ST_CTR_COUNT_RST;
            end
        end
        
// ************************************************************************** 
        S_LOAD_SALT: begin
            state_ctr_rst_val       = ST_CTR_RST_VAL_SMALL;
            force_state_update_next = state_ctr_timeout;
            load_seed_next          = ~update_state;
            load_salt_next          = ~update_state;
            data_mem_addr_opc       = PLUS_1_D_M;
            key_sig_mem_addr_opc    = PLUS_1_KS_M;
        end
     
// ************************************************************************** 
        S_ARRAY_EXPAND: begin
            state_ctr_opc        = ST_CTR_COUNT_DOWN;
            
            if (aes_core_ready_in_1) begin
                store_res_next          = 1'b1;
                load_key_from_res_0_next= node_idx_p1_is_2;
            end
            
            if (aes_comm_next_pip) // Was aes_comm_next
                node_idx_opc = N_IDX_ADD_1;
            
            if (|aes_ready_train[6:5]) // Verification only
                node_idx_opc = N_IDX_SUB_1;
            if (aes_ready_train[7]) begin
                node_idx_opc = N_IDX_SH_LEFT;
            end else if (aes_ready_train[8]) begin
                node_idx_opc = N_IDX_SH_RIGHT;
            end else if (aes_ready_train[9]) begin
                node_idx_opc = N_IDX_ADD_2;
            end
                
            if (verify && v_bit_mem_dout_prev_2) begin // Verification only
//                if (|aes_ready_train[6:5])
//                    node_idx_opc = N_IDX_SUB_1;
                
                if (aes_ready_train[7]) begin
                    v_bit_mem_wren_next_0 = 1'b1;
                    v_bit_mem_wren_next_1 = 1'b1;
                    v_bit_mem_din_next_0  = 1'b1;
                    v_bit_mem_din_next_1  = 1'b1;
//                    node_idx_opc = N_IDX_SH_LEFT;
//                end else if (aes_ready_train[8]) begin
//                    node_idx_opc = N_IDX_SH_RIGHT;
//                end else if (aes_ready_train[9]) begin
//                    node_idx_opc = N_IDX_ADD_2;
                end
            end
            
//            if (aes_ready_train[0]) begin
            if (aes_core_ready_in_2)
                data_mem_wr_addr_opc_next = GOTO_PREV_NODE_LEFT_CHILD_STORAGE_WR; // Go to store address
            if (aes_core_ready_in_1) begin
//                data_mem_addr_opc = GOTO_PREV_NODE_LEFT_CHILD_STORAGE; // Go to store address
                data_mem_wr_addr_opc = GOTO_PREV_NODE_LEFT_CHILD_STORAGE_WR; // Go to store address
            end
            
            if (aes_ready_train[5]) begin
                data_mem_addr_opc = GOTO_STORAGE;
            end else if (aes_ready_train[6]) begin
                data_mem_addr_opc = PLUS_1_D_M;
            end
            
            if (|aes_ready_train[7:6]) begin
                load_seed_next = 1'b1;
            end
            
            aes_init_next   = (state_ctr_timeout || (aes_core_ready_in_1 && ~(verify && exit_node_reached)));
        end // S_ARRAY_EXPAND
        
// ************************************************************************** 
        S_EXP_STORED_N: begin // When storing for the previous round (mpc_round_0), we can take into account that
        // the leaf_node LSB always switches. We distinguish 2 cases: mpc_round==0 and mpc_round!=0
            if (aes_core_ready_in_1) begin
                aes_init_next            = 1'b1;
                store_res_next           = 1'b1;
                
                if (target_node_idx[3])
                    load_key_from_res_1_next = 1'b1;
                else
                    load_key_from_res_0_next = 1'b1;
            end
            
            node_idx_sh_left_lsb_next = target_node_idx[3];
            
            if (aes_comm_next)
                node_idx_opc = N_IDX_SH_LEFT;
//            else if (aes_comm_next_pip && target_node_idx[3])
//                node_idx_opc = N_IDX_ADD_1;
        
//            if (ances_state_next_data_mem_addr)
//                data_mem_addr_opc = GOTO_GR_GP;
        
            if (ances_state_next_data_mem_addr)
                data_mem_wr_addr_opc_next = GOTO_GR_GP_WR;
            if (ances_state_next_data_mem_addr)
                data_mem_wr_addr_opc = GOTO_GR_GP_WR;
        end // S_EXP_STORED_N
        
// ************************************************************************** 
        S_EXP_GREAT_GP: begin
            if (aes_core_ready_in_1) begin
                aes_init_next            = 1'b1;
                store_res_next           = 1'b1;
                
                if (target_node_idx[2])
                    load_key_from_res_1_next = 1'b1;
                else
                    load_key_from_res_0_next = 1'b1;
            end
            
            node_idx_sh_left_lsb_next = target_node_idx[2];
            
            if (aes_comm_next)
                node_idx_opc = N_IDX_SH_LEFT;
//            else if (aes_comm_next_pip && target_node_idx[2])
//                node_idx_opc = N_IDX_ADD_1;
            
//            if (aes_ready_train[6])
//            if (ances_state_next_data_mem_addr)
//                data_mem_addr_opc = GOTO_GRANDPA;
            if (ances_state_next_data_mem_addr)
                data_mem_wr_addr_opc_next = GOTO_GRANDPA_WR;
            if (ances_state_next_data_mem_addr)
                data_mem_wr_addr_opc = GOTO_GRANDPA_WR;
        end // S_EXP_GREAT_GP
        
// **************************************************************************        
        S_EXP_GP: begin
            if (aes_core_ready_in_1) begin
                aes_init_next            = 1'b1;
                store_res_next           = 1'b1;
                
                if (target_node_idx[1]) // NOTE: first MPC round "exception" case
                    load_key_from_res_1_next = 1'b1;
                else
                    load_key_from_res_0_next = 1'b1;
            end
            
            node_idx_sh_left_lsb_next = target_node_idx[1];
            
            if (aes_comm_next)
                node_idx_opc = N_IDX_SH_LEFT;
//            else if (aes_comm_next_pip && target_node_idx[1])
//                node_idx_opc = N_IDX_ADD_1;
            
//            if (aes_ready_train[5]) // Prep the address for the next state
            if (ances_state_next_data_mem_addr)
                data_mem_wr_addr_opc_next = GOTO_PARENT_WR;
            if (ances_state_next_data_mem_addr)
                data_mem_wr_addr_opc = GOTO_PARENT_WR;
                    
//            if (ances_state_next_data_mem_addr)
//                data_mem_addr_opc = GOTO_PARENT;
                    
        end // S_EXP_GP

// **************************************************************************        
        S_EXPAND_BRIDGE: begin
//            mode_aes_next = MODE_EXP_LEAF; // update_mode
//            node_idx_opc  = N_IDX_GOTO_TARG_NODE;
            
            force_state_update_next = key_unpack_done;
            
//            if (leave_expand_bridge) begin
            if (update_state) begin
                aes_init_next           = 1'b1;
                store_res_next          = 1'b1;
//                node_idx_opc            = N_IDX_ADD_1; // Overwrites the above just before we leave the state
            end
        end

// **************************************************************************        
        S_EXPAND: begin
            mode_aes_next = MODE_EXP_LEAF; // update_mode
            
            if (aes_core_ready_in_1)
                node_idx_opc  = N_IDX_ADD_1;
            else
                node_idx_opc  = N_IDX_GOTO_TARG_NODE;
            
            
            if (aes_core_ready_in_1) begin
                aes_init_next           = 1'b1;
                store_res_next          = 1'b1;
            end
            if (prev_state_aes!=S_EXPAND) begin
                if (aes_core_ready_in_3) begin
                    key_sig_mem_addr_opc    = GOTO_ALPHA_MID;
                    if (verify)
                        a_mid_valid_to_a_wrap_next = 1'b1;
                end
                else if (aes_core_ready_in_2)
                    key_sig_mem_addr_opc    = PLUS_1_KS_M;
                else if (aes_core_ready_in_1)
                    key_sig_mem_addr_opc    = GOTO_ACC_STORAGE;
            end
        end

// **************************************************************************         
        S_COMMIT: begin
            mode_aes_next = MODE_COMMIT; // update_mode
            
            if (aes_ready_train[0] && last_party_idx)
                init_mpc_round_next = 1;
                
            if (aes_ready_train[1]) begin
                party_idx_opc     = PARTY_IDX_SH_STEP;
                four_counter_incr = next_sub_phase;
            end
            
//            if (aes_ready_train[4])
//                node_idx_opc = N_IDX_GOTO_TARG_NODE;
//            else if (aes_ready_train[5])
//                node_idx_opc = N_IDX_ADD_1;
            
            if (aes_ready_train[5])
                data_mem_addr_opc = GOTO_LEAF;
            else if (aes_ready_train[6])
                data_mem_addr_opc = PLUS_1_D_M;
//            else if (aes_ready_train[7])
//                data_mem_addr_opc = GOTO_COMMIT_BUFF;
            
            if (|aes_ready_train[7:6])
                load_seed_next=1;
            
            if (aes_core_ready_in_1) begin
                aes_init_next           = 1'b1;
                store_res_next          = 1'b1;
            end
        end // S_COMMIT

// **************************************************************************        
        S_COMMIT_PREP_GGM: begin
            mode_aes_next = MODE_COMMIT; // update_mode
            
//            if (aes_ready_train[0] /*&& ~mpc_round_aes[$clog2(TAU)-1]*/) begin
            if (aes_ready_train[0] && ~share_split_done) begin
                init_mpc_round_next = 1'b1;
                party_idx_opc = PARTY_IDX_SH_STEP;
//                party_idx_opc = PARTY_IDX_ADD_1;
            end
            
            if (aes_ready_train[0] && next_sub_phase && ~share_split_done)
                four_counter_incr = 1'b1;
            
//            if (aes_ready_train[4])
//                node_idx_opc = N_IDX_GOTO_TARG_NODE;
            
//            if (aes_ready_train[2] && mpc_round_aes[0])
//                party_idx_opc = PARTY_IDX_ADD_1; // Add another one
                
            if (aes_core_ready_in_1) begin
                if (next_state_aes_reg!=S_CLEAN_V_MEM && !verify)
                    aes_init_next           = 1'b1;
                store_res_next          = 1'b1;
            end
            
            // Node_idx updates:
            if (aes_ready_train[4])
                node_idx_opc = N_IDX_GOTO_TARG_NODE;
            else if (aes_ready_train[5])
                node_idx_opc = N_IDX_SH_RIGHT;
            else if (~target_node_idx[1] && aes_ready_train[6])
                node_idx_opc = N_IDX_SH_RIGHT;
            else if (~target_node_idx[1] && ~target_node_idx[2] && aes_ready_train[7])
                node_idx_opc = N_IDX_SH_RIGHT;
            else if (~target_node_idx[1] && ~target_node_idx[2] && ~target_node_idx[3] && aes_ready_train[8])
                node_idx_opc = N_IDX_SH_RIGHT;
                
            // Here we need something smart to know where to go next:
            if (target_node_idx[1]) begin // We have the parent
                if (aes_ready_train[4]) begin
                    data_mem_addr_opc = GOTO_PARENT;
                end
                    
            end else if (target_node_idx[2]) begin // We have the grandparent
                if (aes_ready_train[4]) begin
                    data_mem_addr_opc = GOTO_GRANDPA;
                end
                
            end else if (target_node_idx[3]) begin // We have the great grandparent
                if (aes_ready_train[4]) begin
                    data_mem_addr_opc = GOTO_GR_GP;
                end
                
            end else begin // We need to start from the node storage GOTO_STORAGE
                if (aes_ready_train[10])
                    data_mem_addr_opc = GOTO_STORAGE;
            end // Find-where-to-go-next if-else
            
            if (aes_ready_train[11]) // Time to store
                data_mem_addr_opc = PLUS_1_D_M;
//            else if (aes_ready_train[12])
//                data_mem_addr_opc = GOTO_COMMIT_BUFF;
                    
            if (|aes_ready_train[12:11])
                    load_seed_next=1;
        end // S_COMMIT_PREP_GGM

// **************************************************************************
        S_EXP_PARENT: begin // Find where to go next, then load seed, then prep the addr for the next state to store the leaves
            if (aes_core_ready_in_1) begin
                aes_init_next           = !(sub_round_0 && new_round);
                store_res_next          = 1'b1;
            end
            
            if (aes_ready_train[0]) begin
                party_idx_opc = PARTY_IDX_ADD_4;
//                party_idx_opc = PARTY_IDX_ADD_2;
            end
            
            if (aes_ready_train[4])
                node_idx_opc = N_IDX_GOTO_TARG_NODE;
                
            // Here we need something smart to know where to go next:
             if (exp_parent_case_sel[1]) begin // We have the parent
                if (aes_ready_train[5]) begin
                    data_mem_addr_opc = GOTO_PARENT;
                    node_idx_opc = N_IDX_SH_RIGHT;
                end
                    
            end else if (exp_parent_case_sel[2]) begin // We have the grandparent
                if (aes_ready_train[5]) begin
                    data_mem_addr_opc = GOTO_GRANDPA;
                    node_idx_opc = N_IDX_SH_RIGHT;
                end else if (aes_ready_train[6])
                    node_idx_opc = N_IDX_SH_RIGHT;
                
            end else if (exp_parent_case_sel[3]) begin // We have the great grandparent
                if (aes_ready_train[5]) begin
                    data_mem_addr_opc = GOTO_GR_GP;
                    node_idx_opc = N_IDX_SH_RIGHT;
                end else if (aes_ready_train[6]) begin
                    node_idx_opc = N_IDX_SH_RIGHT;
                end else if (aes_ready_train[7])
                    node_idx_opc = N_IDX_SH_RIGHT;
                
            end else begin // We need to start from the node storage GOTO_STORAGE
                if (aes_ready_train[5]) begin
                    node_idx_opc = N_IDX_SH_RIGHT;
                end else if (aes_ready_train[6]) begin
                    node_idx_opc = N_IDX_SH_RIGHT;
                end else if (aes_ready_train[7]) begin
                    node_idx_opc = N_IDX_SH_RIGHT;
                end else if (aes_ready_train[8])
                    node_idx_opc = N_IDX_SH_RIGHT;
                
                if (aes_ready_train[10] && !new_round)
                    data_mem_addr_opc = GOTO_STORAGE;
            end // Find-where-to-go-next if-else
            
            if (new_round) begin
                if (aes_ready_train[5])
                    data_mem_addr_opc = GOTO_LEAF;                    
             end
            
            if (aes_ready_train[11]) // Time to store
                data_mem_addr_opc = PLUS_1_D_M;
//            else if (aes_ready_train[12])
//                data_mem_addr_opc = GOTO_PREV_LEAF;

            if (aes_ready_train[11])
                data_mem_wr_addr_opc_next = GOTO_PREV_LEAF_WR;
            if (aes_ready_train[12])
                data_mem_wr_addr_opc = GOTO_PREV_LEAF_WR;

            if (|aes_ready_train[12:11])
                    load_seed_next=1;
        end // S_EXP_PARENT
        
// **************************************************************************
        S_CLEAN_V_MEM: begin // exit at v_mem_clean_exit
//            party_idx_opc = RST_PARTY_IDX;
            rst_mpc_round_next = 1'b1;
            path_len_opc       = PATH_LEN_RST;
            path_pos_opc       = PATH_POS_RST;
            four_counter_rst   = 1'b1;
            party_idx_opc = PARTY_IDX_GOTO_I_S; // Bug-fix
            
            if (rst_idx_S_CLEAN)
                node_idx_opc = N_IDX_RST;
            else
                node_idx_opc = N_IDX_ADD_2;
            
            v_bit_mem_wren_next_0 = 1'b1;
            v_bit_mem_wren_next_1 = 1'b1;
            
            force_state_update_next = v_mem_clean_exit;
        end // S_CLEAN_V_MEM
        
// **************************************************************************        
        S_WAIT_I_STAR: begin // exit at i_star_mem_full
            force_state_update_next = i_star_mem_full;
            party_idx_opc = PARTY_IDX_GOTO_I_S;
            node_idx_opc  = N_IDX_GOTO_TARG_NODE;
            state_ctr_opc = ST_CTR_COUNT_RST;
        end // 
        
// **************************************************************************         
        S_GET_SIBL_PATH: begin // This part implements the C function mirath_ggm_tree_print_sibling_path
            party_idx_opc = PARTY_IDX_GOTO_I_S;
            
            if (first_cycles_train[0]) // First cycle in state: update mpc_round_aes to have the next i* idx ready.
                init_mpc_round_next = 1'b1;
            
            if (state_ctr_timeout) begin // Check given index: if in list, remove. If not, add sibling.
//                if (~v_bit_mem_dout_pip) begin
                if (~v_bit_mem_dout) begin
                    v_bit_mem_wren_next_0 = 1'b1;
                    v_bit_mem_din_next_0  = 1'b1; // Add sibling
                    path_len_opc          = PATH_LEN_ADD_1;    
                end else begin
                    v_bit_mem_wren_next_1 = 1'b1;
                    v_bit_mem_din_next_1  = 1'b0; // Remove node
                    path_len_opc          = PATH_LEN_SUB_1;    
                end
            end
            
            if (state_ctr_timeout_train[0]) begin
                 node_idx_opc            = N_IDX_SH_RIGHT; // Gets overwritten anyway on exit.
                if (v_bit_mem_dout_pip || node_idx_sub_lvl_2) begin
//                    node_idx_opc            = N_IDX_GOTO_TARG_NODE; // Go to next hidden leaf
                    init_mpc_round_next     = 1'b1;
                    force_state_update_next = mpc_round_0;
                    state_ctr_opc           = ST_CTR_COUNT_FREEZE;
                end
            end
            
            if (state_ctr_frozen) begin // Note: The state_ctr freezing solves a bug where you get
            // multiple leaves in a row and the mpc_round_aes update is too slow.
                node_idx_opc    = N_IDX_GOTO_TARG_NODE; // Go to next hidden leaf
            end
            
            if (update_state) begin // Signal path_len evaluation result to the top-level FSM
                node_idx_opc    = N_IDX_RST;
                
                if (path_too_big)
                    reject_path_next = 1'b1;
                else
                    accept_path_next = 1'b1;
            end            
        end // S_GET_SIBL_PATH
        
// **************************************************************************
        S_FILL_SIBL_PATH_0: begin // Fill the key-sig mem with the members of the sibling
        // path that are already stored in the "permanent" storage
            read_source_next = 1'b1; // Steer data_mem output to key_sig_mem input
            
            if (state_ctr_timeout) begin
                data_mem_addr_opc    = GOTO_STORAGE;
                if (v_bit_mem_dout_pip) begin
//                    data_mem_addr_opc    = GOTO_STORAGE;
                    state_ctr_opc        = ST_CTR_COUNT_FREEZE;
                end else
                    node_idx_opc = N_IDX_ADD_1; // Go to next node_idx
            
            end else if (state_ctr_timeout_train[0]) begin
                if (state_ctr_frozen) begin
                    state_ctr_opc        = ST_CTR_COUNT_FREEZE;
                    data_mem_addr_opc    = PLUS_1_D_M;
                end
//                else
//                    force_state_update_next = ~node_in_storage; // Move to S_FILL_SIBL_PATH_1_SEARCH.
            end else if (state_ctr_timeout_train[1]) begin
                if (state_ctr_frozen) begin
//                    key_sig_mem_addr_opc = PLUS_1_KS_M;
                    state_ctr_opc        = ST_CTR_COUNT_FREEZE;
                    path_pos_opc         = PATH_POS_ADD_1;
                    key_sig_mem_addr_opc = GOTO_PATH_POS;
                    wren_key_sig_mem_next= 1'b1;
                    node_idx_opc = N_IDX_ADD_1; // Go to next node_idx
                end
            end else if (state_ctr_timeout_train[2]) begin
                if (state_ctr_frozen) begin
                    key_sig_mem_addr_opc = PLUS_1_KS_M;
                    wren_key_sig_mem_next= 1'b1;
                    state_ctr_opc        = ST_CTR_COUNT_RST;
//                    force_state_update_next = ~node_in_storage; // Move to S_FILL_SIBL_PATH_1_SEARCH: not 100% verified for correctness
                end
            end
            
            force_state_update_next = ~node_in_storage && ~update_state && ~|first_cycles_train[1:0];
            
            if (update_state)
                state_ctr_opc        = ST_CTR_COUNT_RST;
        end // S_FILL_SIBL_PATH_0
        
// **************************************************************************
        S_FILL_SIBL_PATH_1_SEARCH: begin
//            state_ctr_rst_val   = ST_CTR_RST_VAL_MEDIUM; // Need a few extra cycles to store the revealed nodes
            targ_node_idx_opc = TARG_N_IDX_FREEZE; // We make freezing the state_ctr the default in this state
//            state_ctr_opc     = ST_CTR_COUNT_DOWN;
            data_mem_addr_opc    = GOTO_STORAGE; // We make this the default here
            
            if (state_ctr_timeout) begin
                if (~searching_node) begin
                    if (v_bit_mem_dout_pip) begin
                        targ_node_idx_opc    = TARG_N_IDX_GOTO_NODE_IDX_P1;
                        node_idx_opc         = N_IDX_SH_RIGHT;
                        searching_node_next  = 1'b1;
                    end else begin
                        node_idx_opc    = N_IDX_ADD_1; // Go to next node_idx
                    end
//                end else if (~node_found)   begin // Note: Can probably remove node_found
                end else begin
                    if (node_in_storage) begin
                        force_state_update_next = 1'b1;
                        node_found_next         = 1'b1;
                        searching_node_next     = 1'b1; // TODO: change after debugging
                    end else begin
                        node_idx_opc            = N_IDX_SH_RIGHT;
                        four_counter_incr       = 1'b1;
                    end
                end 
            end   
            
            // Store res of S_FILL_SIBL_PATH_1_STORE: key_sig_mem_addr_opc = PLUS_1_KS_M;
            if ((|aes_ready_train[2:1] && ~four_counter_bit_sel) || (|aes_ready_train[4:3] && four_counter_bit_sel)) begin
                wren_key_sig_mem_next= 1'b1;
                key_sig_mem_addr_opc = PLUS_1_KS_M;
            end
                   
        end // S_FILL_SIBL_PATH_1_SEARCH
 
 // **************************************************************************
        S_FILL_SIBL_PATH_1_STORE: begin
            rst_mpc_round_next  = 1'b1;
            party_idx_opc       = PARTY_IDX_GOTO_I_S;
            state_ctr_opc       = ST_CTR_COUNT_RST;
            targ_node_idx_opc   = TARG_N_IDX_FREEZE;
            searching_node_next = 1'b0;
            node_found_next     = 1'b0;
            data_mem_addr_opc   = PLUS_1_D_M;
            
            if (first_cycles_train[0]) begin
                path_pos_opc        = PATH_POS_ADD_1;
//                data_mem_addr_opc   = PLUS_1_D_M;
                load_seed_next      = 1'b1;
            end else if (first_cycles_train[1]) begin
                load_seed_next      = 1'b1;
            end else if (first_cycles_train[2]) begin
                aes_init_next       = 1'b1;
            end
            
            node_idx_sh_left_lsb_next = four_counter_bit_sel;
            
            if (first_cycles_train[5]) begin
                node_idx_opc        = N_IDX_SH_LEFT;
            end 
//            else if (first_cycles_train[6] && four_counter_bit_sel) begin
//                node_idx_opc        = N_IDX_ADD_1;
//            end
            
            if (aes_core_ready_in_1) begin
                aes_init_next           = ~node_idx_is_target;
//                four_counter_decr       = ~node_idx_is_target;
                store_res_next          = 1'b1;
                
                if (four_counter_bit_sel)
                    load_key_from_res_1_next = 1'b1;
                else
                    load_key_from_res_0_next = 1'b1;
                
                if (node_idx_is_target)
                    node_idx_opc        = N_IDX_ADD_1;
            end
            
            if (aes_ready_train[0]) begin
                four_counter_decr       = 1'b1;
            end 
            
            if (aes_ready_train[3]) begin
                node_idx_opc            = N_IDX_SH_LEFT;
            end 
//            else if (four_counter_bit_sel && aes_ready_train[3])
//                node_idx_opc            = N_IDX_ADD_1;
            
        end // S_FILL_SIBL_PATH_1_STORE

// **************************************************************************
        S_FILL_COMMITS: begin
            party_idx_opc = PARTY_IDX_GOTO_I_S;
            data_mem_addr_opc = GOTO_STORAGE;
            
            if  (node_idx_is_target_pip)
                mode_aes_next     = MODE_COMMIT;
        
            if (node_idx_is_target_delayed) begin
                if (aes_ready_train[2])
                    node_idx_opc = N_IDX_GOTO_TARG_NODE;
                else if (|aes_ready_train[6:3])
                    node_idx_opc = N_IDX_SH_RIGHT;
                    
                if (|aes_ready_train[5:3])
                    four_counter_incr = 1'b1;
                    
                if (aes_ready_train[9])
                    data_mem_addr_opc = PLUS_1_D_M;
                    
                if (|aes_ready_train[10:9])
                    load_seed_next = 1'b1;
                
                if (aes_ready_train[10])
                    aes_init_next = 1'b1;
            end
            
            node_idx_sh_left_lsb_next = four_counter_bit_sel;
            
            if (aes_core_ready_in_2) begin
                node_idx_opc      = N_IDX_SH_LEFT;
                
                if (node_idx_is_target) // Note: This is a quick bugfix to compensate for an extra decr. There's a more optimal way to do this.
                    four_counter_incr = 1'b1;
                else
                    four_counter_decr = 1'b1;
            end
            
            if (aes_core_ready_in_1) begin
                store_res_next    = 1'b1;
                aes_init_next     = ~node_idx_is_target_delayed;
                
//                if (four_counter_bit_sel)
//                    node_idx_opc = N_IDX_ADD_1;
                
                if (four_counter_bit_sel)
                    load_key_from_res_1_next = 1'b1;
                else
                    load_key_from_res_0_next = 1'b1;
            end
            
            if (aes_ready_train[0] && node_idx_is_target)
                node_idx_opc            = N_IDX_ADD_1;
        end // S_FILL_COMMITS

// **************************************************************************        
        S_COPY_NODES_0: begin
            rst_mpc_round_next = 1'b1;
            key_sig_mem_addr_opc = GOTO_PATH_POS;
            read_source_next = 1'b1; // Steer key_sig_mem output to data_mem input
//            data_mem_addr_opc    = GOTO_STORAGE;
            data_mem_wr_addr_opc = GOTO_STORAGE_WR;
            data_mem_wr_addr_opc_next = GOTO_STORAGE_WR;
            force_state_update_next = ~|first_cycles_train[1:0] && ~node_in_storage && ~update_state; // Move to S_FILL_SIBL_PATH_1_SEARCH
            
            if (state_ctr_timeout) begin
                if (v_bit_mem_dout_pip) begin
                    state_ctr_opc = ST_CTR_COUNT_FREEZE;
                end else
                    node_idx_opc  = N_IDX_ADD_1; // Go to next node_idx
            
            end else if (state_ctr_timeout_train[0]) begin
                key_sig_mem_addr_opc = PLUS_1_KS_M;
                
                if (state_ctr_frozen) begin
                    state_ctr_opc        = ST_CTR_COUNT_FREEZE;
//                    key_sig_mem_addr_opc = PLUS_1_KS_M;
                end
//                else
//                    force_state_update_next = ~node_in_storage; // Move to S_FILL_SIBL_PATH_1_SEARCH.
            end else if (state_ctr_timeout_train[1]) begin
                data_mem_wr_addr_opc_next = PLUS_1_D_M_WR;
                if (state_ctr_frozen) begin
                    state_ctr_opc        = ST_CTR_COUNT_FREEZE;
                    path_pos_opc         = PATH_POS_ADD_1;
                    wren_data_mem_next   = 1'b1;
                    node_idx_opc = N_IDX_ADD_1; // Go to next node_idx
                end
            end else if (state_ctr_timeout_train[2]) begin
                key_sig_mem_addr_opc = PLUS_1_KS_M;
                data_mem_wr_addr_opc = PLUS_1_D_M_WR;
                
                if (state_ctr_frozen) begin
//                    key_sig_mem_addr_opc = PLUS_1_KS_M;
//                    data_mem_addr_opc    = PLUS_1_D_M;
//                    data_mem_wr_addr_opc = PLUS_1_D_M_WR;
                    wren_data_mem_next   = 1'b1;
                    state_ctr_opc        = ST_CTR_COUNT_RST;
//                    force_state_update_next = ~node_in_storage; // Move to S_FILL_SIBL_PATH_1_SEARCH: not 100% verified for correctness
                end
            end
            
            if (|first_cycles_train[3:2]) // In the case that node_index_p1=='h2 is valid, this will load it to the seed registers. If not, we don't care about whatever is loaded at this point.
                load_seed_next = 1'b1;
            
            if (update_state)
                state_ctr_opc = ST_CTR_COUNT_RST;
        end
              
// **************************************************************************        
        S_COPY_NODES_1: begin
            if (state_ctr_timeout) begin
                node_idx_opc  = N_IDX_ADD_1; // Go to next node_idx
                if (v_bit_mem_dout_pip) begin
//                    state_ctr_opc = ST_CTR_COUNT_FREEZE;
                    part_ord_list_wren  = 1'b1;
                    path_pos_opc        = PATH_POS_ADD_1;
                end          
            end
            
            if (state_ctr_timeout_train[0] && path_end_reached)
                force_state_update_next = 1'b1;
                
            if (update_forced) begin
                node_idx_opc  = N_IDX_RST;
//                aes_init_next = 1'b1;
            end
            
            party_idx_opc = RST_PARTY_IDX;
        end // S_COPY_NODES_0
        
// **************************************************************************        
        S_FAKE_EXPAND: begin
            // First finish with previous routine
            if (|aes_ready_train[6:5] || aes_ready_train[9])
                node_idx_opc = N_IDX_SUB_1;
            
            if (aes_ready_train[7])
                node_idx_opc = N_IDX_SH_LEFT;
            else if (aes_ready_train[8])
                node_idx_opc = N_IDX_SH_RIGHT;
//            else if (aes_ready_train[9])
//                node_idx_opc = N_IDX_ADD_1;
                
            if (v_bit_mem_dout_prev_2) begin
                if (|aes_ready_train[6:5])
                    node_idx_opc = N_IDX_SUB_1;
                
                if (aes_ready_train[7]) begin
                    v_bit_mem_wren_next_0 = 1'b1;
                    v_bit_mem_wren_next_1 = 1'b1;
                    v_bit_mem_din_next_0  = 1'b1;
                    v_bit_mem_din_next_1  = 1'b1;
                    node_idx_opc = N_IDX_SH_LEFT;
//                end else if (aes_ready_train[8]) begin
//                    node_idx_opc = N_IDX_SH_RIGHT;
//                end else if (aes_ready_train[9]) begin
//                    node_idx_opc = N_IDX_ADD_1;
//                    node_idx_opc = N_IDX_ADD_2;
                end
            end
            
            if (|aes_ready_train[4:1] && v_bit_mem_dout_prev) begin
                wren_data_mem_next = 1'b1;
            end
            
            if (|aes_ready_train[4:2]) begin
                data_mem_addr_opc = PLUS_1_D_M;
            end else if (aes_ready_train[5]) begin
                data_mem_addr_opc = GOTO_STORAGE;
            end
//            else if (aes_ready_train[6]) begin // Not needed here?
//                data_mem_addr_opc = PLUS_1_D_M;
//            end
            
            // Now with the actual work of this routine
            if (aes_ready_train[9])
                state_ctr_opc = ST_CTR_COUNT_RST;
            
            if (state_ctr_timeout) begin
                force_state_update_next = exit_fake_expand;
                if (v_bit_mem_dout_pip) begin
                        v_bit_mem_wren_next_0 = 1'b1;
                        v_bit_mem_wren_next_1 = 1'b1;
                        v_bit_mem_din_next_0  = 1'b1;
                        v_bit_mem_din_next_1  = 1'b1;
                        node_idx_opc  = N_IDX_SH_LEFT;
                        state_ctr_opc = ST_CTR_COUNT_FREEZE;
                end else begin
                    node_idx_opc  = N_IDX_ADD_1; // Go to next node_idx
                end
            end else if (state_ctr_timeout_train[0]) begin
                if (state_ctr_frozen) begin
                    node_idx_opc  = N_IDX_SH_RIGHT;
                    state_ctr_opc = ST_CTR_COUNT_FREEZE;
                end
            end else if (state_ctr_timeout_train[1] && state_ctr_frozen) begin
                if (state_ctr_frozen) begin
                    node_idx_opc  = N_IDX_ADD_1;
                    state_ctr_opc = ST_CTR_COUNT_RST;
//                    force_state_update_next = exit_fake_expand;
                end
            end
            
//            party_idx_opc = RST_PARTY_IDX;
            
            if (update_forced) begin
                searching_node_next = 1'b1;
//                node_idx_opc  = N_IDX_GOTO_TARG_NODE;
            end
            
//            path_pos_opc = PATH_POS_GOTO_LIST;
            path_pos_opc = PATH_POS_GOTO_LIST_MIN_1;
        end // S_FAKE_EXPAND
        
// **************************************************************************        
        S_SEARCH_NEXT_NODE: begin // par_rev_next, gp_rev_next, gr_gp_rev_next, gr_2_gp_rev_next
            search_ctr_opc = SEARCH_CTR_INCR;
            hold_aes_core_ready_next = 1'b1;
            
//            if (mpc_round_0) begin
                if (new_round) begin
                    case (search_ctr)
//                        (SEARCH_CTR_INIT_DELAY-'h2):
//                            node_idx_opc  = N_IDX_GOTO_TARG_NODE;
                        
//                        (SEARCH_CTR_INIT_DELAY-'h1):
//                            node_idx_opc  = N_IDX_ADD_1; // Note: Can remove?
                        
                        ('h1+SEARCH_CTR_INIT_DELAY): begin
                            data_mem_addr_opc = GOTO_LEAF;
                        end
                         
                         ('h2+SEARCH_CTR_INIT_DELAY): begin
//                            node_idx_opc      = N_IDX_ADD_1;
                            data_mem_addr_opc = PLUS_1_D_M;
                            load_seed_next    = 1'b1;
                         end
                         
                         ('h3+SEARCH_CTR_INIT_DELAY): begin
                            load_seed_next          = 1'b1;
                         end
                         
                         'hd: 
                            force_state_update_next = 1'b1; // exit
                    endcase // search_ctr
                    
                end else if (party_idx_is_i_star) begin // Check if sibling is revealed, if so store it from the key-sig mem to the buffer
//                    read_source_next = 1'b1;
                    case (search_ctr)
                        (SEARCH_CTR_INIT_DELAY-'h2): begin
                            node_idx_opc  = N_IDX_GOTO_TARG_NODE;
                        end
                            
                        (SEARCH_CTR_INIT_DELAY-'h1):
                            node_idx_opc  = N_IDX_ADD_1; 
                               
//                        (SEARCH_CTR_INIT_DELAY):
//                            data_mem_wr_addr_opc_next = GOTO_LEAF_WR;
                        
                            
                        ('h1 +SEARCH_CTR_INIT_DELAY): begin // We should have the correct node now
//                            data_mem_addr_opc = GOTO_LEAF;
                            data_mem_wr_addr_opc = GOTO_LEAF_WR;
                            data_mem_wr_addr_opc_next = GOTO_LEAF_WR;
                            
                            if (part_list_dout_is_node_idx) begin
                                search_ctr_opc = SEARCH_CTR_INCR;
                                path_pos_opc  = PATH_POS_SUB_1;
                            end else begin
                                search_ctr_opc = SEARCH_CTR_FREEZE;
                                path_pos_opc  = PATH_POS_ADD_1;
                            end
                            
                            // IF 2 SIBLING LEAVES ARE HIDDEN: exit directly -> Causes bug
//                            if (~v_bit_mem_dout)
//                                force_state_update_next = 1'b1; // exit
                            if (~v_bit_mem_dout)
                                raise_consec_siblings_hidden_flag_next = 1'b1;
                            
                            if (consec_siblings_hidden_flag)
                                search_ctr_opc = SEARCH_CTR_INCR;
                        end
                        
                        
//                        ('h1 +SEARCH_CTR_INIT_DELAY): data_mem_wr_addr_opc_next = PLUS_1_D_M_WR;
                        
                        ('h2 +SEARCH_CTR_INIT_DELAY), ('h3 +SEARCH_CTR_INIT_DELAY): begin
                            path_pos_opc  = PATH_POS_SUB_1;
                            data_mem_wr_addr_opc = PLUS_1_D_M_WR;
                            data_mem_wr_addr_opc_next = PLUS_1_D_M_WR;
                        end
                        
//                        ('h3 +SEARCH_CTR_INIT_DELAY): begin
//                            path_pos_opc  = PATH_POS_SUB_1;
//                            data_mem_wr_addr_opc = PLUS_1_D_M_WR;
//                        end
                        
                        ('h4 +SEARCH_CTR_INIT_DELAY): begin // We should have the correct path_pos
                            key_sig_mem_addr_opc = GOTO_PATH_POS;
                        end
                        
                        ('h5 +SEARCH_CTR_INIT_DELAY): begin
//                            if (~consec_siblings_hidden_flag)
                                key_sig_mem_addr_opc = PLUS_1_KS_M;
                            read_source_next = 1'b1;
                        end
                            
                        ('h6 +SEARCH_CTR_INIT_DELAY): begin
//                            if (~consec_siblings_hidden_flag)
                                wren_data_mem_next =1'b1;
                            read_source_next = 1'b1;
                            data_mem_wr_addr_opc_next = PLUS_1_D_M_WR;
                        end
                        
                        ('h7 +SEARCH_CTR_INIT_DELAY): begin
                            read_source_next = 1'b1; // Note: Can remove?
                            wren_data_mem_next = 1'b1;
//                            data_mem_addr_opc = PLUS_1_D_M;
                            data_mem_wr_addr_opc = PLUS_1_D_M_WR;
                            force_state_update_next  = 1'b1;
                        end
                            
                        
                    endcase // search_ctr
                end else if (searching_node) begin
                    case (search_ctr)
                        (SEARCH_CTR_INIT_DELAY-'h2): begin
                            node_idx_opc  = N_IDX_GOTO_TARG_NODE;
                        end
                            
                        (SEARCH_CTR_INIT_DELAY-'h1):
                            node_idx_opc  = N_IDX_SH_RIGHT;
                        
                        ('h0+SEARCH_CTR_INIT_DELAY): begin
                            node_idx_opc  = N_IDX_SH_RIGHT;
                        end
                        
                        ('h1+SEARCH_CTR_INIT_DELAY): begin
                            node_idx_opc  = N_IDX_SH_RIGHT;
                            par_rev_next  = v_bit_mem_dout;
                        end
                        
                        ('h2+SEARCH_CTR_INIT_DELAY): begin
                            node_idx_opc  = N_IDX_SH_RIGHT;
                            gp_rev_next   = v_bit_mem_dout;
                        end
                        
                        ('h3+SEARCH_CTR_INIT_DELAY): begin
                            gr_gp_rev_next= v_bit_mem_dout;
                        end
                        
                        ('h4+SEARCH_CTR_INIT_DELAY): begin
                            searching_node_next = 1'b0;
                            search_ctr_opc = SEARCH_CTR_RST;
                            gr_2_gp_rev_next = v_bit_mem_dout;
                        end
                    endcase //search_ctr
                    
                end else if (~node_ready) begin
//                end else begin
                    data_mem_addr_opc = GOTO_STORAGE; // Here it should be fine
                    
                    if (gr_2_gp_rev) begin
                        case (search_ctr)
                            ('h1): begin 
                                data_mem_addr_opc = PLUS_1_D_M;
                                load_seed_next = 1'b1;
                             end
                             
                             ('h2): begin 
                                load_seed_next = 1'b1;
                             end
                             
                             ('h3): begin
                                node_ready_next         = 1'b1;
                             end
                        endcase // search_ctr
                    end else begin
                        read_source_next = 1'b1; // We now have to read directly from the sig
//                        data_mem_addr_opc = GOTO_LEAF; // Only for the case the revealed leaf is the sibling of a hidden leaf
                        data_mem_wr_addr_opc = GOTO_LEAF_WR; // Only for the case the revealed leaf is the sibling of a hidden leaf
                        data_mem_wr_addr_opc_next = GOTO_LEAF_WR; // Only for the case the revealed leaf is the sibling of a hidden leaf
                        case (search_ctr)
                            ('h0): begin // Go to gr_gp
                                node_idx_sh_left_lsb_next = target_node_idx[3];
//                                node_idx_opc  = N_IDX_SH_LEFT;
                            end
                            ('h1): begin
                                node_idx_opc  = N_IDX_SH_LEFT;
                                node_idx_sh_left_lsb_next = target_node_idx[2];
//                                if (target_node_idx[3])
//                                    node_idx_opc  = N_IDX_ADD_1;
                            end
                            ('h2): begin // Go to gp
                                if (~gr_gp_rev)
                                    node_idx_opc  = N_IDX_SH_LEFT;
                            end
                            ('h3): begin
                                node_idx_sh_left_lsb_next = target_node_idx[1];
//                                if (~gr_gp_rev && target_node_idx[2])
//                                    node_idx_opc  = N_IDX_ADD_1;
                            end
                            ('h4): begin // Go to par
                                if (~gp_rev)
                                    node_idx_opc  = N_IDX_SH_LEFT;
                            end
//                            ('h5): begin
////                                if (~gp_rev && target_node_idx[1])
////                                    node_idx_opc  = N_IDX_ADD_1;
//                            end
                            ('h5): begin // Go to leaf?
                                if (~par_rev)
                                    node_idx_opc  = N_IDX_SH_LEFT;
                            end
                            ('h6): begin
                                if (~par_rev)// && target_node_idx[1])
                                    path_pos_opc  = PATH_POS_ADD_1;
                            end
                            ('h8): begin // We should have the correct node now
                                if (part_list_dout_is_node_idx) begin
                                    search_ctr_opc = SEARCH_CTR_INCR;
                                    path_pos_opc  = PATH_POS_SUB_1;
                                end else begin
                                    search_ctr_opc = SEARCH_CTR_FREEZE;
                                    path_pos_opc  = PATH_POS_ADD_1;
                                end
                            end
                            ('h9): path_pos_opc  = PATH_POS_SUB_1;
                            ('ha): path_pos_opc  = PATH_POS_SUB_1;
                            
                            ('hc): begin // We should have the correct path_pos
                                key_sig_mem_addr_opc = GOTO_PATH_POS;
                            end
                            ('hd): begin
                                key_sig_mem_addr_opc = PLUS_1_KS_M;
                                load_seed_next = 1'b1;
                            end
                            ('he): begin 
                                load_seed_next = 1'b1;
                                data_mem_wr_addr_opc_next = PLUS_1_D_M_WR;
                                if(~par_rev)
                                    wren_data_mem_next = 1'b1;
                            end
                            ('hf): begin
                                node_ready_next= 1'b1;
//                                data_mem_addr_opc = PLUS_1_D_M;
                                data_mem_wr_addr_opc = PLUS_1_D_M_WR;
                                if(~par_rev)
                                    wren_data_mem_next = 1'b1;
                            end
                            
                        endcase // search_ctr
                    end               
//                end else if (update_forced) begin
//                    aes_init_next = 1'b1;
                end
//            end // if (mpc_round_0)
            
//            if (aes_core_ready_in_1 && node_ready) begin
            if (node_ready && ~update_forced) begin // We make the observation that the datapath always has to wait
                force_state_update_next  = 1'b1;
                hold_aes_core_ready_next = 1'b0;
//                aes_init_next = par_rev || new_round;
            end
            
//            if ((aes_ready_train[0] && ~aes_core_ready_in_1 && prev_state_aes!=S_COMMIT_PREP_GGM) || (update_forced && party_idx_is_i_star && prev_state_aes==S_COMMIT_PREP_GGM))
            if (update_state_pip && prev_state_aes!=S_COMMIT_PREP_GGM && prev_state_aes!=S_FAKE_EXPAND)
                party_idx_opc = PARTY_IDX_ADD_4;
            
            if (update_forced) begin
                lower_consec_siblings_hidden_flag_next = 1'b1;
                hold_aes_core_ready_next = 1'b0;
                store_res_next    = 1'b1;
                search_ctr_opc      = SEARCH_CTR_RST;
                aes_init_next       = (~(party_idx_is_i_star || ~par_rev) || new_round); // Todo: check if this fixes all issues
                node_ready_next     = 1'b0;
                par_rev_next        = 1'b0;
                gp_rev_next         = 1'b0;
                gr_gp_rev_next      = 1'b0;
                gr_2_gp_rev_next    = 1'b0;
                searching_node_next = 1'b1;
//                data_mem_addr_opc   = GOTO_PREV_LEAF;
                data_mem_wr_addr_opc= GOTO_PREV_LEAF_WR;
                data_mem_wr_addr_opc_next= GOTO_PREV_LEAF_WR;
                force_state_update_next  = 1'b0;    // Safety net
//                node_ready_next     = 1'b0;
                path_pos_opc        = PATH_POS_GOTO_LIST_MIN_1; // Note: this avoids a bug where a direct "lucky" landing directly on the correct position breaks the algo
            end
            
        end // S_SEARCH_NEXT_NODE
    endcase // state_aes
 
// **************************************************************************
    case(prev_state_aes)
        S_ARRAY_EXPAND: begin
            if (|aes_ready_train[4:1] && (~verify || v_bit_mem_dout_prev)) begin
                wren_data_mem_next = 1'b1;
            end
            
//            if (aes_ready_train[1])
//                data_mem_wr_addr_opc = GOTO_PREV_NODE_LEFT_CHILD_STORAGE_WR;
//            else
            if (|aes_ready_train[4:2])
                data_mem_wr_addr_opc = PLUS_1_D_M_WR;
            if (|aes_ready_train[3:1])
                data_mem_wr_addr_opc_next = PLUS_1_D_M_WR;
            
        end
    
        S_EXP_PARENT: begin
            if (|aes_ready_train[4:1])
                wren_data_mem_next = 1'b1;

//            if (|aes_ready_train[4:2])
//                data_mem_addr_opc = PLUS_1_D_M;
            
            if (|aes_ready_train[4:2])
                data_mem_wr_addr_opc = PLUS_1_D_M_WR;
            if (|aes_ready_train[3:1])
                data_mem_wr_addr_opc_next = PLUS_1_D_M_WR;
        end
        
        S_EXP_GREAT_GP, S_EXP_GP, S_EXP_STORED_N: begin                    
            if (|aes_ready_train[4:3])
                wren_data_mem_next = 1'b1; // Store right child
                
            if (aes_ready_train[4])
                data_mem_wr_addr_opc = PLUS_1_D_M_WR;
            if (aes_ready_train[3])
                data_mem_wr_addr_opc_next = PLUS_1_D_M_WR;
        end
    
        S_FILL_SIBL_PATH_1_STORE: begin
            if (((|aes_ready_train[2:1] && ~four_counter_bit_sel_pip_3) || (|aes_ready_train[4:3] && four_counter_bit_sel_pip_3)) && state_aes!=S_FILL_SIBL_PATH_1_STORE) begin // Note: might be a good idea to pipeline this one
                wren_key_sig_mem_next= 1'b1;
                key_sig_mem_addr_opc = PLUS_1_KS_M;
            end
        end  // S_FILL_SIBL_PATH_1_STORE
        
        S_COMMIT, S_COMMIT_PREP_GGM: begin
             if (|aes_ready_train[1+:`COMMIT_WORDS])
                cmt_fifo_incr_addr_wr_next = 1'b1;
        end
        
        S_FILL_COMMITS: begin
            key_sig_mem_addr_opc = GOTO_COMMIT_STORAGE;
            
            if (node_idx_is_target_delayed) begin
                if (|aes_ready_train[4:2])
                    key_sig_mem_addr_opc = PLUS_1_KS_M;
                
                if (|aes_ready_train[4:1])
                    wren_key_sig_mem_next= 1'b1;
            end
            
            if (aes_ready_train[9] && node_idx_is_target)
                init_mpc_round_next = 1'b1;
        end
        
        S_SEARCH_NEXT_NODE: begin // Same as S_EXP_PARENT
            if (|aes_ready_train[4:1])
                wren_data_mem_next = 1'b1;
                
//            if (aes_ready_train[4:2])
//                data_mem_addr_opc = PLUS_1_D_M;
                  
            if (aes_ready_train[4:2])
                data_mem_wr_addr_opc = PLUS_1_D_M_WR;
            if (|aes_ready_train[3:1])
                data_mem_wr_addr_opc_next = PLUS_1_D_M_WR;      
        end
        
        S_EXPAND: begin
            if (incr_ks_mem_addr_s_expand)
                key_sig_mem_addr_opc = PLUS_1_KS_M;
        end
   endcase
end

// *******************
// Next_state logic
//******************
always_comb begin
    next_state_aes = state_aes; // Default
    
    case (state_aes)
        S_IDLE: next_state_aes = S_LOAD_SALT;
            
        S_LOAD_SALT: next_state_aes = verify ? S_CLEAN_V_MEM : S_ARRAY_EXPAND;
        
        S_ARRAY_EXPAND: next_state_aes = ~exit_node_reached ? S_ARRAY_EXPAND : 
                                          verify            ? S_FAKE_EXPAND  : S_EXP_STORED_N; // We exploit that the first stored node to expand can be loaded directly in the ARRAY_EXPAND state        
        
//        S_EXP_STORED_N_BRIDGE: next_state_aes = S_EXP_GREAT_GP;

        S_EXP_STORED_N: next_state_aes = S_EXP_GREAT_GP;
        
        S_EXP_GREAT_GP: next_state_aes = S_EXP_GP;
                
        S_EXP_GP: next_state_aes = verify       ? S_SEARCH_NEXT_NODE    : S_EXP_PARENT;
//                                   mpc_round_0  ? S_EXP_PARENT_ROUND_0  : S_EXP_PARENT;
        
//        S_EXP_PARENT_ROUND_0: next_state_aes = new_round ? S_EXPAND_BRIDGE : S_EXP_STORED_N;  // TODO: exit cond
    
        S_EXPAND_BRIDGE: next_state_aes = leave_expand_bridge ? S_EXPAND : S_EXPAND_BRIDGE;
        
        S_EXPAND: next_state_aes          = (ctr!='h2)      ? S_EXPAND          :
                                            (goto_prep_GGM) ? S_COMMIT_PREP_GGM : S_COMMIT;
        
        S_COMMIT: next_state_aes = S_EXPAND;
        
        S_COMMIT_PREP_GGM: begin
            if (share_split_done)           next_state_aes = verify ? S_DONE : S_CLEAN_V_MEM;
            else if (verify)                next_state_aes = S_SEARCH_NEXT_NODE;
            else if (target_node_idx[1])    next_state_aes = S_EXP_PARENT;
            else if (target_node_idx[2])    next_state_aes = S_EXP_GP;
            else if (target_node_idx[3])    next_state_aes = S_EXP_GREAT_GP;
            else                            next_state_aes = S_EXP_STORED_N;
        end
                                         
//        S_EXP_PARENT:      next_state_aes = (new_round)             ? S_EXPAND_BRIDGE :
//                                            (sub_round_0)           ? S_EXP_STORED_N  :
//                                            (target_node_idx[1])    ? S_EXP_PARENT    :
//                                            (target_node_idx[2])    ? S_EXP_GP        :
//                                            (target_node_idx[3])    ? S_EXP_GREAT_GP  : S_EXP_STORED_N;
        S_EXP_PARENT: begin
            if (new_round)
                next_state_aes = sub_round_0 ? S_EXPAND_BRIDGE : S_EXPAND;
            else if (sub_round_0)
                next_state_aes = S_EXP_STORED_N;
            else if (target_node_idx[1])
                next_state_aes = S_EXP_PARENT;
            else if (target_node_idx[2])
                next_state_aes = S_EXP_GP;
            else if (target_node_idx[3])
                next_state_aes = S_EXP_GREAT_GP;
            else
                next_state_aes = S_EXP_STORED_N;
        end                              
                                            
        S_CLEAN_V_MEM:     next_state_aes = S_WAIT_I_STAR;
        
        S_WAIT_I_STAR:     next_state_aes = S_GET_SIBL_PATH;
        
        S_GET_SIBL_PATH: begin  // next_state_aes = (path_too_big) ? S_CLEAN_V_MEM : S_FILL_SIBL_PATH_0;
            if (verify) begin
                if (path_too_big)
                    next_state_aes = S_DONE; // BAD_SIG
                else
                    next_state_aes = S_COPY_NODES_0;
            end else begin
                if (path_too_big)
                    next_state_aes = S_CLEAN_V_MEM;
                else
                    next_state_aes = S_FILL_SIBL_PATH_0;
            end
        end
        
        S_FILL_SIBL_PATH_0:         next_state_aes = S_FILL_SIBL_PATH_1_SEARCH;
        
        S_FILL_SIBL_PATH_1_SEARCH:  next_state_aes = S_FILL_SIBL_PATH_1_STORE;
        
        S_FILL_SIBL_PATH_1_STORE:   next_state_aes = ~node_idx_is_target ? S_FILL_SIBL_PATH_1_STORE :
                                                    path_end_reached    ? S_FILL_COMMITS           : S_FILL_SIBL_PATH_1_SEARCH;
                                                    
        S_FILL_COMMITS:             next_state_aes = (node_idx_is_target_delayed && mpc_round_0) ? S_DONE : S_FILL_COMMITS;
        
        S_COPY_NODES_0:             next_state_aes = S_COPY_NODES_1;
        
        S_COPY_NODES_1:             next_state_aes = S_ARRAY_EXPAND;
        
        S_FAKE_EXPAND:              next_state_aes = S_SEARCH_NEXT_NODE;
        
        S_SEARCH_NEXT_NODE:         next_state_aes =  new_round     ? S_EXPAND           :
                                                      gr_2_gp_rev   ? S_EXP_STORED_N     :
                                                      gr_gp_rev     ? S_EXP_GREAT_GP     :
                                                      gp_rev        ? S_EXP_GP           : S_SEARCH_NEXT_NODE;
    endcase
end

// ****************************************************************
// Small standalone FSMs & RAMs under:

// Update leaf expansion ctr
always_ff @ (posedge clk) begin // update ctr    
    if (mode_aes==MODE_EXP_LEAF) begin
        if (aes_ready_train[1])
            ctr_alt <= ctr_alt + 1'b1;
    end else
        ctr_alt <= {CTR_BITS{1'b1}};
end

always_ff @ (posedge clk) begin // update ctr    
//    if (state_aes!=S_EXPAND && state_aes!=S_EXPAND_BRIDGE) begin
    if (state_aes!=S_EXPAND) begin
        ctr <= {CTR_BITS{1'b1}};
    end else if (update_state_pip)
        ctr <= ctr + 1'b1;
end

wire ctr_debug_wire = (ctr!=ctr_alt);

// ***********************
// *** Valid_bit RAM ****
// *********************
mem_dual #(
    .WIDTH  (1),
    .DEPTH  ((`TREE_NODES+2)),
    .FILE   ("") // ("init_0_and_1_to_zero.mem")
) valid_bit_ram (
    .clock      (clk),
    .wren_0     (v_bit_mem_wren_0),
    .wren_1     (v_bit_mem_wren_1),
    .address_0  ({node_index_p1_aes[`CLOG2(`TREE_NODES)-1:1], ~node_index_p1_aes[0]}),
    .address_1  (node_index_p1_aes),
    
    .data_0    (v_bit_mem_din_0),
    .data_1    (v_bit_mem_din_1),
    .q_1       (v_bit_mem_dout)
);

// ********************
// *** i_star RAM ****
// ******************
reg [$clog2(N)-1:0]     i_star_pip;
reg                     i_star_valid_pip;
reg                     i_star_mem_full;
reg [$clog2(`TAU)-1:0]  i_star_wr_addr;

simple_dual_port_mem #(
    .WIDTH  (8),
    .DEPTH  (`TAU)
) i_star_ram (
    .clk    (clk),
    .wea    (i_star_valid_pip),
    .addra  (i_star_wr_addr),
    .addrb  (mpc_round_aes),
    
    .dia     (i_star_pip),
    .dob     (i_star_mem_dout)
);

always_ff @ (posedge clk) begin
    i_star_valid_pip    <= i_star_valid;
    i_star_pip          <= i_star_in;
//    i_star_mem_full     <= (i_star_wr_addr==`TAU-1) && i_star_valid_pip;
    
    if (rst) // This avoids the lucky-chance bug where a valid challenge-generating ctr is found too quickly after another challenge is rejected due to large path_len
        i_star_mem_full <= 1'b0;
    else if ((i_star_wr_addr==`TAU-1) && i_star_valid_pip)
        i_star_mem_full <= 1'b1;
    else if (state_aes==S_WAIT_I_STAR)
        i_star_mem_full <= 1'b0;
    
    if (rst) begin
        i_star_wr_addr <= 'h0;
//        unlock_mpc_round_update <= 1'b0;
    end else if (i_star_valid_pip) begin
        i_star_wr_addr <= (i_star_wr_addr==`TAU-1) ? 'h0 : (i_star_wr_addr + 1'b1);
//        unlock_mpc_round_update <= 1'b1;
    end
end

// **********************************
// *** Partial Ordered List RAM ****
// ********************************
single_port_mem #(
    .WIDTH  (NODE_IDX_BITS),
    .DEPTH  (`PART_ORD_IDX_LIST_DEPTH),
    .LAST_ENTRY_0   ( 'h1 )
) part_ord_list_inst (
    .clk    (clk),
    .wea    (part_ord_list_wren),
    .addra  (partial_list_mem_addr),
    
    .dia    (node_index_p1_aes),
    .doa    (partial_list_mem_dout)
);

// **************************************************************
// *** Done signal: a few extra cycles to store last commit ****
// ************************************************************
reg done_in_2, done_in_1, waiting_for_i_pre;
always_ff @ (posedge clk) done_in_2          <= state_aes == S_DONE;
always_ff @ (posedge clk) done_in_1          <= done_in_2;
always_ff @ (posedge clk) done_to_ctrl       <= done_in_1;
//always_ff @ (posedge clk) waiting_for_i_pre  <= rst ? 1'b0 : (state_aes==S_WAIT_I_STAR);
//always_ff @ (posedge clk) waiting_for_i_star <= rst ? 1'b0 : waiting_for_i_pre;

reg state_is_expand_pip;
reg notify_ctrl_i_star, party_is_i_star_pip;
always_ff @ (posedge clk) begin
    party_is_i_star_pip <= party_idx_is_i_star;
    state_is_expand_pip <= state_aes==S_EXPAND;
    
    if (rst)
        notify_ctrl_i_star <= 1'b0;
    else if (party_idx_is_i_star && state_aes!=S_EXPAND && state_is_expand_pip)
        notify_ctrl_i_star <= 1'b1;
    else if (commit_valid)
        notify_ctrl_i_star <= 1'b0;
    
    if (~verify)
        party_is_i_star_to_ctrl <= 1'b0;
    else if (commit_valid)
        party_is_i_star_to_ctrl <= notify_ctrl_i_star;
    
    if (~verify)
        rst_v_regs <= 1'b0;
    else
        rst_v_regs <= notify_ctrl_i_star;
end

// *************************************
// cmt_fifo updates
assign cmt_fifo_wren = cmt_fifo_incr_addr_wr;

always @ (posedge clk) begin
    if (rst) begin
        cmt_fifo_incr_addr_wr <= 1'b0;
        cmt_fifo_addr_wr <= 'h0;
    end else begin
        cmt_fifo_incr_addr_wr <= cmt_fifo_incr_addr_wr_next;
        
        if (cmt_fifo_incr_addr_wr)
            cmt_fifo_addr_wr <= cmt_fifo_addr_wr + 1'b1;
    end
end

endmodule