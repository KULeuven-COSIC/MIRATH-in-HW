/* Some implementation-specific parameters */
`ifndef MIRATH_HW_PARAMS
`define MIRATH_HW_PARAMS

`include "clog2.v"
`include "mirath_params.vh" /* Everything else (that is required) already included in here */
//`include "mirath_math.vh"

`define MAX_AB(A,B)             ( (A>B) ? (A) : (B) )
`define MIN_AB(A,B)             ( (A<B) ? (A) : (B) )
`define POW2(number)            (1 << (number))
`define MOD_SIMPLE(A, B)        (((A)<B) ? (A) : (A-B) )
`define MOD_SIMPLEST(A, B)      (((A)==B) ? (0) : (A) )
`define CEIL(i,j)  (((i+j-1)/j)*j)

`define WORD_SIZE       64 // Fixed to 64 bits
`define NODE_SIZE       `MIRATH_SECURITY // Fixed to 128(L1) / 192(L3) / 256(L5)
`define COMMIT_SIZE     (2*`NODE_SIZE)
`define COMMIT_WORDS    (`COMMIT_SIZE/`WORD_SIZE)

`define GET_WORD_COUNT_FROM_BITS(x)     (`CEIL(x,`WORD_SIZE)/`WORD_SIZE)

`define HALF_WORD (`WORD_SIZE/2)
`define WORD_TO_BYTE_RATIO (`WORD_SIZE/8)

`define GET_WORD_COUNT_FROM_BYTES(x)    ((x+`WORD_TO_BYTE_RATIO-1)/`WORD_TO_BYTE_RATIO)

// Node-to-word ratio (e.g. 128/64=2 for L1). For dual AES,
// most of the time this will need to be multiplied by 2 in the code
`define NODE_WORDS (`NODE_SIZE/`WORD_SIZE)

`define LEVEL_K 4
`define K_EXIT_IDX ((`TREE_LEAVES/(1<<(`LEVEL_K-0)))-2) // Always check parent index.

// Tree size parameters
`define TREE_LEAVES (`TAU*`N)
`define TREE_NODES  (2*`TREE_LEAVES-1)

/* Get word counts */
`define MIRATH_VAR_FF_AUX_WORDS  ((`MIRATH_VAR_FF_AUX_BYTES + `WORD_TO_BYTE_RATIO - 1) / `WORD_TO_BYTE_RATIO)
`define MIRATH_VAR_FF_S_WORDS    ((`MIRATH_VAR_FF_S_BYTES   + `WORD_TO_BYTE_RATIO - 1) / `WORD_TO_BYTE_RATIO)
`define MIRATH_VAR_FF_C_WORDS    ((`MIRATH_VAR_FF_C_BYTES   + `WORD_TO_BYTE_RATIO - 1) / `WORD_TO_BYTE_RATIO)
`define MIRATH_VAR_FF_H_WORDS    ((`MIRATH_VAR_FF_H_BYTES   + `WORD_TO_BYTE_RATIO - 1) / `WORD_TO_BYTE_RATIO)
`define MIRATH_VAR_FF_Y_WORDS    ((`MIRATH_VAR_FF_Y_BYTES   + `WORD_TO_BYTE_RATIO - 1) / `WORD_TO_BYTE_RATIO)
`define MIRATH_VAR_FF_T_WORDS    ((`MIRATH_VAR_FF_T_BYTES   + `WORD_TO_BYTE_RATIO - 1) / `WORD_TO_BYTE_RATIO)
`define MIRATH_VAR_FF_E_WORDS    ((`MIRATH_VAR_FF_E_BYTES   + `WORD_TO_BYTE_RATIO - 1) / `WORD_TO_BYTE_RATIO)

`define MIRATH_SALT_WORDS    (`MIRATH_PARAM_SALT_BITS/`WORD_SIZE)
`define EXPAND_LENGTH       5

`define ALPHA_E_WORDS   (`M_PARAM_RHO/`WORD_TO_BYTE_RATIO)
`define TOTAL_A_WORDS   (`ALPHA_E_WORDS*2*`TAU)
`define TOTAL_A_BITS    (`TOTAL_A_WORDS*`WORD_SIZE)

`define AUX_WORDS   `MIRATH_VAR_FF_AUX_WORDS

// *************************************************
// Start commands:
`define START_KEYGEN    2'b01
`define START_SIGN      2'b10
`define START_VERIFY    2'b11

// **************************************************
// Keccak command defines
`define RATE_128 1344
`define RATE_256 1088
`define RATE_384 832
`define RATE_512 576

`define ABSORB_WORDS_128 (`RATE_128/`WORD_SIZE)
`define ABSORB_WORDS_256 (`RATE_256/`WORD_SIZE)
`define ABSORB_WORDS_384 (`RATE_384/`WORD_SIZE)
`define ABSORB_WORDS_512 (`RATE_512/`WORD_SIZE)

// Calculate how many (maximum) words to abosrb for 1 round of keccak absorb
`define RATE_WORDS_SHAKE ( (`SHAKE_RATE==128) ? (`ABSORB_WORDS_128) : (`ABSORB_WORDS_256) )
`define RATE_WORDS_SHA3  ( (`SHA3_RATE==256) ? (`ABSORB_WORDS_256) : ((`SHA3_RATE==384) ? (`ABSORB_WORDS_384) : (`ABSORB_WORDS_512)) )

`define RATE_BITS_SHAKE ((`RATE_WORDS_SHAKE*`WORD_SIZE))
`define RATE_BITS_SHA3  ((`RATE_WORDS_SHA3*`WORD_SIZE))

`define MAX_SINGLE_ROUND_INPUT_SHAKE      ((`RATE_BITS_SHAKE-6))
`define MAX_SINGLE_ROUND_INPUT_SHA3       ((`RATE_BITS_SHA3 -5))

`define SHAKE_CMD_0_ZERO_OUTLEN             ((`SHAKE_RATE == 128) ? 32'h80000000 : 32'h80000000) // REMOVED MUX256
`define SHAKE_CMD_O_CALC(OUTLEN)            (`SHAKE_CMD_0_ZERO_OUTLEN | OUTLEN)
`define SHAKE_CMD_1_CALC(INPUT_LENGTH)      ({ ((INPUT_LENGTH <= `MAX_SINGLE_ROUND_INPUT_SHAKE) ? 1'b1 : 1'b0), {31{1'b0}} } | (INPUT_LENGTH)) 
`define SHAKE_CMD_64(INPUT_LENGTH, OUTLEN)  {`SHAKE_CMD_1_CALC(INPUT_LENGTH), `SHAKE_CMD_O_CALC(OUTLEN)}

`define SHA3_CMD_0                 ((`SHA3_RATE == 256)   ? (32'h00000100) : ((`SHA3_RATE==384) ? (32'h00000180) : (32'h00000200)) ) // REMOVED MUX256
`define SHA3_CMD_1_CALC(INPUT_LENGTH)  ({ ((INPUT_LENGTH <= `MAX_SINGLE_ROUND_INPUT_SHA3) ? 1'b1 : 1'b0), {31{1'b0}} } | (INPUT_LENGTH)) 

`define SHA3_CMD_64(INPUT_LENGTH)                {`SHA3_CMD_1_CALC(INPUT_LENGTH), `SHA3_CMD_0}



// ******************************************
// Data mem defines:
`define DATA_MEM_DEPTH (1800)
`define DATA_MEM_ADDR_BITS `CLOG2(`DATA_MEM_DEPTH)

`define ROOT_SEED_ADDR_OFFSET   'h0
`define ROOT_SEED_ADDR          (`ROOT_SEED_ADDR_OFFSET*`NODE_WORDS)
`define GREAT_GP_ADDR_OFFSET    ((`K_EXIT_IDX<<1) + 3)
`define GREAT_GP_ADDR           (`GREAT_GP_ADDR_OFFSET*`NODE_WORDS)
`define GP_ADDR_OFFSET          (`GREAT_GP_ADDR_OFFSET + `N/4)
`define GP_ADDR                 (`GP_ADDR_OFFSET*`NODE_WORDS)
`define PAR_ADDR_OFFSET         (`GP_ADDR_OFFSET + `N/4)
`define PAR_ADDR                (`PAR_ADDR_OFFSET*`NODE_WORDS)
`define LEAF_ADDR_OFFSET        (`PAR_ADDR_OFFSET + `N/4)
`define LEAF_ADDR               (`LEAF_ADDR_OFFSET*`NODE_WORDS)
`define COMMIT_ADDR_OFFSET      (`LEAF_ADDR_OFFSET+ `N/2)
`define COMMIT_ADDR             (`COMMIT_ADDR_OFFSET*`NODE_WORDS)

//`define S_MATRIX_ADDR           (`COMMIT_ADDR+2*`COMMIT_WORDS)
//`define C_MATRIX_ADDR           (`S_MATRIX_ADDR+3) // TODO: 3 comes from the word # to store S for L1. Parametrize.
`define SUBCTX_0_ADDR           (`COMMIT_ADDR+2*`COMMIT_WORDS)


// ******************************************
// Key - Signature mem defines:
`define MSG_LEN_BYTES       'h16
`define MSG_LEN_WORDS       `GET_WORD_COUNT_FROM_BITS(8*`MSG_LEN_BYTES)
`define LAST_MSG_WORD_BITS  ((`MSG_LEN_BYTES<<3)%`WORD_SIZE)
`define LAST_MSG_WORD_BYTES (((`MSG_LEN_BYTES % `WORD_TO_BYTE_RATIO) == 0) ? `WORD_TO_BYTE_RATIO : (`MSG_LEN_BYTES % `WORD_TO_BYTE_RATIO))

`define PK_WORDS            `GET_WORD_COUNT_FROM_BITS(`PK_BITS)
`define PK_LAST_WORD_BITS   (`PK_BITS%`WORD_SIZE)

// First the signature addresses:
`define KEY_SIG_MEM_DEPTH (512 * 2)
`define KEY_SIG_MEM_ADDR_BITS `CLOG2(`KEY_SIG_MEM_DEPTH)

`define SK_SEED_ADDR 'h0
`define PK_SEED_ADDR (`SK_SEED_ADDR +`NODE_WORDS)
`define Y_ADDR  (`PK_SEED_ADDR +`NODE_WORDS)

`define SALT_ADDR           (`Y_ADDR+`GET_WORD_COUNT_FROM_BITS(`MIRATH_VAR_FF_Y_BITS))
`define MSG_ADDR            (`SALT_ADDR+`MIRATH_SALT_WORDS)
`define HASH_SH_ADDR        (`MSG_ADDR +`MSG_LEN_WORDS)

`define ALPHA_BASE_0_ADDR (`HASH_SH_ADDR +`COMMIT_WORDS)
`define ALPHA_MID_0_ADDR    (`ALPHA_BASE_0_ADDR+`ALPHA_E_WORDS)

//`define SIBL_PATH_ADDR_OFFSET (`SALT_ADDR_OFFSET+(`MIRATH_SALT_WORDS/`WORD_SIZE))
`define SIBL_PATH_ADDR        (`ALPHA_BASE_0_ADDR+`TOTAL_A_WORDS)

//`define KEY_SIG_MEM_COMMIT_ADDR_OFFSET   (`T_OPEN_LIMIT+`SIBL_PATH_ADDR_OFFSET)
`define KEY_SIG_MEM_COMMIT_ADDR         (`SIBL_PATH_ADDR+`T_OPEN_LIMIT*`NODE_WORDS)

`define AUX_ADDR    (`TAU*`COMMIT_WORDS+`KEY_SIG_MEM_COMMIT_ADDR)

`define H_MPC_ADDR  (`AUX_ADDR+`TAU*`AUX_WORDS)
// Then sk and pk addresses
//`define Y_ADDR  (`KEY_SIG_MEM_DEPTH - `GET_WORD_COUNT_FROM_BITS(`MIRATH_VAR_FF_Y_BITS))
//`define PK_SEED_ADDR (`Y_ADDR       -`NODE_WORDS)
//`define SK_SEED_ADDR (`PK_SEED_ADDR -`NODE_WORDS)

//`define ALPHA_BASE_0_ADDR (`SK_SEED_ADDR -`TOTAL_A_WORDS)
//`define ALPHA_MID_0_ADDR    (`ALPHA_BASE_0_ADDR+`ALPHA_E_WORDS)

//`define HASH_SH_ADDR (`ALPHA_BASE_0_ADDR -`COMMIT_WORDS)
//`define MSG_ADDR     (`HASH_SH_ADDR-`MSG_LEN_WORDS)

// ***************************************
// set_to_ff masks (L1 version)
`define S_C_SET_TO_FF_MASK(x) ( \
  (x == 0) ? 64'hffff03ffffffffff : \
  (x == 1) ? 64'hffffffff03ffffff : \
  (x == 2) ? 64'h03ffffffffff03ff : \
  (x == 7) ? 64'h00000f0f0f0f0f0f : 64'h0f0f0f0f0f0f0f0f)

// ****************************
// Different SHA-3 input lengths
`define HASH_COM_SUBCTX_IN_LEN ('h8 + (`TAU*(`N/4)*`COMMIT_SIZE))
`define DOM_SEP_COMMIT      8'h5;
`define DOM_SEP_HASH1       8'h1;
`define DOM_SEP_HASH2_PART  8'h2;

// **********************************
// hash_subctx_mem defines
`define H_SUBCTX_MEM_DEPTH      (4*`COMMIT_WORDS)
`define H_SUBCTX_MEM_ADDR_BITS  (`CLOG2(`H_SUBCTX_MEM_DEPTH))

// **********************************
// commit_fifo defines
`define CMT_FIFO_DEPTH      (2*`COMMIT_WORDS)
`define CMT_FIFO_ADDR_BITS  (`CLOG2(`H_SUBCTX_MEM_DEPTH))

`endif