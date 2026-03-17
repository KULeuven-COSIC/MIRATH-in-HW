`include "clog2.v"
`include "mirath_hw_params.vh"

// Word size & node size (both in bits)
//`define WORD_SIZE 64 // Fixed to 64
//`define NODE_SIZE `MIRATH_SECURITY // Fixed to 128(L1) / 192(L3) / 256(L5)


// FSM / external module interaction defines
`define START_BIT       1
`define VERIFY_MODE_BIT 0

// Domain separators
`define DOMAIN_SEPARATOR_PRG (8'h4)
`define DOMAIN_SEPARATOR_CMT (8'h3)

// Counter checks for read/write and initializing
//`define COUNTER_START_READ 'h3
//`define COUNTER_INIT_AES   'h1

//`define COMM_DUR_LARGE (`READ_COMM_DUR+3)
//`define COUNTER_AES_bits `CLOG2(`COMM_DUR_LARGE)

// Command exit conditions
//`define DIRECT_EXIT         0
//`define WAIT_FOR_LAST_IDX   1
//`define WAIT_AES_READY_NEXT 1 // was 2

// leaf buffer defines
`define NUM_LEAF_BUFFERS 2

`define PART_ORD_IDX_LIST_DEPTH  (`LEVEL_K*`TAU+1) // test if good enough. +1 = permanent zero safe location
`define P_O_I_LIST_AD_BITS       `CLOG2(`PART_ORD_IDX_LIST_DEPTH)

//`define KEY_SIG_MEM_COMMIT_ADDR_OFFSET   (`T_OPEN_LIMIT+2)
//`define KEY_SIG_MEM_COMMIT_ADDR         ((`T_OPEN_LIMIT+2)*`NODE_WORDS)

//`define KEY_SIG_MEM_ACC_ADDR_OFFSET   (`KEY_SIG_MEM_COMMIT_ADDR_OFFSET+(`TAU*2))
`define KEY_SIG_MEM_ACC_ADDR         (`AUX_ADDR)

`define KEY_SIG_MEM_SALT_ADDR `SALT_ADDR
`define DATA_MEM_ROOT_SEED_ADDR 'h0