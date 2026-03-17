// ********************************************
// E_mul opcodes to share amongst math modules
`ifndef E_MUL_OPCODES
`define E_MUL_OPCODES

typedef enum logic [1:0] { // E_mul
    hold_e,
    acc_e,
    init_e
} E_mul_opcode_t;

`endif