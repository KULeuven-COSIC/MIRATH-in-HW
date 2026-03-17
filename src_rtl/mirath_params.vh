/* Taken straight from the C implementation */
`ifndef MIRATH_PARAMETER_H
`define MIRATH_PARAMETER_H

//`include "mirath_tcith_1a_f.vh"
`include "mirath_tcith_1b_f.vh"
//`include "mirath_tcith_5b_f.vh"

`define DOMAIN_SEPARATOR_MESSAGE 0
`define DOMAIN_SEPARATOR_HASH1 1
`define DOMAIN_SEPARATOR_HASH2_PARTIAL 2
`define DOMAIN_SEPARATOR_HASH2 3
`define DOMAIN_SEPARATOR_TREE 4
`define DOMAIN_SEPARATOR_COMMITMENT 5

`define MIRATH_VAR_GAMMA (`MIRATH_PARAM_RHO * (`MIRATH_PARAM_M * `MIRATH_PARAM_N - `MIRATH_PARAM_K))
`define MIRATH_VAR_S (`MIRATH_PARAM_M * `MIRATH_PARAM_R)
`define MIRATH_VAR_C (`MIRATH_PARAM_R * (`MIRATH_PARAM_N - `MIRATH_PARAM_R))
`define MIRATH_VAR_BASE_MID (`MIRATH_PARAM_M * (`MIRATH_PARAM_N - `MIRATH_PARAM_R))
`define MIRATH_VAR_E_A (`MIRATH_PARAM_M * `MIRATH_PARAM_N - `MIRATH_PARAM_K)
`define MIRATH_VAR_T (`MIRATH_PARAM_M * `MIRATH_PARAM_N)

/************************************************************************/
/* Also include MATRIX_FF_ARITH_H contents to avoid adding more headers */

/* Conditional, depending on the field ( Q value) */
`ifdef MIRATH_FIELD_GF16 // For Q = 16: 4-bit field elements, 2 per byte.
  `define MIRATH_MATRIX_FF_BYTES_PER_COLUMN(n_rows) (((n_rows) >> 1) + ((n_rows) & 1))
  
`elsif MIRATH_FIELD_GF2 // For Q = 2: 1-bit field elements, 8 per byte.
  `define MIRATH_MATRIX_FF_BYTES_PER_COLUMN(n_rows) (((n_rows) >> 3) + ((((n_rows) % 8) == 0) ? 0 : 1))
`endif

/* Continue unconditionally */
`define MIRATH_MATRIX_FF_BYTES_SIZE(N_ROWS, N_COLS) ((`MIRATH_MATRIX_FF_BYTES_PER_COLUMN(N_ROWS)) * (N_COLS))

/* Conditional again, depending on the field ( Q value) */
`ifdef MIRATH_FIELD_GF16 // For Q = 16: 4-bit field elements, 2 per byte.
  `define OFF_E_A ((8 * `MIRATH_VAR_FF_Y_BYTES) - 4 * (`MIRATH_PARAM_M * `MIRATH_PARAM_N - `MIRATH_PARAM_K))
  `define OFF_E_B ((8 * `MIRATH_MATRIX_FF_BYTES_SIZE(`MIRATH_PARAM_K, 1)) - 4 * `MIRATH_PARAM_K)
  
`elsif MIRATH_FIELD_GF2 // For Q = 2: 1-bit field elements, 8 per byte.
  `define OFF_E_A ((8 * `MIRATH_VAR_FF_Y_BYTES) - (`MIRATH_PARAM_M * `MIRATH_PARAM_N - `MIRATH_PARAM_K))
  `define OFF_E_B ((8 * `MIRATH_MATRIX_FF_BYTES_SIZE(`MIRATH_PARAM_K, 1)) - `MIRATH_PARAM_K)
`endif



`define MIRATH_VAR_FF_AUX_BYTES ( `MIRATH_MATRIX_FF_BYTES_SIZE(`MIRATH_PARAM_M, `MIRATH_PARAM_R) + `MIRATH_MATRIX_FF_BYTES_SIZE(`MIRATH_PARAM_R, `MIRATH_PARAM_N - `MIRATH_PARAM_R) )
`define MIRATH_VAR_FF_S_BYTES   ( `MIRATH_MATRIX_FF_BYTES_SIZE(`MIRATH_PARAM_M, `MIRATH_PARAM_R) )
`define MIRATH_VAR_FF_C_BYTES   ( `MIRATH_MATRIX_FF_BYTES_SIZE(`MIRATH_PARAM_R, `MIRATH_PARAM_N - `MIRATH_PARAM_R) )
//`define MIRATH_VAR_FF_H_BYTES   ( `MIRATH_MATRIX_FF_BYTES_SIZE(`MIRATH_PARAM_M * `MIRATH_PARAM_N - `MIRATH_PARAM_K), `MIRATH_PARAM_K) )
`define MIRATH_VAR_FF_H_BYTES   ( `MIRATH_MATRIX_FF_BYTES_SIZE(((`MIRATH_PARAM_M * `MIRATH_PARAM_N) - `MIRATH_PARAM_K), `MIRATH_PARAM_K) )

`define MIRATH_VAR_FF_Y_BYTES   ( `MIRATH_MATRIX_FF_BYTES_SIZE(`MIRATH_PARAM_M * `MIRATH_PARAM_N - `MIRATH_PARAM_K, 1) )
`define MIRATH_VAR_FF_T_BYTES   ( `MIRATH_MATRIX_FF_BYTES_SIZE(`MIRATH_PARAM_M, `MIRATH_PARAM_N - `MIRATH_PARAM_R) )
`define MIRATH_VAR_FF_E_BYTES   ( `MIRATH_MATRIX_FF_BYTES_SIZE(`MIRATH_PARAM_M * `MIRATH_PARAM_N, 1) )

`define MIRATH_VAR_FF_AUX_BITS     (8*`MIRATH_VAR_FF_AUX_BYTES)
`define MIRATH_VAR_FF_S_BITS       (8*`MIRATH_VAR_FF_S_BYTES)
`define MIRATH_VAR_FF_C_BITS       (8*`MIRATH_VAR_FF_C_BYTES)
`define MIRATH_VAR_FF_H_BITS       (8*`MIRATH_VAR_FF_H_BYTES)
`define MIRATH_VAR_FF_Y_BITS       (`MIRATH_PARAM_M * `MIRATH_PARAM_N - `MIRATH_PARAM_K)
`define MIRATH_VAR_FF_T_BITS       (8*`MIRATH_VAR_FF_T_BYTES)
`define MIRATH_VAR_FF_E_BITS       (8*`MIRATH_VAR_FF_E_BYTES)

`define MIRATH_VAR_FF_S_ROWS                `MIRATH_PARAM_M
`define MIRATH_VAR_FF_S_COLS                `MIRATH_PARAM_R
`define MIRATH_VAR_FF_C_ROWS                `MIRATH_PARAM_R
`define MIRATH_VAR_FF_C_COLS                (`MIRATH_PARAM_N - `MIRATH_PARAM_R)
`define MIRATH_VAR_FF_H_ROWS                ((`MIRATH_PARAM_M * `MIRATH_PARAM_N) - `MIRATH_PARAM_K)
`define MIRATH_VAR_FF_H_COLS                `MIRATH_PARAM_K
`define MIRATH_VAR_FF_E_MATRIX_FORM_ROWS    `MIRATH_PARAM_M
`define MIRATH_VAR_FF_E_MATRIX_FORM_COLS    `MIRATH_PARAM_N

`define S_ROWS                `MIRATH_PARAM_M
`define S_COLS                `MIRATH_PARAM_R
`define C_ROWS                `MIRATH_PARAM_R
`define C_COLS                (`MIRATH_PARAM_N - `MIRATH_PARAM_R)
`define H_ROWS                ((`MIRATH_PARAM_M * `MIRATH_PARAM_N) - `MIRATH_PARAM_K)
`define H_COLS                `MIRATH_PARAM_K
`define E_MATRIX_FORM_ROWS    `MIRATH_PARAM_M
`define E_MATRIX_FORM_COLS    `MIRATH_PARAM_N

/* Shorthand param definitions */
`define M_PARAM_Q       `MIRATH_PARAM_Q			        //**< Parameter q of the scheme (finite field GF(q^m)) */
`define M_PARAM_M       `MIRATH_PARAM_M			        //**< Parameter m of the scheme (finite field GF(q^m)) */
`define M_PARAM_K       `MIRATH_PARAM_K			        //**< Parameter k of the scheme (code dimension) */
`define M_PARAM_N       `MIRATH_PARAM_N		            //**< Parameter n of the scheme (code length) */
`define M_PARAM_R       `MIRATH_PARAM_R			        //**< Parameter r of the scheme (rank of vectors) */
`define M_PARAM_RHO     `MIRATH_PARAM_RHO	

`define N       `MIRATH_PARAM_N_1			    //**< Parameter N_1 of the scheme */
`define N_1     `MIRATH_PARAM_N_1			    //**< Parameter N_1 of the scheme */
`define N_2     `MIRATH_PARAM_N_2			    //**< Parameter N_2 of the scheme */
`define TAU     `MIRATH_PARAM_TAU			        //**< Parameter tau of the scheme (number of iterations) */
`define TAU_1   `MIRATH_PARAM_TAU_1			    //**< Parameter tau_1 of the scheme (number of iterations concerning N1) */
`define TAU_2   `MIRATH_PARAM_TAU_2		        //**< Parameter tau_2 of the scheme (number of iterations concerning N2) */		       // //**< Parameter rho of the scheme (dimension of the extension)*/
`define MU      `MIRATH_PARAM_MU

`define T_OPEN_LIMIT            `MIRATH_PARAM_T_OPEN

`define S_ROWS  `MIRATH_VAR_FF_S_ROWS
`define S_COLS  `MIRATH_VAR_FF_S_COLS
`define C_ROWS  `MIRATH_VAR_FF_C_ROWS
`define C_COLS  `MIRATH_VAR_FF_C_COLS

`define PK_BITS (`MIRATH_PUBLIC_KEY_BYTES<<3)

`endif