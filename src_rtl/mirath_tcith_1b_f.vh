/* Taken straight from the C reference implementation */
`ifndef MIRATH_TCITH_MIRATH_PARAM_IB_H
`define MIRATH_TCITH_MIRATH_PARAM_IB_H


`define MIRATH_SECURITY 128			        //**< Expected security level (bits) */
`define MIRATH_SECURITY_BYTES 16		    //**< Expected security level (bytes) */

`define MIRATH_PARAM_SALT_BITS  (2*`MIRATH_SECURITY)
`define MIRATH_PARAM_SALT_BYTES (2 * `MIRATH_SECURITY_BYTES)

`define MIRATH_SECRET_KEY_BYTES 32		    //**< Secret key size */
`define MIRATH_PUBLIC_KEY_BYTES 57		    //**< Public key size */
`define MIRATH_SIGNATURE_BYTES 3456		    /**< Signature size */

`define MIRATH_FIELD_GF2                   /* Added for conditional macro inclusion */
`define MIRATH_PARAM_Q 2			        //**< Parameter q of the scheme (finite field GF(q^m)) */
`define MIRATH_PARAM_M 42			        //**< Parameter m of the scheme (finite field GF(q^m)) */
`define MIRATH_PARAM_K 1443			        //**< Parameter k of the scheme (code dimension) */
`define MIRATH_PARAM_N 42			        //**< Parameter n of the scheme (code length) */
`define MIRATH_PARAM_R 4			        //**< Parameter r of the scheme (rank of vectors) */

`define MIRATH_PARAM_N_1 256			    //**< Parameter N_1 of the scheme */
`define MIRATH_PARAM_N_2 256			    //**< Parameter N_2 of the scheme */
`define MIRATH_PARAM_N_1_BITS 8		        //**< Parameter tau_1 of the scheme (bits) */
`define MIRATH_PARAM_N_2_BITS 8		        //**< Parameter tau_2 of the scheme (bits) */
`define MIRATH_PARAM_N_1_BYTES 1		    //**< Parameter tau_1 of the scheme (bytes) */
`define MIRATH_PARAM_N_2_BYTES 1		    //**< Parameter tau_2 of the scheme (bytes) */
`define MIRATH_PARAM_N_1_MASK 0xff		    //**< Parameter tau_1 of the scheme (mask) */
`define MIRATH_PARAM_N_2_MASK 0xff		    //**< Parameter tau_2 of the scheme (mask) */
`define MIRATH_PARAM_TAU 17			        //**< Parameter tau of the scheme (number of iterations) */
`define MIRATH_PARAM_TAU_1 17			    //**< Parameter tau_1 of the scheme (number of iterations concerning N1) */
`define MIRATH_PARAM_TAU_2 0		        //**< Parameter tau_2 of the scheme (number of iterations concerning N2) */
`define MIRATH_PARAM_RHO 16			        //**< Parameter rho of the scheme (dimension of the extension)*/
`define MIRATH_PARAM_MU 8			        //**< Parameter mu of the scheme */

`define MIRATH_PARAM_TREE_LEAVES 4352	    //**< Number of leaves in the tree */

`define MIRATH_PARAM_CHALLENGE_2_BYTES 17	//**< Number of bytes required to store the second challenge */
`define MIRATH_PARAM_HASH_2_MASK_BYTES 2	//**< Number of last bytes in the second hash to be zero */
`define MIRATH_PARAM_HASH_2_MASK 0x01	    //**< Mask for the last byte in the second hash to be zero */
`define MIRATH_PARAM_T_OPEN 118		        //**< Maximum sibling path length allowed */

/* Note: Add extra hardware-relevant security parameters here */
`define SHAKE_RATE 128
`define SHA3_RATE  256
`define SECURITY_LEVEL "L1"

`endif //MIRATH_TCITH_MIRATH_PARAM_IB_H
