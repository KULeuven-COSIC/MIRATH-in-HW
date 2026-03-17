## :hammer_and_wrench: How to Simulate

**Requirements**
Our design was developed and tested with **AMD Vivado**, so we recommend using a machine with Vivado installed. 

**Vivado source setup**
Add the folder [src_rtl](../src_rtl/) to Vivado as **design sources**.

Add the sources in the folder [src_tb](../src_tb/) as **simulation sources**:

## :crossed_swords: Running the Simulations
1. The testbenches `tb_keygen.sv` and `tb_sign_verify.sv` use fixed input values for `sk_seed`, `pk_seed`, `salt`, `root_seed`, and the input message to perform a single keygen or sign/verify run.
2. The testbench `tb_multiple_sign_verify.sv` runs a predefined number (DEFAULT_NUM_RUNS) of **sign/verify** pairs by randomizing `sk_seed`, `pk_seed`, `salt`, and `root_seed` internally for each run. Once all runs have completed, the average latency for **sign** and **verify** is printed. Since the **keygen** routine is constant-time, we do not provide a multiple run testbench for it.

* **`tb_keygen.sv`**  
  Runs **key generation**. This is mainly useful for timing evaluation.

* **`tb_sign_verify.sv`**  
  Runs one **signature generation** followed by one **signature verification**. At the end of the simulation, the cycle counts for both routines are printed, together with the verification result (`bad_sig == 0` means success).

* **`tb_multiple_sign_verify.sv`**  
  Runs multiple back-to-back **sign/verify** tests. The number of runs is controlled by the parameter `DEFAULT_NUM_RUNS` near the top of the file. For each run, cycle counts are printed, and if all runs complete successfully, average signing and verification cycle counts are reported at the end.

## :gear: Changing test vectors
The provided testbenches already include preloaded values, so it is best to first run them as-is before modifying seeds or messages.

For the **NIST Level I** configuration, for example in branch `Mirath_1b_fast`:

* `sk_seed`, `pk_seed`, and `root_seed` are **16 bytes** each
* `salt` is **32 bytes**
* the architecture uses **64-bit words** in **little-endian** format

This means:

* `sk_seed`, `pk_seed`, and `root_seed` each occupy **2 memory words**
* `salt` occupies **4 memory words**

The message length is set through the macro `MSG_LEN_BYTES` in `mirath_hw_params.vh`.

**Memory initialization**

* Place `root_seed` in `data_mem_init.mem`, starting from the first line.
* Place `sk_seed` and `pk_seed` at the beginning of `key_sig_mem_init.mem`.
* Place `salt` and the message in `key_sig_mem_init.mem`, starting from address `@a`.

Suppose we want to initialize the values as follows:

```text
sk_seed:
3dc51e057ad9aba1
6a2ccede84d7f740

pk_seed:
727dbc3b4365b522
be3f60c4b794c2cf

salt:
bc6cba0f3042da40
73a88f63286c2fa2
6000f271aeea764a
5f2d508ab1e7f027

root_seed:
5a541fc91c6dac80
4b5822ca52d5dfed

and the message to

msg:
6e5973206b6e6152
454420656d6f7264
0000676e69646f63
```

Example layout:

`data_mem_init.mem`

```text
5a541fc91c6dac80
4b5822ca52d5dfed
```

`key_sig_mem_init.mem`

```text
3dc51e057ad9aba1
6a2ccede84d7f740
727dbc3b4365b522
be3f60c4b794c2cf
@a
bc6cba0f3042da40
73a88f63286c2fa2
6000f271aeea764a
5f2d508ab1e7f027
6e5973206b6e6152
454420656d6f7264
0000676e69646f63
```

## :shield: Important Notes

* Our implementation supports varying message lengths. The message length is controlled by the defined constant **`MSG_LEN_BYTES`**, in the verilog header file [mirath_hw_params.vh](../src_rtl/mirath_hw_params.vh). If you wish to try out different message lengths, the constant `MSG_LEN_BYTES` should *always* match the intended message size in bytes.
* Vivado reads memory initialization files when the simulation starts. After changing a `.mem` file, close and restart the simulation so the new contents are loaded.
