# Mirath in HW
This repository contains the verilog/vhdl (RTL) sources of an efficient HW (FPGA) implementation of [Mirath](https://pqc-mirath.org/index.html), an MPC-in-the-Head-based post-quantum digital signature algorithm. 
The design supports NIST security level I/V and all main routines (KeyGen, SigGen, SigVer). Our paper describing the design be found on [IACR ePrint](https://eprint.iacr.org/2026/206).

This branch (`L5`) contains the level V RTL code. The level I code can be found on the `main` branch.

## :file_folder: Contents

* [RTL sources](src_rtl): verilog/vhdl (design) source files.
* [TB sources](src_tb): testbenches for different operations (see [README](src_tb/README.md) for instructions).
* [Other sources](src_util): constraint file for ZYNQ board.
* [AES RTL sources](aes): open-source AES core, used in our design.
* [SHA-3 RTL sources](cshake-core): open-source SHA-3 core, used in our design.
* [Patches](patches): a collection of patches for applying to the open-source designs, according to the needs of our design

## :hammer_and_wrench: Running Code & Testbench
Download/clone this repository using:
```bash
git clone
```
Afterwards, it is necessary to initialize the submodules:
```bash
git submodule update --init --recursive
```
Finally, before using the RTL in an IDE (e.g. Vivado), apply patches to the open-source code:
```bash
source patch.sh
```

## :book: Bibliography

If you use or build upon the code in this repository, please cite our paper using our [CITATION](CITATION) key.

## Licensing

See our [LICENSE](LICENSE) and further details of original sources in the accompanying [NOTICE](NOTICE).
