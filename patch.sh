# Apply patches to SHA-3 code
patch -p1 -o src_rtl/keccak_top_64.v cshake-core/verilog/keccak_top.v < patches/keccak_top.patch
patch -p1 -o src_rtl/keccak_control_64.v cshake-core/verilog/control_path.v < patches/keccak_control.patch
patch -p1 -o src_rtl/rc_64.v cshake-core/verilog/rc.v < patches/rc.patch
patch -p1 -o src_rtl/keccak_pkg_64.v cshake-core/verilog/keccak_pkg.v < patches/keccak_pkg.patch

# Apply patches to AES code
patch -p1 -o src_rtl/rijndael256_core_dual_fast.v aes/src/rtl/aes_core.v < patches/aes_core.patch
patch -p1 -o src_rtl/rijndael_encipher_256.v aes/src/rtl/aes_encipher_block.v < patches/aes_encipher.patch
patch -p1 -o src_rtl/aes_sbox_parallel.v aes/src/rtl/aes_sbox.v < patches/aes_sbox_parallel.patch
patch -p1 -o src_rtl/aes_sbox.v aes/src/rtl/aes_sbox.v < patches/aes_sbox.patch
patch -p1 -o src_rtl/keymem_256_stream.v aes/src/rtl/aes_key_mem.v < patches/keymem_256_stream.patch