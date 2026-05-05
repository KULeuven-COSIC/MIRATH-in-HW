/*
 * tb_multiple_sign_verify.sv
 * --------------------
 * This is a testbench that runs multiple sign & verify runs and computes
 * the average latency (in clock cycles) for sign & verify.
 *
 * Copyright (c) 2026 KU Leuven - COSIC
 * Author: Stelios Manasidis    
 *        
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
 
`timescale 1ns / 1ps

`include "mirath_hw_params.vh"

`default_nettype none

module tb_multiple_sign_verify;

  localparam int clk_period      = 10;
  localparam int half_clk_period = clk_period/2;

  localparam int DEFAULT_NUM_RUNS = 50;

  logic tb_rst   = 0;
  logic tb_clk   = 0;

  logic [1:0] tb_start = 0;
  logic       tb_done;
  logic       tb_bad_sig;

  time t_start_sign, t_done_sign;
  time t_start_ver,  t_done_ver;

  // Saved randomized inputs for the current run (64-bit words)
  logic [63:0] sk_seed [0:3];
  logic [63:0] pk_seed [0:3];
  logic [63:0] salt    [0:7];
  logic [63:0] root    [0:3];

  longint unsigned sign_sum_ns = 0;
  longint unsigned ver_sum_ns  = 0;

  int num_runs;

  // ---------------- Existing clear tasks ----------------

task clear_data_mem_to_x;
    begin
      for (int i = 0; i < `DATA_MEM_DEPTH; i = i + 1)
        DUT.data_mem_inst.ram[i] = 'x; // all Xs
//        DUT.data_mem_inst.mem[i] = 'x; // all Xs
    end
endtask

task clear_aux_acc_to_x;
    begin
      for (int i = 0; i < 8; i = i + 1)
        DUT.u_mirath_top_lvl_ctrl_v2.u_aux_acc.shift_regs[i] = 'x; // all Xs DUT/u_mirath_top_lvl_ctrl_v2/u_aux_acc/shift_regs
    end
endtask

task clear_S_C_mem_to_x;
    begin
      for (int i = 0; i < 8; i = i + 1)
        DUT.u_mirath_top_lvl_ctrl_v2.S_C_mem.ram[i] = 'x; // all Xs DUT/u_mirath_top_lvl_ctrl_v2/u_aux_acc/shift_regs
    end
endtask

task clear_A_mem_to_x; // /tb_sign_verify/DUT/base_mem_wrapper_inst/Sb_mem_acc_inst/\TAU_S_base_mems[0].S_base_mem 
    begin
        for (int j = 0; j < `MIRATH_PARAM_RHO; j = j + 1) begin
          DUT.a_mem_wrapper_inst.A_mem_acc_inst.TAU_A_mems_x2[0].A_mem[j] = 'x;
          DUT.a_mem_wrapper_inst.A_mem_acc_inst.TAU_A_mems_x2[2].A_mem[j] = 'x;
          DUT.a_mem_wrapper_inst.A_mem_acc_inst.TAU_A_mems_x2[4].A_mem[j] = 'x;
          DUT.a_mem_wrapper_inst.A_mem_acc_inst.TAU_A_mems_x2[6].A_mem[j] = 'x;
          DUT.a_mem_wrapper_inst.A_mem_acc_inst.TAU_A_mems_x2[8].A_mem[j] = 'x;
          DUT.a_mem_wrapper_inst.A_mem_acc_inst.TAU_A_mems_x2[10].A_mem[j] = 'x;
          DUT.a_mem_wrapper_inst.A_mem_acc_inst.TAU_A_mems_x2[12].A_mem[j] = 'x;
          DUT.a_mem_wrapper_inst.A_mem_acc_inst.TAU_A_mems_x2[14].A_mem[j] = 'x;
          DUT.a_mem_wrapper_inst.A_mem_acc_inst.TAU_A_mems_x2[16].A_mem[j] = 'x;
        end
    end
endtask

task clear_TMP_mem_to_x; // /tb_sign_verify/DUT/base_mem_wrapper_inst/Sb_mem_acc_inst/\TAU_S_base_mems[0].S_base_mem 
    begin
        for (int j = 0; j < 80; j = j + 1) begin
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[0].TMP_base_mem_0[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[0].TMP_base_mem_1[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[0].TMP_base_mem_2[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[0].TMP_base_mem_3[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[0].TMP_base_mem_4[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[0].TMP_base_mem_5[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[0].TMP_base_mem_6[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[0].TMP_base_mem_7[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[9].TMP_base_mem_0[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[9].TMP_base_mem_1[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[9].TMP_base_mem_2[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[9].TMP_base_mem_3[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[9].TMP_base_mem_4[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[9].TMP_base_mem_5[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[9].TMP_base_mem_6[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[9].TMP_base_mem_7[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[18].TMP_base_mem_0[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[18].TMP_base_mem_1[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[18].TMP_base_mem_2[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[18].TMP_base_mem_3[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[18].TMP_base_mem_4[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[18].TMP_base_mem_5[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[18].TMP_base_mem_6[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[18].TMP_base_mem_7[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[27].TMP_base_mem_0[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[27].TMP_base_mem_1[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[27].TMP_base_mem_2[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[27].TMP_base_mem_3[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[27].TMP_base_mem_4[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[27].TMP_base_mem_5[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[27].TMP_base_mem_6[j] = 'x;
          DUT.TMP_wrapper_inst.TAU_TMP_mems_x9[27].TMP_base_mem_7[j] = 'x;
        end
    end
endtask

task clear_S_base_mem_to_x; // /tb_sign_verify/DUT/base_mem_wrapper_inst/Sb_mem_acc_inst/\TAU_S_base_mems[0].S_base_mem 
    begin
//      for (int i = 0; i < `TAU; i = i + 1) begin
        for (int j = 0; j < 42; j = j + 1) begin
          DUT.base_mem_wrapper_inst.Sb_mem_acc_inst.TAU_S_base_mems[0].S_base_mem[j] = 'x; // all Xs DUT/u_mirath_top_lvl_ctrl_v2/u_aux_acc/shift_regs
          DUT.base_mem_wrapper_inst.Sb_mem_acc_inst.TAU_S_base_mems[1].S_base_mem[j] = 'x; // all Xs DUT/u_mirath_top_lvl_ctrl_v2/u_aux_acc/shift_regs
          DUT.base_mem_wrapper_inst.Sb_mem_acc_inst.TAU_S_base_mems[2].S_base_mem[j] = 'x; // all Xs DUT/u_mirath_top_lvl_ctrl_v2/u_aux_acc/shift_regs
          DUT.base_mem_wrapper_inst.Sb_mem_acc_inst.TAU_S_base_mems[3].S_base_mem[j] = 'x; // all Xs DUT/u_mirath_top_lvl_ctrl_v2/u_aux_acc/shift_regs
          DUT.base_mem_wrapper_inst.Sb_mem_acc_inst.TAU_S_base_mems[4].S_base_mem[j] = 'x; // all Xs DUT/u_mirath_top_lvl_ctrl_v2/u_aux_acc/shift_regs
          DUT.base_mem_wrapper_inst.Sb_mem_acc_inst.TAU_S_base_mems[5].S_base_mem[j] = 'x; // all Xs DUT/u_mirath_top_lvl_ctrl_v2/u_aux_acc/shift_regs
          DUT.base_mem_wrapper_inst.Sb_mem_acc_inst.TAU_S_base_mems[6].S_base_mem[j] = 'x; // all Xs DUT/u_mirath_top_lvl_ctrl_v2/u_aux_acc/shift_regs
          DUT.base_mem_wrapper_inst.Sb_mem_acc_inst.TAU_S_base_mems[7].S_base_mem[j] = 'x; // all Xs DUT/u_mirath_top_lvl_ctrl_v2/u_aux_acc/shift_regs
          DUT.base_mem_wrapper_inst.Sb_mem_acc_inst.TAU_S_base_mems[8].S_base_mem[j] = 'x; // all Xs DUT/u_mirath_top_lvl_ctrl_v2/u_aux_acc/shift_regs
          DUT.base_mem_wrapper_inst.Sb_mem_acc_inst.TAU_S_base_mems[9].S_base_mem[j] = 'x; // all Xs DUT/u_mirath_top_lvl_ctrl_v2/u_aux_acc/shift_regs
          DUT.base_mem_wrapper_inst.Sb_mem_acc_inst.TAU_S_base_mems[10].S_base_mem[j] = 'x; // all Xs DUT/u_mirath_top_lvl_ctrl_v2/u_aux_acc/shift_regs
          DUT.base_mem_wrapper_inst.Sb_mem_acc_inst.TAU_S_base_mems[11].S_base_mem[j] = 'x; // all Xs DUT/u_mirath_top_lvl_ctrl_v2/u_aux_acc/shift_regs
          DUT.base_mem_wrapper_inst.Sb_mem_acc_inst.TAU_S_base_mems[35].S_base_mem[j] = 'x; // all Xs DUT/u_mirath_top_lvl_ctrl_v2/u_aux_acc/shift_regs
        end
//      end
    end
endtask

task clear_a_base_mem_to_x; // /tb_sign_verify/DUT/base_mem_wrapper_inst/Sb_mem_acc_inst/\TAU_S_base_mems[0].S_base_mem 
    begin
      for (int j = 0; j < `M_PARAM_RHO; j = j + 1) begin
//        DUT.a_mem_wrapper_inst.A_mem_acc_inst.TAU_A_mems[16].A_base_mem[j] = 'x; // all Xs DUT/u_mirath_top_lvl_ctrl_v2/u_aux_acc/shift_regs
        DUT.a_mem_wrapper_inst.A_mem_acc_inst.TAU_A_mems_x2[16].A_mem[j] = 'x; // all Xs DUT/u_mirath_top_lvl_ctrl_v2/u_aux_acc/shift_regs
      end
    end
endtask

  // ---------------- Helpers you asked for ----------------

  function automatic logic [63:0] rand64();
    rand64 = { $urandom(), $urandom() };
  endfunction

  task automatic program_random_inputs_and_save();
    begin
      // Randomize + save
      for (int i = 0; i < 4; i++) sk_seed[i] = rand64();
      for (int i = 0; i < 4; i++) pk_seed[i] = rand64();
      for (int i = 0; i < 8; i++) salt[i]    = rand64();
      for (int i = 0; i < 4; i++) root[i]    = rand64();

      // Program DUT memories (addresses come from included header macros)
      for (int i = 0; i < 4; i++)
        DUT.key_sign_mem_inst.mem[`SK_SEED_ADDR + i] = sk_seed[i];

      for (int i = 0; i < 4; i++)
        DUT.key_sign_mem_inst.mem[`PK_SEED_ADDR + i] = pk_seed[i];

      for (int i = 0; i < 8; i++)
        DUT.key_sign_mem_inst.mem[`SALT_ADDR + i]    = salt[i];

      for (int i = 0; i < 4; i++)
        DUT.data_mem_inst.ram[`ROOT_SEED_ADDR + i]  = root[i];
    end
  endtask

  task automatic reprogram_saved_inputs();
    begin
      for (int i = 0; i < 4; i++)
        DUT.key_sign_mem_inst.mem[`SK_SEED_ADDR + i] = sk_seed[i];

      for (int i = 0; i < 4; i++)
        DUT.key_sign_mem_inst.mem[`PK_SEED_ADDR + i] = pk_seed[i];

      for (int i = 0; i < 8; i++)
        DUT.key_sign_mem_inst.mem[`SALT_ADDR + i]    = salt[i];

      for (int i = 0; i < 4; i++)
        DUT.data_mem_inst.ram[`ROOT_SEED_ADDR + i]  = root[i];
    end
  endtask

  task automatic dump_saved_inputs();
    begin
      $display("---- DUMP (64-bit hex words) ----");

      $display("key_sig_mem SK_SEED_ADDR (%0d..%0d):", `SK_SEED_ADDR, `SK_SEED_ADDR + 1);
      for (int i = 0; i < 4; i++)
        $display("  mem[%0d] = %016h", `SK_SEED_ADDR + i, sk_seed[i]);

      $display("key_sig_mem PK_SEED_ADDR (%0d..%0d):", `PK_SEED_ADDR, `PK_SEED_ADDR + 1);
      for (int i = 0; i < 4; i++)
        $display("  mem[%0d] = %016h", `PK_SEED_ADDR + i, pk_seed[i]);

      $display("key_sig_mem SALT_ADDR (%0d..%0d):", `SALT_ADDR, `SALT_ADDR + 3);
      for (int i = 0; i < 8; i++)
        $display("  mem[%0d] = %016h", `SALT_ADDR + i, salt[i]);

      $display("data_mem ROOT_SEED_ADDR (%0d..%0d):", `ROOT_SEED_ADDR, `ROOT_SEED_ADDR + 1);
      for (int i = 0; i < 4; i++)
        $display("  ram[%0d] = %016h", `ROOT_SEED_ADDR + i, root[i]);

      $display("-------------------------------");
    end
  endtask
  
  task automatic dump_h_mpc();
    begin
      $display("---- h_mpc ----");for (int i = 0; i < 8; i++)
        $display("  mem[%0d] = %016h", `H_MPC_ADDR + i, DUT.key_sign_mem_inst.mem[`H_MPC_ADDR + i]);

      $display("-------------------------------");
    end
  endtask

  // ---------------- DUT + clock ----------------

  mirath_wrapper_v2 DUT (
    .rst     ( tb_rst   ),
    .clk     ( tb_clk   ),
    .start   ( tb_start ),
    .done    ( tb_done  ),
    .bad_sig ( tb_bad_sig )
  );

  always #(half_clk_period) tb_clk = ~tb_clk;

  // ---------------- Main loop ----------------

  initial begin
    // Optional override: +NUM_RUNS=<n>
    if (!$value$plusargs("NUM_RUNS=%d", num_runs)) num_runs = DEFAULT_NUM_RUNS;

    @(posedge tb_clk); // warm-up edge

    for (int run = 0; run < num_runs; run++) begin
      // Randomize BEFORE tb_rst goes high for SIGN test (as requested)
      program_random_inputs_and_save();

      // reset (same style as your original)
      #(2.1*clk_period);
      tb_rst = 1'b1;
      #(2*clk_period);
      tb_rst = 1'b0;

      // ----- SIGN -----
      wait (!tb_done);
      tb_start      = `START_SIGN;
      t_start_sign  = $time;
      #(1*clk_period);
      tb_start      = 'h0;

      wait (tb_done);
      t_done_sign   = $time;
      $display("[%0t] RUN %0d SIGN  : start=%0t done=%0t delta=%0t ps",
               $time, run, t_start_sign, t_done_sign, t_done_sign - t_start_sign);

      sign_sum_ns += (t_done_sign - t_start_sign);

      # (10*clk_period);

      // reset + clear between SIGN and VERIFY (your original behavior)
      tb_rst = 1'b1;
      #(2*clk_period);
      clear_data_mem_to_x; // Clear data_mem contents
      clear_aux_acc_to_x; // Clear aux regs contents
      clear_S_C_mem_to_x; // Clear S_C_mem contents
      clear_S_base_mem_to_x; // Clear S_base_mem contents
      clear_A_mem_to_x; // Clear A_mem contents
      clear_TMP_mem_to_x;

      // Re-program the saved randomized words (because data_mem was cleared to X)
      reprogram_saved_inputs();

      tb_rst = 1'b0;

      // ----- VERIFY -----
      wait (!tb_done);
      tb_start     = `START_VERIFY;
      dump_saved_inputs();
      dump_h_mpc();
      t_start_ver  = $time;
      #(1*clk_period);
      tb_start     = 'h0;
    
      wait (tb_done);
      t_done_ver   = $time;
      $display("[%0t] RUN %0d VERIFY: start=%0t done=%0t delta=%0t ps",
               $time, run, t_start_ver, t_done_ver, t_done_ver - t_start_ver);

      ver_sum_ns += (t_done_ver - t_start_ver);

      if (tb_bad_sig) begin
        $display("[%0t] RUN %0d VERIFY RESULT: FAIL (tb_bad_sig=1)", $time, run);
//        dump_saved_inputs();
        # (10*clk_period);
        $finish;
      end else begin
        $display("[%0t] RUN %0d VERIFY RESULT: PASS (tb_bad_sig=0)", $time, run);
      end

      # (10*clk_period);
    end

    // All runs passed if we got here
    $display("==== AVERAGES over %0d runs ====", num_runs);
    $display("AVG SIGN  delta = %0t ps", sign_sum_ns / num_runs);
    $display("AVG VERIFY delta = %0t ps", ver_sum_ns  / num_runs);

    $finish;
  end

endmodule
