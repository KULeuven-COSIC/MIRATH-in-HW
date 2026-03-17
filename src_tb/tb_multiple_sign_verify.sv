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
  logic [63:0] sk_seed [0:1];
  logic [63:0] pk_seed [0:1];
  logic [63:0] salt    [0:3];
  logic [63:0] root    [0:1];

  longint unsigned sign_sum_ns = 0;
  longint unsigned ver_sum_ns  = 0;

  int num_runs;

  // ---------------- Existing clear tasks ----------------

  task clear_data_mem_to_x;
    begin
      for (int i = 0; i < `DATA_MEM_DEPTH; i = i + 1)
        DUT.data_mem_inst.ram[i] = 'x;
    end
  endtask

  task clear_aux_acc_to_x;
    begin
      for (int i = 0; i < 8; i = i + 1)
        DUT.u_mirath_top_lvl_ctrl_v2.u_aux_acc.shift_regs[i] = 'x;
    end
  endtask

  task clear_S_C_mem_to_x;
    begin
      for (int i = 0; i < 8; i = i + 1)
        DUT.u_mirath_top_lvl_ctrl_v2.S_C_mem.ram[i] = 'x;
    end
  endtask

  task clear_S_base_mem_to_x;
    begin
      for (int j = 0; j < 44; j = j + 1)
        DUT.base_mem_wrapper_inst.Sb_mem_acc_inst.TAU_S_base_mems[0].S_base_mem[j] = 'x;
    end
  endtask

  task clear_a_base_mem_to_x;
    begin
      for (int j = 0; j < `M_PARAM_RHO; j = j + 1)
        DUT.a_mem_wrapper_inst.A_mem_acc_inst.TAU_A_mems[16].A_base_mem[j] = 'x;
    end
  endtask

  // ---------------- Helpers you asked for ----------------

  function automatic logic [63:0] rand64();
    rand64 = { $urandom(), $urandom() };
  endfunction

  task automatic program_random_inputs_and_save();
    begin
      // Randomize + save
      for (int i = 0; i < 2; i++) sk_seed[i] = rand64();
      for (int i = 0; i < 2; i++) pk_seed[i] = rand64();
      for (int i = 0; i < 4; i++) salt[i]    = rand64();
      for (int i = 0; i < 2; i++) root[i]    = rand64();

      // Program DUT memories (addresses come from included header macros)
      for (int i = 0; i < 2; i++)
        DUT.key_sign_mem_inst.mem[`SK_SEED_ADDR + i] = sk_seed[i];

      for (int i = 0; i < 2; i++)
        DUT.key_sign_mem_inst.mem[`PK_SEED_ADDR + i] = pk_seed[i];

      for (int i = 0; i < 4; i++)
        DUT.key_sign_mem_inst.mem[`SALT_ADDR + i]    = salt[i];

      for (int i = 0; i < 2; i++)
        DUT.data_mem_inst.ram[`ROOT_SEED_ADDR + i]  = root[i];
    end
  endtask

  task automatic reprogram_saved_inputs();
    begin
      for (int i = 0; i < 2; i++)
        DUT.key_sign_mem_inst.mem[`SK_SEED_ADDR + i] = sk_seed[i];

      for (int i = 0; i < 2; i++)
        DUT.key_sign_mem_inst.mem[`PK_SEED_ADDR + i] = pk_seed[i];

      for (int i = 0; i < 4; i++)
        DUT.key_sign_mem_inst.mem[`SALT_ADDR + i]    = salt[i];

      for (int i = 0; i < 2; i++)
        DUT.data_mem_inst.ram[`ROOT_SEED_ADDR + i]  = root[i];
    end
  endtask

  task automatic dump_saved_inputs();
    begin
      $display("---- DUMP (64-bit hex words) ----");

      $display("key_sig_mem SK_SEED_ADDR (%0d..%0d):", `SK_SEED_ADDR, `SK_SEED_ADDR + 1);
      for (int i = 0; i < 2; i++)
        $display("  mem[%0d] = %016h", `SK_SEED_ADDR + i, sk_seed[i]);

      $display("key_sig_mem PK_SEED_ADDR (%0d..%0d):", `PK_SEED_ADDR, `PK_SEED_ADDR + 1);
      for (int i = 0; i < 2; i++)
        $display("  mem[%0d] = %016h", `PK_SEED_ADDR + i, pk_seed[i]);

      $display("key_sig_mem SALT_ADDR (%0d..%0d):", `SALT_ADDR, `SALT_ADDR + 3);
      for (int i = 0; i < 4; i++)
        $display("  mem[%0d] = %016h", `SALT_ADDR + i, salt[i]);

      $display("data_mem ROOT_SEED_ADDR (%0d..%0d):", `ROOT_SEED_ADDR, `ROOT_SEED_ADDR + 1);
      for (int i = 0; i < 2; i++)
        $display("  ram[%0d] = %016h", `ROOT_SEED_ADDR + i, root[i]);

      $display("-------------------------------");
    end
  endtask
  
  task automatic dump_h_mpc();
    begin
      $display("---- h_mpc ----");for (int i = 0; i < 4; i++)
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

      // reset + clear between SIGN and VERIFY
      tb_rst = 1'b1;
      #(2*clk_period);
      clear_data_mem_to_x;
      clear_aux_acc_to_x;
      clear_S_C_mem_to_x;
      clear_S_base_mem_to_x;
      clear_a_base_mem_to_x;

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