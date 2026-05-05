`timescale 1ns / 1ps

`include "mirath_hw_params.vh"

`default_nettype none

module tb_sign_verify;

localparam clk_period = 10;
localparam half_clk_period = clk_period/2;
    
logic tb_rst = 0;
logic tb_clk = 0;

logic [1:0] tb_start = 0;
logic       tb_done;
logic       tb_bad_sig;

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

time t_start_sign, t_done_sign;
time t_start_ver,  t_done_ver;

mirath_wrapper_v2 DUT (
    .rst    ( tb_rst ),
    .clk    ( tb_clk ),

    .start  ( tb_start ),

    .done   ( tb_done ),
    .bad_sig( tb_bad_sig )
);

always #(half_clk_period) tb_clk = ~tb_clk;

initial begin
    @(posedge tb_clk); // one warm-up edge
    // reset
    #(2.1*clk_period);
    tb_rst = 1'b1;
    #(2*clk_period);
    tb_rst = 1'b0;
//    #(2*clk_period);
    
    tb_start = `START_SIGN;
//    tb_start = `START_KEYGEN;
    t_start_sign = $time; // start went high (asserted here)
    #(1*clk_period);
    tb_start = 'h0;
    
//    # (16500*clk_period);
//    # (320000*clk_period);
//    # (430400*clk_period);
    wait (tb_done);
    t_done_sign = $time;               // tb_done asserted here
    $display("[%0t] SIGN  : start=%0t done=%0t delta=%0t ps",
             $time, t_start_sign, t_done_sign, t_done_sign - t_start_sign);
    # (10*clk_period);
//    $finish;
    
    tb_rst = 1'b1;
    #(2*clk_period);
    clear_data_mem_to_x; // Clear data_mem contents
    clear_aux_acc_to_x; // Clear aux regs contents
    clear_S_C_mem_to_x; // Clear S_C_mem contents
    clear_S_base_mem_to_x; // Clear S_base_mem contents
    clear_A_mem_to_x; // Clear A_mem contents
    clear_TMP_mem_to_x;
    tb_rst = 1'b0;
    tb_start = `START_VERIFY;
    t_start_ver = $time;               // start asserted here
    #(1*clk_period);
    tb_start = 'h0;
    
    wait (tb_done);
    t_done_ver = $time;                // tb_done asserted here
    $display("[%0t] VERIFY: start=%0t done=%0t delta=%0t ps",
             $time, t_start_ver, t_done_ver, t_done_ver - t_start_ver);
             
    if (tb_bad_sig)
      $display("[%0t] VERIFY RESULT: FAIL (tb_bad_sig=1)", $time);
    else
      $display("[%0t] VERIFY RESULT: PASS (tb_bad_sig=0)", $time);
    # (10*clk_period);
    $finish;
    # (224000*clk_period);
end

endmodule
