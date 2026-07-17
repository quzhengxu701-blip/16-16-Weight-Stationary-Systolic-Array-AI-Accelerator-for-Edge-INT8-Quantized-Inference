`timescale 1ns/1ps
`include "uvm_macros.svh"
`include "qzx_tb_pkg.sv"
`include "qzx_interfaces.sv"
`include "qzx_transactions.sv"
`include "qzx_axil_agent.sv"
`include "qzx_axis_weight_agent.sv"
`include "qzx_axis_act_agent.sv"
`include "qzx_axis_result_agent.sv"
`include "qzx_scoreboard.sv"
`include "qzx_coverage_subscriber.sv"
`include "qzx_env.sv"
`include "qzx_sequences.sv"
`include "tests/qzx_base_test.sv"
`include "qzx_sva.sv"
`include "test_files.sv"

import uvm_pkg::*;
import qzx_pkg::*;
import qzx_tb_pkg::*;
 



module tb_top;
  
  logic clk,rst_n;
  logic irq_out,busy,done;
  logic [2:0] state_out;
  
  
  initial begin
    
    clk = 0;
    rst_n = 0;
    #100;
    rst_n = 1;
    
  end
  
  always #2.5 clk = ~ clk;
  
  
  //Interface instantiation
  qzx_axil_if axil_if(clk,rst_n);
  
  qzx_axis_weight_if weight_if(clk,rst_n);
  
  qzx_axis_activation_if act_if(clk,rst_n);
  
  qzx_axis_result_if result_if(clk,rst_n);
  
  qzx_dut_probes_if probe_if(clk);
  
  
  assign probe_if.state           = dut.u_control.u_compute.state_q;
  assign probe_if.wgt_buf_tile_ready = dut.wgt_buf_tile_ready;
  assign probe_if.done            = done;
  assign probe_if.mode_dense = dut.u_control.mode_dense;
  
  
  // ---- Fault injection bridge: probe_if.inject_parity_error -> DUT ----
  always @(posedge probe_if.clk) begin
    if (probe_if.inject_parity_error) begin
      $display("*** inject_parity_error detected — forcing parity_error for 1 cycle ***");
      // Assuming the DUT's parity error signal is at this path:
      force dut.u_control.u_csr.ctrl_abort = 1'b1;
      // Release on the next clock edge
      @(posedge probe_if.clk);
      release dut.u_control.u_csr.ctrl_abort;
      // Autonomously clear the injection flag so the test knows it fired
      probe_if.inject_parity_error <= 0;
    end
  end
    
  
  //DUT instantiation

  qzx_top 
     #(
       .ROWS_P(TB_ROWS), //16 replace with ROWS
       .COLS_P(TB_COLS),  //16 replace with COLS
       .WEIGHT_FIFO_DEP(8),
        .OUTPUT_FIFO_DEP(OFIFO_DEPTH),
        .ENABLE_ICG(1),
        .ENABLE_PARITY(1),
        .ENABLE_POSTPROC(1)
       ) dut (
       .clk(clk),
       .rst_n(rst_n),
       .scan_enable(1'b0),
       
       //AXIL Signals
       .s_axil_awaddr(axil_if.awaddr),
       .s_axil_awprot(axil_if.awprot),
       .s_axil_awvalid(axil_if.awvalid),
       .s_axil_awready(axil_if.awready),
       .s_axil_wdata(axil_if.wdata),
       .s_axil_wstrb(axil_if.wstrb),
       .s_axil_wvalid(axil_if.wvalid),
       .s_axil_wready(axil_if.wready),
       .s_axil_bresp(axil_if.bresp),
       .s_axil_bvalid(axil_if.bvalid),
       .s_axil_bready(axil_if.bready),
       .s_axil_araddr(axil_if.araddr),
       .s_axil_arprot(axil_if.arprot),
       .s_axil_arvalid(axil_if.arvalid),
       .s_axil_arready(axil_if.arready),
       .s_axil_rdata(axil_if.rdata),
       .s_axil_rresp(axil_if.rresp),
       .s_axil_rvalid(axil_if.rvalid),
       .s_axil_rready(axil_if.rready),
       
       //AXIS Weight Input
       .s_axis_weight_tdata(weight_if.tdata),
       .s_axis_weight_tkeep(weight_if.tkeep),
       .s_axis_weight_tlast(weight_if.tlast),
       .s_axis_weight_tuser(weight_if.tuser),
       .s_axis_weight_tvalid(weight_if.tvalid),
       .s_axis_weight_tready(weight_if.tready),
       
       //AXIS Activation Input
       .s_axis_act_tdata(act_if.tdata),
       .s_axis_act_tkeep(act_if.tkeep),
       .s_axis_act_tlast(act_if.tlast),
       .s_axis_act_tvalid(act_if.tvalid),
       .s_axis_act_tready(act_if.tready),
       
       //AXIS Result Output
       .m_axis_result_tdata(result_if.tdata),
       .m_axis_result_tkeep(result_if.tkeep),
       .m_axis_result_tlast(result_if.tlast),
       .m_axis_result_tuser(result_if.tuser),
       .m_axis_result_tvalid(result_if.tvalid),
       .m_axis_result_tready(result_if.tready),
       
       //Status Signals
       .irq_out (irq_out),
       .busy (busy),
       .done (done),
       .state_out (state_out)     
     );
 
  
  
  bind qzx_top qzx_sva u_sva(
    
    .clk(clk),
    .rst_n(rst_n),
    //AXIL
    .awvalid(s_axil_awvalid),
    .awready(s_axil_awready),
    .wvalid(s_axil_wvalid),
    .wready(s_axil_wready),
    .bready(s_axil_bready),
    .bvalid(s_axil_bvalid),
    .bresp(s_axil_bresp),
    .arvalid(s_axil_arvalid),
    .arready(s_axil_arready),
    .rvalid(s_axil_rvalid),
    .rresp(s_axil_rresp),
    .rready(s_axil_rready),
    //AXIS Weight
    .w_tvalid(s_axis_weight_tvalid),
    .w_tlast(s_axis_weight_tlast),
    .w_tdata(s_axis_weight_tdata),
    .w_tready(s_axis_weight_tready),
    //AXIS Activation
    .a_tvalid(s_axis_act_tvalid),
    .a_tlast(s_axis_act_tlast),
    .a_tdata(s_axis_act_tdata),
    .a_tready(s_axis_act_tready),
    //AXIS Result
    .r_tvalid(m_axis_result_tvalid),
    .r_tready(m_axis_result_tready),
    .r_tdata(m_axis_result_tdata),
    .r_tlast(m_axis_result_tlast),
    
    .wgt_tile_start(u_control.wgt_tile_start),
    .state(state_out),
    .compute_en(u_control.compute_en),
    .busy(busy),
    .sparsity_mode(dut.sparsity_cfg),
    .ofifo_full(dut.ofifo_full),
    .done(done)
  
  );
  

  initial begin
    
    //Configuration set
    uvm_config_db #(virtual qzx_axil_if)::set(null,"*","axil_if",axil_if);
    uvm_config_db #(virtual qzx_axis_weight_if)::set(null,"*","weight_if",weight_if);
    uvm_config_db #(virtual qzx_axis_activation_if)::set(null,"*","act_if",act_if);
    uvm_config_db #(virtual qzx_axis_result_if)::set(null,"*","result_if",result_if);
    uvm_config_db #(virtual qzx_dut_probes_if)::set(null,"*","probe_if",probe_if);
    
    
  end
  
  // =========================================================================
  // FSDB dumping — for Verdi waveform viewing
  // =========================================================================
  // VCS runtime option +fsdbfile+<filename> (set in Makefile) handles the
  // output file name automatically.  The calls below give explicit control
  // over dump depth and ensure MDA/array contents are included.
  initial begin
    $fsdbDumpvars(0, tb_top);   // 0 = dump entire tb_top hierarchy
    $fsdbDumpMDA();             // dump memory/MDA (buffers, arrays, etc.)
  end

  initial begin
    run_test();
  end
  
  initial begin
    #3000000;
    $finish();
    
  end
  
endmodule