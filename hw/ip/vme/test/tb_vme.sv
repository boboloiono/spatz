// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Pei-Yu Lin, ETH Zürich

module tb_vme;
  import vme_pkg::*;

  localparam time ClockPeriod = 10ns;
  localparam int unsigned TimeoutCycles = 100;

  logic clk;
  logic rst_n;

  request_t vme_req;
  logic vme_req_valid;
  logic vme_req_ready;
  logic [VME_NR_INFLIGHT-1:0] matrix_enable;

  logic ope_rsp_valid;
  logic ope_rsp_ready;
  response_t ope_rsp;

  vrf_addr_t vrf_waddr;
  vrf_data_t vrf_wdata;
  logic vrf_we;
  vrf_be_t vrf_wbe;
  logic vrf_wvalid;

  vrf_addr_t [2:0] vrf_raddr;
  logic [2:0] vrf_re;
  logic [2:0] vrf_rgnt;
  vrf_data_t [2:0] vrf_rdata;
  logic [2:0] vrf_rvalid;
  instruction_id_t [3:0] vrf_id;

  logic tile_wvalid;
  tile_write_req_t tile_w_req;
  logic tile_wready;
  logic tile_rvalid;
  tile_read_req_t tile_r_req;
  tile_word_t tile_rdata;
  logic tile_rready;

  vme i_dut (
    .clk_i            (clk),
    .rst_ni           (rst_n),
    .req_i        (vme_req),
    .req_valid_i  (vme_req_valid),
    .req_ready_o  (vme_req_ready),
    .matrix_enable_i  (matrix_enable),
    .rsp_valid_o  (ope_rsp_valid),
    .rsp_ready_i  (ope_rsp_ready),
    .rsp_o        (ope_rsp),
    .vrf_waddr_o      (vrf_waddr),
    .vrf_wdata_o      (vrf_wdata),
    .vrf_we_o         (vrf_we),
    .vrf_wbe_o        (vrf_wbe),
    .vrf_wvalid_i     (vrf_wvalid),
    .vrf_raddr_o      (vrf_raddr),
    .vrf_re_o         (vrf_re),
    .vrf_rgnt_i       (vrf_rgnt),
    .vrf_rdata_i      (vrf_rdata),
    .vrf_rvalid_i     (vrf_rvalid),
    .vrf_id_o         (vrf_id),
    .tile_wvalid_i    (tile_wvalid),
    .tile_w_req_i     (tile_w_req),
    .tile_wready_o    (tile_wready),
    .tile_rvalid_i    (tile_rvalid),
    .tile_r_req_i     (tile_r_req),
    .tile_rdata_o     (tile_rdata),
    .tile_rready_o    (tile_rready)
  );

  always #(ClockPeriod / 2) clk = ~clk;

  task write_tile_row(input mt_t tile, input vew_e tew, input tile_word_t data);
    int unsigned cycles;
    begin
      @(negedge clk);
      tile_w_req = '0;
      tile_w_req.idx = tile;
      tile_w_req.row = tile_dim_t'(0);
      tile_w_req.col = tile_dim_t'(0);
      tile_w_req.elems = tile_count_t'(VME_TILE_EDGE);
      tile_w_req.tew = tew;
      tile_w_req.data = data;
      tile_wvalid = 1'b1;

      cycles = 0;
      do begin
        @(posedge clk);
        cycles++;
        if (cycles == TimeoutCycles)
          $fatal(1, "Timed out waiting for tile write readiness");
      end while (!tile_wready);

      @(negedge clk);
      tile_wvalid = 1'b0;
      tile_w_req = '0;
    end
  endtask

  task read_and_check_tile_row(
    input mt_t tile,
    input vew_e tew,
    input tile_word_t expected
  );
    int unsigned cycles;
    begin
      @(negedge clk);
      tile_r_req = '0;
      tile_r_req.idx = tile;
      tile_r_req.row = tile_dim_t'(0);
      tile_r_req.col = tile_dim_t'(0);
      tile_r_req.elems = tile_count_t'(VME_TILE_EDGE);
      tile_r_req.tew = tew;
      tile_rvalid = 1'b1;

      cycles = 0;
      do begin
        @(posedge clk);
        cycles++;
        if (cycles == TimeoutCycles)
          $fatal(1, "Timed out waiting for tile read readiness");
      end while (!tile_rready);

      #1ns;
      if (tile_rdata !== expected)
        $fatal(1, "Tile row mismatch: expected %h, got %h", expected, tile_rdata);

      @(negedge clk);
      tile_rvalid = 1'b0;
      tile_r_req = '0;
    end
  endtask

  task issue_zero(input instruction_id_t id, input mt_t tile, input vew_e tew);
    int unsigned cycles;
    begin
      @(negedge clk);
      vme_req = '0;
      vme_req.id = id;
      vme_req.mtd = tile;
      vme_req.op_ope.is_zero_tile = 1'b1;
      vme_req.op_ope.tew = tew;
      vme_req_valid = 1'b1;

      cycles = 0;
      do begin
        @(posedge clk);
        cycles++;
        if (cycles == TimeoutCycles)
          $fatal(1, "Timed out waiting for command readiness");
      end while (!vme_req_ready);

      @(negedge clk);
      vme_req_valid = 1'b0;
      vme_req = '0;

      cycles = 0;
      do begin
        @(posedge clk);
        cycles++;
        if (cycles == TimeoutCycles)
          $fatal(1, "Timed out waiting for zero completion");
      end while (!ope_rsp_valid);

      if (ope_rsp.id !== id)
        $fatal(1, "Completion ID mismatch: expected %0d, got %0d", id, ope_rsp.id);
    end
  endtask

  initial begin : test_sequence
    tile_word_t pattern8, pattern16, pattern32;

    clk = 1'b0;
    rst_n = 1'b0;
    vme_req = '0;
    vme_req_valid = 1'b0;
    matrix_enable = '1;
    ope_rsp_ready = 1'b1;
    vrf_wvalid = 1'b1;
    vrf_rgnt = '1;
    vrf_rdata = '0;
    vrf_rvalid = '0;
    tile_wvalid = 1'b0;
    tile_w_req = '0;
    tile_rvalid = 1'b0;
    tile_r_req = '0;
    pattern8 = '0;
    pattern16 = '0;
    pattern32 = '0;

    for (int unsigned element = 0; element < VME_TILE_EDGE; element++) begin
      pattern8[element*8 +: 8] = 8'h20 + element;
      pattern16[element*16 +: 16] = 16'h3c00 + element;
      pattern32[element*32 +: 32] = 32'h3f80_0000 + element;
    end

    repeat (5) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    write_tile_row(mt_t'(0), EW_8, pattern8);
    write_tile_row(mt_t'(2), EW_16, pattern16);
    write_tile_row(mt_t'(4), EW_32, pattern32);
    read_and_check_tile_row(mt_t'(0), EW_8, pattern8);
    read_and_check_tile_row(mt_t'(2), EW_16, pattern16);
    read_and_check_tile_row(mt_t'(4), EW_32, pattern32);

    issue_zero(instruction_id_t'(3), mt_t'(2), EW_16);
    read_and_check_tile_row(mt_t'(2), EW_16, '0);
    read_and_check_tile_row(mt_t'(0), EW_8, pattern8);

    issue_zero(instruction_id_t'(4), mt_t'(0), EW_8);
    read_and_check_tile_row(mt_t'(0), EW_8, '0);

    issue_zero(instruction_id_t'(5), mt_t'(4), EW_32);
    read_and_check_tile_row(mt_t'(4), EW_32, '0);

    $display("[PASS] Standalone VME runtime-TEW tile write/read/zero smoke test");
    $finish;
  end

endmodule
