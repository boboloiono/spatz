// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Pei-Yu Lin, ETH Zürich

module tb_vme_scoreboard;
  timeunit 1ns;
  timeprecision 1ps;

  import vme_pkg::*;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  request_t dispatch_req;
  logic dispatch_valid;
  deps_t dispatch_dependencies;
  logic [3:0] complete_valid;
  instruction_id_t [3:0] complete_id;
  deps_t outstanding;

  always #5ns clk = ~clk;

  tile_scoreboard i_dut (
    .clk_i                    (clk),
    .rst_ni                   (rst_n),
    .dispatch_req_i           (dispatch_req),
    .dispatch_valid_i         (dispatch_valid),
    .dispatch_deps_o          (dispatch_dependencies),
    .dispatch_generations_o   (),
    .complete_valid_i         (complete_valid),
    .complete_id_i            (complete_id),
    .outstanding_o            (outstanding),
    .generations_o            ()
  );

  task dispatch(input request_t req);
    dispatch_req = req;
    dispatch_valid = 1'b1;
    @(posedge clk);
    #1ps;
    dispatch_valid = 1'b0;
  endtask

  initial begin
    request_t req;
    dispatch_req = '0;
    dispatch_valid = 1'b0;
    complete_valid = '0;
    complete_id = '0;

    repeat (2) @(posedge clk);
    rst_n = 1'b1;

    // ID 0 performs a read-modify-write on mt0.
    req = '0;
    req.id = instruction_id_t'(0);
    req.mtd = mt_t'(0);
    req.op_ope.is_mac = 1'b1;
    #1ps;
    assert (dispatch_dependencies == '0);
    dispatch(req);

    // A younger tile read has a RAW dependency on ID 0.
    req = '0;
    req.id = instruction_id_t'(1);
    req.use_vd = 1'b1;
    req.op_ope.tss.tile_valid = 1'b1;
    req.op_ope.tss.tile_id = mt_t'(0);
    dispatch_req = req;
    #1ps;
    assert (dispatch_dependencies == deps_t'(1 << 0));
    dispatch(req);

    // A write waits for both the older writer and older reader (WAW + WAR).
    req = '0;
    req.id = instruction_id_t'(2);
    req.use_vs2 = 1'b1;
    req.op_ope.tss.tile_valid = 1'b1;
    req.op_ope.tss.tile_id = mt_t'(0);
    dispatch_req = req;
    #1ps;
    assert (dispatch_dependencies == deps_t'((1 << 0) | (1 << 1)));

    // A different tile remains independent.
    req = '0;
    req.id = instruction_id_t'(3);
    req.mtd = mt_t'(1);
    req.op_ope.is_mac = 1'b1;
    dispatch_req = req;
    #1ps;
    assert (dispatch_dependencies == '0);

    complete_id[0] = instruction_id_t'(0);
    complete_valid[0] = 1'b1;
    @(posedge clk);
    #1ps;
    complete_valid = '0;
    assert (!outstanding[0] && outstanding[1]);

    $display("[PASS] VME scoreboard RAW/WAR/WAW dependency test");
    $finish;
  end

endmodule : tb_vme_scoreboard
