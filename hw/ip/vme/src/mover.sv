// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Pei-Yu Lin, ETH Zürich

// Sequences tile-to-vector and vector-to-tile transfers. Tile geometry and
// accumulator steering remain outside this unit; this block owns request
// lifetime, VRF response buffering, word counting, and retirement.
module mover
  import vme_pkg::*;
#(
  parameter int unsigned TileEdge          = VME_TILE_EDGE,
  parameter int unsigned AccWordWidth      = VME_ACC_WORD_WIDTH,
  parameter int unsigned AccWordBytes      = AccWordWidth >> 3,
  parameter int unsigned MaxVregGroupWords = VME_MAX_VREG_GROUP_WORDS
) (
  input  logic                                    clk_i,
  input  logic                                    rst_ni,

  input  request_t vt_req_i,
  input  logic     vt_accept_i,
  input  logic     vt_commit_ready_i,
  output request_t vt_req_o,
  output logic     vt_busy_o,
  output logic     vt_commit_valid_o,
  output logic [$clog2(MaxVregGroupWords)-1:0] vt_word_idx_o,
  output move_shape_t vt_shape_o,

  input  request_t tv_req_i,
  input  logic     tv_accept_i,
  input  logic     tv_commit_ready_i,
  output request_t tv_req_o,
  output logic     tv_busy_o,
  output logic     tv_commit_valid_o,
  output logic [$clog2(MaxVregGroupWords)-1:0] tv_word_idx_o,
  output move_shape_t tv_shape_o,

  input  logic      vrf_wvalid_i,
  input  logic      tv_vrf_rgnt_i,
  input  logic      tv_vrf_rvalid_i,
  input  vrf_data_t tv_vrf_rdata_i,
  output logic      tv_vrf_re_o,
  output logic [$clog2(MaxVregGroupWords):0] tv_vrf_word_idx_o,
  output logic      tv_acc_wen_o,
  output logic      tv_data_pending_o,
  output vrf_data_t tv_acc_wdata_o
);

  `include "common_cells/registers.svh"

  localparam int unsigned MaxWords = MaxVregGroupWords;
  request_t vt_req_q, tv_req_q;
  move_shape_t vt_shape_d, vt_shape_q, tv_shape_d, tv_shape_q;
  logic vt_busy_d, vt_busy_q, tv_busy_d, tv_busy_q;
  logic [$clog2(MaxWords)-1:0] vt_word_idx_d, vt_word_idx_q;
  logic [$clog2(MaxWords)-1:0] tv_word_idx_d, tv_word_idx_q;
  logic vt_word_fire, vt_finishing, tv_finishing;
  vrf_data_t tv_data_d, tv_data_q;
  logic tv_data_latched_d, tv_data_latched_q;
  logic tv_vrf_outstanding_d, tv_vrf_outstanding_q;

  always_comb begin : decode_move_shapes
    logic vt_line_valid, tv_line_valid;
    int unsigned vt_tew_bytes, tv_tew_bytes;
    int unsigned vt_elems_per_word, tv_elems_per_word;

    vt_tew_bytes = 1 << int'(vt_req_i.op_vme.tew);
    tv_tew_bytes = 1 << int'(tv_req_i.op_vme.tew);
    vt_elems_per_word = VME_VRF_WORD_BYTES / vt_tew_bytes;
    tv_elems_per_word = VME_VRF_WORD_BYTES / tv_tew_bytes;

    vt_shape_d = '0;
    vt_shape_d.line_elems =
        (vt_req_i.op_vme.tss.is_row ? vt_req_i.op_vme.tn : vt_req_i.op_vme.tm) > TileEdge ?
        tile_count_t'(TileEdge) :
        tile_count_t'(vt_req_i.op_vme.tss.is_row ? vt_req_i.op_vme.tn : vt_req_i.op_vme.tm);
    vt_line_valid = vt_req_i.op_vme.tss.is_row ?
                    (vt_req_i.op_vme.tm > vt_req_i.op_vme.tss.index) :
                    (vt_req_i.op_vme.tn > vt_req_i.op_vme.tss.index);
    if (vt_line_valid && (vt_shape_d.line_elems != 0) && (vt_req_i.vl != 0)) begin
      vt_shape_d.elems = (vt_req_i.vl < vt_shape_d.line_elems) ?
                         vt_req_i.vl : vlen_t'(vt_shape_d.line_elems);
      vt_shape_d.words =
          (vt_shape_d.elems + vt_elems_per_word - 1) / vt_elems_per_word;
    end

    tv_shape_d = '0;
    tv_shape_d.line_elems =
        (tv_req_i.op_vme.tss.is_row ? tv_req_i.op_vme.tn : tv_req_i.op_vme.tm) > TileEdge ?
        tile_count_t'(TileEdge) :
        tile_count_t'(tv_req_i.op_vme.tss.is_row ? tv_req_i.op_vme.tn : tv_req_i.op_vme.tm);
    tv_line_valid = tv_req_i.op_vme.tss.is_row ?
                    (tv_req_i.op_vme.tm > tv_req_i.op_vme.tss.index) :
                    (tv_req_i.op_vme.tn > tv_req_i.op_vme.tss.index);
    if (tv_line_valid && (tv_shape_d.line_elems != 0) && (tv_req_i.vl != 0)) begin
      tv_shape_d.elems = (tv_req_i.vl < tv_shape_d.line_elems) ?
                         tv_req_i.vl : vlen_t'(tv_shape_d.line_elems);
      tv_shape_d.words =
          (tv_shape_d.elems + tv_elems_per_word - 1) / tv_elems_per_word;
    end
  end : decode_move_shapes

  assign vt_word_fire = vt_busy_q && vt_req_q.use_vd && vrf_wvalid_i;
  assign vt_finishing = vt_busy_q && ((vt_shape_q.words == 0) ||
      (vt_word_fire && (({1'b0, vt_word_idx_q} + 1'b1) >= vt_shape_q.words)));
  assign vt_commit_valid_o = vt_finishing;

  always_comb begin : update_tile_to_vector
    vt_busy_d = vt_busy_q;
    vt_word_idx_d = vt_word_idx_q;
    if (vt_word_fire && !vt_finishing)
      vt_word_idx_d = vt_word_idx_q + 1'b1;
    if (vt_finishing && vt_commit_ready_i) begin
      vt_busy_d = vt_accept_i;
      vt_word_idx_d = '0;
    end else if (!vt_busy_q && vt_accept_i) begin
      vt_busy_d = 1'b1;
      vt_word_idx_d = '0;
    end
  end

  assign tv_acc_wen_o = tv_busy_q && tv_data_latched_q && !vt_busy_q;
  assign tv_vrf_word_idx_o = {1'b0, tv_word_idx_q} + tv_data_latched_q;
  assign tv_vrf_re_o = tv_busy_q && !tv_vrf_outstanding_q &&
                       (!tv_data_latched_q || tv_acc_wen_o) &&
                       (tv_vrf_word_idx_o < tv_shape_q.words);
  assign tv_finishing = tv_busy_q && ((tv_shape_q.words == 0) ||
      (tv_acc_wen_o && (({1'b0, tv_word_idx_q} + 1'b1) >= tv_shape_q.words)));
  assign tv_commit_valid_o = tv_finishing;

  always_comb begin : update_vector_to_tile
    tv_busy_d = tv_busy_q;
    tv_word_idx_d = tv_word_idx_q;
    if (tv_acc_wen_o && !tv_finishing)
      tv_word_idx_d = tv_word_idx_q + 1'b1;
    if (tv_finishing && tv_commit_ready_i) begin
      tv_busy_d = tv_accept_i;
      tv_word_idx_d = '0;
    end else if (!tv_busy_q && tv_accept_i) begin
      tv_busy_d = 1'b1;
      tv_word_idx_d = '0;
    end
  end

  always_comb begin : buffer_vector_data
    tv_data_d = tv_data_q;
    tv_data_latched_d = tv_data_latched_q;
    if (!tv_busy_q) begin
      tv_data_latched_d = 1'b0;
    end else begin
      if (tv_acc_wen_o)
        tv_data_latched_d = 1'b0;
      if (tv_vrf_rvalid_i && (!tv_data_latched_q || tv_acc_wen_o)) begin
        tv_data_d = tv_vrf_rdata_i;
        tv_data_latched_d = 1'b1;
      end
    end
  end

  always_comb begin : track_vrf_request
    tv_vrf_outstanding_d = tv_vrf_outstanding_q;
    if (!tv_busy_q || tv_vrf_rvalid_i)
      tv_vrf_outstanding_d = 1'b0;
    else if (tv_vrf_rgnt_i)
      tv_vrf_outstanding_d = 1'b1;
  end

  `FFL(vt_req_q, vt_req_i, vt_accept_i, request_t'(0))
  `FFL(vt_shape_q, vt_shape_d, vt_accept_i, move_shape_t'(0))
  `FF(vt_busy_q, vt_busy_d, 1'b0)
  `FF(vt_word_idx_q, vt_word_idx_d, '0)
  `FFL(tv_req_q, tv_req_i, tv_accept_i, request_t'(0))
  `FFL(tv_shape_q, tv_shape_d, tv_accept_i, move_shape_t'(0))
  `FF(tv_busy_q, tv_busy_d, 1'b0)
  `FF(tv_word_idx_q, tv_word_idx_d, '0)
  `FF(tv_data_q, tv_data_d, vrf_data_t'(0))
  `FF(tv_data_latched_q, tv_data_latched_d, 1'b0)
  `FF(tv_vrf_outstanding_q, tv_vrf_outstanding_d, 1'b0)

  assign vt_req_o = vt_req_q;
  assign vt_busy_o = vt_busy_q;
  assign vt_word_idx_o = vt_word_idx_q;
  assign vt_shape_o = vt_shape_q;
  assign tv_req_o = tv_req_q;
  assign tv_busy_o = tv_busy_q;
  assign tv_word_idx_o = tv_word_idx_q;
  assign tv_shape_o = tv_shape_q;
  assign tv_acc_wdata_o = tv_data_q;
  assign tv_data_pending_o = tv_data_latched_q;

endmodule : mover
