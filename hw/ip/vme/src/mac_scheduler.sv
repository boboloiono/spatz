// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Pei-Yu Lin, ETH Zürich

// VTFMM instruction buffering, spatial/reduction beat sequencing, and VRF
// operand lookahead.  FPU latency, accumulator forwarding, and result
// write-back belong to the compute array.
module mac_scheduler
  import vme_pkg::*;
#(
  parameter int unsigned ComputeEdge  = VME_COMPUTE_EDGE,
  parameter int unsigned TileEdge     = VME_TILE_EDGE,
  parameter int unsigned EdgeBeats    = 2,
  parameter int unsigned SpatialBeats = EdgeBeats * EdgeBeats,
  parameter int unsigned AccWordWidth = VME_ACC_WORD_WIDTH,
  parameter int unsigned AccWordBytes = AccWordWidth >> 3
) (
  input  logic                                    clk_i,
  input  logic                                    rst_ni,

  input  request_t                                req_i,
  input  logic                                    req_valid_i,
  output logic                                    req_ready_o,

  input  logic       [1:0]                       vrf_rvalid_i,
  output logic                                    vrf_read_valid_o,
  output vrf_addr_t [1:0]                        vrf_read_addr_o,

  input  logic                                    commit_ready_i,
  output logic                                    commit_valid_o,
  output instruction_id_t                         commit_id_o,

  input  logic                                    tile_read_valid_i,
  input  mt_t                                     tile_read_idx_i,
  input  logic [VME_NR_LOGICAL_TILES-1:0][SpatialBeats-1:0] pipe_tile_busy_i,
  input  logic                                    pipe_result_valid_i,
  input  mt_t                                     pipe_result_tile_i,
  input  logic [((SpatialBeats > 1) ? $clog2(SpatialBeats) : 1)-1:0]
         pipe_result_beat_i,

  output logic                                    fire_o,
  output logic                                    last_beat_o,
  output logic                                    empty_complete_o,
  output logic                                    busy_o,
  output logic [VME_NR_LOGICAL_TILES-1:0] pending_tiles_o,

  output instruction_id_t id_o,
  output mt_t             tile_o,
  output vew_e            sew_o,
  output vew_e            tew_o,
  output logic [2:0]      tk_o,
  output logic [((SpatialBeats > 1) ? $clog2(SpatialBeats) : 1)-1:0]
               beat_o,
  output logic [((EdgeBeats > 1) ? $clog2(EdgeBeats) : 1)-1:0] edge_row_o,
  output logic [((EdgeBeats > 1) ? $clog2(EdgeBeats) : 1)-1:0] edge_col_o,
  output logic [ComputeEdge-1:0][ComputeEdge-1:0] lane_active_o,
  output logic use_sdotp_o
);

  `include "common_cells/registers.svh"

  localparam int unsigned EdgeBeatW     = EdgeBeats > 1 ? $clog2(EdgeBeats) : 1;
  localparam int unsigned SpatialBeatW  = SpatialBeats > 1 ? $clog2(SpatialBeats) : 1;
  localparam int unsigned ReductionIdxW = VME_K_MAX > 1 ? $clog2(VME_K_MAX) : 1;

  typedef logic [$clog2(EdgeBeats + 1)-1:0] edge_beat_count_t;
  typedef logic [5:0] operand_width_t;

  typedef struct packed {
    instruction_id_t id;
    vrf_addr_t       vs1_base;
    vrf_addr_t       vs2_base;
    vew_e            sew;
    vew_e            tew;
    mt_t             tile;
    tile_count_t     tm;
    tile_count_t     tn;
    logic [2:0]      tk;
  } mac_op_t;

  typedef struct packed {
    mac_op_t op;
    logic    valid;
  } mac_op_slot_t;

  typedef enum logic [2:0] {
    MacRdHold,
    MacRdSpatial,
    MacRdReduction,
    MacRdNextOp,
    MacRdNewKeep,
    MacRdNewReset
  } mac_rd_sel_e;

  mac_op_slot_t current_d, current_q;
  mac_op_slot_t next_d, next_q;
  logic [SpatialBeatW-1:0] beat_d, beat_q;
  logic [ReductionIdxW-1:0] reduction_d, reduction_q;

  mac_op_t new_op;
  logic current_empty, op_empty, exec_valid;
  logic has_more_spatial, has_more_reductions, has_more_beats;
  logic step_fire, nonfpu_ready;
  logic result_match, tile_hazard;

  logic [EdgeBeatW-1:0] edge_row, edge_col;
  edge_beat_count_t edge_row_next, edge_col_next;
  edge_beat_count_t m_edge_beats, n_edge_beats;
  tile_count_t active_tm, active_tn;
  logic [SpatialBeatW-1:0] next_spatial_beat;

  logic rd_valid_d, rd_valid_q;
  vrf_addr_t [1:0] rd_addr_d, rd_addr_q;
  mac_rd_sel_e rd_sel;
  logic rd_cur_valid, rd_next_valid, rd_new_valid;
  mac_op_t [5:0] rd_candidate_op;
  logic [5:0][SpatialBeatW-1:0] rd_candidate_beat;
  logic [5:0][ReductionIdxW-1:0] rd_candidate_reduction;
  vrf_addr_t [5:0][1:0] rd_candidate_addr;

  assign new_op = '{
    id      : req_i.id,
    vs1_base: vrf_addr_t'(req_i.vs1) * VME_WORDS_PER_VREG,
    vs2_base: vrf_addr_t'(req_i.vs2) * VME_WORDS_PER_VREG,
    sew     : req_i.vtype.vsew,
    tew     : req_i.op_vme.tew,
    tile    : req_i.mtd,
    tm      : req_i.op_vme.tm > TileEdge ? tile_count_t'(TileEdge) : req_i.op_vme.tm,
    tn      : req_i.op_vme.tn > TileEdge ? tile_count_t'(TileEdge) : req_i.op_vme.tn,
    tk      : req_i.op_vme.tk
  };

  assign edge_row = EdgeBeatW'(beat_q / EdgeBeats);
  assign edge_col = EdgeBeatW'(beat_q % EdgeBeats);
  assign edge_row_next = {1'b0, edge_row} + 1'b1;
  assign edge_col_next = {1'b0, edge_col} + 1'b1;

  always_comb begin : active_shape
    active_tm = current_q.op.tm < TileEdge ? current_q.op.tm : tile_count_t'(TileEdge);
    active_tn = current_q.op.tn < TileEdge ? current_q.op.tn : tile_count_t'(TileEdge);
    m_edge_beats = (active_tm + ComputeEdge - 1) / ComputeEdge;
    n_edge_beats = (active_tn + ComputeEdge - 1) / ComputeEdge;
    next_spatial_beat = beat_q;

    if (edge_col_next < n_edge_beats)
      next_spatial_beat = beat_q + 1'b1;
    else
      next_spatial_beat = SpatialBeatW'(edge_row_next * EdgeBeats);

    lane_active_o = '0;
    for (int unsigned row = 0; row < ComputeEdge; row++) begin
      for (int unsigned col = 0; col < ComputeEdge; col++) begin
        lane_active_o[row][col] =
            ((edge_row * ComputeEdge + row) < active_tm) &&
            ((edge_col * ComputeEdge + col) < active_tn);
      end
    end
  end

  assign result_match = pipe_result_valid_i &&
                        (pipe_result_tile_i == current_q.op.tile) &&
                        (pipe_result_beat_i == beat_q);
  assign tile_hazard = pipe_tile_busy_i[current_q.op.tile][beat_q] && !result_match;
  assign current_empty = (current_q.op.tm == tile_count_t'(0)) ||
                         (current_q.op.tn == tile_count_t'(0)) ||
                         (current_q.op.tk == 3'd0);
  assign exec_valid = current_q.valid &&
      (current_empty || (!tile_hazard &&
       !(tile_read_valid_i && (tile_read_idx_i == current_q.op.tile))));
  assign op_empty = exec_valid && current_empty;

  assign use_sdotp_o = current_q.op.sew != current_q.op.tew;
  assign has_more_spatial = (edge_col_next < n_edge_beats) ||
                            (edge_row_next < m_edge_beats);
  assign has_more_reductions = !use_sdotp_o &&
      (({1'b0, reduction_q} + 1'b1) < current_q.op.tk);
  assign has_more_beats = has_more_spatial || has_more_reductions;

  assign nonfpu_ready = (&vrf_rvalid_i) && (has_more_beats || commit_ready_i);
  assign step_fire = (op_empty && commit_ready_i) ||
                     (exec_valid && !op_empty && nonfpu_ready);
  assign fire_o = exec_valid && !op_empty && nonfpu_ready;
  assign last_beat_o = !has_more_beats;
  assign commit_valid_o = op_empty ||
      (exec_valid && !op_empty && !has_more_beats && (&vrf_rvalid_i));
  assign commit_id_o = current_q.op.id;
  assign empty_complete_o = op_empty && commit_valid_o && commit_ready_i;

  assign req_ready_o = req_valid_i &&
      !(tile_read_valid_i && (tile_read_idx_i == req_i.mtd)) &&
      (!next_q.valid || (commit_valid_o && commit_ready_i));

  always_comb begin : operation_update
    current_d = current_q;
    next_d = next_q;
    beat_d = beat_q;
    reduction_d = reduction_q;

    if (step_fire) begin
      if (op_empty) begin
        current_d = next_q;
        next_d = '0;
        beat_d = '0;
        reduction_d = '0;
      end else if (has_more_spatial) begin
        beat_d = next_spatial_beat;
      end else if (has_more_reductions) begin
        beat_d = '0;
        reduction_d = reduction_q + 1'b1;
      end else begin
        current_d = next_q;
        next_d = '0;
        beat_d = '0;
        reduction_d = '0;
      end
    end

    if (req_valid_i && req_ready_o) begin
      if (!current_q.valid) begin
        current_d.op = new_op;
        current_d.valid = 1'b1;
      end else if (commit_valid_o && commit_ready_i && !next_q.valid) begin
        current_d.op = new_op;
        current_d.valid = 1'b1;
        beat_d = '0;
        reduction_d = '0;
      end else begin
        next_d.op = new_op;
        next_d.valid = 1'b1;
      end
    end
  end

  `FF(current_q, current_d, mac_op_slot_t'(0))
  `FF(next_q, next_d, mac_op_slot_t'(0))
  `FF(beat_q, beat_d, '0)
  `FF(reduction_q, reduction_d, '0)

  assign rd_cur_valid = current_q.valid && !current_empty;
  assign rd_next_valid = next_q.valid &&
      (next_q.op.tm != tile_count_t'(0)) &&
      (next_q.op.tn != tile_count_t'(0)) && (next_q.op.tk != 3'd0);
  assign rd_new_valid = (new_op.tm != tile_count_t'(0)) &&
      (new_op.tn != tile_count_t'(0)) && (new_op.tk != 3'd0);

  assign rd_candidate_op[MacRdHold] = current_q.op;
  assign rd_candidate_op[MacRdSpatial] = current_q.op;
  assign rd_candidate_op[MacRdReduction] = current_q.op;
  assign rd_candidate_op[MacRdNextOp] = next_q.op;
  assign rd_candidate_op[MacRdNewKeep] = new_op;
  assign rd_candidate_op[MacRdNewReset] = new_op;
  assign rd_candidate_beat[MacRdHold] = beat_q;
  assign rd_candidate_beat[MacRdSpatial] = next_spatial_beat;
  assign rd_candidate_beat[MacRdReduction] = '0;
  assign rd_candidate_beat[MacRdNextOp] = '0;
  assign rd_candidate_beat[MacRdNewKeep] = beat_q;
  assign rd_candidate_beat[MacRdNewReset] = '0;
  assign rd_candidate_reduction[MacRdHold] = reduction_q;
  assign rd_candidate_reduction[MacRdSpatial] = reduction_q;
  assign rd_candidate_reduction[MacRdReduction] = reduction_q + 1'b1;
  assign rd_candidate_reduction[MacRdNextOp] = '0;
  assign rd_candidate_reduction[MacRdNewKeep] = reduction_q;
  assign rd_candidate_reduction[MacRdNewReset] = '0;

  always_comb begin : read_address_candidates
    for (int unsigned candidate = 0; candidate < 6; candidate++) begin
      logic [3:0] row_stride_regs;
      operand_width_t operand_bits;
      vrf_addr_t reduction_word_offset, row_word_offset, col_word_offset;
      logic [EdgeBeatW-1:0] candidate_edge_row, candidate_edge_col;
      logic candidate_sdotp;

      candidate_edge_row = EdgeBeatW'(rd_candidate_beat[candidate] / EdgeBeats);
      candidate_edge_col = EdgeBeatW'(rd_candidate_beat[candidate] % EdgeBeats);
      candidate_sdotp = rd_candidate_op[candidate].sew !=
                        rd_candidate_op[candidate].tew;
      unique case (rd_candidate_op[candidate].sew)
        EW_8: begin
          operand_bits = 6'd8;
          row_stride_regs = 4'd2;
        end
        EW_16: begin
          operand_bits = 6'd16;
          row_stride_regs = 4'd4;
        end
        default: begin
          operand_bits = 6'd32;
          row_stride_regs = 4'd8;
        end
      endcase
      if (candidate_sdotp)
        operand_bits = 6'(1 << (3 +
                               int'(rd_candidate_op[candidate].tew)));
      reduction_word_offset = candidate_sdotp ? '0 :
          vrf_addr_t'(rd_candidate_reduction[candidate]) *
          row_stride_regs * VME_WORDS_PER_VREG;
      row_word_offset = vrf_addr_t'((candidate_edge_row * ComputeEdge * operand_bits) /
                                     VME_VRF_WORD_WIDTH);
      col_word_offset = vrf_addr_t'((candidate_edge_col * ComputeEdge * operand_bits) /
                                     VME_VRF_WORD_WIDTH);
      rd_candidate_addr[candidate][0] =
          rd_candidate_op[candidate].vs2_base + reduction_word_offset + row_word_offset;
      rd_candidate_addr[candidate][1] =
          rd_candidate_op[candidate].vs1_base + reduction_word_offset + col_word_offset;
    end
  end

  always_comb begin : read_lookahead
    rd_sel = MacRdHold;
    if (step_fire) begin
      if (op_empty)
        rd_sel = MacRdNextOp;
      else if (has_more_spatial)
        rd_sel = MacRdSpatial;
      else if (has_more_reductions)
        rd_sel = MacRdReduction;
      else
        rd_sel = MacRdNextOp;
    end
    if (req_valid_i && req_ready_o) begin
      if (!current_q.valid)
        rd_sel = MacRdNewKeep;
      else if (commit_valid_o && commit_ready_i && !next_q.valid)
        rd_sel = MacRdNewReset;
    end

    unique case (rd_sel)
      MacRdSpatial: begin
        rd_valid_d = rd_cur_valid;
        rd_addr_d = rd_candidate_addr[MacRdSpatial];
      end
      MacRdReduction: begin
        rd_valid_d = rd_cur_valid;
        rd_addr_d = rd_candidate_addr[MacRdReduction];
      end
      MacRdNextOp: begin
        rd_valid_d = rd_next_valid;
        rd_addr_d = rd_candidate_addr[MacRdNextOp];
      end
      MacRdNewKeep: begin
        rd_valid_d = rd_new_valid;
        rd_addr_d = rd_candidate_addr[MacRdNewKeep];
      end
      MacRdNewReset: begin
        rd_valid_d = rd_new_valid;
        rd_addr_d = rd_candidate_addr[MacRdNewReset];
      end
      default: begin
        rd_valid_d = rd_cur_valid;
        rd_addr_d = rd_candidate_addr[MacRdHold];
      end
    endcase
    if (!rd_valid_d)
      rd_addr_d = '0;
  end

  `FF(rd_valid_q, rd_valid_d, 1'b0)
  `FF(rd_addr_q, rd_addr_d, '0)

  always_comb begin : pending_tiles
    pending_tiles_o = '0;
    if (current_q.valid)
      pending_tiles_o[current_q.op.tile] = 1'b1;
    if (next_q.valid)
      pending_tiles_o[next_q.op.tile] = 1'b1;
  end

  assign vrf_read_valid_o = rd_valid_q;
  assign vrf_read_addr_o = rd_addr_q;
  assign busy_o = current_q.valid || next_q.valid;
  assign id_o = current_q.op.id;
  assign tile_o = current_q.op.tile;
  assign sew_o = current_q.op.sew;
  assign tew_o = current_q.op.tew;
  assign tk_o = current_q.op.tk;
  assign beat_o = beat_q;
  assign edge_row_o = edge_row;
  assign edge_col_o = edge_col;

  if ((ComputeEdge == 0) || ((ComputeEdge & (ComputeEdge-1)) != 0))
    $error("[mac_scheduler] ComputeEdge must be a power of two.");
  if ((ComputeEdge > TileEdge) || ((TileEdge % ComputeEdge) != 0))
    $error("[mac_scheduler] ComputeEdge must divide TileEdge.");

endmodule : mac_scheduler
