// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Pei-Yu Lin, ETH Zürich
//
// Vector-Matrix Engine command scheduler and unit integration.

// The accumulator is banked by ComputeEdge x ComputeEdge lanes. A spatial beat
// selects one compute-array region of a logical tile; a resident result holds the newest
// pipeline output until it can be forwarded to the next MAC or written back.

module vme
  import vme_pkg::*;
  import fpnew_pkg::*;
#(
  parameter int unsigned ComputeEdge = VME_COMPUTE_EDGE,
  parameter int unsigned TileEdge = VME_TILE_EDGE,
  parameter int unsigned AccWordWidth = VME_ACC_WORD_WIDTH,
  parameter int unsigned AccWordBytes = AccWordWidth >> 3,
  parameter int unsigned SpatialBeats = 4,
  parameter int unsigned MaxVregGroupWords = VME_MAX_VREG_GROUP_WORDS,
  parameter int unsigned NumPipeRegs = VME_FPU_PIPE_REGS,
  parameter bit EnableFMA16 = VME_ENABLE_FMA16,
  parameter bit EnableFMA32 = VME_ENABLE_FMA32
) (
  input  logic             clk_i              ,
  input  logic             rst_ni             ,

  // interface with Controller
  input  request_t         req_i          ,
  input  logic             req_valid_i    ,
  output logic             req_ready_o    ,
  input  logic [VME_NR_INFLIGHT-1:0] matrix_enable_i,

  output logic             rsp_valid_o    ,
  input  logic             rsp_ready_i    ,
  output response_t         rsp_o          ,

  // interface with VRF
  output vrf_addr_t        vrf_waddr_o        ,
  output vrf_data_t        vrf_wdata_o        ,
  output logic             vrf_we_o           ,
  output vrf_be_t          vrf_wbe_o          ,
  input  logic             vrf_wvalid_i       ,

  output vrf_addr_t  [2:0] vrf_raddr_o        ,
  output logic       [2:0] vrf_re_o           ,
  input  logic       [2:0] vrf_rgnt_i         ,
  input  vrf_data_t  [2:0] vrf_rdata_i        ,
  input  logic       [2:0] vrf_rvalid_i       ,
  output instruction_id_t    [3:0] vrf_id_o           ,

  // interface with VLSU
  input  logic             tile_wvalid_i      ,
  input  tile_write_req_t  tile_w_req_i       ,
  output logic             tile_wready_o      ,
  input  logic             tile_rvalid_i      ,
  input  tile_read_req_t  tile_r_req_i       ,
  output tile_word_t       tile_rdata_o       ,
  output logic             tile_rready_o
);

`include "common_cells/registers.svh"

  // MAC latency is owned by the compute array.  This integration layer only
  // tracks architectural tile state and cross-unit dependencies.
  localparam int unsigned EdgeBeats = TileEdge / ComputeEdge;
  localparam int unsigned EdgeBeatW = EdgeBeats > 1 ? $clog2(EdgeBeats) : 1;
  localparam int unsigned SpatialBeatW = (SpatialBeats > 1) ? $clog2(SpatialBeats) : 1;
  /*------------------------------------------------------------*/
  /*                  Shared Types and Signals                  */
  /*------------------------------------------------------------*/

  typedef logic [5:0] operand_width_t;
  typedef logic [$clog2(VME_VRF_WORD_WIDTH)-1:0] vrf_bit_offset_t;

  // Request queues and arbitration.
  request_t compute_req;
  issue_entry_t dispatch_entry, mac_entry, move_entry, clean_entry;
  deps_t scoreboard_deps, scoreboard_outstanding;
  dep_generations_t scoreboard_dispatch_generations;
  dep_generations_t scoreboard_generations;
  logic [3:0] scoreboard_complete_valid;
  instruction_id_t [3:0] scoreboard_complete_id;
  logic scoreboard_dispatch;
  logic       mac_req_valid ;
  logic       mac_req_ready ;
  request_t move_req;
  logic       tv_req_valid ;
  logic       tv_req_ready ;
  logic       vt_req_valid ;
  logic       vt_req_ready ;
  request_t control_req ;
  logic       clean_req_valid ;
  logic       clean_req_ready ;
  logic mac_in_ready, move_in_ready, clean_in_ready;
  logic mac_queue_valid, move_queue_valid, clean_queue_valid;
  logic mac_deps_ready, move_deps_ready, clean_deps_ready;
  logic input_is_tile_to_vreg, input_is_vreg_to_tile;
  logic move_is_tile_to_vreg, move_is_vreg_to_tile;

  // MAC scheduler interface.
  logic [SpatialBeatW-1:0] mac_beat;
  logic mac_vrf_read_valid;
  vrf_addr_t [1:0] mac_vrf_read_addr;
  logic [EdgeBeatW-1:0] mac_edge_row, mac_edge_col;
  logic [ComputeEdge-1:0][ComputeEdge-1:0] mac_lane_active;
  logic mac_fire, mac_last_beat, mac_empty_complete, mac_scheduler_busy;
  logic [VME_NR_LOGICAL_TILES-1:0] mac_pending_tiles;
  instruction_id_t mac_id;
  mt_t mac_tile;
  vew_e mac_sew, mac_tew;
  logic [2:0] mac_tk;
  logic mac_commit_valid, mac_commit_ready;

  // FPU pipeline and MAC completion.
  logic fpu_pipe_result_fire;
  logic fpu_result_completion;
  instruction_id_t fpu_result_id;
  mt_t fpu_result_tile;
  logic [SpatialBeatW-1:0] fpu_result_beat;
  logic mac_scoreboard_complete;
  instruction_id_t mac_scoreboard_complete_id;
  logic [VME_NR_LOGICAL_TILES-1:0][SpatialBeats-1:0] fpu_pipe_tile_busy;
  logic fpu_pipe_busy;
  response_t mac_commit_rsp, mac_done_rsp;
  logic     mac_done_valid, mac_done_ready;

  // Tile -> VRF move.
  request_t    vt_req_q                       ;
  logic        vt_busy_q                       ;
  logic        vt_commit_valid, vt_commit_ready;
  response_t    vt_commit_rsp, vt_done_rsp;
  logic        vt_done_valid , vt_done_ready  ;
  logic [$clog2(MaxVregGroupWords)-1:0] vt_word_idx_q;
  move_shape_t vt_shape_q;
  vrf_data_t vt_vrf_data;
  vrf_be_t vt_vrf_be;

  // VRF -> tile move.
  request_t    tv_req_q                       ;
  logic        tv_busy_q                       ;
  logic        tv_commit_valid, tv_commit_ready;
  response_t    tv_commit_rsp, tv_done_rsp      ;
  logic        tv_done_valid , tv_done_ready  ;
  logic [$clog2(MaxVregGroupWords)-1:0] tv_word_idx_q;
  move_shape_t tv_shape_q;
  vrf_data_t tv_vrf_data;
  logic tv_vrf_avail;
  logic tv_acc_wen;
  logic tv_rd_en;
  logic [$clog2(MaxVregGroupWords):0] tv_rd_idx;

  // Tile clean and busy tracking.
  response_t clean_commit_rsp, clean_done_rsp;
  logic     clean_done_valid, clean_done_ready ;
  logic     clean_commit_ready                  ;
  logic     clean_acc_ready                     ;
  logic [VME_NR_LOGICAL_TILES-1:0] mac_tile_busy;


  logic mac_use_sdotp;
  logic [VME_NR_LOGICAL_TILES-1:0] tile_read_pipe_conflict, tile_write_pipe_conflict;
  logic tile_write_mac_conflict, tile_write_zero_conflict;
  logic tile_write_blocked, tile_read_blocked;

  request_t ingress_req;
  logic ingress_valid, ingress_ready;

  /*------------------------------------------------------------*/
  /*               Request Queues and Arbitration               */
  /*------------------------------------------------------------*/

  assign input_is_tile_to_vreg = ingress_req.op_ope.tss.tile_valid &&
                                 ingress_req.use_vd;
  assign input_is_vreg_to_tile = ingress_req.op_ope.tss.tile_valid &&
                                 ingress_req.use_vs2 &&
                                 !ingress_req.use_vd;

  assign dispatch_entry = '{
    req: ingress_req,
    deps: scoreboard_deps,
    dep_generations: scoreboard_dispatch_generations
  };
  assign scoreboard_dispatch = ingress_valid && ingress_ready;

  assign scoreboard_complete_valid = {
    clean_req_valid && clean_req_ready,
    tv_commit_valid && tv_commit_ready,
    vt_commit_valid && vt_commit_ready,
    mac_scoreboard_complete
  };
  assign scoreboard_complete_id = {
    control_req.id,
    tv_req_q.id,
    vt_req_q.id,
    mac_scoreboard_complete_id
  };

  tile_scoreboard i_scoreboard (
    .clk_i,
    .rst_ni,
    .dispatch_req_i          (ingress_req),
    .dispatch_valid_i        (scoreboard_dispatch),
    .dispatch_deps_o         (scoreboard_deps),
    .dispatch_generations_o  (scoreboard_dispatch_generations),
    .complete_valid_i        (scoreboard_complete_valid),
    .complete_id_i           (scoreboard_complete_id),
    .outstanding_o           (scoreboard_outstanding),
    .generations_o           (scoreboard_generations)
  );

  assign compute_req = mac_entry.req;
  assign move_req = move_entry.req;
  assign control_req = clean_entry.req;
  assign move_is_tile_to_vreg = move_req.op_ope.tss.tile_valid &&
                                move_req.use_vd;
  assign move_is_vreg_to_tile = move_req.op_ope.tss.tile_valid &&
                                move_req.use_vs2 &&
                                !move_req.use_vd;

  always_comb begin : dep_generation_check
    mac_deps_ready = 1'b1;
    move_deps_ready = 1'b1;
    clean_deps_ready = 1'b1;
    for (int unsigned id = 0; id < VME_NR_INFLIGHT; id++) begin
      if (mac_entry.deps[id] && scoreboard_outstanding[id] &&
          (mac_entry.dep_generations[id] == scoreboard_generations[id]))
        mac_deps_ready = 1'b0;
      if (move_entry.deps[id] && scoreboard_outstanding[id] &&
          (move_entry.dep_generations[id] == scoreboard_generations[id]))
        move_deps_ready = 1'b0;
      if (clean_entry.deps[id] && scoreboard_outstanding[id] &&
          (clean_entry.dep_generations[id] == scoreboard_generations[id]))
        clean_deps_ready = 1'b0;
    end
  end : dep_generation_check

  // matrix_enable_i covers host architectural dependencies. The local
  // scoreboard independently covers cross-unit logical-tile dependencies.
  assign mac_req_valid = mac_queue_valid && mac_deps_ready &&
                         matrix_enable_i[compute_req.id];
  assign vt_req_valid = move_queue_valid && move_deps_ready &&
                        move_is_tile_to_vreg &&
                        matrix_enable_i[move_req.id];
  assign tv_req_valid = move_queue_valid && move_deps_ready &&
                        move_is_vreg_to_tile &&
                        matrix_enable_i[move_req.id];
  assign clean_req_valid = clean_queue_valid && clean_deps_ready &&
                           matrix_enable_i[control_req.id];

  spill_register #(.T(request_t)) i_op_ingress_queue (
    .clk_i,
    .rst_ni,
    .data_i  (req_i),
    .valid_i (req_valid_i),
    .ready_o (req_ready_o),
    .data_o  (ingress_req),
    .valid_o (ingress_valid),
    .ready_i (ingress_ready)
  );

  stream_fifo #(
    .FALL_THROUGH (1'b0             ),
    .DEPTH        (4                ),
    .T            (issue_entry_t)
  ) i_compute_operand_queue (
    .clk_i,
    .rst_ni,
    .flush_i    (1'b0),
    .testmode_i (1'b0),
    .usage_o    (),
    .data_i     (dispatch_entry),
    .valid_i    (ingress_valid && ingress_req.op_ope.is_mac),
    .ready_o    (mac_in_ready),
    .data_o     (mac_entry),
    .valid_o    (mac_queue_valid),
    .ready_i    (mac_req_ready)
  );

  stream_fifo #(
    .FALL_THROUGH (1'b0             ),
    .DEPTH        (4                ),
    .T            (issue_entry_t)
  ) i_move_operand_queue (
    .clk_i,
    .rst_ni,
    .flush_i    (1'b0),
    .testmode_i (1'b0),
    .usage_o    (),
    .data_i     (dispatch_entry),
    .valid_i    (ingress_valid &&
                 (input_is_tile_to_vreg || input_is_vreg_to_tile)),
    .ready_o    (move_in_ready),
    .data_o     (move_entry),
    .valid_o    (move_queue_valid),
    .ready_i    (vt_req_ready || tv_req_ready)
  );

  stream_fifo #(
    .FALL_THROUGH (1'b0             ),
    .DEPTH        (2                ),
    .T            (issue_entry_t)
  ) i_control_operand_queue (
    .clk_i,
    .rst_ni,
    .flush_i    (1'b0),
    .testmode_i (1'b0),
    .usage_o    (),
    .data_i     (dispatch_entry),
    .valid_i    (ingress_valid &&
                 (ingress_req.op_ope.is_zero_tile ||
                  ingress_req.op_ope.is_discard)),
    .ready_o    (clean_in_ready),
    .data_o     (clean_entry),
    .valid_o    (clean_queue_valid),
    .ready_i    (clean_req_ready)
  );

  // Each unit can accept an independent ready operation in the same cycle.
  // Cross-unit ordering is enforced by dependency tokens rather than by a
  // fixed clean/move/compute priority chain.
  always_comb begin : req_ready_proc
    vt_req_ready = vt_req_valid &&
        (!vt_busy_q || (vt_commit_valid && vt_commit_ready));
    tv_req_ready = tv_req_valid &&
        (!tv_busy_q || (tv_commit_valid && tv_commit_ready));

    clean_req_ready = clean_req_valid && clean_commit_ready && clean_acc_ready;
  end : req_ready_proc

  always_comb begin : selected_request_ready
    ingress_ready = 1'b0;
    if (ingress_req.op_ope.is_mac)
      ingress_ready = mac_in_ready;
    else if (input_is_tile_to_vreg || input_is_vreg_to_tile)
      ingress_ready = move_in_ready;
    else if (ingress_req.op_ope.is_zero_tile ||
             ingress_req.op_ope.is_discard)
      ingress_ready = clean_in_ready;
  end : selected_request_ready

  ////////////////////////////////////////////////////////////////////////
  ////                         MAC Operation                          ////
  ////////////////////////////////////////////////////////////////////////

  mac_scheduler #(
    .ComputeEdge  (ComputeEdge),
    .TileEdge     (TileEdge),
    .AccWordWidth (AccWordWidth),
    .AccWordBytes (AccWordBytes)
  ) i_mac_scheduler (
    .clk_i,
    .rst_ni,
    .req_i                  (compute_req),
    .req_valid_i            (mac_req_valid),
    .req_ready_o            (mac_req_ready),
    .vrf_rvalid_i           (vrf_rvalid_i[1:0]),
    .vrf_read_valid_o       (mac_vrf_read_valid),
    .vrf_read_addr_o        (mac_vrf_read_addr),
    .commit_ready_i         (mac_commit_ready),
    .commit_valid_o         (mac_commit_valid),
    .commit_id_o            (),
    .tile_read_valid_i      (tile_rvalid_i),
    .tile_read_idx_i        (tile_r_req_i.idx),
    .pipe_tile_busy_i       (fpu_pipe_tile_busy),
    .pipe_result_valid_i    (fpu_pipe_result_fire),
    .pipe_result_tile_i     (fpu_result_tile),
    .pipe_result_beat_i     (fpu_result_beat),
    .fire_o                 (mac_fire),
    .last_beat_o            (mac_last_beat),
    .empty_complete_o       (mac_empty_complete),
    .busy_o                 (mac_scheduler_busy),
    .pending_tiles_o        (mac_pending_tiles),
    .id_o                   (mac_id),
    .tile_o                 (mac_tile),
    .sew_o                  (mac_sew),
    .tew_o                  (mac_tew),
    .tk_o                   (mac_tk),
    .beat_o                 (mac_beat),
    .edge_row_o             (mac_edge_row),
    .edge_col_o             (mac_edge_col),
    .lane_active_o          (mac_lane_active),
    .use_sdotp_o            (mac_use_sdotp)
  );

  assign mac_scoreboard_complete =
      (fpu_pipe_result_fire && fpu_result_completion) ||
      mac_empty_complete;
  assign mac_scoreboard_complete_id =
      (fpu_pipe_result_fire && fpu_result_completion) ? fpu_result_id : mac_id;

  ////////////////////////////////////////////////////////////////////////
  ////                  Tile -> VRF Move Operation                    ////
  ////////////////////////////////////////////////////////////////////////

  mover #(
    .TileEdge     (TileEdge),
    .AccWordWidth (AccWordWidth),
    .AccWordBytes (AccWordBytes),
    .MaxVregGroupWords(MaxVregGroupWords)
  ) i_mover (
    .clk_i,
    .rst_ni,
    .vt_req_i             (move_req),
    .vt_accept_i          (vt_req_valid && vt_req_ready),
    .vt_commit_ready_i    (vt_commit_ready),
    .vt_req_o             (vt_req_q),
    .vt_busy_o            (vt_busy_q),
    .vt_commit_valid_o    (vt_commit_valid),
    .vt_word_idx_o        (vt_word_idx_q),
    .vt_shape_o           (vt_shape_q),
    .tv_req_i             (move_req),
    .tv_accept_i          (tv_req_valid && tv_req_ready),
    .tv_commit_ready_i    (tv_commit_ready),
    .tv_req_o             (tv_req_q),
    .tv_busy_o            (tv_busy_q),
    .tv_commit_valid_o    (tv_commit_valid),
    .tv_word_idx_o        (tv_word_idx_q),
    .tv_shape_o           (tv_shape_q),
    .vrf_wvalid_i,
    .tv_vrf_rgnt_i        (vrf_rgnt_i[2]),
    .tv_vrf_rvalid_i      (vrf_rvalid_i[2]),
    .tv_vrf_rdata_i       (vrf_rdata_i[2]),
    .tv_vrf_re_o          (tv_rd_en),
    .tv_vrf_word_idx_o    (tv_rd_idx),
    .tv_acc_wen_o         (tv_acc_wen),
    .tv_data_pending_o    (tv_vrf_avail),
    .tv_acc_wdata_o       (tv_vrf_data)
  );

  ////////////////////////////////////////////////////////////////////////
  ////                    Tile Clean Operation                        ////
  ////////////////////////////////////////////////////////////////////////

  always_comb begin : mac_tile_busy_proc
    mac_tile_busy = mac_pending_tiles;
    for (int unsigned tile = 0; tile < VME_NR_LOGICAL_TILES; tile++) begin
      if (|fpu_pipe_tile_busy[tile])
        mac_tile_busy[tile] = 1'b1;
    end
  end : mac_tile_busy_proc

  always_comb begin : clean_acc_ready_proc
    clean_acc_ready = 1'b0;
    // Discard is global state invalidation and therefore takes priority if a
    // malformed request ever asserts both command bits.  It waits for every
    // producer/consumer of accumulator state to become idle.
    if (control_req.op_ope.is_discard)
      clean_acc_ready = !mac_scheduler_busy &&
                        !fpu_pipe_busy && !vt_busy_q && !tv_busy_q &&
                        !tile_rvalid_i && !tile_wvalid_i;
    // Zero only touches its selected tile and may proceed while unrelated
    // tiles are active, provided no reader/writer owns that tile.
    else if (control_req.op_ope.is_zero_tile)
      clean_acc_ready = !mac_tile_busy[control_req.mtd]
                        && !(tile_rvalid_i && (tile_r_req_i.idx == control_req.mtd));
  end : clean_acc_ready_proc

  /*------------------------------------------------------------*/
  /*                      VRF Port Access                       */
  /*------------------------------------------------------------*/

  // Read ports [0:1] feed MAC vs2/vs1; read port [2] feeds TV.
  // ID [3] accompanies the independent VT write port.
  always_comb begin : sb_ids
    vrf_id_o[0] = mac_id;
    vrf_id_o[1] = mac_id;
    vrf_id_o[2] = tv_req_q.id;
    vrf_id_o[3] = vt_req_q.id;
  end

  /*------------------------------------------------------------*/
  /*                   VRF -> Tile Read Ports                   */
  /*------------------------------------------------------------*/

  always_comb begin : vrf_re_proc
    vrf_re_o = $bits(vrf_re_o)'(0);
    vrf_raddr_o = $bits(vrf_raddr_o)'(0);

    if (mac_vrf_read_valid) begin
      vrf_re_o[1:0] = 2'b11;
      vrf_raddr_o[1:0] = mac_vrf_read_addr;
    end

    if (tv_rd_en) begin
      vrf_re_o[2]    = 1'b1;
      vrf_raddr_o[2] = (vrf_addr_t'(tv_req_q.vs2) << $clog2(VME_WORDS_PER_VREG)) + vrf_addr_t'(tv_rd_idx);
    end
  end

  /*------------------------------------------------------------*/
  /*                   Tile -> VRF Write Port                   */
  /*------------------------------------------------------------*/

  always_comb begin : vrf_wr_proc
    vrf_waddr_o = vrf_addr_t'(0);
    vrf_we_o    = 1'b0;
    vrf_wbe_o  = vt_vrf_be;
    vrf_wdata_o = vt_vrf_data;

    if (vt_busy_q) begin
      // A flat word offset crosses physical-register boundaries when LMUL>1;
      // the controller has already checked that vd is an aligned group base.
      vrf_waddr_o =(vrf_addr_t'(vt_req_q.vd) << $clog2(VME_WORDS_PER_VREG)) + vrf_addr_t'(vt_word_idx_q);
      vrf_we_o = vt_req_q.use_vd;
    end
  end

  /*------------------------------------------------------------*/
  /*     Shared Accumulator Access and Tile Slice Selection     */
  /*------------------------------------------------------------*/

  // Compare IDs in parallel with per-tile beat reduction, avoiding a wide beat-vector mux before reduction.
  for (genvar tile = 0; tile < VME_NR_LOGICAL_TILES; tile++) begin : gen_tile_access_conflict
    assign tile_read_pipe_conflict[tile] = (tile_r_req_i.idx == mt_t'(tile)) && (|fpu_pipe_tile_busy[tile]);
    assign tile_write_pipe_conflict[tile] = (tile_w_req_i.idx == mt_t'(tile)) && (|fpu_pipe_tile_busy[tile]);
  end

  assign tile_write_mac_conflict =
      mac_pending_tiles[tile_w_req_i.idx];

  // When write-ready can be high, MAC conflicts and tile_rvalid_i are already excluded.
  // For a matching VTZERO these are precisely the clean_acc_ready conditions, so no ready feedback is needed.
  assign tile_write_zero_conflict = clean_req_valid && control_req.op_ope.is_zero_tile &&
                                   clean_commit_ready && (control_req.mtd == tile_w_req_i.idx);

  assign tile_write_blocked = (|tile_write_pipe_conflict) ||
      tile_write_mac_conflict || tile_write_zero_conflict ||
      (vt_busy_q && (vt_req_q.op_ope.tss.tile_id == tile_w_req_i.idx)) ||
      (tv_busy_q && tv_vrf_avail &&
       (tv_req_q.op_ope.tss.tile_id == tile_w_req_i.idx));
  assign tile_read_blocked = (|tile_read_pipe_conflict) ||
      (tv_busy_q && tv_vrf_avail &&
       (tv_req_q.op_ope.tss.tile_id == tile_r_req_i.idx));

  /*------------------------------------------------------------*/
  /*            Distributed Accumulator Compute Array           */
  /*------------------------------------------------------------*/

  logic [ComputeEdge-1:0][AccWordWidth-1:0] fpu_x_operand, fpu_w_operand;

  always_comb begin : fpu_operand_select
    operand_width_t operand_bits;
    vrf_bit_offset_t x_bit_offset;
    vrf_bit_offset_t w_bit_offset;

    fpu_x_operand = $bits(fpu_x_operand)'(0);
    fpu_w_operand = $bits(fpu_w_operand)'(0);

    unique case (mac_sew)
      EW_8:    operand_bits = 6'd8;
      EW_16:   operand_bits = 6'd16;
      default: operand_bits = 6'd32;
    endcase
    if (mac_use_sdotp)
      operand_bits = 6'(1 << (3 + int'(mac_tew)));
    x_bit_offset = vrf_bit_offset_t'(
        (mac_edge_row * ComputeEdge * operand_bits) % VME_VRF_WORD_WIDTH);
    w_bit_offset = vrf_bit_offset_t'(
        (mac_edge_col * ComputeEdge * operand_bits) % VME_VRF_WORD_WIDTH);
    if (mac_use_sdotp) begin
      for (int unsigned el = 0; el < ComputeEdge; el++) begin
        fpu_x_operand[el] = vrf_rdata_i[0][x_bit_offset + el*32 +: 32];
        fpu_w_operand[el] = vrf_rdata_i[1][w_bit_offset + el*32 +: 32];
        if (mac_tk < 2) begin
          fpu_x_operand[el][31:16] = '0;
          fpu_w_operand[el][31:16] = '0;
        end
      end
    end else unique case (mac_sew)
      EW_16: begin
        for (int unsigned el = 0; el < ComputeEdge; el++) begin
          fpu_x_operand[el][15:0] = vrf_rdata_i[0][x_bit_offset + el*16 +: 16];
          fpu_w_operand[el][15:0] = vrf_rdata_i[1][w_bit_offset + el*16 +: 16];
        end
      end
      EW_8: begin
        for (int unsigned el = 0; el < ComputeEdge; el++) begin
          fpu_x_operand[el][7:0] = vrf_rdata_i[0]
              [x_bit_offset + (el << 3) +: 1 << 3];
          fpu_w_operand[el][7:0] = vrf_rdata_i[1]
              [w_bit_offset + (el << 3) +: 1 << 3];
        end
      end
      default: begin
        for (int unsigned el = 0; el < ComputeEdge; el++) begin
          fpu_x_operand[el] = vrf_rdata_i[0][x_bit_offset + el*AccWordWidth +: AccWordWidth];
          fpu_w_operand[el] = vrf_rdata_i[1][w_bit_offset + el*AccWordWidth +: AccWordWidth];
        end
      end
    endcase
  end : fpu_operand_select

  compute_array #(
    .ComputeEdge              (ComputeEdge),
    .TileEdge                 (TileEdge),
    .AccWordWidth             (AccWordWidth),
    .AccWordBytes             (AccWordBytes),
    .SpatialBeats             (SpatialBeats),
    .MaxVregGroupWords        (MaxVregGroupWords),
    .NumPipeRegs              (NumPipeRegs),
    .EnableFMA16              (EnableFMA16),
    .EnableFMA32              (EnableFMA32)
  ) i_compute_array (
    .clk_i,
    .rst_ni,
    .active_i                  (mac_queue_valid || mac_scheduler_busy || fpu_pipe_busy),
    .compute_valid_i           (mac_fire),
    .compute_id_i              (mac_id),
    .compute_tile_i            (mac_tile),
    .compute_beat_i            (mac_beat),
    .compute_completion_i      (mac_last_beat),
    .compute_sdotp_i           (mac_use_sdotp),
    .compute_sew_i             (mac_sew),
    .compute_tew_i             (mac_tew),
    .x_operand_i               (fpu_x_operand),
    .w_operand_i               (fpu_w_operand),
    .lane_active_i             (mac_lane_active),
    .vt_req_i                  (vt_req_q),
    .vt_busy_i                 (vt_busy_q),
    .vt_word_idx_i             (vt_word_idx_q),
    .vt_shape_i                (vt_shape_q),
    .vt_data_o                 (vt_vrf_data),
    .vt_be_o                   (vt_vrf_be),
    .tv_req_i                  (tv_req_q),
    .tv_write_i                (tv_acc_wen),
    .tv_word_idx_i             (tv_word_idx_q),
    .tv_shape_i                (tv_shape_q),
    .tv_data_i                 (tv_vrf_data),
    .zero_tile_i               (clean_req_valid && clean_req_ready &&
                                control_req.op_ope.is_zero_tile),
    .discard_i                 (clean_req_valid && clean_req_ready &&
                                control_req.op_ope.is_discard),
    .clean_tile_i              (control_req.mtd),
    .clean_tew_i               (control_req.op_ope.tew),
    .tile_wvalid_i,
    .tile_w_req_i,
    .tile_w_blocked_i          (tile_write_blocked),
    .tile_wready_o,
    .tile_rvalid_i,
    .tile_r_req_i,
    .tile_r_blocked_i          (tile_read_blocked),
    .tile_rready_o,
    .tile_rdata_o,
    .pipe_tile_busy_o          (fpu_pipe_tile_busy),
    .pipe_busy_o               (fpu_pipe_busy),
    .result_valid_o            (fpu_pipe_result_fire),
    .result_completion_o       (fpu_result_completion),
    .result_id_o               (fpu_result_id),
    .result_tile_o             (fpu_result_tile),
    .result_beat_o             (fpu_result_beat)
  );
  /*                           Commit                           */
  /*------------------------------------------------------------*/

  // The host response releases the architectural instruction ID once all
  // operands are consumed. Local tile ordering remains valid across ID reuse
  // through the dependency-generation snapshots in the issue queues.
  always_comb begin : mac_commit_rsp_proc
    mac_commit_rsp.id = mac_id;
  end : mac_commit_rsp_proc

  spill_register #(.T(response_t)) i_mac_commit (
    .clk_i  (clk_i           ),
    .rst_ni (rst_ni          ),
    .data_i (mac_commit_rsp  ),
    .valid_i(mac_commit_valid),
    .ready_o(mac_commit_ready),
    .data_o (mac_done_rsp    ),
    .valid_o(mac_done_valid  ),
    .ready_i(mac_done_ready  )
  );

  always_comb begin : vt_commit_rsp_proc
    vt_commit_rsp.id = vt_req_q.id;
  end : vt_commit_rsp_proc

  spill_register #(.T(response_t)) i_vt_commit (
    .clk_i  (clk_i            ),
    .rst_ni (rst_ni           ),
    .data_i (vt_commit_rsp    ),
    .valid_i(vt_commit_valid  ),
    .ready_o(vt_commit_ready  ),
    .data_o (vt_done_rsp   ),
    .valid_o(vt_done_valid    ),
    .ready_i(vt_done_ready    )
  );

  always_comb begin : tv_commit_rsp_proc
    tv_commit_rsp.id = tv_req_q.id;
  end : tv_commit_rsp_proc

  spill_register #(.T(response_t)) i_tv_commit (
    .clk_i  (clk_i            ),
    .rst_ni (rst_ni           ),
    .data_i (tv_commit_rsp    ),
    .valid_i(tv_commit_valid  ),
    .ready_o(tv_commit_ready  ),
    .data_o (tv_done_rsp      ),
    .valid_o(tv_done_valid    ),
    .ready_i(tv_done_ready    )
  );

  always_comb begin : clean_commit_rsp_proc
    clean_commit_rsp.id = control_req.id;
  end : clean_commit_rsp_proc

  spill_register #(.T(response_t)) i_clean_commit (
    .clk_i  (clk_i                                    ),
    .rst_ni (rst_ni                                   ),
    .data_i (clean_commit_rsp                        ),
    .valid_i(clean_req_valid && clean_acc_ready     ),
    .ready_o(clean_commit_ready                      ),
    .data_o (clean_done_rsp                          ),
    .valid_o(clean_done_valid                        ),
    .ready_i(clean_done_ready                        )
  );

  // Completion response arbitration.
  response_t [3:0] arb_inp_data ;
  logic     [3:0] arb_inp_valid;
  logic     [3:0] arb_inp_ready;

  typedef enum logic [1:0] {
    RSP_MAC,
    RSP_VT,
    RSP_TV,
    RSP_SIMPLE
  } ope_rsp_sel_e;

  assign arb_inp_data[RSP_MAC]    = mac_done_rsp;
  assign arb_inp_data[RSP_VT]     = vt_done_rsp;
  assign arb_inp_data[RSP_TV]     = tv_done_rsp;
  assign arb_inp_data[RSP_SIMPLE] = clean_done_rsp;

  assign arb_inp_valid[RSP_MAC]    = mac_done_valid;
  assign arb_inp_valid[RSP_VT]     = vt_done_valid;
  assign arb_inp_valid[RSP_TV]     = tv_done_valid;
  assign arb_inp_valid[RSP_SIMPLE] = clean_done_valid;

  assign mac_done_ready    = arb_inp_ready[RSP_MAC];
  assign vt_done_ready     = arb_inp_ready[RSP_VT];
  assign tv_done_ready     = arb_inp_ready[RSP_TV];
  assign clean_done_ready = arb_inp_ready[RSP_SIMPLE];

  stream_arbiter #(
    .DATA_T  (response_t),
    .N_INP   (4        ),
    .ARBITER ("rr"     )
  ) i_rsp_arb (
    .clk_i       (clk_i           ),
    .rst_ni      (rst_ni          ),
    .inp_data_i  (arb_inp_data    ),
    .inp_valid_i (arb_inp_valid   ),
    .inp_ready_o (arb_inp_ready   ),
    .oup_data_o  (rsp_o       ),
    .oup_valid_o (rsp_valid_o ),
    .oup_ready_i (rsp_ready_i )
  );

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin : check_tile_rw_exclusive
    if (rst_ni) assert (!(tile_rvalid_i && tile_wvalid_i))
      else $error("Tile read and write requested in the same cycle");
  end
`endif

  /*------------------------------------------------------------*/
  /*                      Parameter Checks                      */
  /*------------------------------------------------------------*/

  if ((TileEdge == 0) || ((TileEdge & (TileEdge-1)) != 0))
    $error("[VME] TileEdge must be a power of two.");
  if ((ComputeEdge == 0) || ((ComputeEdge & (ComputeEdge-1)) != 0))
    $error("[VME] ComputeEdge must be a power of two.");
  if (TileEdge < ComputeEdge)
    $error("[VME] TileEdge must be greater than or equal to ComputeEdge.");
  if ((ComputeEdge != 0) && ((TileEdge % ComputeEdge) != 0))
    $error("[VME] TileEdge must be an integer multiple of ComputeEdge.");
  if (AccWordBytes != (AccWordWidth >> 3))
    $error("[VME] AccWordBytes must match AccWordWidth.");
  if (SpatialBeats != EdgeBeats * EdgeBeats)
    $error("[VME] SpatialBeats must match TileEdge and ComputeEdge.");
  if (MaxVregGroupWords > VME_MAX_VREG_GROUP_WORDS)
    $error("[VME] MaxVregGroupWords exceeds the architectural interface width.");
  if (NumPipeRegs < 2)
    $error("[VME] NumPipeRegs must be at least two.");
endmodule : vme
