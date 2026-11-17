// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Pei-Yu Lin, ETH Zürich

// Distributed compute array. Every PE contains its arithmetic datapath and its
// local accumulator, so the MAC feedback path never leaves the PE hierarchy.
module compute_array
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
  input logic clk_i,
  input logic rst_ni,
  input logic active_i,

  input logic compute_valid_i,
  input instruction_id_t compute_id_i,
  input mt_t compute_tile_i,
  input logic [((SpatialBeats > 1) ? $clog2(SpatialBeats) : 1)-1:0]
              compute_beat_i,
  input logic compute_completion_i,
  input logic compute_sdotp_i,
  input vew_e compute_sew_i,
  input vew_e compute_tew_i,
  input logic [ComputeEdge-1:0][AccWordWidth-1:0] x_operand_i,
  input logic [ComputeEdge-1:0][AccWordWidth-1:0] w_operand_i,
  input logic [ComputeEdge-1:0][ComputeEdge-1:0] lane_active_i,

  input request_t vt_req_i,
  input logic vt_busy_i,
  input logic [$clog2(MaxVregGroupWords)-1:0] vt_word_idx_i,
  input move_shape_t vt_shape_i,
  output vrf_data_t vt_data_o,
  output vrf_be_t vt_be_o,

  input request_t tv_req_i,
  input logic tv_write_i,
  input logic [$clog2(MaxVregGroupWords)-1:0] tv_word_idx_i,
  input move_shape_t tv_shape_i,
  input vrf_data_t tv_data_i,

  input logic zero_tile_i,
  input logic discard_i,
  input mt_t clean_tile_i,
  input vew_e clean_tew_i,

  input logic tile_wvalid_i,
  input tile_write_req_t tile_w_req_i,
  input logic tile_w_blocked_i,
  output logic tile_wready_o,
  input logic tile_rvalid_i,
  input tile_read_req_t tile_r_req_i,
  input logic tile_r_blocked_i,
  output logic tile_rready_o,
  output tile_word_t tile_rdata_o,

  output logic [VME_NR_LOGICAL_TILES-1:0]
               [SpatialBeats-1:0]
               pipe_tile_busy_o,
  output logic pipe_busy_o,
  output logic result_valid_o,
  output logic result_completion_o,
  output instruction_id_t result_id_o,
  output mt_t result_tile_o,
  output logic [((SpatialBeats > 1) ? $clog2(SpatialBeats) : 1)-1:0]
               result_beat_o
);

  localparam int unsigned EdgeBeats = TileEdge / ComputeEdge;
  localparam int unsigned AccMaskWidth = AccWordBytes;
  localparam int unsigned EdgeBeatW = EdgeBeats > 1 ? $clog2(EdgeBeats) : 1;
  localparam int unsigned SpatialBeatW = SpatialBeats > 1 ? $clog2(SpatialBeats) : 1;
  localparam int unsigned NrAccBankedTiles = VME_NR_LOGICAL_TILES / AccWordBytes;
  localparam int unsigned AccDepth = NrAccBankedTiles * SpatialBeats;
  localparam int unsigned AccAddrW = AccDepth > 1 ? $clog2(AccDepth) : 1;
  localparam int unsigned AccTileIdxW = NrAccBankedTiles > 1 ?
      $clog2(NrAccBankedTiles) : 1;
  localparam int unsigned AccByteIdxW = AccWordBytes > 1 ? $clog2(AccWordBytes) : 1;

  typedef logic [$clog2((TileEdge<<1)+1)-1:0] tile_sum_t;
  typedef struct packed {
    logic [AccTileIdxW-1:0] tile;
    logic [AccByteIdxW-1:0] byte_offset;
    logic [AccMaskWidth-1:0] mask_enable;
  } acc_sel_t;

  typedef struct packed {
    logic valid;
    logic completion;
    instruction_id_t id;
    mt_t tile;
    logic [SpatialBeatW-1:0] beat;
    logic [AccMaskWidth-1:0] mask;
    logic [ComputeEdge-1:0][ComputeEdge-1:0] lane_active;
  } pipe_tag_t;

  logic fpu_clk;
  logic [ComputeEdge-1:0][AccWordWidth-1:0] x_operand_q, w_operand_q;
  logic [ComputeEdge-1:0][ComputeEdge-1:0][AccWordWidth-1:0] addend_q;
  logic [ComputeEdge-1:0][ComputeEdge-1:0] lane_active_q;
  logic compute_valid_q, compute_sdotp_q;
  vew_e compute_sew_q, compute_tew_q;
  pipe_tag_t [NumPipeRegs-1:0] pipe_tag_d, pipe_tag_q;
  pipe_tag_t pipe_tag_in;
  logic [AccTileIdxW-1:0] compute_acc_tile, result_acc_tile;
  logic [AccByteIdxW-1:0] compute_byte_offset, result_byte_offset;
  logic [AccMaskWidth-1:0] compute_mask, result_mask;
  logic [AccAddrW-1:0] compute_acc_raddr, result_acc_waddr;
  logic [ComputeEdge-1:0][ComputeEdge-1:0][AccMaskWidth-1:0] compute_read_zero;
  logic result_forward;
  acc_sel_t vt_acc_sel, tv_acc_sel, tile_r_acc_sel, tile_w_acc_sel, clean_acc_sel;
  logic [NrAccBankedTiles-1:0][SpatialBeats-1:0]
      [ComputeEdge-1:0][ComputeEdge-1:0][AccMaskWidth-1:0] acc_zero_d, acc_zero_q;
  logic [ComputeEdge-1:0][ComputeEdge-1:0][EdgeBeats-1:0][AccWordWidth-1:0]
      mover_acc_rdata, mover_acc_wdata, tile_acc_rdata, tile_acc_wdata;
  logic [ComputeEdge-1:0][ComputeEdge-1:0][EdgeBeats-1:0][AccMaskWidth-1:0]
      mover_acc_wen, tile_acc_wen;
  logic [AccAddrW-1:0] mover_acc_raddr, mover_acc_waddr;
  logic [AccAddrW-1:0] tile_acc_raddr, tile_acc_waddr;
  logic [SpatialBeatW-1:0] mover_write_beat_base, tile_write_beat_base;

  // One local control branch per PE row. The root drives ComputeEdge branches
  // and each branch drives ComputeEdge PEs, keeping fanout below 32.
  (* keep = "true" *) logic [ComputeEdge-1:0] compute_valid_row, compute_sdotp_row;
  (* keep = "true" *) logic [ComputeEdge-1:0] forward_row, result_write_row;
  (* keep = "true" *) vew_e [ComputeEdge-1:0] compute_sew_row;
  (* keep = "true" *) vew_e [ComputeEdge-1:0] compute_tew_row;
  (* keep = "true" *) logic [ComputeEdge-1:0][AccAddrW-1:0] acc_raddr_row;
  (* keep = "true" *) logic [ComputeEdge-1:0][AccAddrW-1:0] acc_waddr_row;
  (* keep = "true" *) logic [ComputeEdge-1:0][AccAddrW-1:0]
      mover_acc_raddr_row;
  (* keep = "true" *) logic [ComputeEdge-1:0][AccAddrW-1:0]
      mover_acc_waddr_row;
  (* keep = "true" *) logic [ComputeEdge-1:0][AccAddrW-1:0]
      tile_acc_raddr_row;
  (* keep = "true" *) logic [ComputeEdge-1:0][AccAddrW-1:0]
      tile_acc_waddr_row;
  (* keep = "true" *) logic [ComputeEdge-1:0][AccByteIdxW-1:0]
      acc_read_byte_offset_row;
  (* keep = "true" *) logic [ComputeEdge-1:0][AccByteIdxW-1:0]
      acc_write_byte_offset_row;
  (* keep = "true" *) logic [ComputeEdge-1:0][AccMaskWidth-1:0] acc_read_mask_row;
  (* keep = "true" *) logic [ComputeEdge-1:0][AccMaskWidth-1:0] acc_write_mask_row;

  for (genvar branch = 0; branch < ComputeEdge; branch++) begin : gen_control_branch
    assign compute_valid_row[branch] = compute_valid_q;
    assign compute_sdotp_row[branch] = compute_sdotp_q;
    assign compute_sew_row[branch] = compute_sew_q;
    assign compute_tew_row[branch] = compute_tew_q;
    assign forward_row[branch] = result_forward;
    assign result_write_row[branch] = result_valid_o;
    assign acc_raddr_row[branch] = compute_acc_raddr;
    assign acc_waddr_row[branch] = result_acc_waddr;
    assign mover_acc_raddr_row[branch] = mover_acc_raddr;
    assign mover_acc_waddr_row[branch] = mover_acc_waddr;
    assign tile_acc_raddr_row[branch] = tile_acc_raddr;
    assign tile_acc_waddr_row[branch] = tile_acc_waddr;
    assign acc_read_byte_offset_row[branch] = compute_byte_offset;
    assign acc_write_byte_offset_row[branch] = result_byte_offset;
    assign acc_read_mask_row[branch] = compute_mask;
    assign acc_write_mask_row[branch] = result_mask;
  end

  assign compute_acc_tile = AccTileIdxW'(compute_tile_i / AccWordBytes);
  assign compute_byte_offset = AccByteIdxW'(compute_tile_i % AccWordBytes);
  always_comb begin : compute_byte_mask
    int unsigned tew_bytes;
    compute_mask = '0;
    tew_bytes = 1 << int'(compute_tew_i);
    for (int unsigned byte_idx = 0; byte_idx < AccMaskWidth; byte_idx++) begin
      if ((byte_idx >= unsigned'(compute_byte_offset)) &&
          (byte_idx < unsigned'(compute_byte_offset) + tew_bytes))
        compute_mask[byte_idx] = 1'b1;
    end
  end
  assign compute_acc_raddr = AccAddrW'(compute_acc_tile * SpatialBeats + compute_beat_i);
  assign compute_read_zero = acc_zero_q[compute_acc_tile][compute_beat_i];

  assign result_acc_tile = AccTileIdxW'(result_tile_o / AccWordBytes);
  assign result_byte_offset = AccByteIdxW'(result_tile_o % AccWordBytes);
  assign result_mask = pipe_tag_q[NumPipeRegs-1].mask;
  assign result_acc_waddr = AccAddrW'(result_acc_tile * SpatialBeats + result_beat_o);
  assign result_forward = result_valid_o && (result_tile_o == compute_tile_i) &&
                          (result_beat_o == compute_beat_i);
  assign mover_write_beat_base = SpatialBeatW'(mover_acc_waddr);
  assign tile_write_beat_base = SpatialBeatW'(tile_acc_waddr);

  assign pipe_tag_in = '{
    valid      : compute_valid_i,
    completion : compute_valid_i && compute_completion_i,
    id         : compute_id_i,
    tile       : compute_tile_i,
    beat       : compute_beat_i,
    mask       : compute_mask,
    lane_active: lane_active_i
  };
  assign pipe_tag_d = {pipe_tag_q[NumPipeRegs-2:0], pipe_tag_in};

  always_comb begin : pipe_occupancy
    pipe_tile_busy_o = '0;
    pipe_busy_o = 1'b0;
    for (int unsigned stage = 0; stage < NumPipeRegs; stage++) begin
      if (pipe_tag_q[stage].valid) begin
        pipe_busy_o = 1'b1;
        pipe_tile_busy_o[pipe_tag_q[stage].tile][pipe_tag_q[stage].beat] = 1'b1;
      end
    end
  end

  assign result_valid_o = pipe_tag_q[NumPipeRegs-1].valid;
  assign result_completion_o = pipe_tag_q[NumPipeRegs-1].completion;
  assign result_id_o = pipe_tag_q[NumPipeRegs-1].id;
  assign result_tile_o = pipe_tag_q[NumPipeRegs-1].tile;
  assign result_beat_o = pipe_tag_q[NumPipeRegs-1].beat;

  // Logical tiles may select a byte slice within one physical accumulator.
  // Decode that mapping once at the accumulator boundary instead of carrying
  // expanded PE-array buses through the VME integration layer.
  always_comb begin : accumulator_select
    int unsigned vt_tew_bytes, tv_tew_bytes;
    int unsigned tile_r_tew_bytes, tile_w_tew_bytes, clean_tew_bytes;

    vt_tew_bytes = 1 << int'(vt_req_i.op_ope.tew);
    tv_tew_bytes = 1 << int'(tv_req_i.op_ope.tew);
    tile_r_tew_bytes = 1 << int'(tile_r_req_i.tew);
    tile_w_tew_bytes = 1 << int'(tile_w_req_i.tew);
    clean_tew_bytes = 1 << int'(clean_tew_i);

    vt_acc_sel.tile = AccTileIdxW'(vt_req_i.op_ope.tss.tile_id / AccWordBytes);
    vt_acc_sel.byte_offset = AccByteIdxW'(vt_req_i.op_ope.tss.tile_id % AccWordBytes);
    vt_acc_sel.mask_enable = '0;

    tv_acc_sel.tile = AccTileIdxW'(tv_req_i.op_ope.tss.tile_id / AccWordBytes);
    tv_acc_sel.byte_offset = AccByteIdxW'(tv_req_i.op_ope.tss.tile_id % AccWordBytes);
    tv_acc_sel.mask_enable = '0;

    tile_r_acc_sel.tile = AccTileIdxW'(tile_r_req_i.idx / AccWordBytes);
    tile_r_acc_sel.byte_offset = AccByteIdxW'(tile_r_req_i.idx % AccWordBytes);
    tile_r_acc_sel.mask_enable = '0;

    tile_w_acc_sel.tile = AccTileIdxW'(tile_w_req_i.idx / AccWordBytes);
    tile_w_acc_sel.byte_offset = AccByteIdxW'(tile_w_req_i.idx % AccWordBytes);
    tile_w_acc_sel.mask_enable = '0;

    clean_acc_sel.tile = AccTileIdxW'(clean_tile_i / AccWordBytes);
    clean_acc_sel.byte_offset = AccByteIdxW'(clean_tile_i % AccWordBytes);
    clean_acc_sel.mask_enable = '0;

    for (int unsigned byte_idx = 0; byte_idx < AccMaskWidth; byte_idx++) begin
      if ((byte_idx >= unsigned'(vt_acc_sel.byte_offset)) &&
          (byte_idx < unsigned'(vt_acc_sel.byte_offset) + vt_tew_bytes))
        vt_acc_sel.mask_enable[byte_idx] = 1'b1;
      if ((byte_idx >= unsigned'(tv_acc_sel.byte_offset)) &&
          (byte_idx < unsigned'(tv_acc_sel.byte_offset) + tv_tew_bytes))
        tv_acc_sel.mask_enable[byte_idx] = 1'b1;
      if ((byte_idx >= unsigned'(tile_r_acc_sel.byte_offset)) &&
          (byte_idx < unsigned'(tile_r_acc_sel.byte_offset) + tile_r_tew_bytes))
        tile_r_acc_sel.mask_enable[byte_idx] = 1'b1;
      if ((byte_idx >= unsigned'(tile_w_acc_sel.byte_offset)) &&
          (byte_idx < unsigned'(tile_w_acc_sel.byte_offset) + tile_w_tew_bytes))
        tile_w_acc_sel.mask_enable[byte_idx] = 1'b1;
      if ((byte_idx >= unsigned'(clean_acc_sel.byte_offset)) &&
          (byte_idx < unsigned'(clean_acc_sel.byte_offset) + clean_tew_bytes))
        clean_acc_sel.mask_enable[byte_idx] = 1'b1;
    end
  end

  // The mover sees one compact VRF word. Gather/scatter remains beside the
  // distributed accumulators, so only request metadata and one data word
  // cross the compute-array boundary.
  always_comb begin : mover_read_address
    logic [EdgeBeatW-1:0] edge_row;
    tile_dim_t line_word_idx;
    int unsigned tew_bytes, elems_per_word;

    mover_acc_raddr = '0;
    edge_row = '0;
    line_word_idx = '0;
    tew_bytes = 1 << int'(vt_req_i.op_ope.tew);
    elems_per_word = VME_VRF_WORD_BYTES / tew_bytes;
    if (vt_busy_i) begin
      line_word_idx = tile_dim_t'(vt_word_idx_i);
      edge_row = vt_req_i.op_ope.tss.is_row ?
          EdgeBeatW'(vt_req_i.op_ope.tss.index / ComputeEdge) :
          EdgeBeatW'((line_word_idx * elems_per_word) / ComputeEdge);
      mover_acc_raddr = AccAddrW'(vt_acc_sel.tile * SpatialBeats + edge_row * EdgeBeats);
    end
  end

  always_comb begin : mover_read_pack
    tile_dim_t row_i, col_i, line_word_idx;
    tile_sum_t elem_idx;
    vlen_t flat_elem_idx;
    int unsigned tew_bytes, elems_per_word;

    vt_data_o = '0;
    vt_be_o = '0;
    row_i = '0;
    col_i = '0;
    elem_idx = '0;
    flat_elem_idx = '0;
    line_word_idx = tile_dim_t'(vt_word_idx_i);
    tew_bytes = 1 << int'(vt_req_i.op_ope.tew);
    elems_per_word = VME_VRF_WORD_BYTES / tew_bytes;

    if (vt_busy_i) begin
      for (int lane = 0; lane < VME_VRF_WORD_BYTES; lane++) begin
        elem_idx = line_word_idx * elems_per_word + lane;
        flat_elem_idx = elem_idx;
        if ((flat_elem_idx >= vt_req_i.vstart) &&
            (flat_elem_idx < vt_shape_i.elems) &&
            (elem_idx < vt_shape_i.line_elems) &&
            (lane < elems_per_word)) begin
          logic [AccWordWidth-1:0] acc_word;
          if (vt_req_i.op_ope.tss.is_row) begin
            row_i = vt_req_i.op_ope.tss.index;
            col_i = elem_idx[$clog2(TileEdge)-1:0];
          end else begin
            row_i = elem_idx[$clog2(TileEdge)-1:0];
            col_i = vt_req_i.op_ope.tss.index;
          end
          acc_word = mover_acc_rdata[row_i % ComputeEdge][col_i % ComputeEdge]
                                      [col_i / ComputeEdge];
          for (int unsigned mask_idx = 0; mask_idx < AccMaskWidth; mask_idx++)
            if (acc_zero_q[vt_acc_sel.tile]
                          [(row_i / ComputeEdge) * EdgeBeats + (col_i / ComputeEdge)]
                          [row_i % ComputeEdge][col_i % ComputeEdge]
                          [mask_idx])
              acc_word[(mask_idx << 3) +: 1 << 3] = '0;
          for (int unsigned elem_byte = 0; elem_byte < AccWordBytes; elem_byte++) begin
            if (elem_byte < tew_bytes) begin
              vt_data_o[((lane*tew_bytes + elem_byte) << 3) +: 1 << 3] =
                  acc_word[((unsigned'(vt_acc_sel.byte_offset) + elem_byte) <<
                            3) +: 1 << 3];
              vt_be_o[lane*tew_bytes + elem_byte] = 1'b1;
            end
          end
        end
      end
    end
  end

  always_comb begin : mover_write_scatter
    tile_dim_t idx, line_word_idx;
    logic [EdgeBeatW-1:0] edge_row;
    int unsigned tew_bytes, elems_per_word;

    mover_acc_wdata = '0;
    mover_acc_wen = '0;
    mover_acc_waddr = '0;
    idx = '0;
    edge_row = '0;
    line_word_idx = tile_dim_t'(tv_word_idx_i);
    tew_bytes = 1 << int'(tv_req_i.op_ope.tew);
    elems_per_word = VME_VRF_WORD_BYTES / tew_bytes;

    if (tv_write_i) begin
      idx = tv_req_i.op_ope.tss.index;
      edge_row = tv_req_i.op_ope.tss.is_row ?
          EdgeBeatW'(idx / ComputeEdge) :
          EdgeBeatW'((line_word_idx * elems_per_word) / ComputeEdge);
      mover_acc_waddr = AccAddrW'(tv_acc_sel.tile * SpatialBeats + edge_row * EdgeBeats);

      for (int row = 0; row < ComputeEdge; row++) begin
        for (int col = 0; col < ComputeEdge; col++) begin
          for (int beat = 0; beat < EdgeBeats; beat++) begin
            int unsigned dest_elem, source_lane;
            logic dest_match;
            if (tv_req_i.op_ope.tss.is_row) begin
              dest_elem = beat * ComputeEdge + col;
              dest_match = (unsigned'(idx) % ComputeEdge) == row;
            end else begin
              dest_elem = unsigned'(edge_row) * ComputeEdge + row;
              dest_match = ((unsigned'(idx) % ComputeEdge) == col) &&
                           ((unsigned'(idx) / ComputeEdge) == beat);
            end
            source_lane = dest_elem - unsigned'(line_word_idx) * elems_per_word;
            if (dest_match &&
                (dest_elem >= unsigned'(line_word_idx) * elems_per_word) &&
                (source_lane < elems_per_word) &&
                (dest_elem >= unsigned'(tv_req_i.vstart)) &&
                (dest_elem < unsigned'(tv_shape_i.elems)) &&
                (dest_elem < unsigned'(tv_shape_i.line_elems))) begin
              for (int unsigned elem_byte = 0; elem_byte < AccWordBytes; elem_byte++) begin
                if (elem_byte < tew_bytes)
                  mover_acc_wdata[row][col][beat]
                      [((unsigned'(tv_acc_sel.byte_offset) + elem_byte) <<
                        3) +: 1 << 3] =
                      tv_data_i[((source_lane*tew_bytes + elem_byte) <<
                                 3) +: 1 << 3];
              end
              mover_acc_wen[row][col][beat] = tv_acc_sel.mask_enable;
            end
          end
        end
      end
    end
  end

  tile_lsu #(
    .ComputeEdge  (ComputeEdge),
    .TileEdge     (TileEdge),
    .AccWordWidth (AccWordWidth),
    .AccWordBytes (AccWordBytes),
    .AccMaskWidth (AccMaskWidth)
  ) i_tile_lsu (
    .tile_wvalid_i,
    .tile_w_req_i,
    .tile_w_blocked_i,
    .tile_wready_o,
    .tile_rvalid_i,
    .tile_r_req_i,
    .tile_r_blocked_i,
    .tile_rready_o,
    .tile_rdata_o,
    .read_tile_i         (tile_r_acc_sel.tile),
    .read_byte_offset_i  (tile_r_acc_sel.byte_offset),
    .write_tile_i        (tile_w_acc_sel.tile),
    .write_byte_offset_i (tile_w_acc_sel.byte_offset),
    .write_mask_i        (tile_w_acc_sel.mask_enable),
    .zero_i              (acc_zero_q),
    .acc_raddr_o         (tile_acc_raddr),
    .acc_rdata_i         (tile_acc_rdata),
    .acc_waddr_o         (tile_acc_waddr),
    .acc_wdata_o         (tile_acc_wdata),
    .acc_wen_o           (tile_acc_wen)
  );

  always_comb begin : zero_metadata_update
    acc_zero_d = acc_zero_q;
    if (discard_i) begin
      acc_zero_d = '1;
    end else begin
      if (zero_tile_i) begin
        for (int beat = 0; beat < SpatialBeats; beat++) begin
          for (int row = 0; row < ComputeEdge; row++) begin
            for (int col = 0; col < ComputeEdge; col++)
              acc_zero_d[clean_acc_sel.tile][beat][row][col] |=
                  clean_acc_sel.mask_enable;
          end
        end
      end
      if (result_valid_o) begin
        for (int row = 0; row < ComputeEdge; row++) begin
          for (int col = 0; col < ComputeEdge; col++) begin
            if (pipe_tag_q[NumPipeRegs-1].lane_active[row][col])
              acc_zero_d[result_acc_tile][result_beat_o][row][col] &= ~result_mask;
          end
        end
      end
      if (tv_write_i) begin
        for (int row = 0; row < ComputeEdge; row++) begin
          for (int col = 0; col < ComputeEdge; col++) begin
            for (int beat = 0; beat < EdgeBeats; beat++)
              acc_zero_d[tv_acc_sel.tile]
                  [mover_write_beat_base + beat][row][col] &=
                  ~mover_acc_wen[row][col][beat];
          end
        end
      end
      if (tile_wvalid_i && tile_wready_o) begin
        for (int row = 0; row < ComputeEdge; row++) begin
          for (int col = 0; col < ComputeEdge; col++) begin
            for (int beat = 0; beat < EdgeBeats; beat++)
              acc_zero_d[tile_w_acc_sel.tile]
                  [tile_write_beat_base + beat][row][col] &=
                  ~tile_acc_wen[row][col][beat];
          end
        end
      end
    end
  end

  tc_clk_gating i_fpu_clk_gate (
    .clk_i,
    .en_i      (active_i ),
    .test_en_i (1'b0     ),
    .clk_o     (fpu_clk  )
  );

  for (genvar row = 0; row < ComputeEdge; row++) begin : gen_row
    for (genvar col = 0; col < ComputeEdge; col++) begin : gen_col
      logic [AccWordWidth-1:0] acc_fpu_rdata, acc_fpu_wdata;
      logic [AccMaskWidth-1:0] acc_fpu_wen;
      logic [AccWordWidth-1:0] result;
      logic [AccWordWidth-1:0] addend;
      logic fma16_valid, fma32_valid;
      logic [15:0] fma16_result;
      logic [31:0] fma32_result, sdotp16_result;

      always_comb begin : local_feedback
        result = sdotp16_result;
        if (fma16_valid)
          result = {{(AccWordWidth-16){1'b0}}, fma16_result};
        else if (fma32_valid)
          result = fma32_result;

        addend = '0;
        if (forward_row[row] && pipe_tag_q[NumPipeRegs-1].lane_active[row][col])
          addend = result;
        else if ((compute_read_zero[row][col] & acc_read_mask_row[row]) ==
                 acc_read_mask_row[row])
          addend = '0;
        else begin
          for (int unsigned byte_idx = 0; byte_idx < AccMaskWidth; byte_idx++) begin
            if (acc_read_mask_row[row][byte_idx])
              addend[((byte_idx - unsigned'(acc_read_byte_offset_row[row])) <<
                      3) +: 1 << 3] =
                  acc_fpu_rdata[(byte_idx << 3) +: 1 << 3];
          end
        end

        acc_fpu_wdata = '0;
        acc_fpu_wen = '0;
        if (result_write_row[row] &&
            pipe_tag_q[NumPipeRegs-1].lane_active[row][col]) begin
          acc_fpu_wen = acc_write_mask_row[row];
          for (int unsigned byte_idx = 0; byte_idx < AccMaskWidth; byte_idx++) begin
            if (acc_write_mask_row[row][byte_idx])
              acc_fpu_wdata[(byte_idx << 3) +: 1 << 3] =
                  result[((byte_idx - unsigned'(acc_write_byte_offset_row[row])) <<
                          3) +: 1 << 3];
          end
        end
      end

      accumulator_tile #(
        .AccWordWidth (AccWordWidth),
        .AccWordBytes (AccWordBytes),
        .AccMaskWidth (AccMaskWidth),
        .AccDepth     (NrAccBankedTiles * EdgeBeats * EdgeBeats),
        .AccReadPorts (EdgeBeats),
        .AccWritePorts(EdgeBeats)
      ) i_accumulator (
        .clk_i,
        .fpu_wdata_i  (acc_fpu_wdata          ),
        .fpu_wen_i    (acc_fpu_wen            ),
        .fpu_waddr_i  (acc_waddr_row[row]      ),
        .fpu_raddr_i  (acc_raddr_row[row]      ),
        .fpu_rdata_o  (acc_fpu_rdata          ),
        .vrf_wdata_i  (mover_acc_wdata[row][col] ),
        .vrf_wen_i    (mover_acc_wen[row][col]   ),
        .vrf_waddr_i  (mover_acc_waddr_row[row]     ),
        .vrf_raddr_i  (mover_acc_raddr_row[row]     ),
        .vrf_rdata_o  (mover_acc_rdata[row][col] ),
        .vlsu_wdata_i (tile_acc_wdata[row][col]  ),
        .vlsu_wen_i   (tile_acc_wen[row][col]    ),
        .vlsu_waddr_i (tile_acc_waddr_row[row]      ),
        .vlsu_raddr_i (tile_acc_raddr_row[row]      ),
        .vlsu_rdata_o (tile_acc_rdata[row][col]  )
      );

      if (EnableFMA16) begin : gen_fma16
        ope_fma #(
          .FpFormat(fpnew_pkg::FP16), .NumPipeRegs(NumPipeRegs - 1),
          .PipeConfig(fpnew_pkg::DISTRIBUTED), .Stallable(1'b1)
        ) i_fma16 (
          .clk_i(fpu_clk), .rst_ni,
          .operands_i({w_operand_q[col][15:0], x_operand_q[row][15:0]}),
          .addend_i(addend_q[row][col][15:0]),
          .valid_i(compute_valid_row[row] && lane_active_q[row][col] &&
                   !compute_sdotp_row[row] && (compute_tew_row[row] == EW_16) &&
                   (compute_sew_row[row] == EW_16)),
          .ready_o(), .reg_enable_i(1'b1), .result_valid_o(fma16_valid),
          .result_ready_i(1'b1), .result_o(fma16_result)
        );
      end else begin : gen_no_fma16
        assign fma16_valid = 1'b0;
        assign fma16_result = '0;
      end

      if (EnableFMA32) begin : gen_fma32
        ope_fma #(
          .FpFormat(fpnew_pkg::FP32), .NumPipeRegs(NumPipeRegs - 1),
          .PipeConfig(fpnew_pkg::DISTRIBUTED), .Stallable(1'b1)
        ) i_fma32 (
          .clk_i(fpu_clk), .rst_ni,
          .operands_i({w_operand_q[col], x_operand_q[row]}),
          .addend_i(addend_q[row][col]),
          .valid_i(compute_valid_row[row] && lane_active_q[row][col] &&
                   !compute_sdotp_row[row] && (compute_tew_row[row] == EW_32) &&
                   (compute_sew_row[row] == EW_32)),
          .ready_o(), .reg_enable_i(1'b1), .result_valid_o(fma32_valid),
          .result_ready_i(1'b1), .result_o(fma32_result)
        );
      end else begin : gen_no_fma32
        assign fma32_valid = 1'b0;
        assign fma32_result = '0;
      end

      ope_sdotp #(
        .SrcDotpFpFmtConfig(fpnew_pkg::fmt_logic_t'(6'b001000)),
        .DstDotpFpFmtConfig(fpnew_pkg::fmt_logic_t'(6'b100000)),
        .NumPipeRegs(NumPipeRegs - 1), .PipeConfig(fpnew_pkg::DISTRIBUTED),
        .Stallable(1'b1)
      ) i_sdotp16 (
        .clk_i(fpu_clk), .rst_ni,
        .operand_a_i({{16{1'b0}}, x_operand_q[row][15:0]}),
        .operand_b_i(w_operand_q[col][15:0]),
        .operand_c_i({{16{1'b0}}, x_operand_q[row][31:16]}),
        .operand_d_i(w_operand_q[col][31:16]),
        .dst_operands_i(addend_q[row][col]), .reg_enable_i(1'b1),
        .result_o(sdotp16_result)
      );

      always_ff @(posedge fpu_clk or negedge rst_ni) begin
        if (!rst_ni)
          addend_q[row][col] <= '0;
        else
          addend_q[row][col] <= addend;
      end
    end
  end

  always_ff @(posedge fpu_clk or negedge rst_ni) begin
    if (!rst_ni) begin
      x_operand_q <= '0;
      w_operand_q <= '0;
      lane_active_q <= '0;
      compute_valid_q <= 1'b0;
      compute_sdotp_q <= 1'b0;
      compute_sew_q <= EW_16;
      compute_tew_q <= EW_32;
    end else begin
      x_operand_q <= x_operand_i;
      w_operand_q <= w_operand_i;
      lane_active_q <= lane_active_i;
      compute_valid_q <= compute_valid_i;
      compute_sdotp_q <= compute_sdotp_i;
      compute_sew_q <= compute_sew_i;
      compute_tew_q <= compute_tew_i;
    end
  end

  always_ff @(posedge fpu_clk or negedge rst_ni) begin
    if (!rst_ni)
      pipe_tag_q <= '0;
    else
      pipe_tag_q <= pipe_tag_d;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)
      acc_zero_q <= '1;
    else
      acc_zero_q <= acc_zero_d;
  end

  if (NumPipeRegs < 2)
    $error("[compute_array] NumPipeRegs must be at least two.");
  if (SpatialBeats != EdgeBeats * EdgeBeats)
    $error("[compute_array] SpatialBeats must match TileEdge and ComputeEdge.");

endmodule : compute_array
