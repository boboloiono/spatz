// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Spatz adapter for the processor-independent VME command and data interfaces.
module spatz_ope
  import spatz_pkg::*;
  import rvv_pkg::*;
#(
  parameter int unsigned ComputeEdge = OPEComputeEdge,
  parameter int unsigned TileEdge = spatz_pkg::TileEdge,
  parameter int unsigned AccWordWidth = vme_pkg::VME_ACC_WORD_WIDTH,
  parameter int unsigned AccWordBytes = AccWordWidth >> 3,
  parameter int unsigned SpatialBeats = 4,
  parameter int unsigned MaxVregGroupWords = vme_pkg::VME_MAX_VREG_GROUP_WORDS,
  parameter int unsigned NumPipeRegs = vme_pkg::VME_FPU_PIPE_REGS,
  parameter bit EnableFMA16 = 1'b0,
  parameter bit EnableFMA32 = 1'b1
) (
  input  logic             clk_i,
  input  logic             rst_ni,

  input  spatz_req_t       spatz_req_i,
  input  logic             spatz_req_valid_i,
  output logic             spatz_req_ready_o,
  input  logic [NrParallelInstructions-1:0] matrix_enable_i,

  output logic             ope_rsp_valid_o,
  input  logic             ope_rsp_ready_i,
  output ope_rsp_t         ope_rsp_o,

  output vrf_addr_t        vrf_waddr_o,
  output vrf_data_t        vrf_wdata_o,
  output logic             vrf_we_o,
  output vrf_be_t          vrf_wbe_o,
  input  logic             vrf_wvalid_i,

  output vrf_addr_t  [2:0] vrf_raddr_o,
  output logic       [2:0] vrf_re_o,
  input  logic       [2:0] vrf_rgnt_i,
  input  vrf_data_t  [2:0] vrf_rdata_i,
  input  logic       [2:0] vrf_rvalid_i,
  output spatz_id_t  [3:0] vrf_id_o,

  input  logic             tile_wvalid_i,
  input  tile_w_req_t      tile_w_req_i,
  output logic             tile_wready_o,
  input  logic             tile_rvalid_i,
  input  tile_r_req_t      tile_r_req_i,
  output tile_word_t       tile_rdata_o,
  output logic             tile_rready_o
);

  vme_pkg::request_t        vme_req;
  vme_pkg::response_t       vme_rsp;
  vme_pkg::tile_write_req_t vme_tile_w_req;
  vme_pkg::tile_read_req_t  vme_tile_r_req;
  vme_pkg::tile_word_t      vme_tile_rdata;
  vme_pkg::instruction_id_t [3:0] vme_vrf_id;

  always_comb begin
    vme_req = '0;
    vme_req.id      = vme_pkg::instruction_id_t'(spatz_req_i.id);
    vme_req.vs1     = vme_pkg::vreg_t'(spatz_req_i.vs1);
    vme_req.vs2     = vme_pkg::vreg_t'(spatz_req_i.vs2);
    vme_req.use_vs2 = spatz_req_i.use_vs2;
    vme_req.vd      = vme_pkg::vreg_t'(spatz_req_i.vd);
    vme_req.use_vd  = spatz_req_i.use_vd;
    vme_req.mtd     = vme_pkg::mt_t'(spatz_req_i.mtd);
    vme_req.vtype.vsew = vme_pkg::vew_e'(spatz_req_i.vtype.vsew);
    vme_req.vl      = vme_pkg::vlen_t'(spatz_req_i.vl);
    vme_req.vstart  = vme_pkg::vlen_t'(spatz_req_i.vstart);

    vme_req.op_ope.is_mac       = spatz_req_i.op_ope.is_mac;
    vme_req.op_ope.is_zero_tile = spatz_req_i.op_ope.is_zero_tile;
    vme_req.op_ope.is_discard   = spatz_req_i.op_ope.is_discard;
    vme_req.op_ope.tew = vme_pkg::vew_e'(spatz_req_i.op_ope.tew);
    vme_req.op_ope.tn  = vme_pkg::tile_count_t'(spatz_req_i.op_ope.tn);
    vme_req.op_ope.tm  = vme_pkg::tile_count_t'(spatz_req_i.op_ope.tm);
    vme_req.op_ope.tk  = spatz_req_i.op_ope.tk[2:0];
    vme_req.op_ope.tss.tile_valid = spatz_req_i.op_ope.tss.tile_valid;
    vme_req.op_ope.tss.tile_id = vme_pkg::mt_t'(spatz_req_i.op_ope.tss.tile_id);
    vme_req.op_ope.tss.is_row = spatz_req_i.op_ope.tss.is_row;
    vme_req.op_ope.tss.index = vme_pkg::tile_dim_t'(spatz_req_i.op_ope.tss.index);

  end

  always_comb begin : adapt_tile_requests
    vme_tile_w_req.idx = vme_pkg::mt_t'(tile_w_req_i.idx);
    vme_tile_w_req.row = vme_pkg::tile_dim_t'(tile_w_req_i.row);
    vme_tile_w_req.col = vme_pkg::tile_dim_t'(tile_w_req_i.col);
    vme_tile_w_req.elems = vme_pkg::tile_count_t'(tile_w_req_i.elems);
    vme_tile_w_req.tew = vme_pkg::vew_e'(tile_w_req_i.tew);
    vme_tile_w_req.data = vme_pkg::tile_word_t'(tile_w_req_i.data);
    vme_tile_r_req.idx = vme_pkg::mt_t'(tile_r_req_i.idx);
    vme_tile_r_req.row = vme_pkg::tile_dim_t'(tile_r_req_i.row);
    vme_tile_r_req.col = vme_pkg::tile_dim_t'(tile_r_req_i.col);
    vme_tile_r_req.elems = vme_pkg::tile_count_t'(tile_r_req_i.elems);
    vme_tile_r_req.tew = vme_pkg::vew_e'(tile_r_req_i.tew);
  end : adapt_tile_requests
  assign ope_rsp_o.id = spatz_id_t'(vme_rsp.id);
  assign vrf_id_o = vme_vrf_id;
  assign tile_rdata_o = tile_word_t'(vme_tile_rdata);

  vme #(
    .ComputeEdge             (ComputeEdge),
    .TileEdge               (TileEdge),
    .AccWordWidth            (AccWordWidth),
    .AccWordBytes            (AccWordBytes),
    .SpatialBeats            (SpatialBeats),
    .MaxVregGroupWords       (MaxVregGroupWords),
    .NumPipeRegs             (NumPipeRegs),
    .EnableFMA16             (EnableFMA16),
    .EnableFMA32             (EnableFMA32)
  ) i_vme (
    .clk_i,
    .rst_ni,
    .req_i        (vme_req),
    .req_valid_i  (spatz_req_valid_i),
    .req_ready_o  (spatz_req_ready_o),
    .matrix_enable_i,
    .rsp_valid_o  (ope_rsp_valid_o),
    .rsp_ready_i  (ope_rsp_ready_i),
    .rsp_o        (vme_rsp),
    .vrf_waddr_o,
    .vrf_wdata_o,
    .vrf_we_o,
    .vrf_wbe_o,
    .vrf_wvalid_i,
    .vrf_raddr_o,
    .vrf_re_o,
    .vrf_rgnt_i,
    .vrf_rdata_i,
    .vrf_rvalid_i,
    .vrf_id_o         (vme_vrf_id),
    .tile_wvalid_i,
    .tile_w_req_i     (vme_tile_w_req),
    .tile_wready_o,
    .tile_rvalid_i,
    .tile_r_req_i     (vme_tile_r_req),
    .tile_rdata_o     (vme_tile_rdata),
    .tile_rready_o
  );

  if (TileEdge != vme_pkg::VME_TILE_EDGE)
    $error("[VME] Spatz and VME architectural tile edges must match.");
  if (NrPhysicalTile != vme_pkg::VME_NR_LOGICAL_TILES)
    $error("[VME] Spatz and VME logical tile counts must match.");
  if (KMAX != vme_pkg::VME_K_MAX)
    $error("[VME] Spatz and VME architectural K limits must match.");
  if (NrParallelInstructions != vme_pkg::VME_NR_INFLIGHT)
    $error("[VME] Spatz and VME in-flight ID counts must match.");
  if (VLEN != vme_pkg::VME_VLEN)
    $error("[VME] Spatz and VME vector lengths must match.");
  if (NRVREG != vme_pkg::VME_NR_VREGS)
    $error("[VME] Spatz and VME vector register counts must match.");
  if (VRFWordWidth != vme_pkg::VME_VRF_WORD_WIDTH)
    $error("[VME] Spatz and VME VRF word widths must match.");
  if (AccElemWidth != vme_pkg::VME_ACC_WORD_WIDTH)
    $error("[VME] Spatz and VME accumulator word widths must match.");
  if (AccWordWidth != vme_pkg::VME_ACC_WORD_WIDTH)
    $error("[VME] Adapter and VME accumulator word widths must match.");
  if (AccWordBytes != (AccWordWidth >> 3))
    $error("[VME] AccWordBytes must match AccWordWidth.");
endmodule
