// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Pei-Yu Lin, ETH Zürich

// Spatz integration for the processor-independent VME command and data interfaces.
module spatz_vme
  import spatz_pkg::*;
  import rvv_pkg::*;
#(
  parameter int unsigned ComputeEdge       = VMEComputeEdge,
  parameter int unsigned TileEdge          = spatz_pkg::TileEdge,
  parameter int unsigned AccWordWidth      = vme_pkg::VME_ACC_WORD_WIDTH,
  parameter int unsigned MaxVregGroupWords = vme_pkg::VME_MAX_VREG_GROUP_WORDS,
  parameter bit          EnableFMA16       = 1'b0,
  parameter bit          EnableFMA32       = 1'b1,
  parameter int unsigned NrMemPorts        = 8,
  parameter type         spatz_mem_req_t   = logic,
  parameter type         spatz_mem_rsp_t   = logic
) (
  input  logic                                    clk_i,
  input  logic                                    rst_ni,

  input  spatz_req_t                              spatz_req_i,
  input  logic                                    spatz_req_valid_i,
  output logic                                    spatz_req_ready_o,
  input  logic [NrParallelInstructions-1:0]       matrix_enable_i,

  output logic                                    vme_rsp_valid_o,
  input  logic                                    vme_rsp_ready_i,
  output vme_rsp_t                                vme_rsp_o,

  output vrf_addr_t                               vrf_waddr_o,
  output vrf_data_t                               vrf_wdata_o,
  output logic                                    vrf_we_o,
  output vrf_be_t                                 vrf_wbe_o,
  input  logic                                    vrf_wvalid_i,

  output vrf_addr_t  [2:0]                      vrf_raddr_o,
  output logic       [2:0]                      vrf_re_o,
  input  logic       [2:0]                      vrf_rgnt_i,
  input  vrf_data_t  [2:0]                      vrf_rdata_i,
  input  logic       [2:0]                      vrf_rvalid_i,
  output spatz_id_t  [3:0]                      vrf_id_o,

  output spatz_mem_req_t [NrMemPorts-1:0]       tile_mem_req_o,
  output logic           [NrMemPorts-1:0]       tile_mem_req_valid_o,
  input  logic           [NrMemPorts-1:0]       tile_mem_req_ready_i,
  input  spatz_mem_rsp_t [NrMemPorts-1:0]       tile_mem_rsp_i,
  input  logic           [NrMemPorts-1:0]       tile_mem_rsp_valid_i,
  output logic                                    tile_mem_finished_o,
  output logic                                    tile_mem_store_finished_o
);

  vme_pkg::request_t vme_req;
  vme_pkg::response_t vme_rsp;
  vme_pkg::instruction_id_t [3:0] vme_vrf_id;
  vme_pkg::tile_mem_req_t [NrMemPorts-1:0] vme_mem_req;
  vme_pkg::tile_mem_rsp_t [NrMemPorts-1:0] vme_mem_rsp;

  always_comb begin : adapt_request
    vme_req = '0;
    vme_req.id            = vme_pkg::instruction_id_t'(spatz_req_i.id);
    vme_req.vs1           = vme_pkg::vreg_t'(spatz_req_i.vs1);
    vme_req.vs2           = vme_pkg::vreg_t'(spatz_req_i.vs2);
    vme_req.use_vs2       = spatz_req_i.use_vs2;
    vme_req.vd            = vme_pkg::vreg_t'(spatz_req_i.vd);
    vme_req.use_vd        = spatz_req_i.use_vd;
    vme_req.mtd           = vme_pkg::mt_t'(spatz_req_i.mtd);
    vme_req.vtype.vsew    = vme_pkg::vew_e'(spatz_req_i.vtype.vsew);
    vme_req.vl            = vme_pkg::vlen_t'(spatz_req_i.vl);
    vme_req.vstart        = vme_pkg::vlen_t'(spatz_req_i.vstart);
    vme_req.scalar_addr   = vme_pkg::mem_addr_t'(spatz_req_i.rs1);
    vme_req.is_tile_mem   = spatz_req_i.op_vme.is_mem;
    vme_req.tile_mem_load = spatz_req_i.op_mem.is_load;
    vme_req.tile_mem_ew   = vme_pkg::vew_e'(spatz_req_i.op_mem.ew);

    vme_req.op_vme.is_mac       = spatz_req_i.op_vme.is_mac;
    vme_req.op_vme.is_zero_tile = spatz_req_i.op_vme.is_zero_tile;
    vme_req.op_vme.is_discard   = spatz_req_i.op_vme.is_discard;
    vme_req.op_vme.tew          = vme_pkg::vew_e'(spatz_req_i.op_vme.tew);
    vme_req.op_vme.tn           = vme_pkg::tile_count_t'(spatz_req_i.op_vme.tn);
    vme_req.op_vme.tm           = vme_pkg::tile_count_t'(spatz_req_i.op_vme.tm);
    vme_req.op_vme.tk           = spatz_req_i.op_vme.tk[2:0];
    vme_req.op_vme.tss.tile_valid = spatz_req_i.op_vme.tss.tile_valid;
    vme_req.op_vme.tss.tile_id    = vme_pkg::mt_t'(spatz_req_i.op_vme.tss.tile_id);
    vme_req.op_vme.tss.is_row     = spatz_req_i.op_vme.tss.is_row;
    vme_req.op_vme.tss.index      = vme_pkg::tile_dim_t'(spatz_req_i.op_vme.tss.index);
  end

  assign vme_rsp_o.id = spatz_id_t'(vme_rsp.id);
  assign vrf_id_o     = vme_vrf_id;

  always_comb begin : adapt_tile_memory
    for (int unsigned port = 0; port < NrMemPorts; port++) begin
      tile_mem_req_o[port]       = '0;
      tile_mem_req_o[port].addr  = vme_mem_req[port].addr;
      tile_mem_req_o[port].write = vme_mem_req[port].write;
      tile_mem_req_o[port].data  = vme_mem_req[port].data;
      tile_mem_req_o[port].strb  = vme_mem_req[port].strb;
`ifndef MEMPOOL_SPATZ
      tile_mem_req_o[port].amo  = reqrsp_pkg::AMONone;
      tile_mem_req_o[port].user = '0;
`endif
      vme_mem_rsp[port].data = tile_mem_rsp_i[port].data;
    end
  end

  vme #(
    .ComputeEdge             (ComputeEdge),
    .TileEdge               (TileEdge),
    .AccWordWidth            (AccWordWidth),
    .MaxVregGroupWords       (MaxVregGroupWords),
    .EnableFMA16             (EnableFMA16),
    .EnableFMA32             (EnableFMA32),
    .NrMemPorts              (NrMemPorts)
  ) i_vme_core (
    .clk_i,
    .rst_ni,
    .req_i                   (vme_req                   ),
    .req_valid_i             (spatz_req_valid_i         ),
    .req_ready_o             (spatz_req_ready_o         ),
    .matrix_enable_i,
    .rsp_valid_o             (vme_rsp_valid_o           ),
    .rsp_ready_i             (vme_rsp_ready_i           ),
    .rsp_o                   (vme_rsp                   ),
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
    .vrf_id_o                (vme_vrf_id                ),
    .tile_mem_req_o          (vme_mem_req               ),
    .tile_mem_req_valid_o,
    .tile_mem_req_ready_i,
    .tile_mem_rsp_i          (vme_mem_rsp),
    .tile_mem_rsp_valid_i,
    .tile_mem_finished_o,
    .tile_mem_store_finished_o
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

endmodule : spatz_vme
