// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Danilo Cammarata, ETH Zürich
// Author: Pei-Yu Lin, ETH Zürich

module accumulator_tile
#(
  parameter int unsigned AccWordWidth = vme_pkg::VME_ACC_WORD_WIDTH,
  parameter int unsigned AccWordBytes = AccWordWidth >> 3,
  parameter int unsigned AccMaskWidth = AccWordBytes,
  parameter int unsigned AccDepth      = 4,
  parameter int unsigned AccAddrWidth  = $clog2(AccDepth),
  parameter int unsigned AccReadPorts  = 2,
  parameter int unsigned AccWritePorts = 2
) (
  input  logic                                clk_i              ,
  // interface with FPU Array within OPE
  input  logic [AccWordWidth-1:0]             fpu_wdata_i        ,
  input  logic [AccMaskWidth-1:0]             fpu_wen_i,
  input  logic [AccAddrWidth-1:0]              fpu_waddr_i        ,
  input  logic [AccAddrWidth-1:0]              fpu_raddr_i        ,
  output logic [AccWordWidth-1:0]             fpu_rdata_o        ,
  // interface with VRF
  input  logic [AccWritePorts-1:0][AccWordWidth-1:0] vrf_wdata_i  ,
  input  logic [AccWritePorts-1:0]
               [AccMaskWidth-1:0] vrf_wen_i,
  input  logic [AccAddrWidth-1:0]              vrf_waddr_i        ,
  input  logic [AccAddrWidth-1:0]              vrf_raddr_i        ,
  output logic [AccReadPorts-1:0][AccWordWidth-1:0] vrf_rdata_o   ,
  // interface with VLSU
  input  logic [AccWritePorts-1:0][AccWordWidth-1:0] vlsu_wdata_i ,
  input  logic [AccWritePorts-1:0]
               [AccMaskWidth-1:0] vlsu_wen_i,
  input  logic [AccAddrWidth-1:0]              vlsu_waddr_i       ,
  input  logic [AccAddrWidth-1:0]              vlsu_raddr_i       ,
  output logic [AccReadPorts-1:0][AccWordWidth-1:0] vlsu_rdata_o
);

  localparam int unsigned AccWritePortIdxWidth = $clog2(AccWritePorts);
  localparam int unsigned AccReadPortIdxWidth = $clog2(AccReadPorts);

  logic [AccDepth-1:0][AccWordWidth-1:0] acc_mem;
  logic [AccAddrWidth-1:0] vrf_waddr_base, vrf_raddr_base;
  logic [AccAddrWidth-1:0] vlsu_waddr_base, vlsu_raddr_base;

  always_comb begin : align_vrf_addr
    if (AccWritePorts <= 1)
      vrf_waddr_base = vrf_waddr_i;
    else if (AccWritePortIdxWidth >= AccAddrWidth)
      vrf_waddr_base = AccAddrWidth'(0);
    else
      vrf_waddr_base = {vrf_waddr_i[AccAddrWidth-1:AccWritePortIdxWidth],
                        {AccWritePortIdxWidth{1'b0}}};

    if (AccReadPorts <= 1)
      vrf_raddr_base = vrf_raddr_i;
    else if (AccReadPortIdxWidth >= AccAddrWidth)
      vrf_raddr_base = AccAddrWidth'(0);
    else
      vrf_raddr_base = {vrf_raddr_i[AccAddrWidth-1:AccReadPortIdxWidth],
                        {AccReadPortIdxWidth{1'b0}}};
  end

  always_comb begin : align_vlsu_addr
    if (AccWritePorts <= 1)
      vlsu_waddr_base = vlsu_waddr_i;
    else if (AccWritePortIdxWidth >= AccAddrWidth)
      vlsu_waddr_base = AccAddrWidth'(0);
    else
      vlsu_waddr_base = {vlsu_waddr_i[AccAddrWidth-1:AccWritePortIdxWidth],
                         {AccWritePortIdxWidth{1'b0}}};

    if (AccReadPorts <= 1)
      vlsu_raddr_base = vlsu_raddr_i;
    else if (AccReadPortIdxWidth >= AccAddrWidth)
      vlsu_raddr_base = AccAddrWidth'(0);
    else
      vlsu_raddr_base = {vlsu_raddr_i[AccAddrWidth-1:AccReadPortIdxWidth],
                         {AccReadPortIdxWidth{1'b0}}};
  end

  assign fpu_rdata_o  = acc_mem[fpu_raddr_i];
  assign vrf_rdata_o  = acc_mem[vrf_raddr_base +: AccReadPorts];
  assign vlsu_rdata_o = acc_mem[vlsu_raddr_base +: AccReadPorts];

  // The data array deliberately has no reset or global clear.  Reset/discard/VTZERO
  // are represented by granule-level zero metadata in spatz_ope.  Physically
  // clearing this array created a single control net driving every accumulator bit
  // and its clock gate, which is both unnecessary and a severe timing bottleneck.
  always_ff @(posedge clk_i) begin
    for (int unsigned mask_idx = 0; mask_idx < AccMaskWidth; mask_idx++) begin
      if (fpu_wen_i[mask_idx])
        acc_mem[fpu_waddr_i][(mask_idx << 3) +: 1 << 3] <=
            fpu_wdata_i[(mask_idx << 3) +: 1 << 3];
    end
    for (int unsigned port = 0; port < AccWritePorts; port++) begin
      for (int unsigned mask_idx = 0; mask_idx < AccMaskWidth; mask_idx++) begin
        if (vrf_wen_i[port][mask_idx])
          acc_mem[vrf_waddr_base + port][(mask_idx << 3) +: 1 << 3] <=
              vrf_wdata_i[port][(mask_idx << 3) +: 1 << 3];
        if (vlsu_wen_i[port][mask_idx])
          acc_mem[vlsu_waddr_base + port][(mask_idx << 3) +: 1 << 3] <=
              vlsu_wdata_i[port][(mask_idx << 3) +: 1 << 3];
      end
    end
  end

  if ((AccDepth == 0) || ((AccDepth & (AccDepth-1)) != 0))
    $error("[accumulator_tile] AccDepth must be power of 2.");
  if ((AccReadPorts == 0) || ((AccReadPorts & (AccReadPorts-1)) != 0))
    $error("[accumulator_tile] AccReadPorts must be power of 2.");
  if ((AccWritePorts == 0) || ((AccWritePorts & (AccWritePorts-1)) != 0))
    $error("[accumulator_tile] AccWritePorts must be power of 2.");
  if (AccReadPorts > AccDepth)
    $error("[accumulator_tile] AccReadPorts must be <= AccDepth.");
  if (AccWritePorts > AccDepth)
    $error("[accumulator_tile] AccWritePorts must be <= AccDepth.");
  if (AccWordBytes != (AccWordWidth >> 3))
    $error("[accumulator_tile] AccWordBytes must match AccWordWidth.");
  if (AccMaskWidth != AccWordBytes)
    $error("[accumulator_tile] AccMaskWidth must provide one enable per byte.");

endmodule : accumulator_tile
