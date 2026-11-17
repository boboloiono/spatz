// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Pei-Yu Lin, ETH Zürich

// Tile-side load/store datapath. Host-specific TCDM, AXI, cache, and virtual
// memory protocols terminate outside VME; this unit maps generic tile words to
// the distributed accumulator ports.
module tile_lsu
  import vme_pkg::*;
#(
  parameter int unsigned ComputeEdge = VME_COMPUTE_EDGE,
  parameter int unsigned TileEdge = VME_TILE_EDGE,
  parameter int unsigned AccWordWidth = VME_ACC_WORD_WIDTH,
  parameter int unsigned AccWordBytes = AccWordWidth >> 3,
  parameter int unsigned AccMaskWidth = AccWordBytes
) (
  input  logic            tile_wvalid_i,
  input  tile_write_req_t tile_w_req_i,
  input  logic            tile_w_blocked_i,
  output logic            tile_wready_o,
  input  logic            tile_rvalid_i,
  input  tile_read_req_t tile_r_req_i,
  input  logic            tile_r_blocked_i,
  output logic            tile_rready_o,
  output tile_word_t      tile_rdata_o,

  input logic [$clog2(VME_NR_LOGICAL_TILES/AccWordBytes)-1:0] read_tile_i,
  input logic [$clog2(AccWordBytes)-1:0] read_byte_offset_i,
  input logic [$clog2(VME_NR_LOGICAL_TILES/AccWordBytes)-1:0] write_tile_i,
  input logic [$clog2(AccWordBytes)-1:0] write_byte_offset_i,
  input logic [AccMaskWidth-1:0] write_mask_i,
  input logic [VME_NR_LOGICAL_TILES/AccWordBytes-1:0]
              [(TileEdge/ComputeEdge)*(TileEdge/ComputeEdge)-1:0]
              [ComputeEdge-1:0][ComputeEdge-1:0]
              [AccMaskWidth-1:0] zero_i,

  output logic [$clog2((VME_NR_LOGICAL_TILES/AccWordBytes)*(TileEdge/ComputeEdge)*(TileEdge/ComputeEdge))-1:0] acc_raddr_o,
  input  logic [ComputeEdge-1:0][ComputeEdge-1:0][TileEdge/ComputeEdge-1:0][AccWordWidth-1:0] acc_rdata_i,
  output logic [$clog2((VME_NR_LOGICAL_TILES/AccWordBytes)*(TileEdge/ComputeEdge)*(TileEdge/ComputeEdge))-1:0] acc_waddr_o,
  output logic [ComputeEdge-1:0][ComputeEdge-1:0][TileEdge/ComputeEdge-1:0][AccWordWidth-1:0] acc_wdata_o,
  output logic [ComputeEdge-1:0][ComputeEdge-1:0]
               [TileEdge/ComputeEdge-1:0][AccMaskWidth-1:0] acc_wen_o
);

  localparam int unsigned EdgeBeats = TileEdge / ComputeEdge;
  localparam int unsigned EdgeBeatW = EdgeBeats > 1 ? $clog2(EdgeBeats) : 1;
  localparam int unsigned SpatialBeats = EdgeBeats * EdgeBeats;

  assign tile_wready_o = !tile_w_blocked_i;
  assign tile_rready_o = !tile_r_blocked_i;

  always_comb begin : tile_access
    logic [EdgeBeatW-1:0] write_edge_row, read_edge_row, selected_edge_row;
    logic [$clog2(VME_NR_LOGICAL_TILES/AccWordBytes)-1:0] selected_tile;
    int unsigned write_tew_bytes, read_tew_bytes;

    write_tew_bytes = 1 << int'(tile_w_req_i.tew);
    read_tew_bytes = 1 << int'(tile_r_req_i.tew);

    write_edge_row = EdgeBeatW'(tile_w_req_i.row / ComputeEdge);
    read_edge_row = EdgeBeatW'(tile_r_req_i.row / ComputeEdge);
    selected_edge_row = tile_wvalid_i ? write_edge_row : read_edge_row;
    selected_tile = tile_wvalid_i ? write_tile_i : read_tile_i;

    acc_raddr_o = $bits(acc_raddr_o)'(selected_tile * SpatialBeats +
                                      selected_edge_row * EdgeBeats);

    acc_waddr_o = acc_raddr_o;
    acc_wdata_o = '0;
    acc_wen_o = '0;
    tile_rdata_o = '0;

    if (tile_wvalid_i) begin
      for (int row = 0; row < ComputeEdge; row++) begin
        for (int col = 0; col < ComputeEdge; col++) begin
          for (int beat = 0; beat < EdgeBeats; beat++) begin
            int unsigned dest_col;
            int unsigned source_elem;
            dest_col = beat * ComputeEdge + col;
            source_elem = dest_col - unsigned'(tile_w_req_i.col);
            if ((unsigned'(tile_w_req_i.row) % ComputeEdge == row) &&
                (dest_col >= unsigned'(tile_w_req_i.col)) &&
                (source_elem < unsigned'(tile_w_req_i.elems))) begin
              for (int unsigned elem_byte = 0; elem_byte < AccWordBytes; elem_byte++) begin
                if (elem_byte < write_tew_bytes)
                  acc_wdata_o[row][col][beat]
                      [((unsigned'(write_byte_offset_i) + elem_byte) <<
                        3) +: 1 << 3] =
                      tile_w_req_i.data[((source_elem*write_tew_bytes + elem_byte) <<
                                         3) +: 1 << 3];
              end
              acc_wen_o[row][col][beat] = tile_wready_o ?
                  write_mask_i : AccMaskWidth'(0);
            end
          end
        end
      end
    end

    for (int unsigned elem_idx = 0; elem_idx < TileEdge; elem_idx++) begin
      if (elem_idx < unsigned'(tile_r_req_i.elems)) begin
        logic [$clog2(ComputeEdge)-1:0] bank_col;
        logic [EdgeBeatW-1:0] edge_col;
        logic [AccWordWidth-1:0] acc_word;
        tile_dim_t col_i;
        col_i = tile_dim_t'(tile_r_req_i.col + tile_dim_t'(elem_idx));
        bank_col = $clog2(ComputeEdge)'(col_i % ComputeEdge);
        edge_col = EdgeBeatW'(col_i / ComputeEdge);
        acc_word = acc_rdata_i[tile_r_req_i.row % ComputeEdge][bank_col][edge_col];
        for (int unsigned mask_idx = 0; mask_idx < AccMaskWidth; mask_idx++)
          if (zero_i[read_tile_i][read_edge_row * EdgeBeats + edge_col]
                    [tile_r_req_i.row % ComputeEdge][bank_col][mask_idx])
            acc_word[(mask_idx << 3) +: 1 << 3] = '0;
        for (int unsigned elem_byte = 0; elem_byte < AccWordBytes; elem_byte++) begin
          if (elem_byte < read_tew_bytes)
            tile_rdata_o[((elem_idx*read_tew_bytes + elem_byte) <<
                          3) +: 1 << 3] =
                acc_word[((unsigned'(read_byte_offset_i) + elem_byte) <<
                          3) +: 1 << 3];
        end
      end
    end
  end

endmodule : tile_lsu
