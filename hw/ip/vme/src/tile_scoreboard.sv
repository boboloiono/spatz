// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Pei-Yu Lin, ETH Zürich

// VME-local logical-tile dependency scoreboard. Architectural register
// dependencies remain the host controller's responsibility.
module tile_scoreboard
  import vme_pkg::*;
(
  input  logic              clk_i,
  input  logic              rst_ni,
  input  request_t          dispatch_req_i,
  input  logic              dispatch_valid_i,
  output deps_t            dispatch_deps_o,
  output dep_generations_t dispatch_generations_o,
  input  logic [3:0]        complete_valid_i,
  input  instruction_id_t [3:0]     complete_id_i,
  output deps_t            outstanding_o,
  output dep_generations_t generations_o
);

  logic [VME_NR_LOGICAL_TILES-1:0] read_tiles, write_tiles;
  logic [VME_NR_LOGICAL_TILES-1:0] writer_valid_d, writer_valid_q;
  logic [VME_NR_LOGICAL_TILES-1:0] writer_is_mac_d, writer_is_mac_q;
  instruction_id_t [VME_NR_LOGICAL_TILES-1:0] writer_id_d, writer_id_q;
  deps_t [VME_NR_LOGICAL_TILES-1:0] reader_ids_d, reader_ids_q;
  deps_t outstanding_d, outstanding_q;
  dep_generations_t generations_d, generations_q;

  always_comb begin : access_decode
    read_tiles = '0;
    write_tiles = '0;

    if (dispatch_req_i.op_ope.is_mac) begin
      read_tiles[dispatch_req_i.mtd] = 1'b1;
      write_tiles[dispatch_req_i.mtd] = 1'b1;
    end else if (dispatch_req_i.op_ope.tss.tile_valid && dispatch_req_i.use_vd) begin
      read_tiles[dispatch_req_i.op_ope.tss.tile_id] = 1'b1;
    end else if (dispatch_req_i.op_ope.tss.tile_valid && dispatch_req_i.use_vs2) begin
      write_tiles[dispatch_req_i.op_ope.tss.tile_id] = 1'b1;
    end else if (dispatch_req_i.op_ope.is_zero_tile) begin
      write_tiles[dispatch_req_i.mtd] = 1'b1;
    end else if (dispatch_req_i.op_ope.is_discard) begin
      write_tiles = '1;
    end
  end

  always_comb begin : dep_lookup
    dispatch_deps_o = '0;
    dispatch_generations_o = generations_q;
    for (int unsigned tile = 0; tile < VME_NR_LOGICAL_TILES; tile++) begin
      // Consecutive MACs are kept in program order by the compute queue and
      // use the per-beat accumulator forwarding path.  Waiting for the whole
      // preceding MAC here would drain the FPU pipeline between every
      // VTFMM.  Writers from the mover, tile LSU, or clean unit still require
      // an instruction-level dependency.
      if (read_tiles[tile] && writer_valid_q[tile] &&
          !(dispatch_req_i.op_ope.is_mac && writer_is_mac_q[tile]))
        dispatch_deps_o[writer_id_q[tile]] = 1'b1;
      if (write_tiles[tile]) begin
        if (writer_valid_q[tile] &&
            !(dispatch_req_i.op_ope.is_mac && writer_is_mac_q[tile]))
          dispatch_deps_o[writer_id_q[tile]] = 1'b1;
        dispatch_deps_o |= reader_ids_q[tile];
      end
    end
    // The incoming instruction cannot depend on an older generation of its
    // own host ID; other captured IDs retain their generation snapshots.
    dispatch_deps_o[dispatch_req_i.id] = 1'b0;
  end

  always_comb begin : scoreboard_update
    writer_valid_d = writer_valid_q;
    writer_is_mac_d = writer_is_mac_q;
    writer_id_d = writer_id_q;
    reader_ids_d = reader_ids_q;
    outstanding_d = outstanding_q;
    generations_d = generations_q;

    for (int unsigned port = 0; port < 4; port++) begin
      if (complete_valid_i[port]) begin
        outstanding_d[complete_id_i[port]] = 1'b0;
        generations_d[complete_id_i[port]] =
            generations_d[complete_id_i[port]] + 1'b1;
        for (int unsigned tile = 0; tile < VME_NR_LOGICAL_TILES; tile++) begin
          reader_ids_d[tile][complete_id_i[port]] = 1'b0;
          if (writer_valid_d[tile] && (writer_id_d[tile] == complete_id_i[port]))
            writer_valid_d[tile] = 1'b0;
        end
      end
    end

    if (dispatch_valid_i) begin
      outstanding_d[dispatch_req_i.id] = 1'b1;
      for (int unsigned tile = 0; tile < VME_NR_LOGICAL_TILES; tile++) begin
        // A MAC is an ordered read-modify-write operation.  Its writer entry
        // is sufficient to order later non-MAC consumers; recording it as a
        // reader as well would reintroduce a false MAC-to-MAC dependency.
        if (read_tiles[tile] && !dispatch_req_i.op_ope.is_mac)
          reader_ids_d[tile][dispatch_req_i.id] = 1'b1;
        if (write_tiles[tile]) begin
          writer_valid_d[tile] = 1'b1;
          writer_is_mac_d[tile] = dispatch_req_i.op_ope.is_mac;
          writer_id_d[tile] = dispatch_req_i.id;
        end
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      writer_valid_q <= '0;
      writer_is_mac_q <= '0;
      writer_id_q <= '0;
      reader_ids_q <= '0;
      outstanding_q <= '0;
      generations_q <= '0;
    end else begin
      writer_valid_q <= writer_valid_d;
      writer_is_mac_q <= writer_is_mac_d;
      writer_id_q <= writer_id_d;
      reader_ids_q <= reader_ids_d;
      outstanding_q <= outstanding_d;
      generations_q <= generations_d;
    end
  end

  assign outstanding_o = outstanding_q;
  assign generations_o = generations_q;

endmodule : tile_scoreboard
