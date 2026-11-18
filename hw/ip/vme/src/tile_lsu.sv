// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Pei-Yu Lin, ETH Zürich

// Tile-memory command engine. The Spatz integration supplies decoded VTLE/VTSE
// commands and arbitrates this generic memory client with its vector LSU.
// Memory protocol details remain outside VME; this block owns tile traversal,
// unaligned packing, response tracking, and architectural completion.
module tile_lsu
  import vme_pkg::*;
#(
  parameter int unsigned NrMemPorts  = 8,
  parameter int unsigned TileEdge    = VME_TILE_EDGE,
  parameter int unsigned AccWordBytes = VME_ACC_WORD_BYTES,
  parameter int unsigned MemDataWidth = VME_MEM_DATA_WIDTH,
  parameter int unsigned MemDataBytes = MemDataWidth >> 3
) (
  input  logic                               clk_i,
  input  logic                               rst_ni,
  input  request_t                           req_i,
  input  logic                               req_valid_i,
  output logic                               req_ready_o,
  output response_t                          rsp_o,
  output logic                               rsp_valid_o,
  input  logic                               rsp_ready_i,
  output logic                               mem_finished_o,
  output logic                               mem_store_finished_o,

  output tile_mem_req_t [NrMemPorts-1:0]     mem_req_o,
  output logic          [NrMemPorts-1:0]     mem_req_valid_o,
  input  logic          [NrMemPorts-1:0]     mem_req_ready_i,
  input  tile_mem_rsp_t [NrMemPorts-1:0]     mem_rsp_i,
  input  logic          [NrMemPorts-1:0]     mem_rsp_valid_i,

  output logic                               tile_wvalid_o,
  output tile_write_req_t                    tile_w_req_o,
  input  logic                               tile_wready_i,
  output logic                               tile_rvalid_o,
  output tile_read_req_t                     tile_r_req_o,
  input  tile_word_t                         tile_rdata_i,
  input  logic                               tile_rready_i
);

  localparam int unsigned TileBytes = TileEdge * AccWordBytes;
  localparam int unsigned MaxChunks = (TileBytes + (MemDataBytes << 1) - 2) /
                                      MemDataBytes;
  localparam int unsigned ChunkW      = $clog2(MaxChunks + 1);
  localparam int unsigned ByteCountW  = $clog2(TileBytes + 1);
  localparam int unsigned PortCountW  = $clog2(NrMemPorts + 1);
  localparam int unsigned MemByteOffW = $clog2(MemDataBytes);

  typedef logic [ChunkW-1:0] chunk_t;
  typedef logic [ByteCountW-1:0] byte_count_t;
  typedef enum logic [2:0] {
    Idle,
    Gather,
    MemoryRequest,
    Commit,
    Complete
  } state_e;

  state_e state_d, state_q;
  request_t req_d, req_q;
  tile_word_t data_d, data_q;
  tile_dim_t line_d, line_q;
  tile_count_t elems_d, elems_q;
  byte_count_t bytes_d, bytes_q;
  chunk_t chunk_d, chunk_q;
  logic [NrMemPorts-1:0] accepted_d, accepted_q;
  logic [NrMemPorts-1:0] pending_d, pending_q;
  logic [NrMemPorts-1:0] active_ports;
  logic [PortCountW-1:0] active_count;
  chunk_t num_chunks;
  logic [MemByteOffW-1:0] head_offset;
  mem_addr_t base_aligned;

  always_comb begin : memory_geometry
    head_offset = req_q.scalar_addr[MemByteOffW-1:0];
    base_aligned = {req_q.scalar_addr[VME_MEM_ADDR_WIDTH-1:MemByteOffW],
                    {MemByteOffW{1'b0}}};
    num_chunks = chunk_t'((bytes_q + head_offset + MemDataBytes - 1) >>
                          MemByteOffW);
  end

  always_comb begin : memory_requests
    mem_req_o = '0;
    mem_req_valid_o = '0;
    active_ports = '0;
    active_count = '0;

    for (int unsigned port = 0; port < NrMemPorts; port++) begin
      chunk_t request_chunk;
      request_chunk = chunk_t'(chunk_q + port);
      if ((state_q == MemoryRequest) && (request_chunk < num_chunks)) begin
        active_ports[port] = 1'b1;
        active_count = active_count + 1'b1;
        mem_req_o[port].addr = base_aligned + mem_addr_t'(request_chunk * MemDataBytes);
        mem_req_o[port].write = !req_q.tile_mem_load;
        for (int unsigned byte_idx = 0; byte_idx < MemDataBytes; byte_idx++) begin
          int signed stream_idx;
          stream_idx = int'(request_chunk * MemDataBytes + byte_idx) - int'(head_offset);
          if ((stream_idx >= 0) && (stream_idx < int'(bytes_q))) begin
            mem_req_o[port].data[byte_idx*8 +: 8] = data_q[stream_idx*8 +: 8];
            mem_req_o[port].strb[byte_idx] = !req_q.tile_mem_load;
          end
        end
        mem_req_valid_o[port] = !accepted_q[port];
      end
    end

  end

  always_comb begin : tile_requests
    tile_rvalid_o = 1'b0;
    tile_r_req_o = '0;
    tile_r_req_o.idx = req_q.op_vme.tss.tile_id;
    tile_r_req_o.tew = req_q.op_vme.tew;
    tile_r_req_o.row = req_q.op_vme.tss.is_row ?
                       req_q.op_vme.tss.index : line_q;
    tile_r_req_o.col = req_q.op_vme.tss.is_row ?
                       tile_dim_t'(0) : req_q.op_vme.tss.index;
    tile_r_req_o.elems = req_q.op_vme.tss.is_row ? elems_q : tile_count_t'(1);

    tile_wvalid_o = 1'b0;
    tile_w_req_o = '0;
    tile_w_req_o.idx = req_q.op_vme.tss.tile_id;
    tile_w_req_o.tew = req_q.op_vme.tew;
    tile_w_req_o.row = req_q.op_vme.tss.is_row ?
                       req_q.op_vme.tss.index : line_q;
    tile_w_req_o.col = req_q.op_vme.tss.is_row ?
                       tile_dim_t'(0) : req_q.op_vme.tss.index;
    tile_w_req_o.elems = req_q.op_vme.tss.is_row ? elems_q : tile_count_t'(1);
    tile_w_req_o.data = req_q.op_vme.tss.is_row ? data_q :
        tile_word_t'(data_q[(unsigned'(line_q) * AccWordBytes * 8) +: AccWordBytes * 8]);

    if ((state_q == Idle) && req_valid_i && req_ready_o &&
        !req_i.tile_mem_load) begin
      tile_rvalid_o = 1'b1;
      tile_r_req_o.idx = req_i.op_vme.tss.tile_id;
      tile_r_req_o.tew = req_i.op_vme.tew;
      tile_r_req_o.row = req_i.op_vme.tss.is_row ?
                         req_i.op_vme.tss.index : tile_dim_t'(0);
      tile_r_req_o.col = req_i.op_vme.tss.is_row ?
                         tile_dim_t'(0) : req_i.op_vme.tss.index;
      tile_r_req_o.elems = req_i.op_vme.tss.is_row ?
          ((req_i.op_vme.tn < TileEdge) ? req_i.op_vme.tn : tile_count_t'(TileEdge)) :
          tile_count_t'(1);
    end else if ((state_q == Gather) && !req_q.tile_mem_load) begin
      tile_rvalid_o = 1'b1;
    end
    if ((state_q == Commit) && req_q.tile_mem_load)
      tile_wvalid_o = 1'b1;
  end

  always_comb begin : next_state
    state_d = state_q;
    req_d = req_q;
    data_d = data_q;
    line_d = line_q;
    elems_d = elems_q;
    bytes_d = bytes_q;
    chunk_d = chunk_q;
    accepted_d = accepted_q;
    pending_d = pending_q;

    req_ready_o = (state_q == Idle);
    rsp_o.id = req_q.id;
    rsp_valid_o = (state_q == Complete);
    mem_finished_o = rsp_valid_o && rsp_ready_i;
    mem_store_finished_o = mem_finished_o && !req_q.tile_mem_load;

    if (req_valid_i && req_ready_o) begin
      req_d = req_i;
      data_d = '0;
      line_d = '0;
      chunk_d = '0;
      accepted_d = '0;
      pending_d = '0;
      if (req_i.op_vme.tss.is_row)
        elems_d = (req_i.op_vme.tn < TileEdge) ?
                  req_i.op_vme.tn : tile_count_t'(TileEdge);
      else
        elems_d = (req_i.op_vme.tm < TileEdge) ?
                  req_i.op_vme.tm : tile_count_t'(TileEdge);
      if (req_i.op_vme.tss.is_row)
        bytes_d = byte_count_t'(((req_i.op_vme.tn < TileEdge) ?
                    req_i.op_vme.tn : TileEdge) * AccWordBytes);
      else
        bytes_d = byte_count_t'(((req_i.op_vme.tm < TileEdge) ?
                    req_i.op_vme.tm : TileEdge) * AccWordBytes);
      if (req_i.tile_mem_load) begin
        state_d = MemoryRequest;
      end else if (tile_rready_i) begin
        if (req_i.op_vme.tss.is_row) begin
          data_d = tile_rdata_i;
          state_d = MemoryRequest;
        end else begin
          data_d[0 +: AccWordBytes * 8] = tile_rdata_i[AccWordBytes*8-1:0];
          if ((((req_i.op_vme.tm < TileEdge) ? req_i.op_vme.tm : TileEdge)) <= 1) begin
            line_d = '0;
            state_d = MemoryRequest;
          end else begin
            line_d = tile_dim_t'(1);
            state_d = Gather;
          end
        end
      end else begin
        state_d = Gather;
      end
    end

    unique case (state_q)
      Idle: begin
      end

      Gather: begin
        if (tile_rvalid_o && tile_rready_i) begin
          if (req_q.op_vme.tss.is_row) begin
            data_d = tile_rdata_i;
            state_d = MemoryRequest;
          end else begin
            data_d[(unsigned'(line_q) * AccWordBytes * 8) +: AccWordBytes * 8] =
                tile_rdata_i[AccWordBytes*8-1:0];
            if ((tile_count_t'(line_q) + 1'b1) >= elems_q) begin
              line_d = '0;
              state_d = MemoryRequest;
            end else begin
              line_d = line_q + 1'b1;
            end
          end
        end
      end

      MemoryRequest: begin
        for (int unsigned port = 0; port < NrMemPorts; port++) begin
          if (mem_req_valid_o[port] && mem_req_ready_i[port]) begin
            accepted_d[port] = 1'b1;
            pending_d[port] = 1'b1;
          end
          if (pending_q[port] && mem_rsp_valid_i[port]) begin
            pending_d[port] = 1'b0;
            if (req_q.tile_mem_load) begin
              for (int unsigned byte_idx = 0; byte_idx < MemDataBytes; byte_idx++) begin
                int signed stream_idx;
                stream_idx = int'((chunk_q + port) * MemDataBytes + byte_idx) -
                             int'(head_offset);
                if ((stream_idx >= 0) && (stream_idx < int'(bytes_q)))
                  data_d[stream_idx*8 +: 8] =
                      mem_rsp_i[port].data[byte_idx*8 +: 8];
              end
            end
          end
        end
        if ((active_ports != '0) &&
            ((accepted_d & active_ports) == active_ports) &&
            (pending_d == '0)) begin
          if ((chunk_q + chunk_t'(active_count)) < num_chunks) begin
            chunk_d = chunk_q + chunk_t'(active_count);
            accepted_d = '0;
          end else if (req_q.tile_mem_load) begin
            line_d = '0;
            state_d = Commit;
          end else begin
            rsp_valid_o = 1'b1;
            mem_finished_o = rsp_ready_i;
            mem_store_finished_o = rsp_ready_i;
            state_d = rsp_ready_i ? Idle : Complete;
          end
        end
      end

      Commit: begin
        if (tile_wvalid_o && tile_wready_i) begin
          if (req_q.op_vme.tss.is_row ||
              ((tile_count_t'(line_q) + 1'b1) >= elems_q)) begin
            state_d = Complete;
          end else begin
            line_d = line_q + 1'b1;
          end
        end
      end

      Complete: begin
        if (rsp_valid_o && rsp_ready_i) state_d = Idle;
      end

      default: state_d = Idle;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q <= Idle;
      req_q <= '0;
      data_q <= '0;
      line_q <= '0;
      elems_q <= '0;
      bytes_q <= '0;
      chunk_q <= '0;
      accepted_q <= '0;
      pending_q <= '0;
    end else begin
      state_q <= state_d;
      req_q <= req_d;
      data_q <= data_d;
      line_q <= line_d;
      elems_q <= elems_d;
      bytes_q <= bytes_d;
      chunk_q <= chunk_d;
      accepted_q <= accepted_d;
      pending_q <= pending_d;
    end
  end

endmodule : tile_lsu
