// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Danilo Cammarata <dcammarata@iis.ee.ethz.ch>
//
// Tile-indexed accumulator for the OPE engine.
// Memory is a flat array of DEPTH × DATA_WIDTH bits.
// The read port returns RD_PORTS consecutive aligned entries as a pair.
// ext_ld_i selects bulk write (WR_PORTS aligned entries) vs. single-entry write.

module opope_accumulator #(
  parameter int unsigned DATA_WIDTH = 32       ,
  parameter int unsigned DEPTH      =  4       ,   // must be a power of 2
  parameter int unsigned RD_PORTS   =  2       ,   // must be a power of 2
  parameter int unsigned WR_PORTS   =  2           // must be a power of 2
)(
  input  logic                                  clk_i              ,
  input  logic                                  rst_ni             ,
  input  logic                                  flush_i            ,
  input  logic                                  iteration_change_i ,

  // Write port
  input  logic [WR_PORTS-1:0][DATA_WIDTH-1:0]  wdata_i            ,
  input  logic                                  wen_i              ,
  input  logic [$clog2(DEPTH)-1:0]             waddr_i            ,
  input  logic                                  ext_ld_i           ,   // 1=write WR_PORTS aligned entries, 0=write one

  // Read port  (returns RD_PORTS consecutive aligned entries)
  input  logic [$clog2(DEPTH)-1:0]             raddr_i            ,
  output logic [RD_PORTS-1:0][DATA_WIDTH-1:0]  rdata_o
);

  localparam int unsigned ADDR_W    = $clog2(DEPTH)   ;
  localparam int unsigned WR_ALIGN  = $clog2(WR_PORTS);
  localparam int unsigned RD_ALIGN  = $clog2(RD_PORTS);

  logic [DEPTH-1:0][DATA_WIDTH-1:0] mem_d, mem_q;

  // Aligned base addresses
  logic [ADDR_W-1:0] wr_base, rd_base;
  assign wr_base = {waddr_i[ADDR_W-1:WR_ALIGN], {WR_ALIGN{1'b0}}};
  assign rd_base = {raddr_i[ADDR_W-1:RD_ALIGN], {RD_ALIGN{1'b0}}};

  assign rdata_o = mem_q[rd_base +: RD_PORTS];

  always_comb begin : write_proc
    mem_d = mem_q;
    if (flush_i || (wen_i && iteration_change_i)) begin
      mem_d = '0;
    end else if (wen_i && ext_ld_i) begin
      mem_d[wr_base +: WR_PORTS] = wdata_i;    // bulk write: WR_PORTS aligned entries
    end else if (wen_i) begin
      mem_d[waddr_i] = wdata_i[0];             // single write: exact address, entry 0
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin : mem_seq
    if (!rst_ni) mem_q <= '0;
    else         mem_q <= mem_d;
  end

  if ((DEPTH    == 0) || ((DEPTH    & (DEPTH   -1)) != 0)) $error("[opope_accumulator] DEPTH must be power of 2.");
  if ((RD_PORTS == 0) || ((RD_PORTS & (RD_PORTS-1)) != 0)) $error("[opope_accumulator] RD_PORTS must be power of 2.");
  if ((WR_PORTS == 0) || ((WR_PORTS & (WR_PORTS-1)) != 0)) $error("[opope_accumulator] WR_PORTS must be power of 2.");
  if (RD_PORTS > DEPTH) $error("[opope_accumulator] RD_PORTS must be <= DEPTH.");
  if (WR_PORTS > DEPTH) $error("[opope_accumulator] WR_PORTS must be <= DEPTH.");

endmodule : opope_accumulator
