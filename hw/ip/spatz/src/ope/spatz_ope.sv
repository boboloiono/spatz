// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Pei-Yu Lin <peilin@ethz.ch>
//
// spatz_ope — Outer Product Engine wrapper for Spatz.
//
// Tile state: tile_state_q[ptile][word] is the spec-visible, Zvt-compliant
// architectural tile state. Every access (VTMV_VT/TV, VTLE/VTSE via the VLSU
// tile interface, VTZERO, FMA addend/result) goes through zvt_pun32_te8
// (spatz_pkg) as the single address-generation path for the tile layout.
//
// MAC pipelining: the TE*TE opope_fma array is a real NumPipeRegs-deep
// Stallable pipeline. Up to MacSlots=NumPipeRegs independent vtfmm chains
// (one per destination tile; at most 4 exist since TEW=32 has only
// mt0/mt4/mt8/mt12) can be in flight at once, tracked by a tag shift
// register (mac_tag_q) that mirrors opope_fma's own internal valid-pipe
// shift exactly. A same-cycle chain results in fma_addend_proc bypassing
// tile_state_q with the FMA's own just-retired result, so back-to-back
// vtfmm calls to the same tile (a K-loop) can fire every cycle a
// different tile isn't already occupying the pipe.

module spatz_ope
  import spatz_pkg::*;
  import rvv_pkg::*;
  import fpnew_pkg::*;
(
  input  logic             clk_i              ,
  input  logic             rst_ni             ,

  input  spatz_req_t       spatz_req_i        ,
  input  logic             spatz_req_valid_i  ,
  output logic             spatz_req_ready_o  ,

  output logic             ope_rsp_valid_o    ,
  input  logic             ope_rsp_ready_i    ,
  output vfu_rsp_t         ope_rsp_o          ,

  output vrf_addr_t        vrf_waddr_o        ,
  output vrf_data_t        vrf_wdata_o        ,
  output logic             vrf_we_o           ,
  output vrf_be_t          vrf_wbe_o          ,
  input  logic             vrf_wvalid_i       ,
  output spatz_id_t  [2:0] vrf_id_o           ,

  output vrf_addr_t  [1:0] vrf_raddr_o        ,
  output logic       [1:0] vrf_re_o           ,
  input  vrf_data_t  [1:0] vrf_rdata_i        ,
  input  logic       [1:0] vrf_rvalid_i       ,

  input  logic                      tile_wvalid_i  ,
  input  logic [$clog2(NRTILE)-1:0] tile_widx_i    ,
  input  logic [$clog2(TE)-1:0]     tile_wrow_i    ,
  input  vrf_data_t                 tile_wdata_i   ,
  output logic                      tile_wready_o  ,

  input  logic                      tile_rvalid_i  ,
  input  logic [$clog2(NRTILE)-1:0] tile_ridx_i    ,
  input  logic [$clog2(TE)-1:0]     tile_rrow_i    ,
  output vrf_data_t                 tile_rdata_o   ,
  output logic                      tile_rready_o
);

`include "common_cells/registers.svh"

  localparam int unsigned NumPipeRegs = 4;
  localparam int unsigned TILE_IDX_W = 2;
  localparam int unsigned MacSlots = NumPipeRegs;

  spatz_req_t req       ;
  logic       req_valid ;
  logic       req_ready ;

  spill_register #(.T(spatz_req_t)) i_op_queue (
    .clk_i  (clk_i              ),
    .rst_ni (rst_ni             ),
    .data_i (spatz_req_i        ),
    .valid_i(spatz_req_valid_i  ),
    .ready_o(spatz_req_ready_o  ),
    .data_o (req                ),
    .valid_o(req_valid          ),
    .ready_i(req_ready          )
  );

  logic tss_valid;
  assign tss_valid = ((req.rs1[26:24] == 3'd0) || (req.rs1[26:24] == 3'd1)) && (req.rs1[23:0] < TE);

  // tile_zero_q: set by VTZERO, cleared when FMA first writes result back (not
  // at dispatch) so the VTZERO -> MAC addend-zero semantics are preserved.
  // tile_state_q reads as 0 while the flag is set (fma_addend_proc /
  // vrf_wr_proc / tile_rdata_proc).
  logic [NRTILE-1:0][NrWordsPerTile-1:0][TEW-1:0] tile_state_d, tile_state_q;
  logic [NRTILE-1:0][NrWordsPerTile-1:0]           tile_zero_d, tile_zero_q;

  typedef enum logic [1:0] {
    GRP_MAC    = 2'd0,
    GRP_VT     = 2'd1,
    GRP_TV     = 2'd2,
    GRP_SIMPLE = 2'd3
  } op_grp_e;

  op_grp_e   op_grp;
  logic [3:0] path_valid, path_ready;

  always_comb begin : op_grp_decode
    unique case (req.op)
      VTFMM, VTFMM_ALT, VTMMU, VTMMS : op_grp = GRP_MAC;
      VTMV_VT                          : op_grp = GRP_VT;
      VTMV_TV                          : op_grp = GRP_TV;
      default                          : op_grp = GRP_SIMPLE;
    endcase
  end

  stream_demux #(.N_OUP(4)) i_demux (
    .inp_valid_i(req_valid  ),
    .inp_ready_o(req_ready  ),
    .oup_sel_i  (op_grp     ),
    .oup_valid_o(path_valid ),
    .oup_ready_i(path_ready )
  );

  typedef struct packed {
    logic                     tile_en  ;
    logic                     mac_fire ;
  } ctrl_t;

  ctrl_t ctrl;

  function automatic vfu_rsp_t make_rsp(input spatz_req_t r);
    vfu_rsp_t v;
    v       = '0;
    v.id    = r.id;
    v.rd    = r.rd[GPRWidth-1:0];
    return v;
  endfunction

  typedef struct packed {
    logic [TILE_IDX_W-1:0] tile;
    spatz_id_t             id;
    logic [GPRWidth-1:0]   rd;
  } mac_tag_t;

  // mac_pend_q/mac_pend_busy_q is a fallback latch for a request that
  // couldn't fire the same cycle it was accepted (VRF not ready yet, tile
  // hazard, or pipe full). mac_eff is whichever candidate is live this
  // cycle -- the held one if present, else the fresh incoming one -- so a
  // brand-new request can fire the very cycle it arrives (VRF reads are
  // 0-latency/combinational) instead of always paying one latch cycle.
  spatz_req_t mac_pend_q;
  logic       mac_pend_busy_d, mac_pend_busy_q;

  spatz_req_t mac_eff;
  logic       mac_eff_valid;
  assign mac_eff       = mac_pend_busy_q ? mac_pend_q : req;
  assign mac_eff_valid = mac_pend_busy_q || path_valid[GRP_MAC];

  logic mac_vrf_rdy;
  assign mac_vrf_rdy = (!mac_eff.use_vs2 || vrf_rvalid_i[0]) &&
                       (!mac_eff.use_vs1 || vrf_rvalid_i[1]);

  logic [TE-1:0][TE-1:0] fma_ready_a;
  logic                   fma_ready_all;
  assign fma_ready_all = &fma_ready_a;

  logic fma_result_valid;

  // Per-tile inflight tracking: at most one outstanding vtfmm per
  // destination tile at a time. mac_retire_tile/mac_tag_q are defined
  // below; a same-cycle chain reissue to the tile retiring this cycle is
  // not a hazard (fma_addend_proc bypasses tile_state_q for it).
  logic [3:0] tile_inflight_d, tile_inflight_q;
  logic [$clog2(MacSlots+1)-1:0] mac_inflight_cnt_d, mac_inflight_cnt_q;

  logic [TILE_IDX_W-1:0] mac_retire_tile;
  spatz_id_t             mac_retire_id;
  logic [GPRWidth-1:0]   mac_retire_rd;

  logic mac_tile_hazard;
  assign mac_tile_hazard = tile_inflight_q[mac_eff.vd[3:2]] &&
                            !(fma_result_valid && mac_retire_tile == mac_eff.vd[3:2]);

  logic mac_fire;
  assign mac_fire = mac_eff_valid && mac_vrf_rdy && fma_ready_all &&
                     !mac_tile_hazard && (mac_inflight_cnt_q < MacSlots);

  assign path_ready[GRP_MAC] = !mac_pend_busy_q;

  always_comb begin : mac_pend_handler
    mac_pend_busy_d = mac_fire ? 1'b0 : (mac_pend_busy_q || (path_valid[GRP_MAC] && !mac_fire));
  end : mac_pend_handler

  always_comb begin : mac_inflight_update
    tile_inflight_d    = tile_inflight_q;
    mac_inflight_cnt_d = mac_inflight_cnt_q;

    if (fma_result_valid) begin
      tile_inflight_d[mac_retire_tile] = 1'b0;
      mac_inflight_cnt_d = mac_inflight_cnt_d - 1;
    end

    if (mac_fire) begin
      tile_inflight_d[mac_eff.vd[3:2]] = 1'b1;
      mac_inflight_cnt_d = mac_inflight_cnt_d + 1;
    end
  end : mac_inflight_update

  // Tag shift register mirroring opope_fma's own internal valid-pipe shift.
  // Must shift exactly when opope_fma's internal pipe actually advances:
  // fma_clk only ticks when ctrl.tile_en=1 (else this register would
  // desync by shifting on real clk_i edges with no fma_clk edge), and even
  // when it ticks, opope_fma's own pipe_enable=ready_o may be 0 under
  // backpressure (result_ready_i=mac_commit_ready) -- so gate on both.
  mac_tag_t [MacSlots-1:0] mac_tag_d, mac_tag_q;
  logic                    mac_tag_shift_en;
  mac_tag_t                mac_tag_new;

  assign mac_tag_shift_en = ctrl.tile_en && fma_ready_all;
  assign mac_tag_new      = mac_fire ? '{tile: mac_eff.vd[3:2], id: mac_eff.id, rd: mac_eff.rd[GPRWidth-1:0]} : '0;

  always_comb begin : mac_tag_shift
    mac_tag_d = mac_tag_q;
    if (mac_tag_shift_en) begin
      mac_tag_d = {mac_tag_q[MacSlots-2:0], mac_tag_new};
    end
  end : mac_tag_shift

  assign mac_retire_tile = mac_tag_q[MacSlots-1].tile;
  assign mac_retire_id   = mac_tag_q[MacSlots-1].id;
  assign mac_retire_rd   = mac_tag_q[MacSlots-1].rd;

  logic     mac_commit_valid, mac_commit_ready;
  vfu_rsp_t mac_commit_rsp;
  vfu_rsp_t mac_done_rsp;
  logic     mac_done_valid, mac_done_ready;

  assign mac_commit_valid = fma_result_valid;

  always_comb begin : mac_commit_rsp_proc
    mac_commit_rsp    = '0;
    mac_commit_rsp.id = mac_retire_id;
    mac_commit_rsp.rd = mac_retire_rd;
  end : mac_commit_rsp_proc

  spill_register #(.T(vfu_rsp_t)) i_mac_commit (
    .clk_i  (clk_i             ),
    .rst_ni (rst_ni            ),
    .data_i (mac_commit_rsp    ),
    .valid_i(mac_commit_valid  ),
    .ready_o(mac_commit_ready  ),
    .data_o (mac_done_rsp      ),
    .valid_o(mac_done_valid    ),
    .ready_i(mac_done_ready    )
  );

  spatz_req_t  vt_req_q                       ;
  logic        vt_tss_valid_q                 ;
  logic        vt_busy_d    , vt_busy_q       ;
  logic        vt_commit_valid, vt_commit_ready;
  vfu_rsp_t    vt_done_rsp                    ;
  logic        vt_done_valid , vt_done_ready  ;

  logic [TILE_IDX_W-1:0] vt_tss_tile;
  logic [$clog2(TE)-1:0] vt_tss_row ;
  assign vt_tss_tile = vt_req_q.rs1[30:29];
  assign vt_tss_row  = ($clog2(TE))'(vt_req_q.rs1[23:0]);

  logic mac_pipe_idle;
  assign mac_pipe_idle = !mac_pend_busy_q && (mac_inflight_cnt_q == 0);

  // VT/TV/VLSU tile access requires the MAC pipe fully drained: VT/TV read
  // tile_state_q combinationally while an in-flight MAC could still be
  // committing a result into the same physical tile/word. Requiring full
  // drain (rather than proving per-tile safety while draining) trades away
  // the previous VT/TV-overlaps-MAC-drain optimization for correctness;
  // gemm.c never interleaves vtmv/vtse with its K-loop, so no cost there.
  assign path_ready[GRP_VT] = !vt_busy_q && mac_pipe_idle;

  always_comb begin : vt_handler
    vt_busy_d        = vt_busy_q;
    vt_commit_valid  = 1'b0;
    if (path_valid[GRP_VT] && path_ready[GRP_VT])
      vt_busy_d = 1'b1;
    if (vt_busy_q) begin
      vt_commit_valid = !vt_tss_valid_q || !vt_req_q.use_vd || vrf_wvalid_i;
      if (vt_commit_valid && vt_commit_ready)
        vt_busy_d = 1'b0;
    end
  end : vt_handler

  spill_register #(.T(vfu_rsp_t)) i_vt_commit (
    .clk_i  (clk_i            ),
    .rst_ni (rst_ni           ),
    .data_i (make_rsp(vt_req_q)),
    .valid_i(vt_commit_valid  ),
    .ready_o(vt_commit_ready  ),
    .data_o (vt_done_rsp      ),
    .valid_o(vt_done_valid    ),
    .ready_i(vt_done_ready    )
  );

  spatz_req_t  tv_req_q                       ;
  logic        tv_tss_valid_q                 ;
  logic        tv_busy_d    , tv_busy_q       ;
  logic        tv_commit_valid, tv_commit_ready;
  vfu_rsp_t    tv_done_rsp                    ;
  logic        tv_done_valid , tv_done_ready  ;

  logic [TILE_IDX_W-1:0] tv_tss_tile;
  logic [$clog2(TE)-1:0] tv_tss_row ;
  assign tv_tss_tile = tv_req_q.rs1[30:29];
  assign tv_tss_row  = ($clog2(TE))'(tv_req_q.rs1[23:0]);

  // VRF data is latched (tv_data_q) on first rvalid so that if
  // fma_result_valid blocks the acc write for one cycle, TV can retry
  // without re-reading the VRF.
  vrf_data_t tv_data_q;
  logic      tv_data_latched_q;
  vrf_data_t tv_vrf_data;
  logic      tv_vrf_avail;

  assign tv_vrf_avail = tv_data_latched_q || (tv_busy_q && vrf_rvalid_i[0]);
  assign tv_vrf_data  = tv_data_latched_q ? tv_data_q : vrf_rdata_i[0];

  logic tv_acc_wen;
  assign tv_acc_wen = tv_busy_q && tv_tss_valid_q && tv_vrf_avail && !fma_result_valid;

  assign path_ready[GRP_TV] = !tv_busy_q && mac_pipe_idle;

  always_comb begin : tv_handler
    tv_busy_d        = tv_busy_q;
    tv_commit_valid  = 1'b0;
    if (path_valid[GRP_TV] && path_ready[GRP_TV])
      tv_busy_d = 1'b1;
    if (tv_busy_q) begin
      tv_commit_valid = !tv_tss_valid_q || !tv_req_q.use_vs2 || tv_acc_wen;
      if (tv_commit_valid && tv_commit_ready)
        tv_busy_d = 1'b0;
    end
  end : tv_handler

  spill_register #(.T(vfu_rsp_t)) i_tv_commit (
    .clk_i  (clk_i            ),
    .rst_ni (rst_ni           ),
    .data_i (make_rsp(tv_req_q)),
    .valid_i(tv_commit_valid  ),
    .ready_o(tv_commit_ready  ),
    .data_o (tv_done_rsp      ),
    .valid_o(tv_done_valid    ),
    .ready_i(tv_done_ready    )
  );

  vfu_rsp_t simple_done_rsp                      ;
  logic     simple_done_valid, simple_done_ready ;
  logic     simple_commit_ready                  ;
  assign    path_ready[GRP_SIMPLE] = simple_commit_ready;

  logic [TE-1:0][TE-1:0][TEW-1:0] fma_addend;
  logic [TE-1:0][TE-1:0][TEW-1:0] fma_result        ;
  logic [TE-1:0][TE-1:0]          fma_result_valid_a ;

  assign fma_result_valid = fma_result_valid_a[0][0];

  spill_register #(.T(vfu_rsp_t)) i_simple_commit (
    .clk_i  (clk_i                  ),
    .rst_ni (rst_ni                 ),
    .data_i (make_rsp(req)          ),
    .valid_i(path_valid[GRP_SIMPLE] ),
    .ready_o(simple_commit_ready    ),
    .data_o (simple_done_rsp        ),
    .valid_o(simple_done_valid      ),
    .ready_i(simple_done_ready      )
  );

  always_comb begin : tile_state_update
    zvt_ptile_t ptile;
    zvt_word_t  word;

    logic [3:0] arch_tile;
    logic [2:0] idx;
    logic [2:0] row_i;
    logic [2:0] col_i;

    int unsigned active_len;
    int unsigned vstart_i;

    tile_state_d = tile_state_q;
    tile_zero_d  = tile_zero_q;

    ptile      = '0;
    word       = '0;
    arch_tile  = '0;
    idx        = '0;
    row_i      = '0;
    col_i      = '0;
    active_len = 0;
    vstart_i   = 0;

    if (path_valid[GRP_SIMPLE] && simple_commit_ready && req.op == VTDISCARD) begin
      tile_state_d = '0;
      tile_zero_d  = '1;
    end

    if (path_valid[GRP_SIMPLE] && simple_commit_ready && req.op == VTZERO) begin
      arch_tile = {req.vd[3:2], 2'b00};

      for (int r = 0; r < TE; r++) begin
        for (int c = 0; c < TE; c++) begin
          zvt_pun32_te8(arch_tile, r[2:0], c[2:0], ptile, word);
          tile_state_d[ptile][word] = '0;
          tile_zero_d[ptile][word]  = 1'b1;
        end
      end
    end

    if (tile_wvalid_i && tile_wready_o) begin
      arch_tile = {tile_widx_i[1:0], 2'b00};

      for (int i = 0; i < TE; i++) begin
        zvt_pun32_te8(arch_tile, tile_wrow_i[2:0], i[2:0], ptile, word);
        tile_state_d[ptile][word] = tile_wdata_i[i*TEW +: TEW];
        tile_zero_d[ptile][word]  = 1'b0;
      end
    end

    if (tv_acc_wen) begin
      arch_tile  = {tv_req_q.rs1[30:29], 2'b00};
      idx        = tv_req_q.rs1[2:0];
      active_len = (tv_req_q.vl < TE) ? int'(tv_req_q.vl) : TE;
      vstart_i   = int'(tv_req_q.vstart);

      for (int i = 0; i < TE; i++) begin
        if ((i >= vstart_i) && (i < active_len)) begin
          if (tv_req_q.rs1[26:24] == 3'd0) begin
            row_i = idx;
            col_i = i[2:0];
          end else begin
            row_i = i[2:0];
            col_i = idx;
          end

          zvt_pun32_te8(arch_tile, row_i, col_i, ptile, word);

          tile_state_d[ptile][word] = tv_vrf_data[i*TEW +: TEW];
          tile_zero_d[ptile][word]  = 1'b0;
        end
      end
    end

    if (fma_result_valid) begin
      arch_tile = {mac_retire_tile, 2'b00};

      for (int r = 0; r < TE; r++) begin
        for (int c = 0; c < TE; c++) begin
          zvt_pun32_te8(arch_tile, r[2:0], c[2:0], ptile, word);
          tile_state_d[ptile][word] = fma_result[r][c];
          tile_zero_d[ptile][word]  = 1'b0;
        end
      end
    end
  end

  vfu_rsp_t [3:0] arb_inp_data ;
  logic     [3:0] arb_inp_valid;
  logic     [3:0] arb_inp_ready;

  assign arb_inp_data [GRP_MAC]    = mac_done_rsp   ;
  assign arb_inp_data [GRP_VT]     = vt_done_rsp    ;
  assign arb_inp_data [GRP_TV]     = tv_done_rsp    ;
  assign arb_inp_data [GRP_SIMPLE] = simple_done_rsp;

  assign arb_inp_valid[GRP_MAC]    = mac_done_valid   ;
  assign arb_inp_valid[GRP_VT]     = vt_done_valid    ;
  assign arb_inp_valid[GRP_TV]     = tv_done_valid    ;
  assign arb_inp_valid[GRP_SIMPLE] = simple_done_valid;

  assign mac_done_ready    = arb_inp_ready[GRP_MAC]   ;
  assign vt_done_ready     = arb_inp_ready[GRP_VT]    ;
  assign tv_done_ready     = arb_inp_ready[GRP_TV]    ;
  assign simple_done_ready = arb_inp_ready[GRP_SIMPLE];

  stream_arbiter #(
    .DATA_T  (vfu_rsp_t),
    .N_INP   (4        ),
    .ARBITER ("rr"     )
  ) i_rsp_arb (
    .clk_i       (clk_i           ),
    .rst_ni      (rst_ni          ),
    .inp_data_i  (arb_inp_data    ),
    .inp_valid_i (arb_inp_valid   ),
    .inp_ready_o (arb_inp_ready   ),
    .oup_data_o  (ope_rsp_o       ),
    .oup_valid_o (ope_rsp_valid_o ),
    .oup_ready_i (ope_rsp_ready_i )
  );

  // Port [0] owner: MAC while a candidate is live (pending or fresh), else TV.
  always_comb begin : sb_ids
    vrf_id_o[0] = mac_eff_valid ? mac_eff.id : tv_req_q.id;
    vrf_id_o[1] = mac_eff.id;
    vrf_id_o[2] = vt_req_q.id;
  end

  always_comb begin : vrf_re_proc
    vrf_re_o    = '0;
    vrf_raddr_o = '0;

    if (mac_eff_valid) begin
      vrf_re_o[0]    = mac_eff.use_vs2;
      vrf_re_o[1]    = mac_eff.use_vs1;
      vrf_raddr_o[0] = vrf_addr_t'(mac_eff.vs2) << $clog2(NrWordsPerVector);
      vrf_raddr_o[1] = vrf_addr_t'(mac_eff.vs1) << $clog2(NrWordsPerVector);
    end else if (tv_busy_q && tv_tss_valid_q && !tv_data_latched_q) begin
      vrf_re_o[0]    = tv_req_q.use_vs2;
      vrf_raddr_o[0] = vrf_addr_t'(tv_req_q.vs2) << $clog2(NrWordsPerVector);
    end
  end

  always_comb begin : vrf_wr_proc
    zvt_ptile_t ptile;
    zvt_word_t  word;
    logic [3:0] arch_tile;
    logic [2:0] idx;
    logic [2:0] row_i;
    logic [2:0] col_i;
    int unsigned active_len;
    int unsigned vstart_i;

    vrf_waddr_o = '0;
    vrf_we_o    = 1'b0;
    vrf_wbe_o   = '0;
    vrf_wdata_o = '0;

    arch_tile = '0;
    idx       = '0;
    row_i     = '0;
    col_i     = '0;
    ptile     = '0;
    word      = '0;

    active_len = (vt_req_q.vl < TE) ? int'(vt_req_q.vl) : TE;
    vstart_i   = int'(vt_req_q.vstart);

    if (vt_busy_q && vt_tss_valid_q) begin
      vrf_waddr_o = vrf_addr_t'(vt_req_q.vd) << $clog2(NrWordsPerVector);
      vrf_we_o    = vt_req_q.use_vd;

      arch_tile = {vt_req_q.rs1[30:29], 2'b00};
      idx       = vt_req_q.rs1[2:0];

      for (int i = 0; i < TE; i++) begin
        if ((i >= vstart_i) && (i < active_len)) begin
          if (vt_req_q.rs1[26:24] == 3'd0) begin
            row_i = idx;
            col_i = i[2:0];
          end else begin
            row_i = i[2:0];
            col_i = idx;
          end

          zvt_pun32_te8(arch_tile, row_i, col_i, ptile, word);

          vrf_wdata_o[i*TEW +: TEW] = tile_zero_q[ptile][word] ? '0 : tile_state_q[ptile][word];
          vrf_wbe_o[i*ELENB +: ELENB] = {ELENB{1'b1}};
        end
      end
    end
  end

  assign tile_wready_o = mac_pipe_idle && !vt_busy_q && !tv_busy_q && !req_valid;
  assign tile_rready_o = mac_pipe_idle && !vt_busy_q && !tv_busy_q && !req_valid;

  always_comb begin : tile_rdata_proc
    zvt_ptile_t ptile;
    zvt_word_t  word;
    logic [3:0] arch_tile;

    tile_rdata_o = '0;

    ptile     = '0;
    word      = '0;
    arch_tile = {tile_ridx_i[1:0], 2'b00};

    for (int i = 0; i < TE; i++) begin
      zvt_pun32_te8(arch_tile, tile_rrow_i[2:0], i[2:0], ptile, word);
      tile_rdata_o[i*TEW +: TEW] = tile_zero_q[ptile][word] ? '0 : tile_state_q[ptile][word];
    end
  end

  // ctrl.tile_en gates fma_clk: must be 1 whenever the pipe needs to
  // advance (a new fire this cycle) or hold entries that still need to
  // shift/drain (mac_inflight_cnt_q != 0), including stall cycles caused
  // by commit backpressure.
  always_comb begin : engine_ctrl
    ctrl          = '0;
    ctrl.tile_en  = mac_fire || (mac_inflight_cnt_q != 0);
    ctrl.mac_fire = mac_fire;
  end : engine_ctrl

  logic fma_clk;

  tc_clk_gating i_fma_clk_gate (
    .clk_i     (clk_i       ),
    .en_i      (ctrl.tile_en),
    .test_en_i ('0          ),
    .clk_o     (fma_clk     )
  );

  // VRF reads are combinational/0-latency and re-driven every cycle
  // mac_eff_valid is live (vrf_re_proc), so vrf_rdata_i is already the
  // fresh operand for whichever request is firing this cycle -- no latch
  // needed between VRF and the FMA inputs.
  logic [TE-1:0][TEW-1:0] x_op_fma, w_op_fma;
  assign x_op_fma = vrf_rdata_i[0][TE*TEW-1:0];
  assign w_op_fma = vrf_rdata_i[1][TE*TEW-1:0];

  always_comb begin : fma_addend_proc
    zvt_ptile_t ptile;
    zvt_word_t  word;
    logic [3:0] arch_tile;

    fma_addend = '0;
    ptile      = '0;
    word       = '0;
    arch_tile  = {mac_eff.vd[3:2], 2'b00};

    for (int r = 0; r < TE; r++) begin
      for (int c = 0; c < TE; c++) begin
        zvt_pun32_te8(arch_tile, r[2:0], c[2:0], ptile, word);
        // Chain to the FMA's own result for the tile retiring this cycle
        // instead of tile_state_q, which hasn't been written yet -- this
        // is what lets a K-loop's next vtfmm to the same tile fire the
        // exact cycle the previous one's result emerges.
        if (mac_eff_valid && fma_result_valid && (mac_retire_tile == mac_eff.vd[3:2])) begin
          fma_addend[r][c] = fma_result[r][c];
        end else begin
          fma_addend[r][c] = tile_zero_q[ptile][word] ? '0 : tile_state_q[ptile][word];
        end
      end
    end
  end

  // fma_result_valid is driven representatively from [0][0]: all TE*TE FMAs
  // fire together with identical latency, so any single instance's
  // result_valid_o reflects them all.
  for (genvar row = 0; row < TE; row++) begin : gen_row
    for (genvar col = 0; col < TE; col++) begin : gen_col

      logic this_fma_valid;

      assign fma_result_valid_a[row][col] = this_fma_valid;

      opope_fma #(
        .FpFormat    (fpnew_pkg::FP32       ),
        .NumPipeRegs (NumPipeRegs           ),
        .PipeConfig  (fpnew_pkg::DISTRIBUTED),
        .Stallable   (1'b1                  )
      ) i_fma (
        .clk_i          (fma_clk                                                               ),
        .rst_ni         (rst_ni                                                                ),
        .operands_i     ({fma_addend[row][col],
                          w_op_fma[col],
                          x_op_fma[row]}                                                       ),
        .valid_i        (ctrl.mac_fire                                                         ),
        .ready_o        (fma_ready_a[row][col]                                                 ),
        .reg_enable_i   (ctrl.tile_en                                                          ),
        .result_valid_o (this_fma_valid                                                        ),
        .result_ready_i (mac_commit_ready                                                      ),
        .result_o       (fma_result[row][col]                                                  )
      );

    end : gen_col
  end : gen_row

  always_ff @(posedge clk_i or negedge rst_ni) begin : seq_block
    if (!rst_ni) begin
      mac_pend_q         <= '0;
      mac_pend_busy_q     <= 1'b0;
      tile_inflight_q     <= '0;
      mac_inflight_cnt_q  <= '0;
      mac_tag_q           <= '0;
      vt_busy_q         <= 1'b0;
      vt_req_q          <= '0;
      vt_tss_valid_q    <= 1'b0;
      tv_busy_q         <= 1'b0;
      tv_req_q          <= '0;
      tv_tss_valid_q    <= 1'b0;
      tv_data_q         <= '0;
      tv_data_latched_q <= 1'b0;
      tile_state_q      <= '0;
      tile_zero_q       <= '0;
    end else begin
      tile_state_q       <= tile_state_d;
      tile_zero_q        <= tile_zero_d;
      tile_inflight_q     <= tile_inflight_d;
      mac_inflight_cnt_q  <= mac_inflight_cnt_d;
      mac_tag_q            <= mac_tag_d;
      mac_pend_busy_q      <= mac_pend_busy_d;

      if (!mac_pend_busy_q && path_valid[GRP_MAC] && !mac_fire)
        mac_pend_q <= req;

      if (path_valid[GRP_VT] && path_ready[GRP_VT]) begin
        vt_req_q       <= req;
        vt_tss_valid_q <= tss_valid;
      end
      vt_busy_q <= vt_busy_d;

      if (path_valid[GRP_TV] && path_ready[GRP_TV]) begin
        tv_req_q       <= req;
        tv_tss_valid_q <= tss_valid;
      end
      tv_busy_q <= tv_busy_d;

      // Latch TV VRF data on first rvalid; hold until acc write succeeds.
      // Needed because fma_result_valid (higher priority) may block TV for
      // one cycle -- data must be retained for the retry.
      if (!tv_busy_q) begin
        tv_data_latched_q <= 1'b0;
      end else if (vrf_rvalid_i[0] && !tv_data_latched_q) begin
        tv_data_q         <= vrf_rdata_i[0];
        tv_data_latched_q <= 1'b1;
      end
    end
  end : seq_block

endmodule : spatz_ope
