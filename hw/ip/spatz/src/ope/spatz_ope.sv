// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Pei-Yu Lin <peilin@ethz.ch>
//
// spatz_ope — Outer Product Engine wrapper for Spatz.

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

  input  logic             tile_wvalid_i      ,
  input  tile_w_req_t      tile_w_req_i       ,
  output logic             tile_wready_o      ,
  input  logic             tile_rvalid_i      ,
  input  tile_r_req_t      tile_r_req_i       ,
  output tile_row_t        tile_rdata_o       ,
  output logic             tile_rready_o      
);

`include "common_cells/registers.svh"

  localparam int unsigned NumPipeRegs = 4;
  localparam int unsigned GroupsPerEdge = (TE + CE - 1) / CE;
  localparam int unsigned SpatialBeats = GroupsPerEdge * GroupsPerEdge;
  localparam int unsigned SpatialBeatW = (SpatialBeats > 1) ? $clog2(SpatialBeats) : 1;
  localparam int unsigned ReductionW = (KMAX > 1) ? $clog2(KMAX) : 1;
  localparam int unsigned AccDepth = NrTile * SpatialBeats;
  localparam int unsigned AccAddrW = (AccDepth > 1) ? $clog2(AccDepth) : 1;
  localparam int unsigned MacSeqW = (NrTile > 1) ? $clog2(2 * NrTile) : 1;

  spatz_req_t spatz_req_mac ;
  logic       mac_req_valid ;
  logic       mac_req_ready ;
  
  spatz_req_t spatz_req_tv ;
  logic       tv_req_valid ;
  logic       tv_req_ready ;

  spatz_req_t spatz_req_vt ;
  logic       vt_req_valid ;
  logic       vt_req_ready ;

  spatz_req_t spatz_req_simple ;
  logic       simple_req_valid ;
  logic       simple_req_ready ;

  logic mac_in_ready;
  logic tv_in_ready;
  logic vt_in_ready;
  logic simple_in_ready;

  spill_register #(.T(spatz_req_t)) i_op_mac_queue (
    .clk_i  (clk_i                                                   ),
    .rst_ni (rst_ni                                                  ),
    .data_i (spatz_req_i                                             ),
    .valid_i(spatz_req_valid_i && (spatz_req_i.ex_unit == OPE) && spatz_req_i.op_ope.is_mac),
    .ready_o(mac_in_ready                                       ),
    .data_o (spatz_req_mac                                               ),
    .valid_o(mac_req_valid                                               ),
    .ready_i(mac_req_ready                                               )
  );

  spill_register #(.T(spatz_req_t)) i_op_tv_queue (
    .clk_i  (clk_i                                                   ),
    .rst_ni (rst_ni                                                  ),
    .data_i (spatz_req_i                                             ),
    .valid_i(spatz_req_valid_i && (spatz_req_i.ex_unit == OPE) && (spatz_req_i.op == VTMV_TV)),
    .ready_o(tv_in_ready                                       ),
    .data_o (spatz_req_tv                                               ),
    .valid_o(tv_req_valid                                               ),
    .ready_i(tv_req_ready                                               )
  );

  spill_register #(.T(spatz_req_t)) i_op_vt_queue (
    .clk_i  (clk_i                                                   ),
    .rst_ni (rst_ni                                                  ),
    .data_i (spatz_req_i                                             ),
    .valid_i(spatz_req_valid_i && (spatz_req_i.ex_unit == OPE) && (spatz_req_i.op == VTMV_VT)),
    .ready_o(vt_in_ready                                       ),
    .data_o (spatz_req_vt                                               ),
    .valid_o(vt_req_valid                                               ),
    .ready_i(vt_req_ready                                               )
  );

  spill_register #(.T(spatz_req_t)) i_op_simple_queue (
    .clk_i  (clk_i                                                   ),
    .rst_ni (rst_ni                                                  ),
    .data_i (spatz_req_i                                             ),
    .valid_i(spatz_req_valid_i && (spatz_req_i.ex_unit == OPE) && (spatz_req_i.op inside {VTZERO, VTDISCARD})),
    .ready_o(simple_in_ready                                       ),
    .data_o (spatz_req_simple                                               ),
    .valid_o(simple_req_valid                                               ),
    .ready_i(simple_req_ready                                               )
  );

  always_comb begin
    spatz_req_ready_o = 1'b0;

    if (spatz_req_i.ex_unit == OPE) begin
      if (spatz_req_i.op_ope.is_mac)
        spatz_req_ready_o = mac_in_ready;
      else if (spatz_req_i.op == VTMV_TV)
        spatz_req_ready_o = tv_in_ready;
      else if (spatz_req_i.op == VTMV_VT)
        spatz_req_ready_o = vt_in_ready;
      else if (spatz_req_i.op inside {VTZERO, VTDISCARD})
        spatz_req_ready_o = simple_in_ready;
    end
  end
  typedef struct packed {
    tile_id_t tile;
    logic [SpatialBeatW-1:0] beat;
    logic [ReductionW-1:0]   reduction;
    // logic [7:0]              generation;
    spatz_id_t               id;
    logic [GPRWidth-1:0]     rd;
    logic                    write_acc;
    logic                    retire;
  } mac_tag_t;

  typedef struct packed {
    tile_id_t tile;
    // logic [7:0]             generation;
    spatz_id_t              id;
    logic [GPRWidth-1:0]    rd;
    vreg_t                  vs1;
    vreg_t                  vs2;
    logic                   use_vs1;
    logic                   use_vs2;
    vew_e                   ew;
    logic                   is_alt;
    logic [MacSeqW-1:0]     seq;
    elen_t                  tk;
    logic [ReductionW-1:0]  reduction;
  } mac_ctx_t;

  // One pending context is available for each architectural destination tile.
  localparam int unsigned MacCtxs    = NrTile;
  localparam int unsigned MacCtxIdxW = (MacCtxs > 1) ? $clog2(MacCtxs) : 1;

  mac_ctx_t   [MacCtxs-1:0] mac_ctx_d,       mac_ctx_q;
  logic       [MacCtxs-1:0] mac_ctx_valid_d, mac_ctx_valid_q;
  logic [MacCtxs-1:0][SpatialBeatW-1:0] mac_ctx_beat_d, mac_ctx_beat_q;
  logic [MacCtxIdxW-1:0]    mac_alloc_idx;
  logic                     mac_alloc_valid;
  logic                     mac_alloc_tile_free;
  logic [MacCtxIdxW-1:0]    mac_sel_idx;
  logic                     mac_sel_valid;
  logic [MacCtxIdxW-1:0]    mac_exec_idx;
  logic                     mac_exec_valid;
  logic                     mac_exec_from_req;
  logic                     mac_ctx_final_fire;
  logic [MacSeqW-1:0]       mac_seq_head_d, mac_seq_head_q;
  logic [MacSeqW-1:0]       mac_seq_tail_d, mac_seq_tail_q;

  mac_ctx_t mac_exec_ctx;

  logic [SpatialBeatW-1:0] mac_beat_idx;
  localparam int unsigned GroupIdxW = (GroupsPerEdge > 1) ? $clog2(GroupsPerEdge) : 1;
  logic [GroupIdxW-1:0] mac_group_row, mac_group_col;

  assign mac_group_row = (GroupsPerEdge == 1) ? '0 : mac_beat_idx[SpatialBeatW-1:GroupIdxW];
  assign mac_group_col = (GroupsPerEdge == 1) ? '0 : mac_beat_idx[GroupIdxW-1:0];

  logic mac_operand_word_ready;
  assign mac_operand_word_ready = (!mac_exec_ctx.use_vs2 || vrf_rvalid_i[0]) &&
                                  (!mac_exec_ctx.use_vs1 || vrf_rvalid_i[1]);

  logic [CE-1:0][CE-1:0] fma_ready;

  logic mac_result_valid;
  logic mac_result_fire;
  logic mac_result_forward;
  logic fma_result_ready;
  logic mac_result_continue_ready;
  logic resident_drain_commit;
  logic [NrTile-1:0][SpatialBeats-1:0] tile_inflight_d, tile_inflight_q;
  logic [$clog2(NumPipeRegs+1)-1:0] mac_inflight_cnt_d, mac_inflight_cnt_q;

  typedef enum logic [3:0] {
    DrainNone,
    DrainTileSwitch,
    DrainVtseSameTile,
    DrainVtleSameTile,
    DrainVtmvSameTile,
    DrainVtzeroSameTile,
    DrainDiscard,
    DrainAuto
  } resident_drain_e;

  logic            resident_valid_d, resident_valid_q;
  tile_id_t        resident_tile_d, resident_tile_q;
  logic            resident_drain_valid_d, resident_drain_valid_q;
  resident_drain_e resident_drain_reason_d, resident_drain_reason_q;
  logic            resident_start_fire;
  logic            resident_hit_accept;
  logic            resident_switch_req;
  logic            resident_switch_wait;
  logic            resident_switch_fire;


  logic [MacCtxs-1:0] mac_ctx_tile_hazard;
  logic [MacCtxs-1:0] mac_ctx_sched_ready;
  logic [MacCtxs-1:0] mac_ctx_result_match;
  logic [MacCtxs-1:0] mac_ctx_result_switch;

  assign mac_alloc_idx = mac_req_valid ? MacCtxIdxW'(spatz_req_mac.op_ope.tss.tile_id) : '0;
  assign mac_alloc_valid = !mac_ctx_valid_q[mac_alloc_idx] ||
      (mac_ctx_final_fire && (mac_exec_idx == mac_alloc_idx));
  assign mac_alloc_tile_free = mac_alloc_valid;

  always_comb begin : mac_context_sched
    tile_id_t ctx_tile;

    mac_ctx_tile_hazard = '0;
    mac_ctx_sched_ready = '0;
    mac_ctx_result_match = '0;
    mac_ctx_result_switch = '0;
    mac_sel_valid       = 1'b0;
    mac_sel_idx         = '0;
    ctx_tile            = '0;

    for (int unsigned ctx = 0; ctx < MacCtxs; ctx++) begin
      ctx_tile = mac_ctx_q[ctx].tile;
      mac_ctx_result_match[ctx] = mac_result_valid &&
          !mac_tag_q[NumPipeRegs-1].write_acc &&
          (mac_tag_q[NumPipeRegs-1].tile == ctx_tile) &&
          (mac_tag_q[NumPipeRegs-1].beat == mac_ctx_beat_q[ctx]);
      mac_ctx_result_switch[ctx] = mac_result_valid &&
          !mac_tag_q[NumPipeRegs-1].write_acc &&
          resident_valid_q && !resident_drain_valid_q &&
          (mac_tag_q[NumPipeRegs-1].tile == resident_tile_q) &&
          (ctx_tile != resident_tile_q);
      mac_ctx_tile_hazard[ctx] =
          tile_inflight_q[ctx_tile][mac_ctx_beat_q[ctx]] && !mac_ctx_result_match[ctx];
      mac_ctx_sched_ready[ctx] =
          mac_ctx_valid_q[ctx] &&
          (mac_ctx_q[ctx].seq == mac_seq_head_q) &&
          !mac_ctx_tile_hazard[ctx] &&
          ((mac_inflight_cnt_q < NumPipeRegs) ||
           mac_ctx_result_match[ctx] ||
           mac_ctx_result_switch[ctx] ||
           mac_result_fire);
    end

    // A resident partial sum must be consumed before another context can run.
    for (int unsigned ctx = 0; ctx < MacCtxs; ctx++) begin
      if (!mac_sel_valid && mac_ctx_sched_ready[ctx] && mac_ctx_result_match[ctx]) begin
        mac_sel_valid = 1'b1;
        mac_sel_idx   = MacCtxIdxW'(ctx);
      end
    end

    for (int unsigned tile_ord = 0; tile_ord < NrTile; tile_ord++) begin
      if (!mac_sel_valid && mac_ctx_sched_ready[tile_ord]) begin
        mac_sel_valid = 1'b1;
        mac_sel_idx   = MacCtxIdxW'(tile_ord);
      end
    end
  end : mac_context_sched

  // An empty MAC queue may launch phase zero directly from the accepted
  // request. If either VRF operand is unavailable, the request remains in its
  // context and follows the registered path on the next cycle.
  assign mac_exec_from_req = !mac_sel_valid && mac_req_valid &&
      mac_alloc_valid && mac_alloc_tile_free && !(|mac_ctx_valid_q)
      && mac_req_ready
      ;
  assign mac_exec_valid = mac_sel_valid || mac_exec_from_req;
  assign mac_exec_idx   = mac_exec_from_req ? mac_alloc_idx : mac_sel_idx;
  assign mac_beat_idx   = mac_exec_from_req ? '0 : mac_ctx_beat_q[mac_exec_idx];

  always_comb begin : mac_exec_context
    mac_exec_ctx = mac_ctx_q[mac_exec_idx];
    if (mac_exec_from_req) begin
      mac_exec_ctx = '{
        tile     : spatz_req_mac.op_ope.tss.tile_id,
        id       : spatz_req_mac.id,
        rd       : spatz_req_mac.rd[GPRWidth-1:0],
        vs1      : spatz_req_mac.vs1,
        vs2      : spatz_req_mac.vs2,
        use_vs1  : spatz_req_mac.use_vs1,
        use_vs2  : spatz_req_mac.use_vs2,
        ew       : spatz_req_mac.vtype.vsew,
        is_alt   : (spatz_req_mac.op == VTFMM_ALT),
        seq      : mac_seq_tail_q,
        tk       : spatz_req_mac.op_ope.tk,
        reduction: '0
      };
    end
  end : mac_exec_context

  logic mac_fire;
  logic mac_spatial_more;
  logic mac_reduction_more;
  logic mac_operation_more;
  logic mac_commit_ready;

  assign mac_fire = mac_exec_valid && &fma_ready && mac_operand_word_ready &&
      (mac_operation_more || mac_commit_ready);
  assign mac_spatial_more = ((int'(mac_beat_idx) + 1) < SpatialBeats);
  assign mac_reduction_more = ((int'(mac_exec_ctx.reduction) + 1) < int'(mac_exec_ctx.tk));
  assign mac_operation_more = mac_spatial_more || mac_reduction_more;
  assign mac_ctx_final_fire = mac_fire && !mac_operation_more;

  always_comb begin : mac_context_update
    mac_ctx_d       = mac_ctx_q;
    mac_ctx_valid_d = mac_ctx_valid_q;
    mac_ctx_beat_d  = mac_ctx_beat_q;

    if (mac_fire) begin
      if (mac_spatial_more) begin
        mac_ctx_beat_d[mac_exec_idx] = mac_beat_idx + 1'b1;
      end else if (mac_reduction_more) begin
        mac_ctx_beat_d[mac_exec_idx]       = '0;
        mac_ctx_d[mac_exec_idx].reduction = mac_exec_ctx.reduction + 1'b1;
      end else begin
        mac_ctx_valid_d[mac_exec_idx] = 1'b0;
        mac_ctx_beat_d[mac_exec_idx]  = '0;
      end
    end


    if (mac_req_valid && mac_req_ready) begin
      mac_ctx_d[mac_alloc_idx]       = '{
        tile   : spatz_req_mac.op_ope.tss.tile_id,
        id     : spatz_req_mac.id,
        rd     : spatz_req_mac.rd[GPRWidth-1:0],
        vs1    : spatz_req_mac.vs1,
        vs2    : spatz_req_mac.vs2,
        use_vs1: spatz_req_mac.use_vs1,
        use_vs2: spatz_req_mac.use_vs2,
        ew     : spatz_req_mac.vtype.vsew,
        is_alt : (spatz_req_mac.op == VTFMM_ALT),
        seq    :  mac_seq_tail_q,
        tk       : spatz_req_mac.op_ope.tk,
        reduction: '0
      };
      mac_ctx_valid_d[mac_alloc_idx] = 1'b1;
      mac_ctx_beat_d[mac_alloc_idx]  = '0;
      if (mac_exec_from_req && mac_fire) begin
        mac_ctx_beat_d[mac_alloc_idx] = mac_beat_idx + 1'b1;
      end
    end
  end : mac_context_update

  always_comb begin : mac_sequence_update
    mac_seq_head_d = mac_seq_head_q;
    mac_seq_tail_d = mac_seq_tail_q;

    if (mac_ctx_final_fire)
      mac_seq_head_d = mac_seq_head_q + 1'b1;
    if (mac_req_valid && mac_req_ready)
      mac_seq_tail_d = mac_seq_tail_q + 1'b1;
  end : mac_sequence_update

  always_comb begin : mac_inflight_update
    tile_inflight_d    = tile_inflight_q;
    mac_inflight_cnt_d = mac_inflight_cnt_q;

    // Each tile has four independent quadrant accumulators, so all four FMA
    // stages may hold work for the same architectural tile.
    if (mac_result_fire) begin
      mac_inflight_cnt_d = mac_inflight_cnt_d - 1;
      tile_inflight_d[mac_tag_q[NumPipeRegs-1].tile][mac_tag_q[NumPipeRegs-1].beat] = 1'b0;
    end

    if (mac_fire) begin
      tile_inflight_d[mac_exec_ctx.tile][mac_beat_idx] = 1'b1;
      mac_inflight_cnt_d = mac_inflight_cnt_d + 1;
    end
  end : mac_inflight_update

  mac_tag_t [NumPipeRegs-1:0] mac_tag_d, mac_tag_q;
  logic                    mac_tag_shift_en;
  mac_tag_t                mac_tag_new;
  logic                    mac_resident_drain;
  logic                    mac_tag_drain;
  logic                    mac_result_write_acc;

  logic mac_done;
  assign mac_done = fma_ready[0][0];
  assign mac_tag_shift_en = (mac_fire || (mac_inflight_cnt_q != 0)) && mac_done;
  assign mac_tag_new = mac_fire ? '{tile: mac_exec_ctx.tile, beat: mac_beat_idx,
                                    reduction: mac_exec_ctx.reduction,
                                    id: mac_exec_ctx.id, rd: mac_exec_ctx.rd,
                                    write_acc: 1'b0, retire: 1'b0} : '0;

  always_comb begin : mac_tag_shift
    mac_tag_d = mac_tag_q;
    if (resident_switch_fire) begin
      for (int unsigned stage = 0; stage < NumPipeRegs; stage++)
        mac_tag_d[stage].write_acc = 1'b1;
    end
    if (mac_tag_shift_en) begin
      mac_tag_d = {mac_tag_d[NumPipeRegs-2:0], mac_tag_new};
    end
  end : mac_tag_shift

  logic     mac_commit_valid;
  vfu_rsp_t mac_commit_rsp, mac_done_rsp;
  logic     mac_done_valid, mac_done_ready;

  assign mac_commit_valid = mac_ctx_final_fire;
  assign mac_resident_drain = resident_drain_valid_q;
  assign mac_tag_drain = mac_tag_q[NumPipeRegs-1].write_acc ||
      (resident_switch_req && mac_result_valid &&
       (mac_tag_q[NumPipeRegs-1].tile != mac_exec_ctx.tile));
  assign mac_result_write_acc = mac_tag_drain ||
      (mac_resident_drain && resident_drain_commit);
  assign resident_drain_commit = !(resident_drain_reason_q inside {
      DrainVtzeroSameTile, DrainDiscard});

  assign mac_result_continue_ready = mac_exec_valid && mac_operand_word_ready &&
      (mac_tag_q[NumPipeRegs-1].tile == mac_exec_ctx.tile) &&
      (mac_tag_q[NumPipeRegs-1].beat == mac_beat_idx);
  assign fma_result_ready = (mac_resident_drain || mac_tag_drain) ? 1'b1 :
      (mac_tag_q[NumPipeRegs-1].write_acc ? mac_commit_ready : mac_result_continue_ready);
  assign mac_result_fire  = mac_result_valid && fma_result_ready;
  assign mac_result_forward = mac_fire && mac_result_fire &&
      !mac_resident_drain && !mac_tag_drain &&
      (mac_tag_q[NumPipeRegs-1].tile == mac_exec_ctx.tile) &&
      (mac_tag_q[NumPipeRegs-1].beat == mac_beat_idx) &&
      !mac_tag_q[NumPipeRegs-1].write_acc;

  always_comb begin : mac_commit_rsp_proc
    mac_commit_rsp    = '0;
    mac_commit_rsp.id = mac_exec_ctx.id;
    mac_commit_rsp.rd = mac_exec_ctx.rd;
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
  tss_t        vt_tss_q                       ;
  logic        vt_busy_d    , vt_busy_q       ;
  logic        vt_commit_valid, vt_commit_ready;
  vfu_rsp_t    vt_commit_rsp                  ;
  vfu_rsp_t    vt_done_rsp                    ;
  logic        vt_done_valid , vt_done_ready  ;

  logic mac_pipe_idle;
  assign mac_pipe_idle = !(|mac_ctx_valid_q) && (mac_inflight_cnt_q == 0);

  // VT/TV/VLSU tile access requires the MAC pipe fully drained
  always_comb begin : vt_handler
    vt_busy_d        = vt_busy_q;
    vt_commit_valid  = 1'b0;
    if (vt_req_valid && vt_req_ready)
      vt_busy_d = 1'b1;
    if (vt_busy_q) begin
      vt_commit_valid = !vt_tss_q.tile_valid || !vt_req_q.use_vd || vrf_wvalid_i;
      if (vt_commit_valid && vt_commit_ready)
        vt_busy_d = 1'b0;
    end
  end : vt_handler

  always_comb begin : vt_commit_rsp_proc
    vt_commit_rsp    = '0;
    vt_commit_rsp.id = vt_req_q.id;
    vt_commit_rsp.rd = vt_req_q.rd[GPRWidth-1:0];
  end : vt_commit_rsp_proc

  spill_register #(.T(vfu_rsp_t)) i_vt_commit (
    .clk_i  (clk_i            ),
    .rst_ni (rst_ni           ),
    .data_i (vt_commit_rsp    ),
    .valid_i(vt_commit_valid  ),
    .ready_o(vt_commit_ready  ),
    .data_o (vt_done_rsp      ),
    .valid_o(vt_done_valid    ),
    .ready_i(vt_done_ready    )
  );

  spatz_req_t  tv_req_q                       ;
  tss_t        tv_tss_q                       ;
  logic        tv_busy_d    , tv_busy_q       ;
  logic        tv_commit_valid, tv_commit_ready;
  vfu_rsp_t    tv_commit_rsp                  ;
  vfu_rsp_t    tv_done_rsp                    ;
  logic        tv_done_valid , tv_done_ready  ;

  vrf_data_t tv_data_q;
  logic      tv_data_latched_q;
  vrf_data_t tv_vrf_data;
  logic      tv_vrf_avail;

  assign tv_vrf_avail = tv_data_latched_q || (tv_busy_q && vrf_rvalid_i[0]);
  assign tv_vrf_data  = tv_data_latched_q ? tv_data_q : vrf_rdata_i[0];

  logic tv_acc_wen;
  assign tv_acc_wen = tv_busy_q && tv_tss_q.tile_valid && tv_vrf_avail &&
                      !mac_result_valid && !tile_wvalid_i;

  always_comb begin : tv_handler
    tv_busy_d        = tv_busy_q;
    tv_commit_valid  = 1'b0;
    if (tv_req_valid && tv_req_ready)
      tv_busy_d = 1'b1;
    if (tv_busy_q) begin
      tv_commit_valid = !tv_tss_q.tile_valid || !tv_req_q.use_vs2 || tv_acc_wen;
      if (tv_commit_valid && tv_commit_ready)
        tv_busy_d = 1'b0;
    end
  end : tv_handler

  always_comb begin : tv_commit_rsp_proc
    tv_commit_rsp    = '0;
    tv_commit_rsp.id = tv_req_q.id;
    tv_commit_rsp.rd = tv_req_q.rd[GPRWidth-1:0];
  end : tv_commit_rsp_proc

  spill_register #(.T(vfu_rsp_t)) i_tv_commit (
    .clk_i  (clk_i            ),
    .rst_ni (rst_ni           ),
    .data_i (tv_commit_rsp    ),
    .valid_i(tv_commit_valid  ),
    .ready_o(tv_commit_ready  ),
    .data_o (tv_done_rsp      ),
    .valid_o(tv_done_valid    ),
    .ready_i(tv_done_ready    )
  );

  vfu_rsp_t simple_commit_rsp, simple_done_rsp   ;
  logic     simple_done_valid, simple_done_ready ;
  logic     simple_commit_ready                  ;
  logic     simple_acc_ready                     ;
  logic [NrTile-1:0] vtzero_tile_busy;

  always_comb begin : vtzero_tile_busy_proc
    vtzero_tile_busy = '0;

    for (int unsigned ctx = 0; ctx < MacCtxs; ctx++) begin
      if (mac_ctx_valid_q[ctx])
        vtzero_tile_busy[mac_ctx_q[ctx].tile] = 1'b1;
    end
    for (int unsigned tile = 0; tile < NrTile; tile++) begin
      if (|tile_inflight_q[tile])
        vtzero_tile_busy[tile] = 1'b1;
    end

    if (vt_busy_q && vt_tss_q.tile_valid)
      vtzero_tile_busy[vt_tss_q.tile_id] = 1'b1;
    if (tv_busy_q && tv_tss_q.tile_valid)
      vtzero_tile_busy[tv_tss_q.tile_id] = 1'b1;
    if (tile_rvalid_i)
      vtzero_tile_busy[tile_id_t'(tile_r_req_i.idx / NumAccPerTile)] = 1'b1;
    if (tile_wvalid_i)
      vtzero_tile_busy[tile_id_t'(tile_w_req_i.idx / NumAccPerTile)] = 1'b1;
  end : vtzero_tile_busy_proc

  always_comb begin : simple_acc_ready_proc
    simple_acc_ready = 1'b0;
    if (spatz_req_simple.op == VTZERO)
      simple_acc_ready = !resident_drain_valid_q &&
          (!resident_valid_q ||
           (spatz_req_simple.op_ope.tss.tile_id != resident_tile_q));
    else if (spatz_req_simple.op == VTDISCARD)
      simple_acc_ready = !resident_valid_q && !resident_drain_valid_q &&
                         mac_pipe_idle && !vt_busy_q && !tv_busy_q &&
                         !tile_rvalid_i && !tile_wvalid_i;
  end : simple_acc_ready_proc

  always_comb begin : req_ready_proc
    
    mac_req_ready    = 1'b0;
    vt_req_ready     = 1'b0;
    tv_req_ready     = 1'b0;
    simple_req_ready = 1'b0;
    
    /*
    * Preserve OPE command ordering.
    * Simple commands such as VTZERO/VTDISCARD must complete
    * before a younger MAC may start.
    */
    if (simple_req_valid)
      simple_req_ready = simple_commit_ready && simple_acc_ready;
    else if (vt_req_valid)
      vt_req_ready = !vt_busy_q && !resident_drain_valid_q &&
                  (!resident_valid_q || !spatz_req_vt.op_ope.tss.tile_valid ||
                   (spatz_req_vt.op_ope.tss.tile_id != resident_tile_q));
    else if (tv_req_valid)
      tv_req_ready = !tv_busy_q && !resident_drain_valid_q &&
                  (!resident_valid_q || !spatz_req_tv.op_ope.tss.tile_valid ||
                   (spatz_req_tv.op_ope.tss.tile_id != resident_tile_q));
    else if (mac_req_valid)
      mac_req_ready = (spatz_req_mac.op_ope.tk == 1) && !resident_drain_valid_q &&
                  mac_alloc_valid && mac_alloc_tile_free;
  end : req_ready_proc

  assign resident_start_fire = mac_fire && !resident_valid_q &&
      (mac_beat_idx == '0);
  assign resident_hit_accept = mac_req_valid && mac_req_ready && resident_valid_q &&
      (spatz_req_mac.op_ope.tss.tile_id == resident_tile_q);
  assign resident_switch_req = mac_exec_valid && resident_valid_q &&
      !resident_drain_valid_q && (mac_exec_ctx.tile != resident_tile_q);
  assign resident_switch_wait = resident_switch_req;
  assign resident_switch_fire = resident_switch_req && mac_fire;

  always_comb begin : resident_state_update
    logic start_drain;
    resident_drain_e start_reason;
    tile_id_t tile_read_id;
    tile_id_t tile_write_id;

    resident_valid_d       = resident_valid_q;
    resident_tile_d        = resident_tile_q;
    resident_drain_valid_d = resident_drain_valid_q;
    resident_drain_reason_d = resident_drain_reason_q;

    start_drain = 1'b0;
    start_reason = DrainNone;
    tile_read_id = tile_id_t'(tile_r_req_i.idx / NumAccPerTile);
    tile_write_id = tile_id_t'(tile_w_req_i.idx / NumAccPerTile);

    if (resident_valid_q && !resident_drain_valid_q &&
        !(|mac_ctx_valid_q) && (mac_inflight_cnt_q != 0)) begin
      if (simple_req_valid && (spatz_req_simple.op == VTDISCARD)) begin
        start_drain = 1'b1;
        start_reason = DrainDiscard;
      end else if (simple_req_valid && (spatz_req_simple.op == VTZERO) &&
                   (spatz_req_simple.op_ope.tss.tile_id == resident_tile_q)) begin
        start_drain = 1'b1;
        start_reason = DrainVtzeroSameTile;
      end else if (tile_rvalid_i && (tile_read_id == resident_tile_q)) begin
        start_drain = 1'b1;
        start_reason = DrainVtseSameTile;
      end else if (tile_wvalid_i && (tile_write_id == resident_tile_q)) begin
        start_drain = 1'b1;
        start_reason = DrainVtleSameTile;
      end else if (vt_req_valid && spatz_req_vt.op_ope.tss.tile_valid &&
                   (spatz_req_vt.op_ope.tss.tile_id == resident_tile_q)) begin
        start_drain = 1'b1;
        start_reason = DrainVtmvSameTile;
      end else if (tv_req_valid && spatz_req_tv.op_ope.tss.tile_valid &&
                   (spatz_req_tv.op_ope.tss.tile_id == resident_tile_q)) begin
        start_drain = 1'b1;
        start_reason = DrainVtmvSameTile;
      end
    end

    if (start_drain) begin
      resident_drain_valid_d = 1'b1;
      resident_drain_reason_d = start_reason;
    end

    if (resident_start_fire) begin
      resident_valid_d = 1'b1;
      resident_tile_d = mac_exec_ctx.tile;
    end
    if (resident_switch_fire) begin
      resident_valid_d = 1'b1;
      resident_tile_d = mac_exec_ctx.tile;
    end

    if (mac_result_fire && resident_drain_valid_q &&
        (mac_inflight_cnt_q == 1)) begin
      resident_valid_d = 1'b0;
      resident_drain_valid_d = 1'b0;
      resident_drain_reason_d = DrainNone;
    end
  end : resident_state_update

  logic [CE-1:0][CE-1:0][TEW-1:0] fma_addend;
  logic [CE-1:0][CE-1:0][TEW-1:0] fma_result;
  logic [CE-1:0][CE-1:0] fma_result_valid;
  logic [CE-1:0][CE-1:0][AccDepth-1:0][TEW-1:0] acc_mem_rdata;
  logic [CE-1:0][CE-1:0][SpatialBeats-1:0][TEW-1:0] acc_wdata;
  logic [CE-1:0][CE-1:0] acc_wen;
  logic [AccAddrW-1:0] acc_waddr;
  logic acc_ext_ld, acc_flush;
  logic [NrTile-1:0][SpatialBeats-1:0] acc_zero_d, acc_zero_q;
  logic [NrTile-1:0][TE-1:0][TE-1:0][TEW-1:0] acc_rdata;
  logic acc_tile_read_ready, acc_tile_write_ready;
  logic [NrTile-1:0] mac_acc_rd_req, mac_acc_wr_req, vt_acc_rd_req, tv_acc_wr_req, acc_zero_wr_req;

  assign mac_result_valid = fma_result_valid[0][0];

  always_comb begin : simple_commit_rsp_proc
    simple_commit_rsp    = '0;
    simple_commit_rsp.id = spatz_req_simple.id;
    simple_commit_rsp.rd = spatz_req_simple.rd[GPRWidth-1:0];
  end : simple_commit_rsp_proc

  spill_register #(.T(vfu_rsp_t)) i_simple_commit (
    .clk_i  (clk_i                                    ),
    .rst_ni (rst_ni                                   ),
    .data_i (simple_commit_rsp                        ),
    .valid_i(simple_req_valid && simple_acc_ready     ),
    .ready_o(simple_commit_ready                      ),
    .data_o (simple_done_rsp                          ),
    .valid_o(simple_done_valid                        ),
    .ready_i(simple_done_ready                        )
  );

  always_comb begin : acc_req_proc
    mac_acc_rd_req  = '0;
    mac_acc_wr_req  = '0;
    vt_acc_rd_req   = '0;
    tv_acc_wr_req   = '0;
    acc_zero_wr_req = '0;

    if (mac_exec_valid && (mac_exec_ctx.reduction == '0)
        && (!resident_valid_q || resident_switch_wait)
       )
      mac_acc_rd_req[mac_exec_ctx.tile] = 1'b1;
    if (mac_result_valid && mac_result_write_acc)
      mac_acc_wr_req[mac_tag_q[NumPipeRegs-1].tile] = 1'b1;
    if (vt_busy_q && vt_tss_q.tile_valid)
      vt_acc_rd_req[vt_tss_q.tile_id] = 1'b1;
    if (tv_busy_q && tv_tss_q.tile_valid && tv_vrf_avail)
      tv_acc_wr_req[tv_tss_q.tile_id] = 1'b1;
    if (simple_req_valid && simple_req_ready && (spatz_req_simple.op == VTZERO))
      acc_zero_wr_req[spatz_req_simple.op_ope.tss.tile_id] = 1'b1;
  end

  assign acc_tile_read_ready =
      !mac_acc_rd_req[tile_id_t'(tile_r_req_i.idx / NumAccPerTile)] &&
      !mac_acc_wr_req[tile_id_t'(tile_r_req_i.idx / NumAccPerTile)] &&
      !vt_acc_rd_req[tile_id_t'(tile_r_req_i.idx / NumAccPerTile)] &&
      !tv_acc_wr_req[tile_id_t'(tile_r_req_i.idx / NumAccPerTile)] &&
      !acc_zero_wr_req[tile_id_t'(tile_r_req_i.idx / NumAccPerTile)];
  assign acc_tile_write_ready =
      !mac_acc_rd_req[tile_id_t'(tile_w_req_i.idx / NumAccPerTile)] &&
      !(|mac_acc_wr_req) &&
      !(|tv_acc_wr_req) &&
      !acc_zero_wr_req[tile_id_t'(tile_w_req_i.idx / NumAccPerTile)];
  assign tile_rready_o = acc_tile_read_ready && !resident_drain_valid_q &&
      (!resident_valid_q ||
       (tile_id_t'(tile_r_req_i.idx / NumAccPerTile) != resident_tile_q));
  assign tile_wready_o = acc_tile_write_ready && !tile_rvalid_i
      && !resident_drain_valid_q &&
      (!resident_valid_q ||
       (tile_id_t'(tile_w_req_i.idx / NumAccPerTile) != resident_tile_q))
      ;

  for (genvar tile = 0; tile < NrTile; tile++) begin : gen_tile_view
    for (genvar row = 0; row < TE; row++) begin : gen_tile_view_row
      for (genvar col = 0; col < TE; col++) begin : gen_tile_view_col
        assign acc_rdata[tile][row][col] =
            acc_zero_q[tile][(row / CE) * GroupsPerEdge + (col / CE)] ? '0 :
                acc_mem_rdata[row % CE][col % CE]
                             [tile * SpatialBeats + (row / CE) * GroupsPerEdge + (col / CE)];
      end : gen_tile_view_col
    end : gen_tile_view_row
  end : gen_tile_view

  always_comb begin : acc_access_proc
    logic [$clog2(TE)-1:0] idx;
    logic [$clog2(TE)-1:0] row_i;
    logic [$clog2(TE)-1:0] col_i;

    int unsigned active_len;
    int unsigned vstart_i;

    acc_wdata = '0;
    acc_wen   = '0;
    acc_waddr = '0;
    acc_ext_ld = 1'b0;
    acc_flush = 1'b0;
    tile_rdata_o = '0;

    idx        = '0;
    row_i      = '0;
    col_i      = '0;
    active_len = 0;
    vstart_i   = 0;

    if (simple_req_valid && simple_req_ready && (spatz_req_simple.op == VTDISCARD)) begin
      acc_flush = 1'b1;
    end

    if (tv_acc_wen) begin
      idx        = tv_tss_q.index;
      active_len = (tv_req_q.vl < TE) ? int'(tv_req_q.vl) : TE;
      vstart_i   = int'(tv_req_q.vstart);

      acc_ext_ld = 1'b1;
      acc_waddr = AccAddrW'(int'(tv_tss_q.tile_id) * SpatialBeats);
      for (int row = 0; row < CE; row++) begin
        for (int col = 0; col < CE; col++) begin
          for (int beat = 0; beat < SpatialBeats; beat++) begin
            acc_wdata[row][col][beat] = acc_zero_q[tv_tss_q.tile_id][beat] ? '0 :
                acc_mem_rdata[row][col][int'(tv_tss_q.tile_id) * SpatialBeats + beat];
          end
          if (|acc_zero_q[tv_tss_q.tile_id])
            acc_wen[row][col] = 1'b1;
        end
      end

      for (int i = 0; i < TE; i++) begin
        if ((i >= vstart_i) && (i < active_len)) begin
          if (tv_tss_q.is_row) begin
            row_i = idx;
            col_i = i[$clog2(TE)-1:0];
          end else begin
            row_i = i[$clog2(TE)-1:0];
            col_i = idx;
          end

          acc_wen[int'(row_i) % CE][int'(col_i) % CE] = 1'b1;
          acc_wdata[int'(row_i) % CE][int'(col_i) % CE]
                   [(int'(row_i) / CE) * GroupsPerEdge + (int'(col_i) / CE)] =
              tv_vrf_data[i*TEW +: TEW];
        end
      end
    end

    if (tile_wvalid_i && tile_wready_o) begin
      acc_ext_ld = 1'b1;
      acc_waddr = AccAddrW'(
          int'(tile_id_t'(tile_w_req_i.idx / NumAccPerTile)) * SpatialBeats);
      for (int row = 0; row < CE; row++) begin
        for (int col = 0; col < CE; col++) begin
          for (int beat = 0; beat < SpatialBeats; beat++) begin
            acc_wdata[row][col][beat] =
                acc_zero_q[tile_id_t'(tile_w_req_i.idx / NumAccPerTile)][beat] ? '0 :
                acc_mem_rdata[row][col]
                    [int'(tile_id_t'(tile_w_req_i.idx / NumAccPerTile)) * SpatialBeats + beat];
          end
          if (|acc_zero_q[tile_id_t'(tile_w_req_i.idx / NumAccPerTile)])
            acc_wen[row][col] = 1'b1;
        end
      end
      for (int i = 0; i < TE; i++) begin
        col_i = i[$clog2(TE)-1:0];
        acc_wen[int'(tile_w_req_i.row) % CE][int'(col_i) % CE] = 1'b1;
        acc_wdata[int'(tile_w_req_i.row) % CE][int'(col_i) % CE]
                 [(int'(tile_w_req_i.row) / CE) * GroupsPerEdge + (int'(col_i) / CE)] =
            tile_w_req_i.data[i*TEW +: TEW];
      end
    end

    if (mac_result_fire && mac_result_write_acc) begin
      acc_waddr = AccAddrW'(int'(mac_tag_q[NumPipeRegs-1].tile) * SpatialBeats +
                            int'(mac_tag_q[NumPipeRegs-1].beat));
      for (int row = 0; row < CE; row++) begin
        for (int col = 0; col < CE; col++) begin
          acc_wen[row][col] = 1'b1;
          acc_wdata[row][col][0] = fma_result[row][col];
        end
      end
    end

    if (tile_rvalid_i) begin
      for (int i = 0; i < TE; i++) begin
        col_i = i[$clog2(TE)-1:0];
        tile_rdata_o[i*TEW +: TEW] =
            acc_rdata[tile_id_t'(tile_r_req_i.idx / NumAccPerTile)][tile_r_req_i.row][col_i];
      end
    end
  end

  always_comb begin : acc_zero_update
    acc_zero_d = acc_zero_q;

    if (simple_req_valid && simple_req_ready && (spatz_req_simple.op == VTDISCARD))
      acc_zero_d = '0;
    else begin
      if (simple_req_valid && simple_req_ready && (spatz_req_simple.op == VTZERO))
        acc_zero_d[spatz_req_simple.op_ope.tss.tile_id] = '1;
      if (mac_result_fire && mac_result_write_acc)
        acc_zero_d[mac_tag_q[NumPipeRegs-1].tile][mac_tag_q[NumPipeRegs-1].beat] = 1'b0;
      if (tv_acc_wen)
        acc_zero_d[tv_tss_q.tile_id] = '0;
      if (tile_wvalid_i && tile_wready_o)
        acc_zero_d[tile_id_t'(tile_w_req_i.idx / NumAccPerTile)] = '0;
    end
  end : acc_zero_update

  typedef enum logic [1:0] {
    RSP_MAC,
    RSP_VT,
    RSP_TV,
    RSP_SIMPLE
  } ope_rsp_sel_e;

  vfu_rsp_t [3:0] arb_inp_data ;
  logic     [3:0] arb_inp_valid;
  logic     [3:0] arb_inp_ready;

  assign arb_inp_data[RSP_MAC]    = mac_done_rsp;
  assign arb_inp_data[RSP_VT]     = vt_done_rsp;
  assign arb_inp_data[RSP_TV]     = tv_done_rsp;
  assign arb_inp_data[RSP_SIMPLE] = simple_done_rsp;

  assign arb_inp_valid[RSP_MAC]    = mac_done_valid;
  assign arb_inp_valid[RSP_VT]     = vt_done_valid;
  assign arb_inp_valid[RSP_TV]     = tv_done_valid;
  assign arb_inp_valid[RSP_SIMPLE] = simple_done_valid;

  assign mac_done_ready    = arb_inp_ready[RSP_MAC];
  assign vt_done_ready     = arb_inp_ready[RSP_VT];
  assign tv_done_ready     = arb_inp_ready[RSP_TV];
  assign simple_done_ready = arb_inp_ready[RSP_SIMPLE];

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
    vrf_id_o[0] = mac_exec_valid ? mac_exec_ctx.id : tv_req_q.id;
    vrf_id_o[1] = mac_exec_ctx.id;
    vrf_id_o[2] = vt_req_q.id;
  end

  always_comb begin : vrf_re_proc
    vrf_re_o    = '0;
    vrf_raddr_o = '0;

    if (mac_exec_valid) begin
      int unsigned operand_bits;
      int unsigned reduction_stride;

      operand_bits = 8 << int'(mac_exec_ctx.ew);
      unique case (mac_exec_ctx.ew)
        EW_8:    reduction_stride = 2;
        EW_16:   reduction_stride = 4;
        EW_32:   reduction_stride = 1;
        default: reduction_stride = 8;
      endcase
      vrf_re_o[0] = mac_exec_ctx.use_vs2;
      vrf_re_o[1] = mac_exec_ctx.use_vs1;
      vrf_raddr_o[0] =
          (vrf_addr_t'(int'(mac_exec_ctx.vs2) +
                       int'(mac_exec_ctx.reduction) * reduction_stride) <<
           $clog2(NrWordsPerVector)) +
          vrf_addr_t'((int'(mac_group_row) * CE * operand_bits) / VRFWordWidth);
      vrf_raddr_o[1] =
          (vrf_addr_t'(int'(mac_exec_ctx.vs1) +
                       int'(mac_exec_ctx.reduction) * reduction_stride) <<
           $clog2(NrWordsPerVector)) +
          vrf_addr_t'((int'(mac_group_col) * CE * operand_bits) / VRFWordWidth);
    end else if (tv_busy_q && tv_tss_q.tile_valid && !tv_data_latched_q) begin
      vrf_re_o[0]    = tv_req_q.use_vs2;
      vrf_raddr_o[0] = vrf_addr_t'(tv_req_q.vs2) << $clog2(NrWordsPerVector);
    end
  end

  always_comb begin : vrf_wr_proc
    logic [$clog2(TE)-1:0] idx;
    logic [$clog2(TE)-1:0] row_i;
    logic [$clog2(TE)-1:0] col_i;
    int unsigned active_len;
    int unsigned vstart_i;

    vrf_waddr_o = '0;
    vrf_we_o    = 1'b0;
    vrf_wbe_o   = '0;
    vrf_wdata_o = '0;

    idx   = '0;
    row_i = '0;
    col_i = '0;

    active_len = (vt_req_q.vl < TE) ? int'(vt_req_q.vl) : TE;
    vstart_i   = int'(vt_req_q.vstart);

    if (vt_busy_q && vt_tss_q.tile_valid) begin
      vrf_waddr_o = vrf_addr_t'(vt_req_q.vd) << $clog2(NrWordsPerVector);
      vrf_we_o    = vt_req_q.use_vd;

      idx = vt_tss_q.index;

      for (int i = 0; i < TE; i++) begin
        if ((i >= vstart_i) && (i < active_len)) begin
          if (vt_tss_q.is_row) begin
            row_i = idx;
            col_i = i[$clog2(TE)-1:0];
          end else begin
            row_i = i[$clog2(TE)-1:0];
            col_i = idx;
          end

          vrf_wdata_o[i*TEW +: TEW] = acc_rdata[vt_tss_q.tile_id][row_i][col_i];
          vrf_wbe_o[i*ELENB +: ELENB] = {ELENB{1'b1}};
        end
      end
    end
  end

  logic fma_clk;

  tc_clk_gating i_fma_clk_gate (
    .clk_i     (clk_i       ),
    .en_i      (mac_fire || (mac_inflight_cnt_q != 0)),
    .test_en_i ('0          ),
    .clk_o     (fma_clk     )
  );

  logic [CE-1:0][TEW-1:0] fma_x_operand, fma_w_operand;
  fpnew_pkg::fp_format_e mac_input_format;

  always_comb begin : proc_mac_input_format
    mac_input_format = fpnew_pkg::FP32;
    unique case (mac_exec_ctx.ew)
      EW_16: mac_input_format = mac_exec_ctx.is_alt ? fpnew_pkg::FP16ALT : fpnew_pkg::FP16;
      EW_8:  mac_input_format = mac_exec_ctx.is_alt ? fpnew_pkg::FP8ALT  : fpnew_pkg::FP8;
      default: mac_input_format = fpnew_pkg::FP32;
    endcase
  end


  always_comb begin : fma_operand_select
    int unsigned operand_bits;
    int unsigned x_bit_offset;
    int unsigned w_bit_offset;

    fma_x_operand = '0;
    fma_w_operand = '0;

    operand_bits = 8 << int'(mac_exec_ctx.ew);
    x_bit_offset = (int'(mac_group_row) * CE * operand_bits) % VRFWordWidth;
    w_bit_offset = (int'(mac_group_col) * CE * operand_bits) % VRFWordWidth;
    unique case (mac_exec_ctx.ew)
      EW_16: begin
        for (int unsigned el = 0; el < CE; el++) begin
          fma_x_operand[el][15:0] = vrf_rdata_i[0][x_bit_offset + el*16 +: 16];
          fma_w_operand[el][15:0] = vrf_rdata_i[1][w_bit_offset + el*16 +: 16];
        end
      end
      EW_8: begin
        for (int unsigned el = 0; el < CE; el++) begin
          fma_x_operand[el][7:0] = vrf_rdata_i[0][x_bit_offset + el*8 +: 8];
          fma_w_operand[el][7:0] = vrf_rdata_i[1][w_bit_offset + el*8 +: 8];
        end
      end
      default: begin
        for (int unsigned el = 0; el < CE; el++) begin
          fma_x_operand[el] = vrf_rdata_i[0][x_bit_offset + el*TEW +: TEW];
          fma_w_operand[el] = vrf_rdata_i[1][w_bit_offset + el*TEW +: TEW];
        end
      end
    endcase
  end : fma_operand_select

  always_comb begin : fma_addend_proc
    fma_addend = '0;

    for (int row = 0; row < CE; row++) begin
      for (int col = 0; col < CE; col++) begin
        if (mac_result_forward) begin
          fma_addend[row][col] = fma_result[row][col];
        end else if (acc_zero_q[mac_exec_ctx.tile][mac_beat_idx]) begin
          fma_addend[row][col] = '0;
        end else begin
          fma_addend[row][col] =
              acc_mem_rdata[row][col]
                           [int'(mac_exec_ctx.tile) * SpatialBeats + int'(mac_beat_idx)];
        end
      end
    end
  end

  for (genvar row = 0; row < CE; row++) begin : gen_acc_row
    for (genvar col = 0; col < CE; col++) begin : gen_acc_col
      opope_accumulator #(
        .DATA_WIDTH(TEW         ),
        .DEPTH     (AccDepth    ),
        .RD_PORTS  (AccDepth    ),
        .WR_PORTS  (SpatialBeats)
      ) i_accumulator (
        .clk_i             (clk_i              ),
        .rst_ni            (rst_ni             ),
        .flush_i           (acc_flush          ),
        .iteration_change_i(1'b0               ),
        .wdata_i           (acc_wdata[row][col]),
        .wen_i             (acc_wen[row][col]  ),
        .waddr_i           (acc_waddr          ),
        .raddr_i           ('0                 ),
        .ext_ld_i          (acc_ext_ld         ),
        .rdata_o           (acc_mem_rdata[row][col])
      );
    end : gen_acc_col
  end : gen_acc_row

  for (genvar row = 0; row < CE; row++) begin : gen_fma_row
    for (genvar col = 0; col < CE; col++) begin : gen_fma_col

      logic fma_valid;

      assign fma_result_valid[row][col] = fma_valid;

      opope_fma #(
        .FpFormat    (fpnew_pkg::FP32       ),
        .NumPipeRegs (NumPipeRegs           ),
        .PipeConfig  (fpnew_pkg::DISTRIBUTED),
        .Stallable   (1'b1                  )
      ) i_fma (
        .clk_i          (fma_clk                                                               ),
        .rst_ni         (rst_ni                                                                ),
        .operands_i     ({fma_w_operand[col],
                          fma_x_operand[row]}                                                  ),
        .addend_i       (fma_addend[row][col]                                                   ),
        .input_format_i (mac_input_format                                                       ),
        .valid_i        (mac_fire                                                              ),
        .ready_o        (fma_ready[row][col]                                                   ),
        .reg_enable_i   (mac_fire || (mac_inflight_cnt_q != 0)                                ),
        .result_valid_o (fma_valid                                                             ),
        .result_ready_i (fma_result_ready                                                      ),
        .result_o       (fma_result[row][col]                                                  )
      );

    end : gen_fma_col
  end : gen_fma_row

  initial begin : parameter_check
    if ((CE == 0) || (CE > TE))
      $error("[spatz_ope] CE must be in the range 1..TE");
    if ((TE % CE) != 0)
      $error("[spatz_ope] CE must divide TE exactly");
    if ((AccDepth & (AccDepth - 1)) != 0)
      $error("[spatz_ope] folded accumulator depth must be a power of two");
    if (!(((TE == 16) && ((CE == 2) || (CE == 4) || (CE == 8))) ||
          ((TE == 8) && (CE == 4)) || ((TE == 32) && (CE == 8)) ||
          ((TE == 64) && (CE == 8))) || (TEW != 32))
      $error("[spatz_ope] unsupported no-context TE/CE/TEW configuration");
    if ((TE != 16) || (CE != 8) || (TEW != 32) || (SpatialBeats != NumPipeRegs))
      $error("[spatz_ope] resident mode requires TE=16, CE=8, TEW=32");
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin : seq_block
    if (!rst_ni) begin
      mac_ctx_q           <= '0;
      mac_ctx_valid_q     <= '0;
      mac_ctx_beat_q      <= '0;
      mac_seq_head_q      <= '0;
      mac_seq_tail_q      <= '0;
      tile_inflight_q     <= '0;
      mac_inflight_cnt_q  <= '0;
      mac_tag_q           <= '0;
      vt_busy_q         <= 1'b0;
      vt_req_q          <= '0;
      vt_tss_q          <= '0;
      tv_busy_q         <= 1'b0;
      tv_req_q          <= '0;
      tv_tss_q          <= '0;
      tv_data_q         <= '0;
      tv_data_latched_q <= 1'b0;
      acc_zero_q        <= '0;
      resident_valid_q       <= 1'b0;
      resident_tile_q        <= '0;
      resident_drain_valid_q <= 1'b0;
      resident_drain_reason_q <= DrainNone;
    end else begin
      tile_inflight_q     <= tile_inflight_d;
      mac_inflight_cnt_q  <= mac_inflight_cnt_d;
      mac_tag_q            <= mac_tag_d;
      mac_ctx_q           <= mac_ctx_d;
      mac_ctx_valid_q     <= mac_ctx_valid_d;
      mac_ctx_beat_q      <= mac_ctx_beat_d;
      acc_zero_q          <= acc_zero_d;
      resident_valid_q       <= resident_valid_d;
      resident_tile_q        <= resident_tile_d;
      resident_drain_valid_q <= resident_drain_valid_d;
      resident_drain_reason_q <= resident_drain_reason_d;

      mac_seq_head_q <= mac_seq_head_d;
      mac_seq_tail_q <= mac_seq_tail_d;

      if (vt_req_valid && vt_req_ready) begin
        vt_req_q        <= spatz_req_vt;
        vt_tss_q        <= spatz_req_vt.op_ope.tss;
      end
      vt_busy_q <= vt_busy_d;

      if (tv_req_valid && tv_req_ready) begin
        tv_req_q        <= spatz_req_tv;
        tv_tss_q        <= spatz_req_tv.op_ope.tss;
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
