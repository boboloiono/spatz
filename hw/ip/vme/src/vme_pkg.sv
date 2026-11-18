// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Pei-Yu Lin, ETH Zürich

package vme_pkg;

  // -------------------------------------------------------------------------
  // Architectural command contract
  // -------------------------------------------------------------------------
  // These constants determine packed interface types and therefore must match
  // the host adapter. They are deliberately not module parameters.
  localparam int unsigned VME_TILE_EDGE        = 16;
  localparam int unsigned VME_NR_LOGICAL_TILES = 16;
  localparam int unsigned VME_K_MAX            = 4;

  // -------------------------------------------------------------------------
  // Host integration contract
  // -------------------------------------------------------------------------
  // These values size the VME-to-host interfaces. The adapter must convert or
  // reject a host whose contract differs from these values.
  localparam int unsigned VME_NR_INFLIGHT      = 8;
  localparam int unsigned VME_VLEN             = 512;
  localparam int unsigned VME_NR_VREGS         = 32;
  localparam int unsigned VME_VRF_WORD_WIDTH   = 256;
  localparam int unsigned VME_ACC_WORD_WIDTH   = 32;
  localparam int unsigned VME_MEM_ADDR_WIDTH   = 32;
  localparam int unsigned VME_MEM_DATA_WIDTH   = 64;

  // -------------------------------------------------------------------------
  // Default microarchitecture configuration
  // -------------------------------------------------------------------------
  // These defaults may be overridden at the VME module boundary because they
  // change only implementation structure or supported datapaths.
  localparam int unsigned VME_COMPUTE_EDGE = 8;
  localparam bit          VME_ENABLE_FMA16 = 1'b0;
  localparam bit          VME_ENABLE_FMA32 = 1'b0;

  // -------------------------------------------------------------------------
  // Derived constants: never override independently
  // -------------------------------------------------------------------------
  localparam int unsigned VME_VRF_WORD_BYTES       = VME_VRF_WORD_WIDTH >> 3;
  localparam int unsigned VME_WORDS_PER_VREG       = VME_VLEN / VME_VRF_WORD_WIDTH;
  localparam int unsigned VME_NR_VRF_WORDS         = VME_NR_VREGS * VME_WORDS_PER_VREG;
  localparam int unsigned VME_ACC_WORD_BYTES       = VME_ACC_WORD_WIDTH >> 3;
  localparam int unsigned VME_MAX_VREG_GROUP_WORDS = VME_WORDS_PER_VREG << 3;
  localparam int unsigned VME_ACC_TILE_BYTES       =
      VME_TILE_EDGE * VME_TILE_EDGE * VME_ACC_WORD_BYTES;
  localparam int unsigned VME_TILE_WORD_BYTES      = VME_TILE_EDGE * VME_ACC_WORD_BYTES;

  typedef logic [$clog2(VME_MAX_VREG_GROUP_WORDS + 1)-1:0] move_word_count_t;

  typedef enum logic [2:0] {
    EW_8  = 3'b000,
    EW_16 = 3'b001,
    EW_32 = 3'b010,
    EW_64 = 3'b011
  } vew_e;

  typedef logic [$clog2(VME_NR_INFLIGHT)-1:0] instruction_id_t;
  // Vector Registers
  typedef logic [$clog2(VME_NR_VREGS)-1:0] vreg_t;
  typedef logic [$clog2(VME_NR_VRF_WORDS)-1:0] vrf_addr_t;
  typedef logic [VME_VRF_WORD_WIDTH-1:0] vrf_data_t;
  typedef logic [VME_VRF_WORD_BYTES-1:0] vrf_be_t;
  typedef logic [$clog2(VME_VLEN + 1)-1:0] vlen_t;
  // Tile Registers
  typedef logic [$clog2(VME_NR_LOGICAL_TILES)-1:0] mt_t;
  typedef logic [$clog2(VME_TILE_EDGE)-1:0] tile_dim_t;
  typedef logic [$clog2(VME_TILE_EDGE + 1)-1:0] tile_count_t;
  typedef logic [$clog2(VME_TILE_WORD_BYTES)-1:0] tile_word_elem_idx_t;
  typedef logic [(VME_TILE_WORD_BYTES << 3)-1:0] tile_word_t;
  typedef logic [VME_MEM_ADDR_WIDTH-1:0] mem_addr_t;
  typedef logic [VME_MEM_DATA_WIDTH-1:0] mem_data_t;
  typedef logic [(VME_MEM_DATA_WIDTH >> 3)-1:0] mem_strb_t;

  typedef struct packed {
    mem_addr_t addr;
    logic      write;
    mem_data_t data;
    mem_strb_t strb;
  } tile_mem_req_t;

  typedef struct packed {
    mem_data_t data;
  } tile_mem_rsp_t;

  typedef struct packed {
    move_word_count_t words;
    vlen_t            elems;
    tile_count_t      line_elems;
  } move_shape_t;

  typedef struct packed {
    logic      tile_valid;
    mt_t       tile_id;
    logic      is_row;
    tile_dim_t index;
  } tile_slice_t;

  typedef struct packed {
    logic        is_mac;
    logic        is_zero_tile;
    logic        is_discard;
    vew_e        tew;
    tile_slice_t tss;
    tile_count_t tn;
    tile_count_t tm;
    logic [2:0]  tk;
  } operation_t;

  typedef struct packed {
    vew_e vsew;
  } vector_type_t;

  // Processor-independent command accepted by the VME core. Decode, CSR
  // semantics and scoreboard ownership stay in the processor adapter.
  typedef struct packed {
    instruction_id_t id;
    vreg_t           vs1;
    vreg_t           vs2;
    logic            use_vs2;
    vreg_t           vd;
    logic            use_vd;
    mt_t             mtd;
    operation_t      op_vme;
    vector_type_t    vtype;
    vlen_t           vl;
    vlen_t           vstart;
    mem_addr_t       scalar_addr;
    logic            is_tile_mem;
    logic            tile_mem_load;
    vew_e            tile_mem_ew;
  } request_t;

  typedef struct packed {
    instruction_id_t id;
  } response_t;

  typedef logic [VME_NR_INFLIGHT-1:0] deps_t;
  localparam int unsigned VME_DEP_GENERATION_WIDTH = 4;
  typedef logic [VME_DEP_GENERATION_WIDTH-1:0] dep_generation_t;
  typedef dep_generation_t [VME_NR_INFLIGHT-1:0] dep_generations_t;

  // Entry shared by the independent compute, move, and control issue queues.
  // The generation snapshot distinguishes an older completed producer from a
  // newer instruction that reuses the same host instruction ID.
  typedef struct packed {
    request_t         req;
    deps_t            deps;
    dep_generations_t dep_generations;
  } issue_entry_t;

  typedef struct packed {
    mt_t         idx;
    tile_dim_t   row;
    tile_dim_t   col;
    tile_count_t elems;
    vew_e        tew;
  } tile_read_req_t;

  typedef struct packed {
    mt_t         idx;
    tile_dim_t   row;
    tile_dim_t   col;
    tile_count_t elems;
    vew_e        tew;
    tile_word_t  data;
  } tile_write_req_t;

endpackage
