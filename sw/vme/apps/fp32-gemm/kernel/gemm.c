// Copyright 2026 ETH Zurich and University of Bologna.
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Pei-Yu Lin <peilin@ethz.ch>

// fp32 VME GEMM kernel for TE=16, CE=8, and four accumulator tiles.

#include "gemm.h"

#define LOAD_VECTOR(VR, PTR)                                            \
        "vle32.v " #VR ", (%[" #PTR "])\n"                              \
        "addi %[" #PTR "], %[" #PTR "], 64\n"

#define LOAD_PAIR_FMA(TILE, VA, AP, VB, BP)                            \
        LOAD_VECTOR(VA, AP) LOAD_VECTOR(VB, BP)                         \
        "vtfmm.tvv " #TILE ", " #VA ", " #VB "\n"

#define LOAD_B_FMA(TILE, VA, VB, BP)                                   \
        LOAD_VECTOR(VB, BP)                                             \
        "vtfmm.tvv " #TILE ", " #VA ", " #VB "\n"

#define LOAD_A_FMA(TILE, VA, AP, VB)                                   \
        LOAD_VECTOR(VA, AP)                                             \
        "vtfmm.tvv " #TILE ", " #VA ", " #VB "\n"

#define PAIR_MT0                                                        \
        LOAD_PAIR_FMA(mt0, v0,  a0p, v16, b0p)                         \
        LOAD_PAIR_FMA(mt0, v1,  a0p, v17, b0p)                         \
        LOAD_PAIR_FMA(mt0, v2,  a0p, v18, b0p)                         \
        LOAD_PAIR_FMA(mt0, v3,  a0p, v19, b0p)                         \
        LOAD_PAIR_FMA(mt0, v4,  a0p, v20, b0p)                         \
        LOAD_PAIR_FMA(mt0, v5,  a0p, v21, b0p)                         \
        LOAD_PAIR_FMA(mt0, v6,  a0p, v22, b0p)                         \
        LOAD_PAIR_FMA(mt0, v7,  a0p, v23, b0p)                         \
        LOAD_PAIR_FMA(mt0, v8,  a0p, v24, b0p)                         \
        LOAD_PAIR_FMA(mt0, v9,  a0p, v25, b0p)                         \
        LOAD_PAIR_FMA(mt0, v10, a0p, v26, b0p)                         \
        LOAD_PAIR_FMA(mt0, v11, a0p, v27, b0p)                         \
        LOAD_PAIR_FMA(mt0, v12, a0p, v28, b0p)                         \
        LOAD_PAIR_FMA(mt0, v13, a0p, v29, b0p)                         \
        LOAD_PAIR_FMA(mt0, v14, a0p, v30, b0p)                         \
        LOAD_PAIR_FMA(mt0, v15, a0p, v31, b0p)

#define PAIR_MT4                                                        \
        LOAD_B_FMA(mt4, v0,  v16, b1p)                                 \
        LOAD_B_FMA(mt4, v1,  v17, b1p)                                 \
        LOAD_B_FMA(mt4, v2,  v18, b1p)                                 \
        LOAD_B_FMA(mt4, v3,  v19, b1p)                                 \
        LOAD_B_FMA(mt4, v4,  v20, b1p)                                 \
        LOAD_B_FMA(mt4, v5,  v21, b1p)                                 \
        LOAD_B_FMA(mt4, v6,  v22, b1p)                                 \
        LOAD_B_FMA(mt4, v7,  v23, b1p)                                 \
        LOAD_B_FMA(mt4, v8,  v24, b1p)                                 \
        LOAD_B_FMA(mt4, v9,  v25, b1p)                                 \
        LOAD_B_FMA(mt4, v10, v26, b1p)                                 \
        LOAD_B_FMA(mt4, v11, v27, b1p)                                 \
        LOAD_B_FMA(mt4, v12, v28, b1p)                                 \
        LOAD_B_FMA(mt4, v13, v29, b1p)                                 \
        LOAD_B_FMA(mt4, v14, v30, b1p)                                 \
        LOAD_B_FMA(mt4, v15, v31, b1p)

#define PAIR_MT12                                                       \
        LOAD_A_FMA(mt12, v0,  a1p, v16)                                \
        LOAD_A_FMA(mt12, v1,  a1p, v17)                                \
        LOAD_A_FMA(mt12, v2,  a1p, v18)                                \
        LOAD_A_FMA(mt12, v3,  a1p, v19)                                \
        LOAD_A_FMA(mt12, v4,  a1p, v20)                                \
        LOAD_A_FMA(mt12, v5,  a1p, v21)                                \
        LOAD_A_FMA(mt12, v6,  a1p, v22)                                \
        LOAD_A_FMA(mt12, v7,  a1p, v23)                                \
        LOAD_A_FMA(mt12, v8,  a1p, v24)                                \
        LOAD_A_FMA(mt12, v9,  a1p, v25)                                \
        LOAD_A_FMA(mt12, v10, a1p, v26)                                \
        LOAD_A_FMA(mt12, v11, a1p, v27)                                \
        LOAD_A_FMA(mt12, v12, a1p, v28)                                \
        LOAD_A_FMA(mt12, v13, a1p, v29)                                \
        LOAD_A_FMA(mt12, v14, a1p, v30)                                \
        LOAD_A_FMA(mt12, v15, a1p, v31)

#define PAIR_MT8                                                        \
        LOAD_B_FMA(mt8, v0,  v16, b0p)                                 \
        LOAD_B_FMA(mt8, v1,  v17, b0p)                                 \
        LOAD_B_FMA(mt8, v2,  v18, b0p)                                 \
        LOAD_B_FMA(mt8, v3,  v19, b0p)                                 \
        LOAD_B_FMA(mt8, v4,  v20, b0p)                                 \
        LOAD_B_FMA(mt8, v5,  v21, b0p)                                 \
        LOAD_B_FMA(mt8, v6,  v22, b0p)                                 \
        LOAD_B_FMA(mt8, v7,  v23, b0p)                                 \
        LOAD_B_FMA(mt8, v8,  v24, b0p)                                 \
        LOAD_B_FMA(mt8, v9,  v25, b0p)                                 \
        LOAD_B_FMA(mt8, v10, v26, b0p)                                 \
        LOAD_B_FMA(mt8, v11, v27, b0p)                                 \
        LOAD_B_FMA(mt8, v12, v28, b0p)                                 \
        LOAD_B_FMA(mt8, v13, v29, b0p)                                 \
        LOAD_B_FMA(mt8, v14, v30, b0p)                                 \
        LOAD_B_FMA(mt8, v15, v31, b0p)

static inline uintptr_t gemm_tss_row(int tile, int row)
{
    return (((uintptr_t)(tile & 0xF) << 27) |
            (uintptr_t)(row & 0xFFFFFF));
}

static __attribute__((noinline, aligned(64))) void gemm_ope_spatial_2x2(
    float *C, const float *Apack, const float *Bpack,
    int ti, int output_stride, int col_blocks)
{
    const int tile_stride = 16 * (int)gemm_l.K;
    uintptr_t a0p = (uintptr_t)(Apack + ti * tile_stride);
    uintptr_t a1p = (uintptr_t)(Apack + (ti + 1) * tile_stride);
    uintptr_t b0p = (uintptr_t)Bpack;
    uintptr_t b1p = (uintptr_t)(Bpack + tile_stride);
    uintptr_t tss0 = gemm_tss_row(0, 0);
    uintptr_t tss4 = gemm_tss_row(4, 0);
    uintptr_t tss8 = gemm_tss_row(8, 0);
    uintptr_t tss12 = gemm_tss_row(12, 0);
    uintptr_t p00 = (uintptr_t)(C + ti * 16 * output_stride);
    uintptr_t p01 = p00 + 16 * sizeof(float);
    uintptr_t p10 = (uintptr_t)(C + (ti + 1) * 16 * output_stride);
    uintptr_t p11 = (uintptr_t)(C + (ti + 1) * 16 * output_stride +
                                16);
    const uintptr_t c_stride = (uintptr_t)(output_stride * sizeof(float));
    const uintptr_t row_span = 16 * c_stride;
    const uintptr_t tile_bytes = (uintptr_t)(tile_stride * sizeof(float));

    for (int cb = 0; cb < col_blocks; ++cb) {
    asm volatile(
        "vtzero mt0\n"
        "vtzero mt4\n"
        "vtzero mt8\n"
        "vtzero mt12\n" ::: "memory");

    asm volatile(

#ifdef SPATZ_GEMM_LONG_TILE_BURST
        // K=0..47: process two K-groups per resident tile. One B0 group is
        // reloaded per pair because A0/A1/B0/B1 occupy all 32 vector registers.
        ".rept 3\n"
        PAIR_MT0
        PAIR_MT4
        PAIR_MT12
        "addi %[b0p], %[b0p], -1024\n"
        PAIR_MT8
        ".endr\n"
        // The shared tail expects A0[K48..55] in v0..v7.
        LOAD_VECTOR(v0, a0p)
        LOAD_VECTOR(v1, a0p)
        LOAD_VECTOR(v2, a0p)
        LOAD_VECTOR(v3, a0p)
        LOAD_VECTOR(v4, a0p)
        LOAD_VECTOR(v5, a0p)
        LOAD_VECTOR(v6, a0p)
        LOAD_VECTOR(v7, a0p)
#else
        // K-Group 1
        // vle A0
        // vle B0
        // vtfmm mt0
        "vle32.v v0,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vle32.v v16, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v0, v16\n"
        "vle32.v v1,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vle32.v v17, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v1, v17\n"
        "vle32.v v2,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vle32.v v18, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v2, v18\n"
        "vle32.v v3,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vle32.v v19, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v3, v19\n"
        "vle32.v v4,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vle32.v v20, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v4, v20\n"
        "vle32.v v5,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vle32.v v21, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v5, v21\n"
        "vle32.v v6,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vle32.v v22, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v6, v22\n"
        "vle32.v v7,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vle32.v v23, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v7, v23\n"

        // vle B1
        // vtfmm mt4
        "vle32.v v24,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtfmm.tvv mt4, v0, v24\n"
        "vle32.v v25,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtfmm.tvv mt4, v1, v25\n"
        "vle32.v v26,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtfmm.tvv mt4, v2, v26\n"
        "vle32.v v27,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtfmm.tvv mt4, v3, v27\n"
        "vle32.v v28,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtfmm.tvv mt4, v4, v28\n"
        "vle32.v v29,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtfmm.tvv mt4, v5, v29\n"
        "vle32.v v30,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtfmm.tvv mt4, v6, v30\n"
        "vle32.v v31,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtfmm.tvv mt4, v7, v31\n"

        // vle A1
        // vtfmm mt8
        "vle32.v v8,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt8, v8,  v16\n"
        "vle32.v v9,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt8, v9,  v17\n"
        "vle32.v v10,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt8, v10,  v18\n"
        "vle32.v v11,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt8, v11,  v19\n"
        "vle32.v v12,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt8, v12,  v20\n"
        "vle32.v v13,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt8, v13,  v21\n"
        "vle32.v v14,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt8, v14,  v22\n"
        "vle32.v v15,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt8, v15,  v23\n"

        // preload vle A0
        // vtfmm mt12
        "vle32.v v0,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vtfmm.tvv mt12, v8,  v24\n"
        "vle32.v v1,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vtfmm.tvv mt12, v9,  v25\n"
        "vle32.v v2,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vtfmm.tvv mt12, v10, v26\n"
        "vle32.v v3,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vtfmm.tvv mt12, v11, v27\n"
        "vle32.v v4,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vtfmm.tvv mt12, v12, v28\n"
        "vle32.v v5,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vtfmm.tvv mt12, v13, v29\n"
        "vle32.v v6,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vtfmm.tvv mt12, v14, v30\n"
        "vle32.v v7,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vtfmm.tvv mt12, v15, v31\n"
        
        // K-Group 2 to N-2
        // vle B0
        // vtfmm mt0
        ".rept 5\n" // ".rept (gemm_l.K / 8 -3)\n"
        "vle32.v v16, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v0, v16\n"
        "vle32.v v17, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v1, v17\n"
        "vle32.v v18, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v2, v18\n"
        "vle32.v v19, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v3, v19\n"
        "vle32.v v20, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v4, v20\n"
        "vle32.v v21, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v5, v21\n"
        "vle32.v v22, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v6, v22\n"
        "vle32.v v23, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v7, v23\n"

        // vle B1
        // vtfmm mt4
        "vle32.v v24,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtfmm.tvv mt4, v0, v24\n"
        "vle32.v v25,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtfmm.tvv mt4, v1, v25\n"
        "vle32.v v26,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtfmm.tvv mt4, v2, v26\n"
        "vle32.v v27,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtfmm.tvv mt4, v3, v27\n"
        "vle32.v v28,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtfmm.tvv mt4, v4, v28\n"
        "vle32.v v29,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtfmm.tvv mt4, v5, v29\n"
        "vle32.v v30,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtfmm.tvv mt4, v6, v30\n"
        "vle32.v v31,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtfmm.tvv mt4, v7, v31\n"

        // vle A1
        // vtfmm mt8
        "vle32.v v8,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt8, v8,  v16\n"
        "vle32.v v9,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt8, v9,  v17\n"
        "vle32.v v10,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt8, v10,  v18\n"
        "vle32.v v11,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt8, v11,  v19\n"
        "vle32.v v12,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt8, v12,  v20\n"
        "vle32.v v13,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt8, v13,  v21\n"
        "vle32.v v14,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt8, v14,  v22\n"
        "vle32.v v15,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt8, v15,  v23\n"

        // preload vle A0
        // vtfmm mt12
        "vle32.v v0,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vtfmm.tvv mt12, v8,  v24\n"
        "vle32.v v1,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vtfmm.tvv mt12, v9,  v25\n"
        "vle32.v v2,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vtfmm.tvv mt12, v10, v26\n"
        "vle32.v v3,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vtfmm.tvv mt12, v11, v27\n"
        "vle32.v v4,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vtfmm.tvv mt12, v12, v28\n"
        "vle32.v v5,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vtfmm.tvv mt12, v13, v29\n"
        "vle32.v v6,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vtfmm.tvv mt12, v14, v30\n"
        "vle32.v v7,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vtfmm.tvv mt12, v15, v31\n"
        ".endr\n"
#endif

        // K-Group N-1 & K-Group N
        // vle B0
        // vtfmm mt0
        "vle32.v v16, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v0, v16\n"
        "vle32.v v17, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v1, v17\n"
        "vle32.v v18, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v2, v18\n"
        "vle32.v v19, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v3, v19\n"
        "vle32.v v20, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v4, v20\n"
        "vle32.v v21, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v5, v21\n"
        "vle32.v v22, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v6, v22\n"
        "vle32.v v23, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v7, v23\n"
        
        // vle A0'
        // vle B0'
        // vtfmm mt0
        "vle32.v v8,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vle32.v v24, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v8, v24\n"
        "vle32.v v9,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vle32.v v25, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v9, v25\n"
        "vle32.v v10,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vle32.v v26, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v10, v26\n"
        "vle32.v v11,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vle32.v v27, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v11, v27\n"
        "vle32.v v12,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vle32.v v28, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v12, v28\n"
        "vle32.v v13,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vle32.v v29, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v13, v29\n"
        "vle32.v v14,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vle32.v v30, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v14, v30\n"
        "vle32.v v15,  (%[a0p])\n" "addi %[a0p], %[a0p], 64\n"
        "vle32.v v31, (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt0, v15, v31\n"
        
        // vle B1 (reuse A0)
        // vtfmm mt4
        "vle32.v v16,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtse32 %[tss0], (%[p00])\n"
        "vtfmm.tvv mt4, v0, v16\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"

        "vle32.v v17,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtse32 %[tss0], (%[p00])\n"
        "vtfmm.tvv mt4, v1, v17\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"

        "vle32.v v18,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtse32 %[tss0], (%[p00])\n"
        "vtfmm.tvv mt4, v2, v18\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"

        "vle32.v v19,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtse32 %[tss0], (%[p00])\n"
        "vtfmm.tvv mt4, v3, v19\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"

        "vle32.v v20,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtse32 %[tss0], (%[p00])\n"
        "vtfmm.tvv mt4, v4, v20\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"

        "vle32.v v21,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtse32 %[tss0], (%[p00])\n"
        "vtfmm.tvv mt4, v5, v21\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"

        "vle32.v v22,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtse32 %[tss0], (%[p00])\n"
        "vtfmm.tvv mt4, v6, v22\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"

        "vle32.v v23,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtse32 %[tss0], (%[p00])\n"
        "vtfmm.tvv mt4, v7, v23\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"
        
        // vle B1' (reuse A0')
        // vtfmm mt4
        "vle32.v v24,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtse32 %[tss0], (%[p00])\n"
        "vtfmm.tvv mt4, v8, v24\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"

        "vle32.v v25,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtse32 %[tss0], (%[p00])\n"
        "vtfmm.tvv mt4, v9, v25\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"

        "vle32.v v26,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtse32 %[tss0], (%[p00])\n"
        "vtfmm.tvv mt4, v10, v26\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"

        "vle32.v v27,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtse32 %[tss0], (%[p00])\n"
        "vtfmm.tvv mt4, v11, v27\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"

        "vle32.v v28,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtse32 %[tss0], (%[p00])\n"
        "vtfmm.tvv mt4, v12, v28\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"

        "vle32.v v29,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtse32 %[tss0], (%[p00])\n"
        "vtfmm.tvv mt4, v13, v29\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"

        "vle32.v v30,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtse32 %[tss0], (%[p00])\n"
        "vtfmm.tvv mt4, v14, v30\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"

        "vle32.v v31,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtse32 %[tss0], (%[p00])\n"
        "vtfmm.tvv mt4, v15, v31\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"

        // TO BE DELETED
        "vle32.v v31,  (%[b1p])\n" "addi %[b1p], %[b1p], 64\n"
        "vtse32 %[tss0], (%[p00])\n" "add %[p00], %[p00], %[c_stride]\n"
        "vtfmm.tvv mt4, v15, v31\n" "addi %[tss0], %[tss0], 1\n"
        
        

        // vle A1 (reuse B0)
        // vtfmm mt12
        // vtse mt4
        "vle32.v v0,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt12, v0,  v16\n"
        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"

        "vle32.v v1,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt12, v1,  v17\n"
        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"

        "vle32.v v2,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt12, v2,  v18\n"
        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"

        "vle32.v v3,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt12, v3,  v19\n"
        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"

        "vle32.v v4,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt12, v4,  v20\n"
        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"

        "vle32.v v5,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt12, v5,  v21\n"
        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"

        "vle32.v v6,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt12, v6,  v22\n"
        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"

        "vle32.v v7,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt12, v7,  v23\n"
        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"

        // vle A1' (reuse B0')
        // vtfmm mt12
        // vtse mt4
        "vle32.v v8,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt12, v8,  v24\n"
        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"

        "vle32.v v9,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt12, v9,  v25\n"
        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"

        "vle32.v v10,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt12, v10,  v26\n"
        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"

        "vle32.v v11,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt12, v11,  v27\n"
        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"

        "vle32.v v12,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt12, v12,  v28\n"
        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"

        "vle32.v v13,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt12, v13,  v29\n"
        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"

        "vle32.v v14,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt12, v14,  v30\n"
        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"

        "vle32.v v15,  (%[a1p])\n" "addi %[a1p], %[a1p], 64\n"
        "vtfmm.tvv mt12, v15,  v31\n"
        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"

        // vle B0 (reuse A1)
        // vtfmm mt8
        // vtse mt12
        "addi %[b0p], %[b0p], -1024\n"

        "vle32.v v16,  (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt8, v0,  v16\n"
        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"

        "vle32.v v17,  (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt8, v1,  v17\n"
        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"

        "vle32.v v18,  (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt8, v2,  v18\n"
        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"

        "vle32.v v19,  (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt8, v3,  v19\n"
        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"

        "vle32.v v20,  (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt8, v4,  v20\n"
        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"

        "vle32.v v21,  (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt8, v5,  v21\n"
        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"

        "vle32.v v22,  (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt8, v6,  v22\n"
        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"

        "vle32.v v23,  (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt8, v7,  v23\n"
        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"

        // vle B0' (reuse A1')
        // vtfmm mt8
        // vtse mt12
        "vle32.v v24,  (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt8, v8,  v24\n"
        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"

        "vle32.v v25,  (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt8, v9,  v25\n"
        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"

        "vle32.v v26,  (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt8, v10,  v26\n"
        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"

        "vle32.v v27,  (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt8, v11,  v27\n"
        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"

        "vle32.v v28,  (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt8, v12,  v28\n"
        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"

        "vle32.v v29,  (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt8, v13,  v29\n"
        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"

        "vle32.v v30,  (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt8, v14,  v30\n"
        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"

        "vle32.v v31,  (%[b0p])\n" "addi %[b0p], %[b0p], 64\n"
        "vtfmm.tvv mt8, v15,  v31\n"
        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"

        // vtse mt8
        ".rept 16\n"
        "vtse32 %[tss8], (%[p10])\n"
        "add %[p10], %[p10], %[c_stride]\n"
        "addi %[tss8], %[tss8], 1\n"
        ".endr\n"

        :
          [a0p] "+r"(a0p),
          [a1p] "+r"(a1p),
          [b0p] "+r"(b0p),
          [b1p] "+r"(b1p),
          [tss0] "+r"(tss0),
          [tss4] "+r"(tss4),
          [tss8] "+r"(tss8),
          [tss12] "+r"(tss12),
          [p00] "+r"(p00),
          [p01] "+r"(p01),
          [p10] "+r"(p10),
          [p11] "+r"(p11)
        :
          [c_stride] "r"(c_stride)
        : "memory");

        if (cb + 1 != col_blocks) {
            a0p -= tile_bytes;
            a1p -= tile_bytes;
            b0p += tile_bytes;
            b1p += tile_bytes;
            p00 += 2 * 16 * sizeof(float) - row_span;
            p01 += 2 * 16 * sizeof(float) - row_span;
            p10 += 2 * 16 * sizeof(float) - row_span;
            p11 += 2 * 16 * sizeof(float) - row_span;
            tss0 -= 16;
            tss4 -= 16;
            tss8 -= 16;
            tss12 -= 16;
        }
    }
}

void gemm_fp32(float *C, const float *Apack, const float *Bpack,
                    uint32_t ti_lo, uint32_t ti_hi)
{
    const int output_stride = gemm_l.N < 64 ? 64 : (int)gemm_l.N;
    const int col_blocks = (int)gemm_l.N / 32;

    const uintptr_t TM = 16;
    const uintptr_t TN = 16;

    asm volatile("msetmtypei 1, 2" ::: "memory");
    asm volatile("msettn x0, %0" :: "r"(TN) : "memory");
    asm volatile("msettm x0, %0" :: "r"(TM) : "memory");
    asm volatile("csrc 0xC21, %0" :: "r"((uintptr_t)256) : "memory");

    if (ti_lo != 0)
        asm volatile("nop" ::: "memory");

    for (int ti = (int)ti_lo; ti < (int)ti_hi; ti += 2) {
#ifdef SPATZ_GEMM_CROSS_BLOCK
        gemm_ope_spatial_2x2(C, Apack, Bpack, ti, output_stride, col_blocks);
#else
        const int tile_stride = 16 * (int)gemm_l.K;
        for (int cb = 0; cb < col_blocks; ++cb) {
            gemm_ope_spatial_2x2(C + cb * 32, Apack,
                                 Bpack + cb * 2 * tile_stride,
                                 ti, output_stride, 1);
        }
#endif
    }
}
