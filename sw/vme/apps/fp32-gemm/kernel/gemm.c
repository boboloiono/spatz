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

enum {
    GEMM_TSS_MT0_ROW0 = 0u << 27,
    GEMM_TSS_MT4_ROW0 = 4u << 27,
    GEMM_TSS_MT8_ROW0 = 8u << 27,
    GEMM_TSS_MT12_ROW0 = 12u << 27,
};

__attribute__((noinline, aligned(64))) void gemm_fp32(
    float *C, const float *Apack, const float *Bpack,
    uint32_t ti_lo, uint32_t ti_hi)
{
    const int ti = (int)ti_lo;
    const int output_stride = gemm_l.N < 64 ? 64 : (int)gemm_l.N;
    const int col_blocks = (int)gemm_l.N / 32;
    const int tile_stride = 16 * (int)gemm_l.K;

    // The M64 two-core partition assigns one row-tile pair to each core.
    (void)ti_hi;

    if (ti_lo != 0)
        asm volatile("nop" ::: "memory");

    uintptr_t a0p = (uintptr_t)(Apack + ti * tile_stride);
    uintptr_t a1p = (uintptr_t)(Apack + (ti + 1) * tile_stride);
    uintptr_t b0p = (uintptr_t)Bpack;
    uintptr_t b1p = (uintptr_t)(Bpack + tile_stride);
    uintptr_t tss0 = GEMM_TSS_MT0_ROW0;
    uintptr_t tss4 = GEMM_TSS_MT4_ROW0;
    uintptr_t tss8 = GEMM_TSS_MT8_ROW0;
    uintptr_t tss12 = GEMM_TSS_MT12_ROW0;
    uintptr_t p00 = (uintptr_t)(C + ti * 16 * output_stride);
    uintptr_t p01 = p00 + 16 * sizeof(float);
    uintptr_t p10 = (uintptr_t)(C + (ti + 1) * 16 * output_stride);
    uintptr_t p11 = (uintptr_t)(C + (ti + 1) * 16 * output_stride + 16);
    const uintptr_t c_stride = (uintptr_t)(output_stride * sizeof(float));
    const uintptr_t group_vl = 128;
    const uintptr_t matrix_mtype = 1;
    const uintptr_t matrix_vtype = 0xD0;
    const uintptr_t TM = 16;
    const uintptr_t TN = 16;
    const uintptr_t TK = 8;
    uintptr_t block_counter = (uintptr_t)col_blocks;
    uintptr_t loop_counter;

    asm volatile(
        // K-Group 1
        // vle A0
        "msetmtype %[mtype], %[vtype]\n"
        "msettn x0, %[tn]\n"
        "vsetvli x0, %[group_vl], e32, m8, ta, ma\n"
        "vle32.v v0,  (%[a0p])\n"
        "vle32.v v16,  (%[b0p])\n"
        "msettm x0, %[tm]\n"
        "msettk x0, %[tk]\n"
        "vtzero mt0\n"
        
        // vle B1
        // vtfmm mt0
        "vle32.v v24,  (%[b1p])\n"
        "vtfmm.tvv mt0, v0, v16\n"
        "addi %[a0p], %[a0p], 512\n"
        "vtfmm.tvv mt0, v1, v17\n"
        "addi %[b0p], %[b0p], 512\n"
        "vtfmm.tvv mt0, v2, v18\n"
        "addi %[b1p], %[b1p], 512\n"
        "vtfmm.tvv mt0, v3, v19\n"
        "vtfmm.tvv mt0, v4, v20\n"
        "vtfmm.tvv mt0, v5, v21\n"
        "vtzero mt4\n"
        "vtfmm.tvv mt0, v6, v22\n"
        "vtfmm.tvv mt0, v7, v23\n"

        // vle A1
        // vtfmm mt4
        "vle32.v v8,  (%[a1p])\n"
        "vtfmm.tvv mt4, v0, v24\n"
        "addi %[a1p], %[a1p], 512\n"
        "vtfmm.tvv mt4, v1, v25\n"
        "vtfmm.tvv mt4, v2, v26\n"
        "vtfmm.tvv mt4, v3, v27\n"
        "vtfmm.tvv mt4, v4, v28\n"
        "vtfmm.tvv mt4, v5, v29\n"
        "vtzero mt8\n"
        "vtfmm.tvv mt4, v6, v30\n"
        "vtfmm.tvv mt4, v7, v31\n"

        // preload vle A0
        // vtfmm mt8
        "vle32.v v0,  (%[a0p])\n"
        "vtfmm.tvv mt8, v8,  v16\n"
        "addi %[a0p], %[a0p], 512\n"
        "vtfmm.tvv mt8, v9,  v17\n"
        "vtfmm.tvv mt8, v10,  v18\n"
        "vtfmm.tvv mt8, v11,  v19\n"
        "vtfmm.tvv mt8, v12,  v20\n"
        "vtfmm.tvv mt8, v13,  v21\n"
        "vtzero mt12\n"
        "vtfmm.tvv mt8, v14,  v22\n"
        "vtfmm.tvv mt8, v15,  v23\n"

        // preload vle A1
        // vtfmm mt12
        "vle32.v v16,  (%[b0p])\n"
        "vtfmm.tvv mt12, v8,  v24\n"
        "addi %[b0p], %[b0p], 512\n"
        "vtfmm.tvv mt12, v9,  v25\n"
        "vtfmm.tvv mt12, v10, v26\n"
        "vtfmm.tvv mt12, v11, v27\n"
        "vtfmm.tvv mt12, v12, v28\n"
        "vtfmm.tvv mt12, v13, v29\n"
        "vtfmm.tvv mt12, v14, v30\n"
        "vtfmm.tvv mt12, v15, v31\n"

        "9:\n"
        // K-Group 1 to N-2
        // vle B0
        // vtfmm mt0
        "li %[loop], 5\n"
        "1:\n"
        "vle32.v v24,  (%[b1p])\n"
        "vtfmm.tvv mt0, v0, v16\n"
        "addi %[b1p], %[b1p], 512\n"
        "vtfmm.tvv mt0, v1, v17\n"
        "vtfmm.tvv mt0, v2, v18\n"
        "vtfmm.tvv mt0, v3, v19\n"
        "vtfmm.tvv mt0, v4, v20\n"
        "vtfmm.tvv mt0, v5, v21\n"
        "vtfmm.tvv mt0, v6, v22\n"
        "vtfmm.tvv mt0, v7, v23\n"

        // vle B1
        // vtfmm mt4
        "vle32.v v8,  (%[a1p])\n"
        "vtfmm.tvv mt4, v0, v24\n"
        "addi %[a1p], %[a1p], 512\n"
        "vtfmm.tvv mt4, v1, v25\n"
        "vtfmm.tvv mt4, v2, v26\n"
        "vtfmm.tvv mt4, v3, v27\n"
        "vtfmm.tvv mt4, v4, v28\n"
        "vtfmm.tvv mt4, v5, v29\n"
        "vtfmm.tvv mt4, v6, v30\n"
        "vtfmm.tvv mt4, v7, v31\n"

        // vle A1
        // vtfmm mt8
        "vle32.v v0,  (%[a0p])\n"
        "vtfmm.tvv mt8, v8,  v16\n"
        "addi %[a0p], %[a0p], 512\n"
        "vtfmm.tvv mt8, v9,  v17\n"
        "vtfmm.tvv mt8, v10,  v18\n"
        "vtfmm.tvv mt8, v11,  v19\n"
        "vtfmm.tvv mt8, v12,  v20\n"
        "vtfmm.tvv mt8, v13,  v21\n"
        "vtfmm.tvv mt8, v14,  v22\n"
        "vtfmm.tvv mt8, v15,  v23\n"

        // preload vle A0
        // vtfmm mt12
        "addi %[loop], %[loop], -1\n"
        "vle32.v v16,  (%[b0p])\n"
        "vtfmm.tvv mt12, v8,  v24\n"
        "addi %[b0p], %[b0p], 512\n"
        "vtfmm.tvv mt12, v9,  v25\n"
        "vtfmm.tvv mt12, v10, v26\n"
        "vtfmm.tvv mt12, v11, v27\n"
        "vtfmm.tvv mt12, v12, v28\n"
        "vtfmm.tvv mt12, v13, v29\n"
        "vtfmm.tvv mt12, v14, v30\n"
        "vtfmm.tvv mt12, v15, v31\n"
        "bnez %[loop], 1b\n"

        // K-Group N-1 & K-Group N
        // vle B0
        // vtfmm mt0
        "vle32.v v8,  (%[a0p])\n"
        "vtfmm.tvv mt0, v0, v16\n"
        "addi %[a0p], %[a0p], 512\n"
        "vtfmm.tvv mt0, v1, v17\n"
        "vtfmm.tvv mt0, v2, v18\n"
        "vtfmm.tvv mt0, v3, v19\n"
        "vle32.v v24,  (%[b0p])\n"
        "vtfmm.tvv mt0, v4, v20\n"
        "addi %[b0p], %[b0p], 512\n"
        "vtfmm.tvv mt0, v5, v21\n"
        "vtfmm.tvv mt0, v6, v22\n"
        "vtfmm.tvv mt0, v7, v23\n"
        
        // vle A0'
        // vle B0'
        // vtfmm mt0
        "vle32.v v16,  (%[b1p])\n"
        "vtfmm.tvv mt0, v8, v24\n"
        "addi %[b1p], %[b1p], 512\n"
        "vtfmm.tvv mt0, v9, v25\n"
        "vtfmm.tvv mt0, v10, v26\n"
        "vtfmm.tvv mt0, v11, v27\n"
        "vtfmm.tvv mt0, v12, v28\n"
        "vtfmm.tvv mt0, v13, v29\n"
        "vtfmm.tvv mt0, v14, v30\n"
        "vtfmm.tvv mt0, v15, v31\n"
        
        // vle B1 (reuse A0)
        // vtfmm mt4
        "vle32.v v24,  (%[b1p])\n"

        "vtse32 %[tss0], (%[p00])\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"
        "vtfmm.tvv mt4, v0, v16\n"

        "vtse32 %[tss0], (%[p00])\n"
        "addi %[b1p], %[b1p], 512\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"
        "vtfmm.tvv mt4, v1, v17\n"

        "vtse32 %[tss0], (%[p00])\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"
        "vtfmm.tvv mt4, v2, v18\n"

        "vtse32 %[tss0], (%[p00])\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"
        "vtfmm.tvv mt4, v3, v19\n"

        "vtse32 %[tss0], (%[p00])\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"
        "vtfmm.tvv mt4, v4, v20\n"

        "vtse32 %[tss0], (%[p00])\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"
        "vtfmm.tvv mt4, v5, v21\n"

        "vtse32 %[tss0], (%[p00])\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"
        "vtfmm.tvv mt4, v6, v22\n"

        "vtse32 %[tss0], (%[p00])\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"
        "vtfmm.tvv mt4, v7, v23\n"

        // vle B1' (reuse A0')
        // vtfmm mt4
        "vtse32 %[tss0], (%[p00])\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"
        "vtfmm.tvv mt4, v8, v24\n"

        "vtse32 %[tss0], (%[p00])\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"
        "vtfmm.tvv mt4, v9, v25\n"

        "vtse32 %[tss0], (%[p00])\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"
        "vtfmm.tvv mt4, v10, v26\n"

        "vtse32 %[tss0], (%[p00])\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"
        "vtfmm.tvv mt4, v11, v27\n"

        "vtse32 %[tss0], (%[p00])\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"
        "vtfmm.tvv mt4, v12, v28\n"

        "vtse32 %[tss0], (%[p00])\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"
        "vtfmm.tvv mt4, v13, v29\n"

        "vtse32 %[tss0], (%[p00])\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"
        "vtfmm.tvv mt4, v14, v30\n"

        "vtse32 %[tss0], (%[p00])\n"
        "vle32.v v0,  (%[a1p])\n"
        "add %[p00], %[p00], %[c_stride]\n"
        "addi %[tss0], %[tss0], 1\n"
        "vtfmm.tvv mt4, v15, v31\n"

        // vle A1 (reuse B0)
        // vtfmm mt12
        // vtse mt4

        "vtse32 %[tss4], (%[p01])\n"
        "addi %[a1p], %[a1p], 512\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"
        "vtfmm.tvv mt12, v0,  v16\n"

        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"
        "vtfmm.tvv mt12, v1,  v17\n"

        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"
        "vtfmm.tvv mt12, v2,  v18\n"

        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"
        "vtfmm.tvv mt12, v3,  v19\n"

        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"
        "vtfmm.tvv mt12, v4,  v20\n"

        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"
        "vtfmm.tvv mt12, v5,  v21\n"

        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"
        "vtfmm.tvv mt12, v6,  v22\n"

        "vtse32 %[tss4], (%[p01])\n"
        "vle32.v v8,  (%[a1p])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"
        "vtfmm.tvv mt12, v7,  v23\n"

        // vle A1' (reuse B0')
        // vtfmm mt12
        // vtse mt4

        "vtse32 %[tss4], (%[p01])\n"
        "addi %[a1p], %[a1p], 512\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"
        "vtfmm.tvv mt12, v8,  v24\n"

        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"
        "vtfmm.tvv mt12, v9,  v25\n"

        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"
        "vtfmm.tvv mt12, v10,  v26\n"

        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"
        "vtfmm.tvv mt12, v11,  v27\n"

        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"
        "vtfmm.tvv mt12, v12,  v28\n"

        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"
        "addi %[b0p], %[b0p], -1024\n"
        "vtfmm.tvv mt12, v13,  v29\n"

        "vtse32 %[tss4], (%[p01])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"
        "vtfmm.tvv mt12, v14,  v30\n"

        "vtse32 %[tss4], (%[p01])\n"
        "vle32.v v16,  (%[b0p])\n"
        "add %[p01], %[p01], %[c_stride]\n"
        "addi %[tss4], %[tss4], 1\n"
        "vtfmm.tvv mt12, v15,  v31\n"

        // vle B0 (reuse A1)
        // vtfmm mt8
        // vtse mt12
        "vtse32 %[tss12], (%[p11])\n"
        "addi %[b0p], %[b0p], 512\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"
        "vtfmm.tvv mt8, v0,  v16\n"

        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"
        "vtfmm.tvv mt8, v1,  v17\n"

        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"
        "vtfmm.tvv mt8, v2,  v18\n"

        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"
        "vtfmm.tvv mt8, v3,  v19\n"

        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"
        "vtfmm.tvv mt8, v4,  v20\n"

        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"
        "vtfmm.tvv mt8, v5,  v21\n"

        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"
        "vtfmm.tvv mt8, v6,  v22\n"

        "vtse32 %[tss12], (%[p11])\n"
        "vle32.v v24,  (%[b0p])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"
        "vtfmm.tvv mt8, v7,  v23\n"

        // vle B0' (reuse A1')
        // vtfmm mt8
        // vtse mt12

        "vtse32 %[tss12], (%[p11])\n"
        "addi %[b0p], %[b0p], 512\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"
        "vtfmm.tvv mt8, v8,  v24\n"

        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"
        "vtfmm.tvv mt8, v9,  v25\n"

        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"
        "vtfmm.tvv mt8, v10,  v26\n"

        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"
        "vtfmm.tvv mt8, v11,  v27\n"

        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"
        "vtfmm.tvv mt8, v12,  v28\n"

        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"
        "vtfmm.tvv mt8, v13,  v29\n"

        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"
        "vtfmm.tvv mt8, v14,  v30\n"

        "vtse32 %[tss12], (%[p11])\n"
        "add %[p11], %[p11], %[c_stride]\n"
        "addi %[tss12], %[tss12], 1\n"
        "vtfmm.tvv mt8, v15,  v31\n"

        "addi %[blocks], %[blocks], -1\n"
        "beqz %[blocks], 7f\n"

        "sub %[loop], %[b1p], %[b0p]\n"
        "sub %[a0p], %[a0p], %[loop]\n"
        "vle32.v v0,  (%[a0p])\n"
        "add %[b0p], %[b0p], %[loop]\n"
        "vle32.v v16,  (%[b0p])\n"
        "vtzero mt0\n"
        "sub %[a1p], %[a1p], %[loop]\n"
        "add %[b1p], %[b1p], %[loop]\n"

        "vtse32 %[tss8], (%[p10])\n"
        "add %[p10], %[p10], %[c_stride]\n"
        "addi %[tss8], %[tss8], 1\n"
        "vtfmm.tvv mt0, v0, v16\n"

        "vtse32 %[tss8], (%[p10])\n"
        "add %[p10], %[p10], %[c_stride]\n"
        "addi %[tss8], %[tss8], 1\n"
        "vtfmm.tvv mt0, v1, v17\n"

        "vtse32 %[tss8], (%[p10])\n"
        "add %[p10], %[p10], %[c_stride]\n"
        "addi %[tss8], %[tss8], 1\n"
        "vtfmm.tvv mt0, v2, v18\n"

        "vtse32 %[tss8], (%[p10])\n"
        "add %[p10], %[p10], %[c_stride]\n"
        "addi %[tss8], %[tss8], 1\n"
        "vle32.v v24, (%[b1p])\n"
        "vtfmm.tvv mt0, v3, v19\n"

        "vtse32 %[tss8], (%[p10])\n"
        "add %[p10], %[p10], %[c_stride]\n"
        "addi %[tss8], %[tss8], 1\n"
        "vtfmm.tvv mt0, v4, v20\n"

        "vtse32 %[tss8], (%[p10])\n"
        "add %[p10], %[p10], %[c_stride]\n"
        "addi %[tss8], %[tss8], 1\n"
        "vtzero mt4\n"
        "vtfmm.tvv mt0, v5, v21\n"

        "vtse32 %[tss8], (%[p10])\n"
        "add %[p10], %[p10], %[c_stride]\n"
        "addi %[tss8], %[tss8], 1\n"
        "vle32.v v8, (%[a1p])\n"
        "vtfmm.tvv mt0, v6, v22\n"

        "vtse32 %[tss8], (%[p10])\n"
        "add %[p10], %[p10], %[c_stride]\n"
        "addi %[tss8], %[tss8], 1\n"
        "vtfmm.tvv mt0, v7, v23\n"

        // Rows 8..15 overlap the next block's mt4. p10 remains the old
        // block's store pointer until row 15 has issued.
        "vtse32 %[tss8], (%[p10])\n"
        "add %[p10], %[p10], %[c_stride]\n"
        "addi %[tss8], %[tss8], 1\n"
        "vtfmm.tvv mt4, v0, v24\n"

        "vtse32 %[tss8], (%[p10])\n"
        "add %[p10], %[p10], %[c_stride]\n"
        "addi %[tss8], %[tss8], 1\n"
        "vtfmm.tvv mt4, v1, v25\n"

        "vtse32 %[tss8], (%[p10])\n"
        "add %[p10], %[p10], %[c_stride]\n"
        "addi %[tss8], %[tss8], 1\n"
        "vtfmm.tvv mt4, v2, v26\n"

        "vtse32 %[tss8], (%[p10])\n"
        "add %[p10], %[p10], %[c_stride]\n"
        "addi %[tss8], %[tss8], 1\n"
        "vtfmm.tvv mt4, v3, v27\n"

        "vtse32 %[tss8], (%[p10])\n"
        "add %[p10], %[p10], %[c_stride]\n"
        "addi %[tss8], %[tss8], 1\n"
        "vtfmm.tvv mt4, v4, v28\n"

        "vtse32 %[tss8], (%[p10])\n"
        "add %[p10], %[p10], %[c_stride]\n"
        "addi %[tss8], %[tss8], 1\n"
        "vtfmm.tvv mt4, v5, v29\n"

        "vtse32 %[tss8], (%[p10])\n"
        "add %[p10], %[p10], %[c_stride]\n"
        "addi %[tss8], %[tss8], 1\n"
        "vtfmm.tvv mt4, v6, v30\n"

        "vtse32 %[tss8], (%[p10])\n"
        "add %[p10], %[p10], %[c_stride]\n"
        "addi %[tss8], %[tss8], 1\n"
        "vtfmm.tvv mt4, v7, v31\n"

        // The final old mt8 row has issued, so its tile-row selector can now
        // be reset. p10 was retargeted by the last row's spare slot above.

        // preload vle A0
        // vtfmm mt8
        "addi %[a0p], %[a0p], 512\n"
        "vle32.v v0,  (%[a0p])\n"
        "vtzero mt8\n"
        "vtfmm.tvv mt8, v8,  v16\n"
        "addi %[a0p], %[a0p], 512\n"
        "vtfmm.tvv mt8, v9,  v17\n"
        "vtzero mt12\n"
        "vtfmm.tvv mt8, v10,  v18\n"
        "addi %[b0p], %[b0p], 512\n"
        "vtfmm.tvv mt8, v11,  v19\n"
        "addi %[b1p], %[b1p], 512\n"
        "vtfmm.tvv mt8, v12,  v20\n"
        "addi %[a1p], %[a1p], 512\n"
        "vtfmm.tvv mt8, v13,  v21\n"
        "vle32.v v16,  (%[b0p])\n"
        "vtfmm.tvv mt8, v14,  v22\n"
        "addi %[b0p], %[b0p], 512\n"
        "vtfmm.tvv mt8, v15,  v23\n"

        // preload vle A1
        // vtfmm mt12
        "addi %[tss0], %[tss0], -16\n"
        "vtfmm.tvv mt12, v8,  v24\n"
        "slli %[loop], %[c_stride], 4\n"
        "addi %[tss4], %[tss4], -16\n"
        "vtfmm.tvv mt12, v9,  v25\n"
        "addi %[loop], %[loop], -128\n"
        "addi %[tss12], %[tss12], -16\n"
        "vtfmm.tvv mt12, v10, v26\n"
        "addi %[tss8], %[tss8], -16\n"
        "vtfmm.tvv mt12, v11, v27\n"
        "sub %[p00], %[p00], %[loop]\n"
        "vtfmm.tvv mt12, v12, v28\n"
        "sub %[p01], %[p01], %[loop]\n"
        "vtfmm.tvv mt12, v13, v29\n"
        "sub %[p10], %[p10], %[loop]\n"
        "vtfmm.tvv mt12, v14, v30\n"
        "sub %[p11], %[p11], %[loop]\n"
        "vtfmm.tvv mt12, v15, v31\n"
        "j 9b\n"

        // Final block: compact 16-row mt8 store loop.
        "7:\n"
        "li %[loop], 16\n"
        "2:\n"
        "vtse32 %[tss8], (%[p10])\n"
        "add %[p10], %[p10], %[c_stride]\n"
        "addi %[tss8], %[tss8], 1\n"
        "addi %[loop], %[loop], -1\n"
        "bnez %[loop], 2b\n"
        "8:\n"

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
          [p11] "+r"(p11),
          [blocks] "+r"(block_counter),
          [loop] "=&r"(loop_counter)
        :
          [c_stride] "r"(c_stride),
          [group_vl] "r"(group_vl),
          [mtype] "r"(matrix_mtype),
          [vtype] "r"(matrix_vtype),
          [tn] "r"(TN),
          [tm] "r"(TM),
          [tk] "r"(TK)
        : "memory");
}
