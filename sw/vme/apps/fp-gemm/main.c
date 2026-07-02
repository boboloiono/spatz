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

// main.c — fp32×fp32=fp32 VME GEMM functional check

#include <snrt.h>
#include <string.h>
#include <printf.h>

#include DATAHEADER
#include "kernel/gemm.c"

int main(void)
{
    if (snrt_cluster_core_idx() != 0) {
        snrt_cluster_hw_barrier();
        return 0;
    }

    // enable_vec();
    // enable_fp();
    // enable_vme();

    const uint32_t M  = gemm_l.M;
    const uint32_t N  = gemm_l.N;
    const uint32_t K  = gemm_l.K;
    const uint32_t TM = gemm_l.TM;
    const uint32_t TN = gemm_l.TN;

    float *A    = (float *)snrt_l1alloc(M * K * sizeof(float));
    float *B    = (float *)snrt_l1alloc(K * N * sizeof(float));
    float *C    = (float *)snrt_l1alloc(M * N * sizeof(float));
    float *Cref = (float *)snrt_l1alloc(M * N * sizeof(float));

    snrt_dma_start_1d(A,    gemm_A_dram, M * K * sizeof(float));
    snrt_dma_start_1d(B,    gemm_B_dram, K * N * sizeof(float));
    snrt_dma_start_1d(Cref, gemm_C_dram, M * N * sizeof(float));
    snrt_dma_wait_all();

    memset(C, 0, M * N * sizeof(float));

    uint32_t t0 = get_cycle();
    gemm_fp32(C, A, B, M, N, K, TM, TN);
    uint32_t t1 = get_cycle();
    uint32_t cycles = t1 - t0;

    // Element-wise error check.
    // fdiv.s is not supported by the Spatz FPU; use fmul.s to avoid it.
    // Instead of rel_err = abs_err / den > tol, check abs_err > tol * den.
    int   n_err   = 0;
    float max_abs = 0.0f;
    const float tol = 1.0e-3f;

    uint32_t row_errs[64] = {0};
    for (uint32_t idx = 0; idx < M * N; idx++) {
        float diff    = C[idx] - Cref[idx];
        float abs_err = diff      < 0.0f ? -diff      : diff;
        float den     = Cref[idx] < 0.0f ? -Cref[idx] : Cref[idx];
        if (den < 1.0e-9f) den = 1.0e-9f;
        if (abs_err > max_abs) max_abs = abs_err;
        if (abs_err > tol && abs_err > tol * den) {
            row_errs[idx / N]++;
            n_err++;
        }
    }
    for (uint32_t r = 0; r < M; r++) {
        if (row_errs[r])
            printf("row %u: %u errors\n", (unsigned)r, (unsigned)row_errs[r]);
    }

    printf("\n=== VME fp32 GEMM %ux%ux%u (TM=%u TN=%u) ===\n",
           (unsigned)M, (unsigned)N, (unsigned)K,
           (unsigned)TM, (unsigned)TN);
    printf("Cycles:    %u\n", (unsigned)cycles);
    printf("MaxAbsErr: 0x%08x\n", *(unsigned *)&max_abs);
    printf("Errors:    %d\n",   n_err);
    printf("Status:    %s\n",   n_err == 0 ? "PASS" : "FAIL");

    snrt_cluster_hw_barrier();
    return n_err == 0 ? 0 : 1;
}