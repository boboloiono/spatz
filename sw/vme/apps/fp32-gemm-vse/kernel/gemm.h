#pragma once

#include <stdint.h>

static inline uint32_t gemm_cycle(void) {
    uint32_t value;
    asm volatile("csrr %0, mcycle" : "=r"(value));
    return value;
}

// Tuned two-core kernel for exactly one packed M64xN64xK64 panel. Each core
// calls it with ti_lo = 2 * core_id.
void gemm_fp32(float *C, const float *Apack, const float *Bpack,
               uint32_t ti_lo, uint32_t ti_hi);

// Runtime tail kernel. Each call handles at most one 32x64x64 per-core panel;
// A/B/C retain the physical 64x64x64 packed-panel strides.
void gemm_block_fp32(void *addrA, void *addrB, void *addrC,
                     int K, int N, int M, int alt_fmt);
