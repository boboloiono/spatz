#pragma once

#include <stdint.h>

static inline uint32_t gemm_cycle(void) {
    uint32_t value;
    asm volatile("csrr %0, mcycle" : "=r"(value));
    return value;
}

// Runtime M/N/K panel kernel. The general driver passes one 32-row core panel
// and a padded N/K chunk; the tuned 64x64x64 schedule remains unchanged.
void gemm_block_fp32(void *addrA, void *addrB, void *addrC,
                     int K, int N, int M, int alt_fmt);
