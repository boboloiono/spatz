#include "gemm.h"
#include "../data/layer.h"

// The fixed fp32-matmul kernel is the scheduling reference. This descriptor is
// updated by the runtime adapter below before entering that unchanged kernel.
static vme_gemm_layer gemm_block_l;

#define gemm_l gemm_block_l
#define gemm_fp32 gemm_fixed_block
#include "../../fp32-matmul-vtse/kernel/gemm.c"
#undef gemm_fp32
#undef gemm_l

__attribute__((noinline, aligned(64))) void gemm_block_fp32(
    void *addrA, void *addrB, void *addrC,
    int K, int N, int M, int alt_fmt)
{
    gemm_block_l.M = (uint32_t)M;
    gemm_block_l.N = (uint32_t)N;
    gemm_block_l.K = (uint32_t)K;
    gemm_block_l.Mp = (uint32_t)M;
    gemm_block_l.Np = (uint32_t)N;
    gemm_block_l.Kp = (uint32_t)K;
    (void)alt_fmt;

    gemm_fixed_block((float *)addrC, (const float *)addrA,
                     (const float *)addrB, 0, 2);
}
