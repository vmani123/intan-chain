/*
 * pattern.h - the test pattern of CONTRACTS.md section 3, and nothing else.
 *
 * Pure C, no ESP-IDF dependency, header-only. That is on purpose: the FPGA
 * RTL, this firmware and the host checker have to agree bit for bit, so the
 * ESP-side copy is kept in a file that can be compiled and diffed against the
 * other two implementations on a workstation without an IDF install.
 *
 * ORDERING, which CONTRACTS.md leaves implicit: the value at sample_index i
 * is the state AFTER i advances, so sample 0 reports the seed itself,
 * seed(ch) = ch + 1. That is the only reading under which "seeded at reset
 * with seed(ch) = ch + 1" is observable in the data. The FPGA and the host
 * must use the same convention; if either disagrees, fix CONTRACTS.md and all
 * three, do not patch one side.
 */
#ifndef CHAIN_PATTERN_H
#define CHAIN_PATTERN_H

#include <stdint.h>

/* x^16 + x^14 + x^13 + x^11, Fibonacci form folded into the usual mask. */
#define CHAIN_LFSR_TAPS 0xb400u

static inline uint16_t chain_lfsr_seed(unsigned ch)
{
    /* Never 0: an all-zero LFSR is stuck. */
    return (uint16_t)(ch + 1u);
}

static inline uint16_t chain_lfsr_next(uint16_t s)
{
    return (uint16_t)((s >> 1) ^ ((s & 1u) ? CHAIN_LFSR_TAPS : 0u));
}

/* CHECK: the sample value IS the state, reinterpreted as signed 16-bit. */
static inline int16_t chain_value_check(uint16_t state)
{
    return (int16_t)state;
}

/* REALISTIC: triangle wave, phase-shifted per channel, plus 4 bits of the
 * LFSR as dither. Smooth enough that the codec sees a plausible ratio. */
static inline int16_t chain_value_realistic(uint16_t state,
                                            uint32_t sample_index,
                                            unsigned ch)
{
    uint32_t phase = (sample_index + 8u * (uint32_t)ch) & 0xffu;
    int32_t  tri   = (phase < 128u) ? ((int32_t)phase * 128 - 8192)
                                    : ((int32_t)(255u - phase) * 128 - 8192);
    int32_t  noise = (int32_t)(state >> 12) - 8;   /* -8 .. +7 */
    return (int16_t)(tri + noise);
}

#endif /* CHAIN_PATTERN_H */
