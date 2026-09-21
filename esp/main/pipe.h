/*
 * pipe.h - the fixed buffer pool between ingress and the radio.
 *
 * A pool of PIPE_N_BUFS DMA-capable buffers, each exactly CHAIN_DGRAM_BYTES,
 * allocated once at init and never freed. Two queues carry pointers to them:
 *
 *      free   --pipe_acquire-->  source  --pipe_submit-->  filled
 *      filled --pipe_take----->  sink    --pipe_recycle->  free
 *
 * Two queues rather than one ring so that the source and the sink can sit on
 * different cores and a Wi-Fi stall blocks only the sink: ingress keeps
 * running until the pool is empty, and then reports a drop instead of
 * back-pressuring the FPGA invisibly. A drop that is counted is data; a stall
 * that is not counted is a lie about where the ceiling is.
 *
 * Nothing in the data path allocates. pipe_acquire and pipe_take are a single
 * FreeRTOS queue operation each.
 *
 *
 * WHERE BYTES ARE COUNTED
 * -----------------------
 * Throughput is counted in exactly ONE place: pipe_note_sent(), called by the
 * sink once sendto() has returned, that is once the buffer has been handed to
 * the socket layer. Nowhere else in this firmware increments a byte counter.
 *
 * Moving that call changes what the published number means:
 *   - call it before sendto() and you are measuring the source, not the radio
 *   - call it in the source and you are measuring memcpy
 *   - count it on the host instead and you are measuring the air plus the
 *     host's receive path, which is a different and also useful number, but
 *     not this one
 * So: one call site, in sink_udp.c, immediately after sendto().
 */
#ifndef PIPE_H
#define PIPE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "chain_contract.h"

/* 16 buffers is 16 x 1392 = 22 kB of internal DMA-capable RAM. Enough to ride
 * out a ~4 ms Wi-Fi stall at 4000 datagrams/s, small enough that a longer
 * stall shows up as a drop rather than as latency, which is what we want to
 * see. */
#define PIPE_N_BUFS 16

typedef struct {
    uint8_t *data;    /* CHAIN_DGRAM_BYTES, DMA-capable, 4-byte aligned */
    uint32_t rx_us;   /* esp_timer_get_time() when the source finished it */
} pipe_buf_t;

typedef struct {
    uint64_t dgrams_sent;   /* sendto() returned >= 0 */
    uint64_t bytes_sent;    /* the number. Only pipe_note_sent writes it */
    uint32_t tx_errors;     /* sendto() returned < 0 */
    uint32_t acquire_fails; /* source wanted a buffer, pool was empty */
    uint32_t depth_max;     /* high-water mark of the filled queue, in slots */
} pipe_stats_t;

esp_err_t pipe_init(void);

/* Starts the 1 Hz reporting task. Separate from pipe_init so that app_main
 * can bring the radio up first and keep the log in chronological order. */
void pipe_run(void);

/* Source side. Returns NULL on timeout and counts an acquire failure; the
 * caller should drop the datagram and stay on its schedule rather than slow
 * down, so that the offered load stays at the commanded rate. */
pipe_buf_t *pipe_acquire(uint32_t wait_ms);
void        pipe_submit(pipe_buf_t *b);

/* Sink side. */
pipe_buf_t *pipe_take(uint32_t wait_ms);
void        pipe_recycle(pipe_buf_t *b);

/* The one throughput accounting point. sendto_ret is the raw return of
 * sendto(): >= 0 counts bytes, < 0 counts a tx error and no bytes. */
void pipe_note_sent(const pipe_buf_t *b, int sendto_ret);

/* Bytes sitting in the filled queue right now. Goes into the datagram
 * trailer, so it has to be readable from the sink without taking a lock that
 * the source also wants. */
uint32_t pipe_depth_bytes(void);

void pipe_stats(pipe_stats_t *out);

#endif /* PIPE_H */
