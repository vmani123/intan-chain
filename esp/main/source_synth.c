/*
 * source_synth.c - Stage 0 ingress: the ESP makes the data itself.
 *
 * This is the control arm. It builds byte-identical datagrams to the ones the
 * FPGA will send, at a commanded sample rate, so the whole radio path can be
 * characterised before any FPGA exists. When the QSPI ingress later produces
 * a lower number, the difference is the ingress, and that is only a clean
 * statement because this arm shares the pipe, the sink and the socket code
 * with it exactly.
 *
 * TIMESTAMPS IN THIS MODE ARE ESP-SIDE.
 * t_cap and t_tx are filled from esp_timer_get_time(), a local microsecond
 * counter, NOT from the FPGA 96 MHz tick that CONTRACTS.md section 1
 * describes. So t_tx - t_cap is not "time spent inside the FPGA" here; it is
 * near zero and means nothing. Host analysis must treat FPGA-internal latency
 * as undefined whenever the run was made with the synth ingress. The units
 * also differ: microseconds here, 10.417 ns ticks on the FPGA.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "chain_contract.h"
#include "pattern.h"
#include "pipe.h"

static const char *TAG = "synth";

/* Pacing: wake once per millisecond and emit however many datagrams the clock
 * says are due. Not one timer event per datagram -- at 20 kHz a datagram is
 * due every 250 us, and a periodic esp_timer at that rate spends more CPU in
 * interrupt entry and exit than the pacing is worth.
 *
 * vTaskDelayUntil rather than esp_timer plus a semaphore because it is one
 * call with no callback, no ISR context and no extra object, and because the
 * long-run rate does not depend on it being accurate: the number of datagrams
 * due is recomputed each wake from an absolute start time, so tick jitter
 * cannot accumulate into a rate error. It does depend on
 * CONFIG_FREERTOS_HZ=1000, which sdkconfig.defaults sets; at 100 Hz the
 * source would burst 10 ms of traffic at a time.
 *
 * Cost is up to 1 ms of burstiness at the source. The radio aggregates over
 * more than that anyway. */
#define SYNTH_TICK_MS 1

/* Cap on catch-up after a stall, so that a 200 ms Wi-Fi outage does not turn
 * into an 800-datagram burst that overruns the pool all at once and hides the
 * outage behind a pile of drops. */
#define SYNTH_MAX_BURST 32

static uint16_t          s_lfsr[CHAIN_N_CH];
static uint32_t          s_sample_index;   /* global, monotone, never wraps in practice */
static uint32_t          s_seq;            /* frame counter, wraps at 2^32 per contract */

#ifdef CONFIG_CHAIN_SYNTH_REALISTIC
static const bool s_realistic = true;
#else
static const bool s_realistic = false;
#endif

/* Fill one datagram payload: CHAIN_FRAMES_PER_DGRAM frames back to back. The
 * trailer region at the end is left untouched; the sink owns it. */
static void fill_dgram(uint8_t *dst)
{
    for (int f = 0; f < CHAIN_FRAMES_PER_DGRAM; f++) {
        uint8_t *fr = dst + f * CHAIN_RAW16_FRAME;
        uint32_t t  = (uint32_t)esp_timer_get_time();   /* ESP-side, see file header */

        chain_hdr_t h = {
            .magic       = CHAIN_MAGIC,
            .type        = CHAIN_TYPE_RAW16,
            .flags       = s_realistic ? CHAIN_FLAG_REALISTIC : 0,
            .seq         = s_seq++,
            .t_cap       = t,
            .t_tx        = t,
            .n_ch        = CHAIN_N_CH,
            .payload_len = CHAIN_RAW16_PAYLOAD,
        };
        memcpy(fr, &h, sizeof h);

        /* The target is little-endian and the frame stride (276) and header
         * size (20) are both multiples of 4, so the in-memory int16 array is
         * already the wire layout and is correctly aligned. No byte-swapping
         * loop, which at 640 samples per datagram would be visible. */
        int16_t *p = (int16_t *)(void *)(fr + CHAIN_HDR_BYTES);

        if (s_realistic) {
            for (unsigned ch = 0; ch < CHAIN_N_CH; ch++) {
                p[ch] = chain_value_realistic(s_lfsr[ch], s_sample_index, ch);
            }
        } else {
            for (unsigned ch = 0; ch < CHAIN_N_CH; ch++) {
                p[ch] = chain_value_check(s_lfsr[ch]);
            }
        }

        /* Advance every channel exactly once per sample, after the value has
         * been taken, so sample 0 reports the seed. See pattern.h. */
        for (unsigned ch = 0; ch < CHAIN_N_CH; ch++) {
            s_lfsr[ch] = chain_lfsr_next(s_lfsr[ch]);
        }
        s_sample_index++;
    }
}

static void synth_task(void *arg)
{
    (void)arg;

    for (unsigned ch = 0; ch < CHAIN_N_CH; ch++) {
        s_lfsr[ch] = chain_lfsr_seed(ch);
    }

    const uint32_t sample_hz  = CONFIG_CHAIN_SYNTH_SAMPLE_HZ;
    /* Period in nanoseconds so the integer division does not quantise the
     * rate: at 20 kHz the datagram period is 250000 ns exactly, but at, say,
     * 30 kHz it is 166666.67 ns and rounding in microseconds would be a 0.2 %
     * rate error, which is larger than the effect being looked for. */
    const uint64_t period_ns  = (1000000000ULL * CHAIN_FRAMES_PER_DGRAM) / sample_hz;

    ESP_LOGI(TAG, "%u Hz/ch, %s, %llu ns per datagram, %u dgram/s offered",
             (unsigned)sample_hz, s_realistic ? "REALISTIC" : "CHECK",
             (unsigned long long)period_ns,
             (unsigned)(sample_hz / CHAIN_FRAMES_PER_DGRAM));

    const uint64_t t0 = (uint64_t)esp_timer_get_time() * 1000ULL;
    uint64_t produced = 0;
    TickType_t next = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&next, pdMS_TO_TICKS(SYNTH_TICK_MS));

        uint64_t now_ns = (uint64_t)esp_timer_get_time() * 1000ULL;
        uint64_t due    = (now_ns - t0) / period_ns;

        uint32_t n = 0;
        while (produced < due && n < SYNTH_MAX_BURST) {
            produced++;
            n++;

            pipe_buf_t *b = pipe_acquire(0);
            if (!b) {
                /* Pool empty: the sink is behind. Drop this datagram and stay
                 * on schedule. Slowing down instead would quietly lower the
                 * offered load and make the radio look better than it is.
                 * The drop is already counted inside pipe_acquire. Counting
                 * it against the burst cap too, so that a long stall cannot
                 * turn into an unbounded spin through the backlog. */
                continue;
            }
            fill_dgram(b->data);
            b->rx_us = (uint32_t)esp_timer_get_time();
            pipe_submit(b);
        }

        /* Fell too far behind to catch up inside the burst cap: give up on the
         * backlog rather than chase it forever. Skipped datagrams are already
         * counted as acquire failures when the pool was the cause; here the
         * cause is the source itself, which the log line below makes explicit. */
        if (produced < due) {
            uint64_t skipped = due - produced;
            produced = due;
            ESP_LOGW(TAG, "source behind by %llu datagrams, resynchronised",
                     (unsigned long long)skipped);
        }
    }
}

void source_synth_start(void)
{
    /* Core 1, priority 8. Core 1 is the quiet one: Wi-Fi, lwIP and the sink
     * all live on core 0, so the source never contends with the radio for CPU
     * and the two halves of the measurement stay separable. */
    xTaskCreatePinnedToCore(synth_task, "synth", 4096, NULL, 8, NULL, 1);
}
