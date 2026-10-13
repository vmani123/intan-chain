/* pipe.c - see pipe.h. Bytes are counted only in pipe_note_sent(). */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#include "pipe.h"

static const char *TAG = "pipe";

static pipe_buf_t    s_bufs[PIPE_N_BUFS];
static QueueHandle_t s_free;     /* pipe_buf_t * */
static QueueHandle_t s_filled;   /* pipe_buf_t * */

/* A spinlock, not a mutex: the critical sections are a handful of adds and
 * are entered from two different cores. A mutex would add a context switch to
 * the per-datagram cost we are trying to measure. */
static portMUX_TYPE  s_lock = portMUX_INITIALIZER_UNLOCKED;
static pipe_stats_t  s_st;

esp_err_t pipe_init(void)
{
    s_free   = xQueueCreate(PIPE_N_BUFS, sizeof(pipe_buf_t *));
    s_filled = xQueueCreate(PIPE_N_BUFS, sizeof(pipe_buf_t *));
    if (!s_free || !s_filled) {
        return ESP_ERR_NO_MEM;
    }

    for (int i = 0; i < PIPE_N_BUFS; i++) {
        /* MALLOC_CAP_DMA forces internal RAM: SPI DMA cannot reach PSRAM on
         * the S3 without extra alignment rules, and PSMRAM latency would show
         * up in the measurement anyway. 64-byte alignment rather than the
         * 4-byte minimum so a buffer never shares a cache line with another,
         * which matters once DMA and the CPU touch different buffers at the
         * same time. */
        uint8_t *p = heap_caps_aligned_alloc(64, CHAIN_DGRAM_BYTES,
                                             MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        if (!p) {
            ESP_LOGE(TAG, "only %d of %d buffers fit in DMA RAM", i, PIPE_N_BUFS);
            return ESP_ERR_NO_MEM;
        }
        memset(p, 0, CHAIN_DGRAM_BYTES);
        s_bufs[i].data  = p;
        s_bufs[i].rx_us = 0;

        pipe_buf_t *b = &s_bufs[i];
        xQueueSend(s_free, &b, 0);
    }

    memset(&s_st, 0, sizeof s_st);
    ESP_LOGI(TAG, "%d buffers x %d bytes = %d bytes of DMA RAM",
             PIPE_N_BUFS, (int)CHAIN_DGRAM_BYTES,
             PIPE_N_BUFS * (int)CHAIN_DGRAM_BYTES);
    return ESP_OK;
}

pipe_buf_t *pipe_try_acquire(void)
{
    pipe_buf_t *b = NULL;
    return xQueueReceive(s_free, &b, 0) == pdTRUE ? b : NULL;
}

pipe_buf_t *pipe_acquire(uint32_t wait_ms)
{
    pipe_buf_t *b = NULL;
    if (xQueueReceive(s_free, &b, pdMS_TO_TICKS(wait_ms)) != pdTRUE) {
        portENTER_CRITICAL(&s_lock);
        s_st.acquire_fails++;
        portEXIT_CRITICAL(&s_lock);
        return NULL;
    }
    return b;
}

void pipe_submit(pipe_buf_t *b)
{
    /* The filled queue is exactly as deep as the pool, so this cannot block:
     * a buffer being submitted was, by construction, taken off the free queue
     * and so a filled slot is guaranteed free for it. */
    xQueueSend(s_filled, &b, 0);

    UBaseType_t d = uxQueueMessagesWaiting(s_filled);
    portENTER_CRITICAL(&s_lock);
    if ((uint32_t)d > s_st.depth_max) {
        s_st.depth_max = (uint32_t)d;
    }
    portEXIT_CRITICAL(&s_lock);
}

pipe_buf_t *pipe_take(uint32_t wait_ms)
{
    pipe_buf_t *b = NULL;
    if (xQueueReceive(s_filled, &b, pdMS_TO_TICKS(wait_ms)) != pdTRUE) {
        return NULL;
    }
    return b;
}

void pipe_recycle(pipe_buf_t *b)
{
    xQueueSend(s_free, &b, 0);
}

void pipe_note_sent(const pipe_buf_t *b, int sendto_ret)
{
    (void)b;
    portENTER_CRITICAL(&s_lock);
    if (sendto_ret < 0) {
        s_st.tx_errors++;
    } else {
        s_st.dgrams_sent++;
        s_st.bytes_sent += (uint64_t)sendto_ret;
    }
    portEXIT_CRITICAL(&s_lock);
}

uint32_t pipe_depth_bytes(void)
{
    return (uint32_t)uxQueueMessagesWaiting(s_filled) * (uint32_t)CHAIN_DGRAM_BYTES;
}

void pipe_stats(pipe_stats_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_st;
    portEXIT_CRITICAL(&s_lock);
}

static void report_task(void *arg)
{
    (void)arg;
    pipe_stats_t prev;
    pipe_stats(&prev);
    TickType_t next = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&next, pdMS_TO_TICKS(1000));

        pipe_stats_t now;
        pipe_stats(&now);

        uint64_t dbytes  = now.bytes_sent  - prev.bytes_sent;
        uint64_t ddgrams = now.dgrams_sent - prev.dgrams_sent;

        /* kbit/s of UDP payload, trailer included, IP/UDP/802.11 headers not.
         * That is the number the host counts too, so the two are comparable
         * without first agreeing on header overhead.
         * Integer maths on purpose: newlib-nano, which some IDF configs
         * select, prints %f as a literal "f", and a silently wrong headline
         * number is the worst possible bug in a measurement rig. */
        uint32_t kbit = (uint32_t)(dbytes * 8ULL / 1000ULL);

        ESP_LOGI(TAG,
                 "%6llu dgram/s  %5u.%03u Mbit/s  depth_max %2u  drops %u  txerr %u  heap %u",
                 (unsigned long long)ddgrams,
                 (unsigned)(kbit / 1000u), (unsigned)(kbit % 1000u),
                 (unsigned)now.depth_max,
                 (unsigned)now.acquire_fails,
                 (unsigned)now.tx_errors,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

        prev = now;
    }
}

void pipe_run(void)
{
    /* Priority 1, core 0: reporting must never be the reason a datagram is
     * late. It runs once a second and does nothing but format a line. */
    xTaskCreatePinnedToCore(report_task, "pipe_rep", 3072, NULL, 1, NULL, 0);
}
