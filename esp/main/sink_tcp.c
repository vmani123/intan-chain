/*
 * sink_tcp.c - the egress path over TCP, with a deep spool (CONFIG_CHAIN_SINK_TCP).
 *
 * Same records as sink_udp.c -- the FPGA's 1380-byte payload plus the 12-byte
 * trailer -- written back to back on one TCP connection to CONFIG_CHAIN_HOST_IP
 * on the data port. The point is a stream with no holes: Wi-Fi on a busy
 * channel goes quiet for tenths of a second at a time, and UDP loses whatever
 * the radio could not send meanwhile.
 *
 * Two tasks, with a spool between them:
 *
 *   pipe (16 DMA buffers) --spool_task--> spool (PSRAM, SPOOL slots) --sender_task--> TCP
 *
 * spool_task copies every datagram out of the DMA pipe into PSRAM at once and
 * hands the DMA buffer straight back, so the QSPI receiver always has buffers
 * armed and the FPGA never stalls behind the radio (its FIFO is 4 datagrams,
 * 10 ms at 2 kS/s -- far too short to ride out an outage). sender_task blocks
 * in send() for as long as the air is busy; TCP retransmits underneath; the
 * spool absorbs the backlog (CONFIG_CHAIN_TCP_SPOOL_DGRAMS, 2000 = 5 s at
 * 400 datagrams/s) and drains faster than real time when the air clears.
 * Only an outage longer than the spool loses data, and that is counted
 * (txerr on the 1 Hz pipe line, "drops" on the 1 Hz tcp line).
 *
 * Records are fixed size, so the host cuts the stream by count from the start
 * of each connection. If a send fails, the connection is dropped and the same
 * record is sent again, whole, on the next one; the host discards the partial
 * copy from the old connection. While nobody is listening the spool is emptied
 * (counted), so a viewer that connects late starts on live data, not on a
 * backlog from before it was running.
 */

#include <string.h>
#include <errno.h>
#include <fcntl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "chain_contract.h"
#include "pipe.h"

static const char *TAG = "tcp";

/* CHAIN_TCP_SPOOL_DGRAMS exists only when CHAIN_SINK_TCP is set, but this file
 * is compiled in every build (so the UDP builds still type-check it, as with
 * the two sources). The UDP build never calls sink_tcp_start(). */
#ifndef CONFIG_CHAIN_TCP_SPOOL_DGRAMS
#define CONFIG_CHAIN_TCP_SPOOL_DGRAMS 2000
#endif

/* send() blocked this long means the link is gone, not slow. Long, because a
 * slow link is exactly what the spool is for: reconnecting would throw away
 * the bytes already queued in the socket. */
#define SEND_TIMEOUT_MS     20000
#define CONNECT_TIMEOUT_MS   1000
#define RECONNECT_GAP_MS      300
#define FALLBACK_SLOTS         24   /* internal RAM, if there is no PSRAM */

static uint8_t      *s_spool;
static uint32_t      s_slots;
static QueueHandle_t s_free_q;     /* uint16_t slot indices */
static QueueHandle_t s_fill_q;     /* uint16_t slot indices, oldest first */

static volatile uint32_t s_spool_drops;   /* spool full: datagram lost */
static volatile uint32_t s_spool_high;    /* high-water, slots, since last report */
static volatile uint32_t s_idle_drops;    /* discarded while no host was connected */

static inline uint8_t *slot(uint16_t i)
{
    return s_spool + (size_t)i * CHAIN_DGRAM_BYTES;
}

static struct sockaddr_in host_addr(void)
{
    struct sockaddr_in dst = { 0 };
    dst.sin_family      = AF_INET;
    dst.sin_port        = htons(CHAIN_UDP_PORT);
    dst.sin_addr.s_addr = inet_addr(CONFIG_CHAIN_HOST_IP);
    return dst;
}

/* Non-blocking connect with a timeout: a blocking lwIP connect to a host that
 * does not answer waits out every SYN retry. Returns a connected, blocking
 * socket, or -1. */
static int open_conn(void)
{
    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0) {
        return -1;
    }
    int one = 1;
    /* No Nagle: a record leaves when written, not a round trip later. */
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    struct timeval tv = { .tv_sec = SEND_TIMEOUT_MS / 1000,
                          .tv_usec = (SEND_TIMEOUT_MS % 1000) * 1000 };
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);

    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    struct sockaddr_in dst = host_addr();
    int r = connect(fd, (struct sockaddr *)&dst, sizeof dst);
    if (r != 0 && errno != EINPROGRESS) {
        close(fd);
        return -1;
    }
    if (r != 0) {
        fd_set wfds;
        FD_ZERO(&wfds);
        FD_SET(fd, &wfds);
        struct timeval ct = { .tv_sec = CONNECT_TIMEOUT_MS / 1000,
                              .tv_usec = (CONNECT_TIMEOUT_MS % 1000) * 1000 };
        int soerr = 0;
        socklen_t len = sizeof soerr;
        if (select(fd + 1, NULL, &wfds, NULL, &ct) != 1
            || getsockopt(fd, SOL_SOCKET, SO_ERROR, &soerr, &len) != 0
            || soerr != 0) {
            close(fd);
            return -1;
        }
    }
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    return fd;
}

/* All of it or an error: a short write left in place would shift every
 * record after it on the host. */
static int send_all(int fd, const uint8_t *p, int len)
{
    int done = 0;
    while (done < len) {
        int n = send(fd, p + done, len - done, 0);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }
        done += n;
    }
    return done;
}

/* pipe -> spool, immediately. Never waits on the network. */
static void spool_task(void *arg)
{
    (void)arg;
    for (;;) {
        pipe_buf_t *b = pipe_take(1000);
        if (!b) {
            continue;
        }
        uint16_t i;
        if (xQueueReceive(s_free_q, &i, 0) != pdTRUE) {
            s_spool_drops++;
            pipe_note_sent(b, -1);          /* counted: the spool overflowed */
            pipe_recycle(b);
            continue;
        }
        uint8_t *p = slot(i);
        memcpy(p, b->data, CHAIN_DGRAM_PAYLOAD);
        chain_trailer_t t = { 0 };
        t.esp_rx_us = b->rx_us;
        memcpy(p + CHAIN_DGRAM_PAYLOAD, &t, CHAIN_TRAILER_BYTES);
        pipe_recycle(b);
        xQueueSend(s_fill_q, &i, 0);        /* cannot fail: queues hold every slot */
        uint32_t d = (uint32_t)uxQueueMessagesWaiting(s_fill_q);
        if (d > s_spool_high) {
            s_spool_high = d;
        }
    }
}

/* Empty the spool, counting every record: there is no host to send it to. */
static void discard_spool(void)
{
    uint16_t i;
    while (xQueueReceive(s_fill_q, &i, 0) == pdTRUE) {
        pipe_note_sent(NULL, -1);
        s_idle_drops++;
        xQueueSend(s_free_q, &i, 0);
    }
}

static void report(int connected)
{
    ESP_LOGI(TAG, "%s | spool now %u high %u of %u | spool drops %u | dropped while unconnected %u",
             connected ? "connected" : "no host",
             (unsigned)uxQueueMessagesWaiting(s_fill_q), (unsigned)s_spool_high,
             (unsigned)s_slots, (unsigned)s_spool_drops, (unsigned)s_idle_drops);
    s_spool_high = (uint32_t)uxQueueMessagesWaiting(s_fill_q);
}

/* spool -> TCP. Blocks in send() while the air is busy; that is the design. */
static void sender_task(void *arg)
{
    (void)arg;
    int      fd = -1;
    uint32_t failures = 0;
    int      have = 0;          /* cur holds a record not yet sent in full */
    uint16_t cur = 0;
    int64_t  next_report = esp_timer_get_time() + 1000000;

    for (;;) {
        int64_t now = esp_timer_get_time();
        if (now >= next_report) {
            report(fd >= 0);
            next_report = now + 1000000;
        }

        if (fd < 0) {
            fd = open_conn();
            if (fd < 0) {
                if (failures++ % 20 == 0) {
                    ESP_LOGW(TAG, "no listener at %s:%d (start the viewer with --tcp); "
                             "retrying, data discarded meanwhile",
                             CONFIG_CHAIN_HOST_IP, (int)CHAIN_UDP_PORT);
                }
                if (have) {
                    pipe_note_sent(NULL, -1);
                    s_idle_drops++;
                    xQueueSend(s_free_q, &cur, 0);
                    have = 0;
                }
                discard_spool();
                vTaskDelay(pdMS_TO_TICKS(RECONNECT_GAP_MS));
                continue;
            }
            failures = 0;
            ESP_LOGI(TAG, "connected to %s:%d, spool %u datagrams",
                     CONFIG_CHAIN_HOST_IP, (int)CHAIN_UDP_PORT, (unsigned)s_slots);
        }

        if (!have) {
            if (xQueueReceive(s_fill_q, &cur, pdMS_TO_TICKS(200)) != pdTRUE) {
                continue;
            }
            have = 1;
        }

        uint8_t *p = slot(cur);
        chain_trailer_t t;
        memcpy(&t, p + CHAIN_DGRAM_PAYLOAD, sizeof t);
        t.esp_tx_us  = (uint32_t)esp_timer_get_time();
        t.pipe_depth = (uint32_t)uxQueueMessagesWaiting(s_fill_q) * CHAIN_DGRAM_BYTES;
        memcpy(p + CHAIN_DGRAM_PAYLOAD, &t, sizeof t);

        int n = send_all(fd, p, CHAIN_DGRAM_BYTES);
        int err = errno;
        if (n < 0) {
            /* Part of this record may be on the dead connection; the host drops
             * that partial copy, and `have` keeps the record for the next one. */
            ESP_LOGW(TAG, "send failed (errno %d)%s, reconnecting", err,
                     err == EAGAIN ? ": blocked > 20 s" : "");
            close(fd);
            fd = -1;
            continue;
        }

        /* THE accounting point for this build. Nothing else counts bytes. */
        pipe_note_sent(NULL, n);
        xQueueSend(s_free_q, &cur, 0);
        have = 0;
    }
}

void sink_tcp_start(void)
{
    s_slots = CONFIG_CHAIN_TCP_SPOOL_DGRAMS;
    s_spool = heap_caps_malloc((size_t)s_slots * CHAIN_DGRAM_BYTES,
                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    int in_psram = s_spool != NULL;
    if (!s_spool) {
        ESP_LOGW(TAG, "no PSRAM for a %u-datagram spool; falling back to %d in internal RAM "
                 "(outages over ~%d ms will lose data)",
                 (unsigned)s_slots, FALLBACK_SLOTS, FALLBACK_SLOTS * 1000 / 400);
        s_slots = FALLBACK_SLOTS;
        s_spool = heap_caps_malloc((size_t)s_slots * CHAIN_DGRAM_BYTES, MALLOC_CAP_8BIT);
    }
    s_free_q = xQueueCreate(s_slots, sizeof(uint16_t));
    s_fill_q = xQueueCreate(s_slots, sizeof(uint16_t));
    if (!s_spool || !s_free_q || !s_fill_q) {
        ESP_LOGE(TAG, "spool allocation failed");
        abort();
    }
    for (uint16_t i = 0; i < s_slots; i++) {
        xQueueSend(s_free_q, &i, 0);
    }
    ESP_LOGI(TAG, "spool %u datagrams (%u kB) in %s, to %s:%d",
             (unsigned)s_slots, (unsigned)(s_slots * CHAIN_DGRAM_BYTES / 1024),
             in_psram ? "PSRAM" : "internal RAM",
             CONFIG_CHAIN_HOST_IP, (int)CHAIN_UDP_PORT);

    /* spool_task above the sender, so the DMA pipe empties promptly even while
     * the sender is busy; both below lwIP (18) and the Wi-Fi task (23). */
    xTaskCreatePinnedToCore(spool_task, "spool", 3072, NULL, 7, NULL, 0);
    xTaskCreatePinnedToCore(sender_task, "sink", 4096, NULL, 6, NULL, 0);
}
