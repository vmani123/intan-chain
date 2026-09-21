/*
 * sink_udp.c - the whole egress path.
 *
 * Take a filled buffer, write the 12-byte trailer into its last
 * CHAIN_TRAILER_BYTES, sendto() the whole datagram, count it, give the buffer
 * back. That is the entire job. The ESP never reframes, never parses a frame,
 * never allocates here -- so whatever throughput this rig reports cannot be
 * credited to, or blamed on, clever ESP-side work.
 *
 * pipe_note_sent() is called from here and from nowhere else in the firmware.
 * See the long comment in pipe.h about why the position of that call is part
 * of the definition of the number.
 */

#include <string.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "chain_contract.h"
#include "pipe.h"

static const char *TAG = "sink";

static void sink_task(void *arg)
{
    (void)arg;

    int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) {
        ESP_LOGE(TAG, "socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in dst = { 0 };
    dst.sin_family      = AF_INET;
    dst.sin_port        = htons(CHAIN_UDP_PORT);
    dst.sin_addr.s_addr = inet_addr(CONFIG_CHAIN_HOST_IP);

    /* Unicast to a fixed host, connect() not used: sendto() with an explicit
     * address is one call either way, and leaving the socket unconnected
     * means a wrong CONFIG_CHAIN_HOST_IP shows up as traffic on the air that
     * nobody answers, rather than as a socket error at startup. Easier to see
     * with a capture, which is how this rig gets debugged. */
    ESP_LOGI(TAG, "streaming to %s:%d, %d bytes per datagram",
             CONFIG_CHAIN_HOST_IP, (int)CHAIN_UDP_PORT, (int)CHAIN_DGRAM_BYTES);

    for (;;) {
        pipe_buf_t *b = pipe_take(1000);
        if (!b) {
            continue;   /* idle; the reporting task will show 0 dgram/s */
        }

        chain_trailer_t t;
        t.esp_rx_us  = b->rx_us;
        t.esp_tx_us  = (uint32_t)esp_timer_get_time();
        t.pipe_depth = pipe_depth_bytes();
        memcpy(b->data + CHAIN_DGRAM_PAYLOAD, &t, CHAIN_TRAILER_BYTES);

        int n = sendto(fd, b->data, CHAIN_DGRAM_BYTES, 0,
                       (struct sockaddr *)&dst, sizeof dst);

        /* THE accounting point. Nothing else counts bytes. */
        pipe_note_sent(b, n);

        pipe_recycle(b);

        if (n < 0 && errno == ENOMEM) {
            /* lwIP is out of pbufs, which means the Wi-Fi driver has not
             * drained its queue yet. One tick (1 ms at CONFIG_FREERTOS_HZ
             * 1000) lets it. Spinning instead would starve the very task that
             * has to free the buffers, and turn a 1 ms hiccup into a lockup. */
            vTaskDelay(1);
        }
    }
}

void sink_udp_start(void)
{
    /* Core 0, priority 6. Core 0 because the Wi-Fi task and the lwIP task
     * live there: keeping the sink on the same core keeps the pbuf hand-off
     * out of the cross-core queue. Priority 6 is above the default so a
     * backlog drains promptly, and well below the Wi-Fi driver task (23) and
     * lwIP (18) so it can never delay the thing it is feeding. */
    xTaskCreatePinnedToCore(sink_task, "sink", 4096, NULL, 6, NULL, 0);
}
