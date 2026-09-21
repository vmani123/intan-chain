/*
 * echo.c - UDP echo responder on CHAIN_ECHO_PORT.
 *
 * Bounces whatever arrives straight back to the sender. The host uses it to
 * estimate the one-way air latency and the ESP-to-host clock offset, which is
 * what makes the esp_tx_us field in the datagram trailer comparable with a
 * host arrival timestamp at all.
 *
 * Priority 2, below the sink (6) and the source (8), and below the default.
 * It must not be able to starve the data path: this task exists to measure
 * the data path, and a probe that perturbs what it measures is worse than no
 * probe. The consequence is that a round-trip time measured here is an UPPER
 * bound -- it includes however long this task waited for the CPU under load
 * -- so the host should take the minimum over many probes, not the mean.
 *
 * Stack and buffer are small and static. Nothing here allocates while the
 * data path is running.
 */

#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "esp_log.h"

#include "chain_contract.h"

static const char *TAG = "echo";

/* Probes are tiny. Anything larger than this is not a probe and is ignored by
 * being truncated, which is visible to the host as a short reply. */
#define ECHO_MAX 128

static void echo_task(void *arg)
{
    (void)arg;
    static uint8_t buf[ECHO_MAX];

    int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) {
        ESP_LOGE(TAG, "socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in me = { 0 };
    me.sin_family      = AF_INET;
    me.sin_port        = htons(CHAIN_ECHO_PORT);
    me.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(fd, (struct sockaddr *)&me, sizeof me) < 0) {
        ESP_LOGE(TAG, "bind %d: errno %d", (int)CHAIN_ECHO_PORT, errno);
        close(fd);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "listening on UDP %d", (int)CHAIN_ECHO_PORT);

    for (;;) {
        struct sockaddr_in from;
        socklen_t flen = sizeof from;
        int n = recvfrom(fd, buf, sizeof buf, 0, (struct sockaddr *)&from, &flen);
        if (n <= 0) {
            continue;
        }
        sendto(fd, buf, n, 0, (struct sockaddr *)&from, flen);
    }
}

void echo_start(void)
{
    xTaskCreatePinnedToCore(echo_task, "echo", 3072, NULL, 2, NULL, 0);
}
