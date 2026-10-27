/*
 * sink_check.c - the link-test egress: check every datagram here, send none.
 *
 * Built in only with CONFIG_CHAIN_LINK_TEST_NO_WIFI. The radio never starts,
 * so the QSPI link can be proven before the node is allowed on the network
 * (CMU-DEVICE admits registered MACs only) and with no Wi-Fi current on the
 * shared ground while it is debugged.
 *
 * Each datagram is five frames. A frame passes when its header carries the
 * magic, RAW16, 128 channels and a 256-byte payload, and its seq is one more
 * than the frame before it. Those fields sit at fixed offsets across the
 * whole 1380 bytes, so a slave latching one bit early or late, a swapped data
 * line or a short segment fails every datagram here at once -- the same
 * signature the host's CHECK scoring would show, without needing the host.
 *
 * A seq jump is counted, not failed: the FPGA drops whole datagrams when its
 * FIFO is full (OVERRUNS), and seq is how that is seen from this end. A seq
 * that goes backwards is a restart (clear_counters zeroes it), not a loss. The
 * sample values are not checked; the host does that bit-exact once Wi-Fi is
 * up, and the header check is what tells a bus fault from a radio fault.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "chain_contract.h"
#include "pipe.h"

static const char *TAG = "link";

static bool header_ok(const chain_hdr_t *h)
{
    return h->magic == CHAIN_MAGIC && h->type == CHAIN_TYPE_RAW16 &&
           h->n_ch == CHAIN_N_CH && h->payload_len == CHAIN_RAW16_PAYLOAD;
}

static void check_task(void *arg)
{
    (void)arg;
    uint32_t dgrams = 0, frames_ok = 0, frames_bad = 0, seq_gaps = 0, lost = 0;
    uint32_t restarts = 0;
    uint32_t last_dgrams = 0, last_seq = 0;
    bool have_seq = false, dumped = false;
    int64_t next_report = esp_timer_get_time() + 1000000;

    ESP_LOGW(TAG, "LINK TEST: Wi-Fi off; each datagram is checked here and dropped");

    for (;;) {
        pipe_buf_t *b = pipe_take(100);
        if (b) {
            dgrams++;
            for (int f = 0; f < CHAIN_FRAMES_PER_DGRAM; f++) {
                chain_hdr_t h;
                memcpy(&h, b->data + f * CHAIN_RAW16_FRAME, sizeof h);
                if (!header_ok(&h)) {
                    frames_bad++;
                    if (!dumped) {
                        const uint8_t *p = b->data + f * CHAIN_RAW16_FRAME;
                        ESP_LOGE(TAG, "first bad header, frame %d: %02x %02x %02x %02x "
                                      "%02x %02x %02x %02x .. n_ch %02x %02x len %02x %02x",
                                 f, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7],
                                 p[16], p[17], p[18], p[19]);
                        dumped = true;
                    }
                    continue;
                }
                frames_ok++;
                if (have_seq && h.seq != last_seq + 1) {
                    if (h.seq > last_seq) {
                        seq_gaps++;
                        lost += h.seq - last_seq - 1;
                    } else {
                        restarts++;     /* the FPGA's counters were cleared */
                    }
                }
                last_seq = h.seq;
                have_seq = true;
            }
            pipe_recycle(b);
        }

        int64_t now = esp_timer_get_time();
        if (now >= next_report) {
            ESP_LOGI(TAG, "%lu dgram/s | total %lu | frames ok %lu bad %lu | "
                          "seq gaps %lu (%lu frames) restarts %lu | last seq %lu",
                     (unsigned long)(dgrams - last_dgrams), (unsigned long)dgrams,
                     (unsigned long)frames_ok, (unsigned long)frames_bad,
                     (unsigned long)seq_gaps, (unsigned long)lost,
                     (unsigned long)restarts, (unsigned long)last_seq);
            last_dgrams = dgrams;
            next_report = now + 1000000;
        }
    }
}

void sink_check_start(void)
{
    /* Core 0, priority 6: where sink_udp runs, so the ingress side sees the
     * same competition for the pool as in a real run. */
    xTaskCreatePinnedToCore(check_task, "link", 4096, NULL, 6, NULL, 0);
}
