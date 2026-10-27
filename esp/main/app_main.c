/*
 * app_main.c - bring-up and wiring for the intan-chain node (ESP32-S3 or C5).
 *
 * Order matters: NVS, banner, radio, pipe, sink, echo, source. The source
 * starts last so that no datagram is generated before there is somewhere for
 * it to go; otherwise the first second of every run contains a burst of drops
 * that mean nothing.
 */

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_idf_version.h"
#include "sdkconfig.h"

#include "chain_contract.h"
#include "pipe.h"
#include "target.h"
#include "wifi.h"

/* Task entry points. Declared here rather than in a header: each is called
 * exactly once, from this file, and each is void(void), so a header would add
 * a file without adding a check. */
void source_synth_start(void);
void source_qspi_start(void);
void sink_udp_start(void);
void sink_check_start(void);
void echo_start(void);

static const char *TAG = "chain";

#if defined(BENCH_INGRESS_QSPI)
#  define INGRESS_NAME "qspi"
#elif defined(BENCH_INGRESS_SYNTH)
#  define INGRESS_NAME "synth"
#else
#  error "BENCH_INGRESS not set; pass -DBENCH_INGRESS=synth or -DBENCH_INGRESS=qspi to idf.py"
#endif

/* One line with every size the three implementations have to agree on.
 * Printed before anything else runs, so a contract mismatch between this
 * binary and the FPGA bitstream or the host tool is visible in the first
 * screenful of the serial log instead of being inferred later from corrupt
 * data. If these numbers do not match what host/chain/contract.py prints,
 * stop and regenerate: someone edited CONTRACTS.md and did not run
 * "make contract". */
static void banner(void)
{
    ESP_LOGI(TAG,
             "intan-chain esp | chip=%s ingress=%s | idf=%d.%d.%d | "
             "n_ch=%d hdr=%d frame=%d frames/dgram=%d payload=%d trailer=%d dgram=%d | "
             "udp=%d echo=%d",
             NODE_CHIP, INGRESS_NAME,
             ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR, ESP_IDF_VERSION_PATCH,
             (int)CHAIN_N_CH, (int)CHAIN_HDR_BYTES, (int)CHAIN_RAW16_FRAME,
             (int)CHAIN_FRAMES_PER_DGRAM, (int)CHAIN_DGRAM_PAYLOAD,
             (int)CHAIN_TRAILER_BYTES, (int)CHAIN_DGRAM_BYTES,
             (int)CHAIN_UDP_PORT, (int)CHAIN_ECHO_PORT);
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    banner();

#if !CONFIG_CHAIN_LINK_TEST_NO_WIFI
    /* A node with no link has nothing to measure. Reboot rather than idle, so
     * an unattended bench run recovers by itself when the AP comes back. */
    if (wifi_connect() != ESP_OK) {
        ESP_LOGE(TAG, "no link, rebooting in 5 s");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
    }
#endif

    ESP_ERROR_CHECK(pipe_init());

#if CONFIG_CHAIN_LINK_TEST_NO_WIFI
    sink_check_start();     /* checks and drops: the link without the radio */
#else
    sink_udp_start();
    echo_start();
#endif

#if defined(BENCH_INGRESS_QSPI)
    source_qspi_start();
#else
    source_synth_start();
#endif

    pipe_run();
}
