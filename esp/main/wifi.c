/*
 * wifi.c - station mode bring-up for the benchmark node.
 *
 * Two settings here exist purely for throughput and are worth naming:
 * power save off, and HT40 requested. Everything else is the standard station
 * sequence.
 *
 * The link parameters are logged at startup because a throughput number
 * without them is not a result. 44 Mbit/s at -45 dBm on an HT40 channel and
 * 44 Mbit/s at -75 dBm on a crowded HT20 channel are very different claims,
 * and the second one would be the interesting one.
 */

#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"

#include "wifi.h"

static const char *TAG = "wifi";

#define BIT_GOT_IP  BIT0
#define BIT_GAVE_UP BIT1

static EventGroupHandle_t s_eg;
static int                s_retry;

/* Band, chosen at build time with -DBENCH_WIFI_BAND=2g|5g|auto (CMake, like
 * BENCH_INGRESS), so switching it recompiles only main/. "auto" lets the
 * driver pick, which on a dual-band AP is usually 2.4 GHz -- the crowded one.
 * Only the C5 has a 5 GHz radio; on the S3 this stays at the driver default. */
#if defined(CHAIN_WIFI_BAND_5G) && CONFIG_SOC_WIFI_SUPPORT_5G
#  define BAND_MODE  WIFI_BAND_MODE_5G_ONLY
#  define BAND_NAME  "5 GHz only"
#elif defined(CHAIN_WIFI_BAND_2G) && CONFIG_SOC_WIFI_SUPPORT_5G
#  define BAND_MODE  WIFI_BAND_MODE_2G_ONLY
#  define BAND_NAME  "2.4 GHz only"
#endif

static void on_wifi_event(void *arg, esp_event_base_t base,
                          int32_t id, void *data)
{
    (void)arg; (void)base; (void)data;

    if (id == WIFI_EVENT_STA_START) {
#ifdef BAND_MODE
        /* Has to be after esp_wifi_start() (the API refuses before) and
         * before the first connect, which is exactly here. Setting the band
         * restarts the station, which raises STA_START a second time: set it
         * once. The connect below still runs on both passes -- returning
         * early would leave the node unconnected if a band change ever did
         * not restart the station -- and the redundant one only logs
         * "sta is connecting". */
        static bool band_set;
        if (!band_set) {
            band_set = true;
            esp_err_t e = esp_wifi_set_band_mode(BAND_MODE);
            ESP_LOGI(TAG, "band: %s (%s)", BAND_NAME, esp_err_to_name(e));
        }
#endif
        esp_wifi_connect();
    } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
        bool was_up = xEventGroupGetBits(s_eg) & BIT_GOT_IP;
        if (!was_up && s_retry >= CONFIG_CHAIN_WIFI_MAX_RETRY) {
            /* Never associated: app_main is waiting and reboots on this. */
            xEventGroupSetBits(s_eg, BIT_GAVE_UP);
            return;
        }
        s_retry++;
        /* Once the link has been up, never give up: an AP restart or a walk
         * out of range has to come back on its own, mid-demo, with nobody at
         * the serial console. Log the first and every 20th retry; reboot as a
         * last resort if the driver itself has wedged. */
        if (s_retry == 1 || s_retry % 20 == 0) {
            ESP_LOGW(TAG, "disconnected, retry %d%s", s_retry,
                     was_up ? " (was connected; retrying until it returns)" : "");
        }
        if (was_up && s_retry >= 30 * CONFIG_CHAIN_WIFI_MAX_RETRY) {
            ESP_LOGE(TAG, "no association after %d retries, rebooting", s_retry);
            esp_restart();
        }
        esp_wifi_connect();
    }
}

static void on_ip_event(void *arg, esp_event_base_t base,
                        int32_t id, void *data)
{
    (void)arg; (void)base;

    if (id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "ip " IPSTR, IP2STR(&e->ip_info.ip));
        s_retry = 0;
        xEventGroupSetBits(s_eg, BIT_GOT_IP);
    }
}

void wifi_log_link(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
        ESP_LOGW(TAG, "no association, link parameters unknown");
        return;
    }

    /* phy_11b/g/n/lr are the modes the association actually enabled, which is
     * the closest thing to a negotiated PHY rate that is safe to read here.
     * The instantaneous MCS index changes per frame with rate control and is
     * not exposed by esp_wifi; take it from a monitor-mode capture on the
     * host if the exact rate matters for a given result.
     *
     * Newer IDF also has esp_wifi_sta_get_negotiated_phymode(), which reports
     * the mode as a single enum and is nicer to log. It is left out here
     * because the call is not present in every 5.x release; enable it once
     * confirmed for the IDF in use:
     *
     *   wifi_phy_mode_t pm;
     *   if (esp_wifi_sta_get_negotiated_phymode(&pm) == ESP_OK) {
     *       ESP_LOGI(TAG, "phymode %d", (int)pm);
     *   }
     */
    wifi_bandwidth_t bw = WIFI_BW_HT20;
    esp_wifi_get_bandwidth(WIFI_IF_STA, &bw);

    ESP_LOGI(TAG,
             "link: rssi %d dBm  ch %u (second %u)  bw %s  phy%s%s%s%s  bssid "
             "%02x:%02x:%02x:%02x:%02x:%02x",
             (int)ap.rssi,
             (unsigned)ap.primary, (unsigned)ap.second,
             (bw == WIFI_BW_HT40) ? "40" : "20",
             ap.phy_11b ? " b" : "", ap.phy_11g ? " g" : "",
             ap.phy_11n ? " n" : "", ap.phy_lr ? " lr" : "",
             ap.bssid[0], ap.bssid[1], ap.bssid[2],
             ap.bssid[3], ap.bssid[4], ap.bssid[5]);
}

esp_err_t wifi_connect(void)
{
    s_eg = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, on_ip_event, NULL, NULL));

    wifi_config_t wc = { 0 };
    strncpy((char *)wc.sta.ssid, CONFIG_CHAIN_WIFI_SSID, sizeof wc.sta.ssid - 1);
    strncpy((char *)wc.sta.password, CONFIG_CHAIN_WIFI_PASSWORD,
            sizeof wc.sta.password - 1);
    /* Scan every channel and join the strongest AP. The default fast scan
     * joins the first AP it hears with this SSID: on a campus network with
     * many APs that is a different one after every reboot or replug, and on
     * the bench it once picked one at SNR 11 (MCS0) over one at SNR 27. */
    wc.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wc.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

    /* Keep the Wi-Fi config in RAM. The default writes it to NVS on every
     * start, and a flash write in the middle of bring-up is a stall that has
     * no business being in a timing rig. */
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));

    /* Ask for HT40. If the AP is 20 MHz only this is silently ignored and the
     * association comes up at 20, which wifi_log_link() will report. Set
     * before start because the value is used when the association is made. */
    esp_err_t bw = esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40);
    if (bw != ESP_OK) {
        ESP_LOGW(TAG, "HT40 request refused: %s", esp_err_to_name(bw));
    }

    ESP_ERROR_CHECK(esp_wifi_start());

    /* Power save off. With the default modem sleep the station dozes between
     * beacons and buffers outbound frames, which adds tens of milliseconds of
     * jitter and roughly halves sustained UDP throughput. Must be set after
     * esp_wifi_start(). */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    EventBits_t bits = xEventGroupWaitBits(s_eg, BIT_GOT_IP | BIT_GAVE_UP,
                                           pdFALSE, pdFALSE, portMAX_DELAY);
    if (!(bits & BIT_GOT_IP)) {
        ESP_LOGE(TAG, "could not associate with \"%s\"", CONFIG_CHAIN_WIFI_SSID);
        return ESP_FAIL;
    }

    wifi_log_link();
    return ESP_OK;
}
