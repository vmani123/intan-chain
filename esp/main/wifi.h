/*
 * wifi.h - station bring-up.
 *
 * One blocking call. The data path must not start before there is an IP, and
 * there is nothing useful for this node to do without one, so there is no
 * asynchronous variant.
 */
#ifndef WIFI_H
#define WIFI_H

#include "esp_err.h"

/* Brings up the station, associates, waits for an IPv4 address, then logs the
 * link parameters. Returns ESP_OK once an IP is held. Returns ESP_FAIL after
 * CONFIG_CHAIN_WIFI_MAX_RETRY failed association attempts. */
esp_err_t wifi_connect(void);

/* Logs RSSI, channel and the PHY modes the association is using. Safe to call
 * again later; the host benchmark harness calls it at the end of a run too,
 * because RSSI drifting mid-run is the most common reason two otherwise
 * identical runs disagree. */
void wifi_log_link(void);

#endif /* WIFI_H */
