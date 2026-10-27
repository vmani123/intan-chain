/*
 * target.h - what differs between the ESP32-S3 and ESP32-C5 builds of the
 * node, so the source files read the same on both.
 *
 *   - the pin map (pins_s3.h, pins_c5.h)
 *   - which core the ingress task owns: core 1 on the dual-core S3, leaving
 *     core 0 to the radio and lwIP; the only core on the single-core C5
 *   - the slave registers bus_report() reads. The fields are the same
 *     hardware on both chips, but the C5's register headers prefix them
 *     with slv_ and suffix the interrupt bits with _int_raw.
 *
 * Nothing else in the firmware is target specific: the slave-HD driver, the
 * command protocol and the Wi-Fi/lwIP path are common ESP-IDF code.
 */
#ifndef TARGET_H
#define TARGET_H

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"

#if CONFIG_IDF_TARGET_ESP32S3
#include "pins_s3.h"
#define NODE_CHIP "esp32s3"
#define NODE_PIN_MAP "pins_s3.h"
#elif CONFIG_IDF_TARGET_ESP32C5
#include "pins_c5.h"
#define NODE_CHIP "esp32c5"
#define NODE_PIN_MAP "pins_c5.h"
#else
#error "intan-chain: no pin map for this target; add a pins_<chip>.h"
#endif

#define INGRESS_CORE ((portNUM_PROCESSORS > 1) ? 1 : 0)

/* Slave state for bus_report(); expand where soc/spi_struct.h is included. */
#if CONFIG_IDF_TARGET_ESP32S3
#define SLV_LAST_CMD()      (GPSPI2.slave1.last_command)
#define SLV_LAST_ADDR()     (GPSPI2.slave1.last_addr)
#define SLV_DATA_BITLEN()   (GPSPI2.slave1.data_bitlen)
#define SLV_SEEN_WR_DONE()  (GPSPI2.dma_int_raw.cmd7)
#define SLV_SEEN_WRDMA()    (GPSPI2.dma_int_raw.wr_dma_done)
#define SLV_SEEN_CMD_ERR()  (GPSPI2.dma_int_raw.cmd_err)
#else
#define SLV_LAST_CMD()      (GPSPI2.slave1.slv_last_command)
#define SLV_LAST_ADDR()     (GPSPI2.slave1.slv_last_addr)
#define SLV_DATA_BITLEN()   (GPSPI2.slave1.slv_data_bitlen)
#define SLV_SEEN_WR_DONE()  (GPSPI2.dma_int_raw.slv_cmd7_int_raw)
#define SLV_SEEN_WRDMA()    (GPSPI2.dma_int_raw.slv_wr_dma_done_int_raw)
#define SLV_SEEN_CMD_ERR()  (GPSPI2.dma_int_raw.slv_cmd_err_int_raw)
#endif

#endif /* TARGET_H */
