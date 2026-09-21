/*
 * source_qspi.c - Stage 2 ingress: datagrams arrive from the FPGA over the
 * ESP32-S3 GP-SPI Slave HD peripheral, 4-line data phase, DMA straight into a
 * pipe buffer.
 *
 * =====================================================================
 *  READ THIS BEFORE THE FIRST HARDWARE BRING-UP. TWO THINGS WILL BITE.
 * =====================================================================
 *
 * (1) THE PHASE TABLE IS NOT NEGOTIABLE AND IS NOT DOCUMENTED HERE.
 *
 *     The ESP32-S3 SPI slave in half-duplex mode implements a fixed,
 *     hardware-defined command protocol. The master does not get to invent
 *     it. Every field of every phase -- the command byte, the address width,
 *     the number of dummy cycles, and how many data lines each phase uses --
 *     must be read out of the ESP-IDF that is actually installed on the build
 *     machine and the FPGA QSPI master configured field for field to match:
 *
 *       components/esp_driver_spi/include/driver/spi_slave_hd.h
 *       components/hal/include/hal/spi_ll.h      (SPI_LL_BASE_CMD_HD_*)
 *       components/hal/esp32s3/include/hal/spi_ll.h
 *       docs/en/api-reference/peripherals/spi_slave_hd.rst
 *         -> the "ESP SPI Slave HD (Half Duplex) Mode Protocol" table
 *
 *     The values written in the MUST-CONFIRM block below are this author's
 *     best reading of that protocol and have NOT been checked against an
 *     installed IDF, because none was available when this file was written.
 *     Treat them as a starting hypothesis.
 *
 *     Why this warning is at the top of the file: a one-cycle dummy mismatch
 *     does not fault. The slave simply latches the data phase one bit early
 *     or late, the DMA completes with the right length, and the bytes are
 *     rotated garbage. Downstream that looks exactly like a radio problem --
 *     intact packet counts, correct sizes, wrong contents -- and it is very
 *     easy to spend a week blaming Wi-Fi for it. The CHECK pattern of
 *     CONTRACTS.md section 3 exists partly to catch this: if the host's
 *     LFSR check fails on byte 0 of every frame while the datagram rate is
 *     nominal, suspect the phase table before anything else.
 *
 * (2) A SLAVE-TO-MASTER READY LINE IS REQUIRED, NOT OPTIONAL.
 *
 *     PIN_QSPI_READY is driven high by this firmware only while at least one
 *     DMA receive buffer is queued in the slave peripheral, and the FPGA
 *     master MUST sample it and hold off the next transaction while it is
 *     low. There is no other back-pressure in the system.
 *
 *     Without it, the master clocks data into a slave with no buffer armed.
 *     The peripheral discards it, silently, and the frames simply vanish --
 *     no error, no counter. That happens precisely when the ESP is busiest,
 *     which is precisely the load being measured, so the failure mode is a
 *     throughput curve that rolls off smoothly and looks like a genuine radio
 *     ceiling. It is not. It is the ingress dropping on the floor.
 *
 *     The FPGA's STALLS register (CONTRACTS.md section 4, address 0x14)
 *     counts cycles spent waiting on this line, which is how the host tells
 *     "the ESP could not keep up" apart from "the radio could not keep up".
 *     A run with STALLS at zero and a flat throughput curve means the line is
 *     not wired or not being sampled.
 *
 * ---------------------------------------------------------------------
 * Segment mode, not append mode. Append mode is faster in principle but the
 * driver owns the descriptor ring and the received length is reported per
 * append rather than per segment; segment mode gives one transaction per
 * datagram, one length to check, and one buffer to hand straight to the pipe.
 * Fixed-size datagrams make the extra throughput of append mode unnecessary.
 *
 * Polling API (queue_trans / get_trans_res) rather than the ISR callbacks.
 * The callbacks would let the ready line be driven from the completion
 * interrupt, which is strictly better, but they are also the part of the API
 * most likely to have drifted between IDF versions. This file has never been
 * compiled. Simple and probably right beats clever and unverifiable; the
 * known cost is documented at drive_ready() below.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_slave_hd.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "soc/soc_caps.h"

#include "chain_contract.h"
#include "pins_s3.h"
#include "pipe.h"

static const char *TAG = "qspi";

/* =====================================================================
 * MUST CONFIRM AGAINST THE INSTALLED ESP-IDF BEFORE FIRST HARDWARE USE
 * =====================================================================
 * Nothing below is used by this firmware -- the slave hardware implements the
 * protocol on its own. These are here so the FPGA master has one authoritative
 * place to be configured from, and so the check has a checklist.
 *
 * Confirm each against hal/spi_ll.h (SPI_LL_BASE_CMD_HD_*) and the slave-HD
 * protocol table in the IDF docs for the INSTALLED version, then delete this
 * banner and record the version the values came from.
 *
 *   [ ] QSPI_CMD_WRDMA_4LINE   command byte for "master writes DMA", 4-line
 *   [ ] QSPI_CMD_WR_DONE_4LINE command byte that terminates a write segment
 *   [ ] QSPI_ADDR_BITS         width of the address phase
 *   [ ] QSPI_DUMMY_BITS_WR     dummy cycles between address and data on WRITE
 *   [ ] QSPI_CMD_BITS          width of the command phase
 *   [ ] which phases are 1-line and which are 4-line (see note below)
 *   [ ] SPI mode (CPOL/CPHA) the FPGA master drives
 *
 * Working hypothesis, from the documented slave-HD protocol:
 *   - command, address and dummy phases are carried on the same number of
 *     lines as the data phase when a 4-line command byte is used; the 4-line
 *     command bytes are the 1-line ones with bit5 set (0x03 -> 0x23).
 *   - a master-write segment is: CMD8 + ADDR8 + DATA(n), no dummy, followed
 *     by a separate WR_DONE command with no data.
 *   - dummy cycles apply to master-READ commands only. This link only ever
 *     does master writes, so QSPI_DUMMY_BITS_WR is expected to be 0 even
 *     though dummy_bits is configured non-zero below for the read direction.
 * If any of that is wrong the symptom is rotated data, see (1) above.
 */
#define QSPI_CMD_BITS            8
#define QSPI_ADDR_BITS           8
#define QSPI_DUMMY_BITS          8     /* read direction only; unused here */
#define QSPI_CMD_WRDMA_4LINE     0x23  /* master -> slave, DMA */
#define QSPI_CMD_WR_DONE_4LINE   0x27  /* ends the write segment */
#define QSPI_CMD_RDDMA_4LINE     0x24  /* unused: this link is one-way */
/* ===================== end MUST CONFIRM block ======================== */

#define QSPI_HOST        SPI2_HOST

/* Four transactions armed at once. One is not enough: with the polling API
 * the gap between a completion being retrieved and the next buffer being
 * queued is a whole scheduling round trip, and at 4000 datagrams/s that gap
 * is a large fraction of the 250 us budget. Four gives the master roughly a
 * millisecond of slack before the ready line has to go low. */
#define QSPI_QUEUE_DEPTH 4

/* The driver keeps the pointer we hand it, so the descriptors must outlive
 * the call. A ring of QUEUE_DEPTH is enough because segment-mode receives
 * complete in the order they were queued. */
static spi_slave_hd_data_t s_desc[QSPI_QUEUE_DEPTH];
static int                 s_head;
static int                 s_outstanding;

static uint32_t s_short_frames;   /* completed with the wrong length */
static uint32_t s_starved;        /* no free pipe buffer to arm */

/* Drive the ready line from the count of armed buffers.
 *
 * KNOWN LIMITATION of the polling flow: this runs in task context, so between
 * the DMA completing and this call there is a window in which the peripheral
 * has one fewer buffer than the line advertises. QSPI_QUEUE_DEPTH of 4 makes
 * that window harmless at the rates of interest -- the line only ever goes
 * low when all four are consumed, and consuming four takes a millisecond --
 * but if the FPGA ever reports lost segments with STALLS at zero, moving this
 * into the cb_recv callback is the fix. */
static void drive_ready(void)
{
    gpio_set_level(PIN_QSPI_READY, s_outstanding > 0 ? 1 : 0);
}

/* Arm one DMA receive into a fresh pipe buffer. */
static bool arm_one(void)
{
    pipe_buf_t *b = pipe_acquire(0);
    if (!b) {
        s_starved++;
        return false;
    }

    spi_slave_hd_data_t *d = &s_desc[s_head];
    s_head = (s_head + 1) % QSPI_QUEUE_DEPTH;

    d->data = b->data;
    /* The FPGA sends the payload only; the trailer region at the end of the
     * buffer belongs to the ESP and is written by the sink. */
    d->len  = CHAIN_DGRAM_PAYLOAD;
    d->arg  = b;

    esp_err_t err = spi_slave_hd_queue_trans(QSPI_HOST, SPI_SLAVE_CHAN_RX,
                                             d, portMAX_DELAY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "queue_trans: %s", esp_err_to_name(err));
        pipe_recycle(b);
        return false;
    }

    s_outstanding++;
    return true;
}

static void qspi_task(void *arg)
{
    (void)arg;

    while (s_outstanding < QSPI_QUEUE_DEPTH && arm_one()) {
        /* prime the ring */
    }
    drive_ready();
    ESP_LOGI(TAG, "armed %d receives of %d bytes, ready line on GPIO%d",
             s_outstanding, (int)CHAIN_DGRAM_PAYLOAD, PIN_QSPI_READY);

    for (;;) {
        spi_slave_hd_data_t *done = NULL;
        esp_err_t err = spi_slave_hd_get_trans_res(QSPI_HOST, SPI_SLAVE_CHAN_RX,
                                                   &done, portMAX_DELAY);
        if (err != ESP_OK || done == NULL) {
            ESP_LOGE(TAG, "get_trans_res: %s", esp_err_to_name(err));
            continue;
        }

        s_outstanding--;
        drive_ready();

        pipe_buf_t *b = (pipe_buf_t *)done->arg;
        b->rx_us = (uint32_t)esp_timer_get_time();

        /* Segments are fixed size always (CONTRACTS.md section 2), so a short
         * transfer is a protocol fault, not a valid variable-length datagram.
         * Dropping it is right: forwarding a partial datagram would make the
         * host's loss statistic count it as received. */
        if (done->trans_len == (size_t)CHAIN_DGRAM_PAYLOAD) {
            pipe_submit(b);
        } else {
            if ((s_short_frames++ % 256u) == 0u) {
                ESP_LOGW(TAG, "short segment: %u bytes, expected %d (x%u)",
                         (unsigned)done->trans_len, (int)CHAIN_DGRAM_PAYLOAD,
                         (unsigned)s_short_frames);
            }
            pipe_recycle(b);
        }

        /* Re-arm. If the pool is empty the ready line stays low and the FPGA
         * stalls, which is exactly the intended back-pressure: the stall is
         * counted on the FPGA side and the run is then honestly labelled
         * ESP-limited rather than radio-limited. */
        while (s_outstanding < QSPI_QUEUE_DEPTH && arm_one()) {
            /* refill */
        }
        drive_ready();
    }
}

void source_qspi_start(void)
{
    gpio_config_t rdy = {
        .pin_bit_mask = 1ULL << PIN_QSPI_READY,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&rdy));
    /* Low until a buffer is armed: the FPGA must never see "ready" during the
     * ESP's boot, or the first datagrams are lost before anything is counted. */
    gpio_set_level(PIN_QSPI_READY, 0);

    spi_bus_config_t bus = { 0 };
    bus.data0_io_num    = PIN_QSPI_D0;
    bus.data1_io_num    = PIN_QSPI_D1;
    bus.data2_io_num    = PIN_QSPI_D2;
    bus.data3_io_num    = PIN_QSPI_D3;
    bus.sclk_io_num     = PIN_QSPI_CLK;
    bus.max_transfer_sz = CHAIN_DGRAM_PAYLOAD;
    bus.flags           = SPICOMMON_BUSFLAG_SLAVE | SPICOMMON_BUSFLAG_QUAD;
#if SOC_SPI_SUPPORT_OCT
    /* These fields exist on the S3 and a zero would be read as GPIO0, a
     * strapping pin. Explicitly unused. */
    bus.data4_io_num = -1;
    bus.data5_io_num = -1;
    bus.data6_io_num = -1;
    bus.data7_io_num = -1;
#endif

    spi_slave_hd_slot_config_t slot = { 0 };
    slot.spics_io_num = PIN_QSPI_CS;
    slot.flags        = 0;              /* segment mode, MSB first */
    slot.mode         = 0;              /* CPOL=0 CPHA=0. MUST CONFIRM vs FPGA */
    slot.command_bits = QSPI_CMD_BITS;
    slot.address_bits = QSPI_ADDR_BITS;
    slot.dummy_bits   = QSPI_DUMMY_BITS;
    slot.queue_size   = QSPI_QUEUE_DEPTH;
    slot.dma_chan     = SPI_DMA_CH_AUTO;

    ESP_ERROR_CHECK(spi_slave_hd_init(QSPI_HOST, &bus, &slot));

    /* Core 1, priority 8: same placement and priority as the synthetic
     * source, so the two ingress arms differ in what they do and in nothing
     * else. Comparing them is the whole point. */
    xTaskCreatePinnedToCore(qspi_task, "qspi", 4096, NULL, 8, NULL, 1);
}
