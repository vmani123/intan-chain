/*
 * source_qspi.c - Stage 2 ingress: datagrams arrive from the FPGA over the
 * ESP32-S3 GP-SPI Slave HD peripheral, 4-line data phase, DMA straight into a
 * pipe buffer.
 *
 * =====================================================================
 *  READ THIS BEFORE THE FIRST HARDWARE BRING-UP. TWO THINGS WILL BITE.
 * =====================================================================
 *
 * (1) THE BUS PROTOCOL IS FIXED BY THE SLAVE HARDWARE.
 *
 *     The ESP32-S3 SPI slave in half-duplex mode implements a fixed command
 *     protocol; the master does not get to invent it. The FPGA side
 *     (fpga/rtl/rx/qspi_master.sv) was matched to it field for field, and
 *     fpga/sim/qspi_hd_monitor.sv models this slave in the testbenches. See
 *     the confirmed table below.
 *
 *     Why this warning is at the top of the file: a one-cycle dummy mismatch
 *     does not fault. The slave simply latches the data phase one bit early
 *     or late, the DMA completes with the right length, and the bytes are
 *     rotated garbage. Downstream that looks exactly like a radio problem --
 *     intact packet counts, correct sizes, wrong contents -- and it is very
 *     easy to spend a week blaming Wi-Fi for it. The CHECK pattern of
 *     CONTRACTS.md section 3 exists partly to catch this: if the host's
 *     LFSR check fails on byte 0 of every frame while the datagram rate is
 *     nominal, suspect the bus protocol before anything else.
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
 * most likely to have drifted between IDF versions. Simple and probably
 * right beats clever and unverifiable; the known cost is documented at
 * drive_ready() below.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_slave_hd.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "soc/gpio_reg.h"
#include "soc/spi_struct.h"

#include "chain_contract.h"
#include "pins_s3.h"
#include "pipe.h"

static const char *TAG = "qspi";

/* =====================================================================
 * THE SLAVE-HD BUS PROTOCOL, AS THE FPGA DRIVES IT
 * =====================================================================
 * Nothing below is used by this firmware -- the slave hardware implements the
 * protocol on its own. It is here so both ends have one place to be checked
 * against. Confirmed against ESP-IDF v5.4 (the CI image) and v6.0-dev:
 *
 *   hal/esp32s3/include/hal/spi_ll.h
 *     spi_ll_get_slave_hd_command():    base command | line-mode mask, where
 *                                       0x20 = CMD 1-bit, ADDR 1-bit, DATA 4-bit
 *     spi_ll_get_slave_hd_dummy_bits(): 8, in every line mode
 *   hal/spi_slave_hd_hal.c:             a DMA receive completes on the CMD7
 *                                       (WR_DONE) interrupt, not on CS rising
 *   esp_serial_slave_link/essl_spi.c:   Espressif's own master sends WRDMA,
 *                                       then a separate WR_DONE transaction
 *
 * One datagram, as qspi_master.sv sends it (SPI mode 0, MSB first):
 *
 *   WRDMA    CS low, 0x23 on D0, 0x00 on D0, 8 dummy SCLKs,
 *            CHAIN_DGRAM_PAYLOAD bytes on D0..D3, CS high
 *   WR_DONE  CS low, 0x27 on D0, 0x00 on D0, 8 dummy SCLKs, CS high
 *   then CS stays high for 10 us while s_spi_slave_hd_segment_isr re-arms
 *   the receive DMA from the queue below.
 *
 * The slot config in source_qspi_start() -- 8 command, 8 address and 8 dummy
 * bits, mode 0, segment mode -- is the one Espressif's segment_mode/seg_slave
 * example uses with that master.
 */
#define QSPI_CMD_BITS            8
#define QSPI_ADDR_BITS           8
#define QSPI_DUMMY_BITS          8
#define QSPI_CMD_WRDMA_4LINE     0x23  /* master -> slave, DMA, data on 4 lines */
#define QSPI_CMD_WR_DONE_4LINE   0x27  /* closes the segment: CMD7 interrupt */
#define QSPI_CMD_RDDMA_4LINE     0x24  /* unused: this link is one-way */

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

/* Which ESP pin carries which bus line. pins_s3.h gives the intended wiring;
 * CONFIG_CHAIN_QSPI_PIN_SCAN can replace it with the wiring actually found. */
typedef struct { int cs, clk, d0, d1, d2, d3; } qspi_pins_t;
static qspi_pins_t s_pins = {
    PIN_QSPI_CS, PIN_QSPI_CLK, PIN_QSPI_D0, PIN_QSPI_D1, PIN_QSPI_D2, PIN_QSPI_D3,
};

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

/* What the bus looks like when nothing is arriving. Called after a second
 * with no completed segment, so it costs nothing while data flows.
 *
 * Two independent views. The slave's own registers say what it decoded: the
 * last command and address byte it latched, and which events it has seen
 * (CMD7 = WR_DONE, the one that completes a receive; cmd_err = a command
 * byte it does not support, i.e. bits arriving misaligned). The pin probe
 * reads the pads directly, a few hundred thousand times, and reports whether
 * each QSPI line was ever seen high and ever seen low: a line that never
 * moves while the FPGA reports DGRAMS climbing is a missing or misplaced
 * wire. Raw interrupt bits are read, never cleared -- the driver owns them. */
static void bus_report(void)
{
    const struct { int gpio; const char *name; } pins[] = {
        { s_pins.cs, "cs" }, { s_pins.clk, "clk" }, { s_pins.d0, "d0" },
        { s_pins.d1, "d1" }, { s_pins.d2, "d2" },   { s_pins.d3, "d3" },
    };
    uint32_t hi = 0, lo = 0;
    for (int i = 0; i < 200000; i++) {
        uint32_t v = REG_READ(GPIO_IN_REG);
        hi |= v;
        lo |= ~v;
    }
    char lines[96];
    int n = 0;
    for (size_t i = 0; i < sizeof pins / sizeof pins[0]; i++) {
        uint32_t m = 1u << pins[i].gpio;
        const char *st = (hi & m) && (lo & m) ? "moves" : (hi & m) ? "high" : "low";
        n += snprintf(lines + n, sizeof lines - n, " %s%d=%s", pins[i].name, pins[i].gpio, st);
    }
    ESP_LOGW(TAG, "no segment for 1 s | slave last cmd 0x%02x addr 0x%02x bits %u"
                  " | seen: wr_done %d wrdma %d cmd_err %d | pins:%s",
             (unsigned)GPSPI2.slave1.last_command, (unsigned)GPSPI2.slave1.last_addr,
             (unsigned)GPSPI2.slave1.data_bitlen,
             (int)GPSPI2.dma_int_raw.cmd7, (int)GPSPI2.dma_int_raw.wr_dma_done,
             (int)GPSPI2.dma_int_raw.cmd_err, lines);
}

/* Arm one DMA receive into a fresh pipe buffer. */
static bool arm_one(void)
{
    pipe_buf_t *b = pipe_try_acquire();
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

    int64_t last_seg_us = esp_timer_get_time();
    int64_t last_report_us = last_seg_us;
    for (;;) {
        /* While short of armed buffers, wake every 2 ms to re-arm: a buffer
         * the sink frees has no other way to reach the slave. Waiting only
         * for completions deadlocks once the pool runs dry with nothing
         * armed -- ready stays low and the FPGA stalls for good. */
        TickType_t wait = (s_outstanding < QSPI_QUEUE_DEPTH) ? pdMS_TO_TICKS(2)
                                                             : pdMS_TO_TICKS(1000);
        spi_slave_hd_data_t *done = NULL;
        esp_err_t err = spi_slave_hd_get_trans_res(QSPI_HOST, SPI_SLAVE_CHAN_RX,
                                                   &done, wait);
        if (err == ESP_OK && done != NULL) {
            s_outstanding--;
            drive_ready();
            last_seg_us = esp_timer_get_time();

            pipe_buf_t *b = (pipe_buf_t *)done->arg;
            b->rx_us = (uint32_t)last_seg_us;

            /* Segments are fixed size always (CONTRACTS.md section 2), so a
             * short transfer is a protocol fault, not a valid variable-length
             * datagram. Dropping it is right: forwarding a partial datagram
             * would make the host's loss statistic count it as received. */
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
        } else if (err != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "get_trans_res: %s", esp_err_to_name(err));
        }

        /* Re-arm. If the pool is empty the ready line stays low and the FPGA
         * stalls, which is exactly the intended back-pressure: the stall is
         * counted on the FPGA side and the run is then honestly labelled
         * ESP-limited rather than radio-limited. */
        while (s_outstanding < QSPI_QUEUE_DEPTH && arm_one()) {
            /* refill */
        }
        drive_ready();

        int64_t now = esp_timer_get_time();
        if (now - last_seg_us > 1000000 && now - last_report_us > 1000000) {
            bus_report();
            last_report_us = now;
        }
    }
}

/* Bring the slave up on the given pins, always through the GPIO matrix.
 *
 * On the bench (Arty S7-25 JC, 200 R series, ~15 cm jumper wires, 8 MHz SCLK)
 * the direct IOMUX path never decoded a command: the slave flagged cmd_err
 * on every transaction and never saw a WR_DONE. The same wires through the
 * GPIO matrix delivered every datagram bit-exact (2-8 kS/s, ~300 Mbit
 * checked). ESP-IDF's slave guide says the two routes behave the same below
 * 80 MHz, so the difference is most likely physical -- e.g. ringing on SCLK
 * or CS that the matrix's input path rides out and the IOMUX path does not.
 * Unconfirmed: needs a scope on GPIO12/GPIO10. Until then the matrix route
 * is the one that works; recheck it at 24 MHz (QSPI_DIV=2). */
static esp_err_t slave_init(const qspi_pins_t *p)
{
    spi_bus_config_t bus = { 0 };
    bus.data0_io_num    = p->d0;
    bus.data1_io_num    = p->d1;
    bus.data2_io_num    = p->d2;
    bus.data3_io_num    = p->d3;
    bus.sclk_io_num     = p->clk;
    bus.max_transfer_sz = CHAIN_DGRAM_PAYLOAD;
    bus.flags           = SPICOMMON_BUSFLAG_SLAVE | SPICOMMON_BUSFLAG_QUAD
                        | SPICOMMON_BUSFLAG_GPIO_PINS;
#if SOC_SPI_SUPPORT_OCT
    /* These fields exist on the S3 and a zero would be read as GPIO0, a
     * strapping pin. Explicitly unused. */
    bus.data4_io_num = -1;
    bus.data5_io_num = -1;
    bus.data6_io_num = -1;
    bus.data7_io_num = -1;
#endif

    spi_slave_hd_slot_config_t slot = { 0 };
    slot.spics_io_num = p->cs;
    slot.flags        = 0;              /* segment mode, MSB first */
    slot.mode         = 0;              /* CPOL=0 CPHA=0, as qspi_master.sv drives */
    slot.command_bits = QSPI_CMD_BITS;
    slot.address_bits = QSPI_ADDR_BITS;
    slot.dummy_bits   = QSPI_DUMMY_BITS;
    slot.queue_size   = QSPI_QUEUE_DEPTH;
    slot.dma_chan     = SPI_DMA_CH_AUTO;

    return spi_slave_hd_init(QSPI_HOST, &bus, &slot);
}

#if CONFIG_CHAIN_QSPI_PIN_SCAN
/* A datagram whose five frame headers all carry the magic and n_ch = 128 can
 * only have come through with every data line in its right place. */
static bool payload_ok(const uint8_t *b)
{
    for (int f = 0; f < CHAIN_FRAMES_PER_DGRAM; f++) {
        const uint8_t *h = b + f * CHAIN_RAW16_FRAME;
        if (h[0] != (CHAIN_MAGIC & 0xff) || h[1] != (CHAIN_MAGIC >> 8) ||
            h[16] != (CHAIN_N_CH & 0xff) || h[17] != (CHAIN_N_CH >> 8)) {
            return false;
        }
    }
    return true;
}

/* Find which of the five non-CS wires is SCLK and which are D0..D3, by
 * trying all 120 assignments while the FPGA streams. CS is taken as wired:
 * it is the one line that idles high, which bus_report() shows directly.
 * Each trial arms one receive and waits 100 ms -- about 40 datagrams at the
 * 2 kS/s test rate -- so the whole scan takes ~15 s. The FPGA must be
 * running (MODE 4 is simplest) while this runs; esp_ready is held high so it
 * sends. */
static void pin_scan(qspi_pins_t *io)
{
    const int cand[5] = { io->clk, io->d0, io->d1, io->d2, io->d3 };
    uint8_t *buf = heap_caps_aligned_alloc(64, CHAIN_DGRAM_PAYLOAD, MALLOC_CAP_DMA);
    if (!buf) {
        ESP_LOGE(TAG, "pin scan: no DMA memory");
        return;
    }
    gpio_set_level(PIN_QSPI_READY, 1);
    ESP_LOGI(TAG, "pin scan: trying 120 wirings, FPGA must be streaming");

    int completed = 0;
    for (int a = 0; a < 5; a++)
    for (int b = 0; b < 5; b++)
    for (int c = 0; c < 5; c++)
    for (int d = 0; d < 5; d++)
    for (int e = 0; e < 5; e++) {
        if (a == b || a == c || a == d || a == e || b == c || b == d || b == e ||
            c == d || c == e || d == e) {
            continue;
        }
        qspi_pins_t p = { io->cs, cand[a], cand[b], cand[c], cand[d], cand[e] };
        if (slave_init(&p) != ESP_OK) {
            continue;
        }
        bool ok = false;
        spi_slave_hd_data_t t = { .data = buf, .len = CHAIN_DGRAM_PAYLOAD };
        spi_slave_hd_data_t *res = NULL;
        if (spi_slave_hd_queue_trans(QSPI_HOST, SPI_SLAVE_CHAN_RX, &t, 0) == ESP_OK &&
            spi_slave_hd_get_trans_res(QSPI_HOST, SPI_SLAVE_CHAN_RX, &res,
                                       pdMS_TO_TICKS(100)) == ESP_OK) {
            completed++;   /* SCLK and D0 right: a segment closed */
            ok = res->trans_len == (size_t)CHAIN_DGRAM_PAYLOAD && payload_ok(buf);
            ESP_LOGI(TAG, "pin scan: clk%d d0=%d d1=%d d2=%d d3=%d closed %u B: "
                          "%02x %02x %02x %02x .. n_ch %02x %02x",
                     p.clk, p.d0, p.d1, p.d2, p.d3, (unsigned)res->trans_len,
                     buf[0], buf[1], buf[2], buf[3], buf[16], buf[17]);
            memset(buf, 0, CHAIN_DGRAM_PAYLOAD);
        }
        spi_slave_hd_deinit(QSPI_HOST);
        if (ok) {
            if (p.clk != PIN_QSPI_CLK || p.d0 != PIN_QSPI_D0 || p.d1 != PIN_QSPI_D1 ||
                p.d2 != PIN_QSPI_D2 || p.d3 != PIN_QSPI_D3) {
                ESP_LOGW(TAG, "pin scan: WIRING DIFFERS from pins_s3.h");
            }
            ESP_LOGW(TAG, "pin scan: found cs=GPIO%d clk=GPIO%d d0=GPIO%d d1=GPIO%d "
                          "d2=GPIO%d d3=GPIO%d", p.cs, p.clk, p.d0, p.d1, p.d2, p.d3);
            *io = p;
            break;
        }
    }
    ESP_LOGI(TAG, "pin scan: done, %d wirings closed a segment", completed);
    gpio_set_level(PIN_QSPI_READY, 0);
    heap_caps_free(buf);
}
#endif

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

#if CONFIG_CHAIN_QSPI_PIN_SCAN
    pin_scan(&s_pins);
#endif
    ESP_ERROR_CHECK(slave_init(&s_pins));

    /* Core 1, priority 8: same placement and priority as the synthetic
     * source, so the two ingress arms differ in what they do and in nothing
     * else. Comparing them is the whole point. */
    xTaskCreatePinnedToCore(qspi_task, "qspi", 4096, NULL, 8, NULL, 1);
}
