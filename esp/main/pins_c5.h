/*
 * pins_c5.h - every pin number for the ESP32-C5 node, in one place.
 *
 * The link moved to an ESP32-C5-DevKitC-1 when the bench S3 board started
 * browning out on Wi-Fi start-up. The FPGA side is unchanged: same Pmod JC
 * pins, same slave-HD protocol (the C5's GP-SPI2 decodes the same command
 * set as the S3's -- WRDMA 0x03, WR_END 0x07, hal/esp32c5/.../spi_ll.h).
 *
 * The C5's SPI2 IOMUX set is CLK 6, CS 10, WP 5, HD 4, MOSI 7, MISO 2, but
 * MOSI (GPIO7) and MISO (GPIO2) are strapping pins. So D0 and D1 sit on
 * plain pins instead, and the slave runs through the GPIO matrix -- which is
 * the route source_qspi.c forces on every target anyway, because it is the
 * one that worked on the S3 bench.
 *
 * ESP32-C5 pins to keep clear, for reference when moving any of these:
 *   GPIO2, 3     strapping (MTMS, MTDI); also JTAG pads
 *   GPIO7        strapping
 *   GPIO11, 12   UART0 TX/RX: the console through the board's CP2102N
 *   GPIO13, 14   USB-Serial-JTAG D- and D+ (the board's native USB port)
 *   GPIO15..22   SPI0/1 flash and PSRAM (GPIO15 is SPICS1 on the N8R8 module)
 *   GPIO25..28   strapping; GPIO27 also drives the on-board RGB LED
 *   GPIO0, 1     XTAL_32K pads; free on the DevKitC-1, but kept spare
 * GPIO29 and up do not exist on the ESP32-C5.
 */
#ifndef PINS_C5_H
#define PINS_C5_H

/* DevKitC-1 header positions in brackets (J1 left, J3 right). */
#define PIN_QSPI_CLK     6   /* FSPICLK   [J1-7]  */
#define PIN_QSPI_D0      8   /* plain     [J1-9]  */
#define PIN_QSPI_D1      9   /* plain     [J1-10] */
#define PIN_QSPI_D2      5   /* FSPIWP    [J3-9]  */
#define PIN_QSPI_D3      4   /* FSPIHD    [J3-8]  */
#define PIN_QSPI_CS     10   /* FSPICS0   [J1-11] */

/* Slave-to-master ready, sampled by the FPGA before each transaction. On the
 * other header from the QSPI group, which keeps a slow-edged handshake line
 * away from the fast clock, as on the S3. */
#define PIN_QSPI_READY  23   /*           [J3-5]  */

#endif /* PINS_C5_H */
