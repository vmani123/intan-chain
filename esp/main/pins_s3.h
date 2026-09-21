/*
 * pins_s3.h - every pin number for the ESP32-S3 node, in one place.
 *
 * The QSPI pins are the SPI2 (FSPI) IOMUX set. That is not cosmetic: routed
 * through the GPIO matrix the slave adds two APB cycles of input delay and
 * tops out near 26 MHz, while on the IOMUX path the same peripheral is
 * specified to 80 MHz. Since the whole point of this board is to find the
 * ceiling, the ingress must not be the thing that sets it.
 *
 * ESP32-S3 pins to keep clear, for reference when moving any of these:
 *   GPIO0        strapping (boot mode), has an external pull-up on most boards
 *   GPIO3        strapping (JTAG source select)
 *   GPIO19, 20   USB-Serial-JTAG D- and D+; taking them kills the USB console
 *   GPIO26..32   SPI0/1, the quad SPI flash. Never usable.
 *   GPIO33..37   used as well when the module has octal flash or octal PSRAM
 *                (any ...R8 part, for example ESP32-S3-WROOM-1-N8R8)
 *   GPIO39..42   JTAG MTCK/MTDO/MTDI/MTMS on the 20-pin header
 *   GPIO45       strapping (VDD_SPI voltage select)
 *   GPIO46       strapping (ROM message print), input only
 *   GPIO48       RGB LED on most dev boards
 * GPIO22..25 do not exist on the ESP32-S3.
 */
#ifndef PINS_S3_H
#define PINS_S3_H

/* SPI2 / FSPI IOMUX. None of GPIO9..14 is a strapping pin on the S3, and
 * none is used by flash or PSRAM on any standard module, so this set is free
 * on WROOM-1 and WROOM-2 alike. */
#define PIN_QSPI_CLK    12   /* FSPICLK */
#define PIN_QSPI_D0     11   /* FSPID   - MOSI when running single line */
#define PIN_QSPI_D1     13   /* FSPIQ   - MISO when running single line */
#define PIN_QSPI_D2     14   /* FSPIWP  */
#define PIN_QSPI_D3      9   /* FSPIHD  */
#define PIN_QSPI_CS     10   /* FSPICS0 */

/* Slave-to-master ready. Plain GPIO, driven by this firmware, sampled by the
 * FPGA master before it starts a transaction. GPIO21 is an ordinary pin on
 * the S3: not strapping, not USB, not flash, and it is not adjacent to the
 * QSPI group on the WROOM-1 pinout, which keeps a slow-edged handshake line
 * away from the fast clock. */
#define PIN_QSPI_READY  21

#endif /* PINS_S3_H */
