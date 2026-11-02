# Checking every sample against the Intan RHD recording controller

The chain's own numbers (`seq` gaps, `LINK_ERR`, the pattern check) prove the
link carries bits faithfully. They cannot prove that the values the Mac shows
are the values the RHD2164s produced. This check does: the Intan RHD recording
controller records the headstage as usual, the chain hears the same SPI
traffic, and the Mac compares the two recordings sample by sample.

```
                      cable A                 cable B
 Intan RHD recording ========= [C3430 X]--[C3430 Y] ========= headstage
 controller (master)               pass-through |              (2 x RHD2164)
       | USB                                    | 5 LVDS pairs + GND, listen only
       v                                        v
     RHX  --> amplifier.dat          Arty S7-25 (tap build) --QSPI--> ESP32-C5 --Wi-Fi--> Mac
                         \                                                               /
                          `------------------ python -m chain.tapcheck compare ---------'
```

An RHD2164 has one SPI master, so the chain cannot run its own sweeps at the
same time. In this check the controller is the master and the Arty only
listens. What is checked, end to end, is everything from the SPI wires onward:
capture, framing, FIFO, QSPI, the ESP, Wi-Fi and the host parser. The chain's
own SPI master is not in the loop here; the bring-up script, `LINK_ERR` and the
emulator loopback cover it.

## Hardware

- Intan RHD recording controller (or RHD USB interface board) with RHX
- the headstage (2 x RHD2164 = 128 channels on one port)
- **two** Intan RHD SPI cables (C3203 / C3206 / C3211 / ...)
- **two** Intan SPI cable adapter boards, **C3430** (12 signals to 0.1" holes,
  no active parts)
- the Arty S7-25 with the tap bitstream, the ESP32-C5 on Pmod JC as before
- short wires; twisted pairs if longer than ~10 cm

The tap is the pass-through Intan describes for commutators (application note
"Adapting SPI Cables to a Commutator"), plus five pairs and a ground to the
Arty. The Arty's inputs are LVDS receivers with internal termination **off**
(the cable is already terminated: CS, SCLK and MOSI on the headstage, both MISO
lines inside the controller's FPGA), so the tap only loads the lines; it never
drives them.

## Wiring

### 1. Pass-through between the two C3430 boards

`X` is the C3430 on the cable from the **controller**, `Y` the C3430 on the
cable from the **headstage**. Every `T` hole on one board goes to the `B` hole
with the same number on the other (the cable flips top and bottom):

| X | Y | Signal |
|---|---|---|
| T1 | B1 | CS+ |
| B1 | T1 | CS− |
| T2 | B2 | SCLK+ |
| B2 | T2 | SCLK− |
| T3 | B3 | MOSI+ |
| B3 | T3 | MOSI− |
| T4 | B4 | MISO1+ |
| B4 | T4 | MISO1− |
| T5 | B5 | MISO2+ |
| B5 | T5 | MISO2− |
| T6 | B6 | power (3.5 V from the controller) |
| B6 | T6 | ground |

Keep each pair together (twist + with −). With this in place and nothing else
connected, RHX must record the headstage exactly as with one cable; check that
before adding the tap.

### 2. Tap from C3430 Y to the Arty

| C3430 Y hole | Signal | Arty pin | FPGA pin |
|---|---|---|---|
| B1 | CS+    | JA1  | L17 |
| T1 | CS−    | JA2  | L18 |
| B2 | SCLK+  | JA3  | M14 |
| T2 | SCLK−  | JA4  | N14 |
| B3 | MOSI+  | JA7  | M16 |
| T3 | MOSI−  | JA8  | M17 |
| B4 | MISO1+ | JA9  | M18 |
| T4 | MISO1− | JA10 | N18 |
| B5 | MISO2+ | JB1  | P17 |
| T5 | MISO2− | JB2  | P18 |
| T6 | ground | JA5 (or JA11) | GND |

- **Never connect Y B6 (cable power) or any Pmod 3V3 pin (pin 6 / 12).** Only
  ground is shared.
- Pairs land on the Arty's differential pairs (JA1/2, JA3/4, JA7/8, JA9/10,
  JB1/2). A swapped + and − inverts that line; `tapcheck status` names which.
- The LVDS converter board (C3490) and its JA wiring are not used in this
  setup; unplug it.
- Optional: a 220–470 Ω resistor in each tap wire, right at the C3430 holes,
  isolates the stub. At 2 kS/s short wires work without them.

MISO1 is chip 0 (channels 0–63), MISO2 chip 1 (64–127), the same split the
chain and RHX use.
