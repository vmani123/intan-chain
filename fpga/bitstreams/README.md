# fpga/bitstreams

Bitstreams are build products and are not committed (`*.bit` is gitignored).
This file records which one the bench runs, so it can be rebuilt bit for bit.

| Bitstream | Built from | Tool | Result |
|---|---|---|---|
| `headstage_arty_s7_25_div6.bit` | `ababfb1` (no `fpga/` change since) | Vivado 2023.2, `QSPI_DIV=6 make bit-headstage` | WNS +0.513 ns, WHS +0.037 ns; ID `0x1c100002`; SHA-1 `2204cd3eb96292d9ad675ca1a5c55a24d69d5383` |

Rebuild on a machine with Vivado, then program the Arty S7-25 from the Mac
(volatile: reload after every power cycle):

```sh
QSPI_DIV=6 make bit-headstage
openFPGALoader -b arty_s7_25 fpga/bitstreams/headstage_arty_s7_25_div6.bit
```

`QSPI_DIV=6` is the 8 MHz QSPI link to the ESP that the bench is proven at.
`docs/headstage-bringup.md` has the wiring and the bring-up steps.
