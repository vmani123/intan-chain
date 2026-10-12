// ============================================================================
// qspi_hd_monitor.sv -- the ESP32-S3 end of the QSPI link, as the slave
// hardware sees it. Testbench only.
// ----------------------------------------------------------------------------
// Models the GP-SPI slave in half-duplex (HD) segment mode, SPI mode 0, from
// ESP-IDF v5.4 (hal/esp32s3/include/hal/spi_ll.h, hal/spi_slave_hd_hal.c) and
// Espressif's reference master (esp_serial_slave_link/essl_spi.c):
//
//   * The command phase is ALWAYS 8 bits on dq[0], MSB first. Its high nibble
//     picks the line count of the later phases: 0x2_ = address on 1 line, data
//     on 4; 0xA_ = address and data on 4; 0x0_ = everything on 1 line.
//   * The address phase is 8 bits on those lines.
//   * 8 dummy cycles follow, whatever the line mode (spi_ll_get_slave_hd_dummy_bits
//     returns 8 on the S3).
//   * Then data. On 4 lines dq[3] carries the most significant bit of each
//     nibble, high nibble first.
//   * A master-write DMA segment is closed ONLY by a separate WR_DONE (base
//     command 0x07) transaction: that raises the CMD7 interrupt, which is what
//     completes the receive. CS rising after the data does not.
//
// So this monitor reassembles bytes exactly where the real slave would latch
// them. A master that drops the dummy cycles, or sends the command on four
// lines, ends up with the wrong length and rotated bytes here, just as it would
// on the ESP. A master that never sends WR_DONE delivers no datagrams at all.
//
// Results are read hierarchically by the testbench: dgcopy[] holds the last
// completed segment, dg_seen counts them, and the error counters say why
// anything was rejected.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module qspi_hd_monitor #(
    parameter int EXP_BYTES = 1380,     // every segment must be exactly this long
    parameter int MAX_BYTES = 2048
) (
    input wire       en,                // low across a reset: drop any partial transaction
    input wire       cs_n,
    input wire       sclk,
    input wire [3:0] dq
);

    localparam int DUMMY_CYC = 8;

    localparam [3:0] BASE_WRDMA   = 4'h3;
    localparam [3:0] BASE_WR_DONE = 4'h7;

    localparam int PH_CMD = 0, PH_ADDR = 1, PH_DUMMY = 2, PH_DATA = 3;

    // ---- results ----
    reg [7:0] dgcopy [0:MAX_BYTES-1];
    integer   dg_seen      = 0;     // segments closed by WR_DONE
    integer   seg_bytes    = 0;     // length of the last closed segment
    integer   bad_len      = 0;     // closed segments not EXP_BYTES long
    integer   proto_err    = 0;     // unknown command, short command, partial byte
    integer   missing_done = 0;     // a second WRDMA arrived before WR_DONE
    integer   orphan_done  = 0;     // WR_DONE with no write before it
    reg [7:0] wr_cmd       = 8'h00; // command byte of the last WRDMA
    reg [7:0] wr_addr      = 8'h00;
    reg [7:0] done_cmd     = 8'h00; // command byte of the last WR_DONE
    real      min_gap_ns   = 1.0e18;// shortest CS-high time from WR_DONE to the next WRDMA

    // ---- per transaction ----
    integer   phase, bits, data_bits, alines, dlines;
    reg [7:0] cmd, addr, cur;
    integer   nbytes;
    reg [7:0] tbuf [0:MAX_BYTES-1];

    // ---- per segment ----
    reg [7:0] seg  [0:MAX_BYTES-1];
    integer   seg_len  = 0;
    reg       seg_open = 1'b0;
    real      t_done   = -1.0;
    integer   k;

    task automatic start_txn;
        begin
            phase = PH_CMD; bits = 0; data_bits = 0; nbytes = 0;
            cmd = 8'h00; addr = 8'h00; cur = 8'h00;
            alines = 1; dlines = 1;
        end
    endtask

    initial start_txn();

    always @(negedge en) begin
        start_txn();
        seg_len  = 0;
        seg_open = 1'b0;
        t_done   = -1.0;
    end

    always @(negedge cs_n) if (en) begin
        start_txn();
        if (t_done >= 0.0 && ($realtime - t_done) < min_gap_ns)
            min_gap_ns = $realtime - t_done;
    end

    // Mode 0: the slave samples on the rising edge.
    always @(posedge sclk) if (en && !cs_n) begin
        case (phase)
            PH_CMD: begin
                cmd  = {cmd[6:0], dq[0]};
                bits = bits + 1;
                if (bits == 8) begin
                    case (cmd[7:4])
                        4'h1:    begin alines = 1; dlines = 2; end  // DOUT
                        4'h5:    begin alines = 2; dlines = 2; end  // DIO
                        4'h2:    begin alines = 1; dlines = 4; end  // QOUT
                        4'hA:    begin alines = 4; dlines = 4; end  // QIO
                        default: begin alines = 1; dlines = 1; end
                    endcase
                    phase = PH_ADDR; bits = 0;
                end
            end
            PH_ADDR: begin
                case (alines)
                    4:       addr = {addr[3:0], dq};
                    2:       addr = {addr[5:0], dq[1:0]};
                    default: addr = {addr[6:0], dq[0]};
                endcase
                bits = bits + alines;
                if (bits >= 8) begin phase = PH_DUMMY; bits = 0; end
            end
            PH_DUMMY: begin
                bits = bits + 1;
                if (bits == DUMMY_CYC) begin phase = PH_DATA; bits = 0; end
            end
            default: begin                             // PH_DATA
                case (dlines)
                    4:       cur = {cur[3:0], dq};
                    2:       cur = {cur[5:0], dq[1:0]};
                    default: cur = {cur[6:0], dq[0]};
                endcase
                data_bits = data_bits + dlines;
                if ((data_bits % 8) == 0) begin
                    if (nbytes < MAX_BYTES) tbuf[nbytes] = cur;
                    nbytes = nbytes + 1;
                end
            end
        endcase
    end

    always @(posedge cs_n) if (en) begin
        if (phase == PH_CMD) begin
            proto_err = proto_err + 1;                 // CS pulse shorter than a command
        end else if (cmd[3:0] == BASE_WRDMA) begin
            if ((data_bits % 8) != 0) proto_err = proto_err + 1;
            if (seg_open) missing_done = missing_done + 1;
            wr_cmd  = cmd;
            wr_addr = addr;
            for (k = 0; k < nbytes && seg_len < MAX_BYTES; k = k + 1) begin
                seg[seg_len] = tbuf[k];
                seg_len = seg_len + 1;
            end
            seg_open = 1'b1;
        end else if (cmd[3:0] == BASE_WR_DONE) begin
            // The master may clock address and dummy bits after the command
            // (Espressif's own does); the slave acts on the command alone.
            done_cmd = cmd;
            if (!seg_open) begin
                orphan_done = orphan_done + 1;
            end else begin
                for (k = 0; k < seg_len; k = k + 1) dgcopy[k] = seg[k];
                seg_bytes = seg_len;
                if (seg_len != EXP_BYTES) bad_len = bad_len + 1;
                dg_seen  = dg_seen + 1;
                seg_len  = 0;
                seg_open = 1'b0;
            end
            t_done = $realtime;
        end else begin
            proto_err = proto_err + 1;
        end
    end

endmodule

`default_nettype wire
