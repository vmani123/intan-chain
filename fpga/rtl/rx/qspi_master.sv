// ============================================================================
// qspi_master.sv -- shift one finished slot out to the ESP32-S3.
// ----------------------------------------------------------------------------
// The ESP32-S3 receives with its GP-SPI slave in half-duplex (HD) segment
// mode, whose bus protocol is fixed by the silicon. One datagram is two
// transactions, mirroring Espressif's own master (essl_spi_wrdma_seg followed
// by essl_spi_wrdma_done, ESP-IDF v5.4):
//
//   WRDMA    cs_n low -> CMD 0x23 on dq[0] -> ADDR 0x00 on dq[0] -> 8 dummy
//            SCLKs -> 1380 data bytes on dq[3:0] -> cs_n high
//   WR_DONE  cs_n low -> CMD 0x27 on dq[0] -> ADDR 0x00 on dq[0] -> 8 dummy
//            SCLKs -> cs_n high
//
// The command phase is always one line, MSB first; 0x2_ in its high nibble
// tells the slave the address is one line and the data four. The 8 dummy
// cycles are the S3's fixed count (spi_ll_get_slave_hd_dummy_bits). WR_DONE is
// not optional: the receive only completes on the slave's CMD7 interrupt, so a
// segment without it is never delivered. Get any of this wrong and nothing
// faults -- the bytes simply land rotated, or never land -- which is why
// fpga/sim/qspi_hd_monitor.sv models the slave and the testbench checks it.
//
// Data goes out most significant nibble first, dq[3] carrying the nibble's
// top bit, and slot words low byte first because the contract is
// little-endian. SCLK is clk / (2 * DIV), so DIV = 2 at 96 MHz gives 24 MHz.
// dq_out changes while SCLK is low and is stable across the rising edge:
// SPI mode 0.
//
// THE GAP AFTER WR_DONE
//   The slave re-arms its receive DMA inside the CMD7 interrupt
//   (s_spi_slave_hd_segment_isr). A WRDMA that starts before that ISR has run
//   lands in no buffer and is lost without trace, so cs_n stays high for
//   SEG_GAP clocks after every WR_DONE. 10 us is a guess at ISR latency with
//   Wi-Fi sharing the core; if the host sees periodic seq gaps with STALLS at
//   zero, raise it.
//
// THE READY LINE IS THE ONLY BACK PRESSURE IN THE SYSTEM
//   esp_ready is high while the ESP has a DMA buffer queued. It comes from
//   another clock domain, so it passes a two-flop synchronizer before the
//   state machine sees it. `stalls` counts every clock spent holding a
//   finished datagram while it is low. A run with a flat throughput curve and
//   stalls at zero means the ready line is not wired; a rolled-off curve with
//   stalls climbing means the ESP, not the radio, is the limit.
//
// THE t_tx PATCH
//   Words 6 and 7 of every 138-word frame are replaced, as the shifter reaches
//   them, by the live `tick`. t_tx is then genuinely the instant the frame left
//   the FPGA, so t_tx - t_cap on the host is the real queueing time including
//   the wait above. The full tick is latched at word 6 and its high half reused
//   at word 7, so the two halves can never straddle a counter wrap.
//
// dq_oe is a constant 1: this link is one-way. It is brought out so that a
// bidirectional variant needs no change here.
// ============================================================================

`default_nettype none

module qspi_master #(
    parameter int N_CH             = 128,
    parameter int HDR_WORDS        = 10,
    parameter int FRAMES_PER_DGRAM = 5,
    parameter int NSLOT            = 4,
    parameter int DIV              = 2,          // SCLK = clk / (2*DIV)
    parameter logic [7:0] CMD_BYTE  = 8'h23,     // slave-HD WRDMA, data on 4 lines
    parameter logic [7:0] DONE_BYTE = 8'h27,     // slave-HD WR_DONE, same line mode
    parameter logic [7:0] ADDR_BYTE = 8'h00,
    parameter int DUMMY_CYC        = 8,          // ESP32-S3 slave-HD, any line mode
    parameter int CS_GAP           = 8,          // clocks of cs_n high between WRDMA and WR_DONE
    parameter int SEG_GAP          = 960,        // clocks of cs_n high after WR_DONE (10 us)
    parameter int SW               = $clog2(NSLOT),
    parameter int AW               = $clog2(NSLOT * FRAMES_PER_DGRAM
                                            * (HDR_WORDS + N_CH))
) (
    input  wire            clk,
    input  wire            rst_n,
    input  wire            clear,          // zero the counters

    input  wire            esp_ready,      // asynchronous, from the ESP

    // slot memory read port
    output reg             rd_en,
    output reg  [AW-1:0]   rd_addr,
    input  wire [15:0]     rd_data,

    // slot FIFO handshake
    input  wire [SW-1:0]   slot_rd,
    input  wire            slot_valid,
    output reg             slot_release,

    input  wire [31:0]     tick,

    output reg             cs_n,
    output reg             sclk,
    output reg  [3:0]      dq_out,
    output wire            dq_oe,

    output reg  [31:0]     dgrams,
    output reg  [31:0]     stalls,
    output reg             dgram_sent
);

    localparam int FRAME_WORDS = HDR_WORDS + N_CH;          // 138
    localparam int SLOT_WORDS  = FRAMES_PER_DGRAM * FRAME_WORDS;
    localparam int DATA_BYTES  = SLOT_WORDS * 2;            // 1380
    localparam int NIB_TOTAL   = 2 * DATA_BYTES;            // data nibbles only
    localparam int NW          = $clog2(NIB_TOTAL);
    localparam int WW          = $clog2(SLOT_WORDS);
    localparam int MW          = $clog2(FRAME_WORDS);
    localparam int DW          = (DIV <= 1) ? 1 : $clog2(DIV);
    localparam int GW          = $clog2(((SEG_GAP > CS_GAP) ? SEG_GAP : CS_GAP) + 1);

    // Bit-selects on parenthesised expressions are not legal, so every sized
    // constant the comparisons need gets a name.
    localparam [NW-1:0]  N_LAST  = NW'(NIB_TOTAL - 1);
    localparam [WW-1:0]  W_LAST  = WW'(SLOT_WORDS - 1);
    localparam [MW-1:0]  M_LAST  = MW'(FRAME_WORDS - 1);
    localparam [MW-1:0]  M_TTXL  = MW'(6);
    localparam [MW-1:0]  M_TTXH  = MW'(7);
    localparam [DW-1:0]  D_LAST  = DW'(DIV - 1);
    localparam [4:0]     PRE_LAST = 5'd15;                 // 8 command + 8 address bits
    localparam [4:0]     DUM_LAST = 5'(DUMMY_CYC - 1);
    localparam [GW-1:0]  CS_GAP_LAST  = GW'(CS_GAP - 1);
    localparam [GW-1:0]  SEG_GAP_LAST = GW'(SEG_GAP - 1);

    // S_PRE, S_DUMMY and S_TAIL serve both transactions; `done_txn` says which.
    localparam [2:0] S_IDLE  = 3'd0,   // waiting for a slot and esp_ready
                     S_PRE   = 3'd1,   // command + address, one line
                     S_DUMMY = 3'd2,
                     S_DATA  = 3'd3,   // WRDMA payload, four lines
                     S_TAIL  = 3'd4,   // cs hold after the last falling edge
                     S_GAP   = 3'd5,   // cs high between WRDMA and WR_DONE
                     S_SEG   = 3'd6;   // cs high after WR_DONE: the ESP re-arms

    reg [2:0]      state;
    reg            done_txn;       // 0: the WRDMA transaction, 1: WR_DONE
    reg [DW-1:0]   dcnt;
    reg [4:0]      bcnt;           // bit / dummy cycle within S_PRE or S_DUMMY
    reg [15:0]     pre;            // command and address, shifted out MSB first
    reg [GW-1:0]   gcnt;
    reg [NW-1:0]   nidx;           // data nibble index within the transaction
    reg [15:0]     cur;            // word currently being shifted
    reg [WW-1:0]   fetch_idx;      // word the read port is presenting
    reg [MW-1:0]   wmod;           // fetch_idx mod FRAME_WORDS
    reg [31:0]     t_tx_lat;
    reg [AW-1:0]   base;

    (* ASYNC_REG = "TRUE" *) reg [1:0] rdy_sync;
    wire ready = rdy_sync[1];

    assign dq_oe = 1'b1;

    // Next nibble, and whether it starts a new slot word.
    wire [NW-1:0] nn       = nidx + NW'(1);
    wire          new_word = (nn[1:0] == 2'd0);

    // The word about to be latched, with the t_tx substitution applied.
    wire [15:0] nxt_word = (wmod == M_TTXL) ? tick[15:0]
                         : (wmod == M_TTXH) ? t_tx_lat[31:16]
                                            : rd_data;

    function automatic [3:0] nib_of(input [15:0] w, input [1:0] s);
        case (s)
            2'd0:    nib_of = w[7:4];      // low byte first, high nibble first
            2'd1:    nib_of = w[3:0];
            2'd2:    nib_of = w[15:12];
            default: nib_of = w[11:8];
        endcase
    endfunction

    wire tock = (dcnt == D_LAST);
    wire fall = tock && sclk;      // this clock ends an SCLK period

    always @(posedge clk) begin
        if (!rst_n) rdy_sync <= 2'b00;
        else        rdy_sync <= {rdy_sync[0], esp_ready};
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            done_txn     <= 1'b0;
            dcnt         <= '0;
            bcnt         <= '0;
            pre          <= 16'd0;
            gcnt         <= '0;
            nidx         <= '0;
            cur          <= 16'd0;
            fetch_idx    <= '0;
            wmod         <= '0;
            t_tx_lat     <= 32'd0;
            base         <= '0;
            rd_en        <= 1'b0;
            rd_addr      <= '0;
            slot_release <= 1'b0;
            cs_n         <= 1'b1;
            sclk         <= 1'b0;
            dq_out       <= 4'h0;
            dgrams       <= 32'd0;
            stalls       <= 32'd0;
            dgram_sent   <= 1'b0;
        end else begin
            slot_release <= 1'b0;
            dgram_sent   <= 1'b0;

            // SCLK runs in every shifting state; the case below acts on the
            // falling edge that ends each SCLK period.
            if (state == S_PRE || state == S_DUMMY || state == S_DATA) begin
                dcnt <= tock ? '0 : dcnt + 1'b1;
                if (tock) sclk <= ~sclk;
            end

            case (state)
                S_IDLE: begin
                    cs_n  <= 1'b1;
                    sclk  <= 1'b0;
                    rd_en <= 1'b0;
                    // slot_release is registered, so on the cycle it is high
                    // the FIFO has not yet advanced its read pointer and
                    // slot_rd still names the slot just sent. Starting here
                    // would transmit that slot a second time.
                    if (slot_valid && !slot_release && ready) begin
                        state     <= S_PRE;
                        done_txn  <= 1'b0;
                        cs_n      <= 1'b0;
                        dcnt      <= '0;
                        bcnt      <= '0;
                        pre       <= {CMD_BYTE, ADDR_BYTE};
                        dq_out    <= {3'b000, CMD_BYTE[7]};
                        base      <= slot_rd * SLOT_WORDS;
                        fetch_idx <= '0;
                        wmod      <= '0;
                        // Hold the read port on the next word for the whole
                        // nibble before it, so a one-clock RAM is fast enough
                        // however DIV is set.
                        rd_en     <= 1'b1;
                        rd_addr   <= slot_rd * SLOT_WORDS;
                    end else if (slot_valid && !slot_release) begin
                        stalls <= stalls + 32'd1;
                    end
                end

                S_PRE: if (fall) begin
                    pre    <= {pre[14:0], 1'b0};
                    dq_out <= {3'b000, pre[14]};
                    if (bcnt == PRE_LAST) begin
                        state  <= S_DUMMY;
                        bcnt   <= '0;
                        dq_out <= 4'h0;
                    end else begin
                        bcnt <= bcnt + 1'b1;
                    end
                end

                S_DUMMY: if (fall) begin
                    if (bcnt != DUM_LAST) begin
                        bcnt <= bcnt + 1'b1;
                    end else if (done_txn) begin
                        state <= S_TAIL;
                    end else begin
                        // First data nibble: the same load as every later
                        // word boundary, at nibble index 0.
                        state  <= S_DATA;
                        nidx   <= '0;
                        cur    <= nxt_word;
                        dq_out <= nxt_word[7:4];
                        if (wmod == M_TTXL) t_tx_lat <= tick;
                        wmod <= (wmod == M_LAST) ? '0 : wmod + 1'b1;
                        if (fetch_idx != W_LAST) begin
                            fetch_idx <= fetch_idx + 1'b1;
                            rd_addr   <= base + AW'(fetch_idx) + AW'(1);
                        end
                    end
                end

                S_DATA: if (fall) begin
                    if (nidx == N_LAST) begin
                        state <= S_TAIL;
                        rd_en <= 1'b0;
                    end else begin
                        nidx <= nn;
                        if (new_word) begin
                            cur    <= nxt_word;
                            dq_out <= nxt_word[7:4];
                            if (wmod == M_TTXL) t_tx_lat <= tick;
                            wmod <= (wmod == M_LAST) ? '0 : wmod + 1'b1;
                            if (fetch_idx != W_LAST) begin
                                fetch_idx <= fetch_idx + 1'b1;
                                rd_addr   <= base + AW'(fetch_idx) + AW'(1);
                            end
                        end else begin
                            dq_out <= nib_of(cur, nn[1:0]);
                        end
                    end
                end

                S_TAIL: begin                      // cs hold time, SCLK low
                    sclk <= 1'b0;
                    if (!tock) begin
                        dcnt <= dcnt + 1'b1;
                    end else begin
                        dcnt   <= '0;
                        cs_n   <= 1'b1;
                        dq_out <= 4'h0;
                        gcnt   <= '0;
                        if (done_txn) begin
                            // The datagram is delivered: the slave has its
                            // CMD7 and the slot can be reused.
                            state        <= S_SEG;
                            slot_release <= 1'b1;
                            dgram_sent   <= 1'b1;
                            dgrams       <= dgrams + 32'd1;
                        end else begin
                            state <= S_GAP;
                        end
                    end
                end

                S_GAP: begin                       // between WRDMA and WR_DONE
                    if (gcnt != CS_GAP_LAST) begin
                        gcnt <= gcnt + 1'b1;
                    end else begin
                        state    <= S_PRE;
                        done_txn <= 1'b1;
                        cs_n     <= 1'b0;
                        dcnt     <= '0;
                        bcnt     <= '0;
                        pre      <= {DONE_BYTE, ADDR_BYTE};
                        dq_out   <= {3'b000, DONE_BYTE[7]};
                    end
                end

                default: begin                     // S_SEG: let the ESP re-arm
                    if (gcnt != SEG_GAP_LAST) gcnt  <= gcnt + 1'b1;
                    else                      state <= S_IDLE;
                end
            endcase

            if (clear) begin
                dgrams <= 32'd0;
                stalls <= 32'd0;
            end
        end
    end

endmodule

`default_nettype wire
