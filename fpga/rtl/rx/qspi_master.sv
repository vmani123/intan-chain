// ============================================================================
// qspi_master.sv -- shift one finished slot out to the ESP32-S3.
// ----------------------------------------------------------------------------
// One quad-SPI write transaction per datagram:
//
//   esp_ready high -> cs_n low -> CMD byte -> ADDR byte -> 1380 data bytes
//   -> cs_n high
//
// All four phases go out on the four data lines, most significant nibble of
// each byte first, and slot words are sent low byte first because the whole
// contract is little-endian. SCLK is clk / (2 * DIV), so DIV = 2 at 96 MHz
// gives 24 MHz; one nibble occupies one SCLK period. dq_out changes while SCLK
// is low and is stable across the rising edge, which is SPI mode 0.
//
// THE READY LINE IS THE ONLY BACK PRESSURE IN THE SYSTEM
//   The ESP slave silently discards a transaction clocked in with no DMA buffer
//   armed -- no error, no counter -- which looks exactly like a radio ceiling.
//   So `stalls` counts every clock spent holding a finished datagram while
//   esp_ready is low. A run with a flat throughput curve and stalls at zero
//   means the ready line is not wired; a rolled-off curve with stalls climbing
//   means the ESP, not the radio, is the limit.
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
    parameter logic [7:0] CMD_BYTE  = 8'h23,     // slave-HD 4-line DMA write
    parameter logic [7:0] ADDR_BYTE = 8'h00,
    parameter int SW               = $clog2(NSLOT),
    parameter int AW               = $clog2(NSLOT * FRAMES_PER_DGRAM
                                            * (HDR_WORDS + N_CH))
) (
    input  wire            clk,
    input  wire            rst_n,
    input  wire            clear,          // zero the counters

    input  wire            esp_ready,

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
    localparam int NIB_TOTAL   = 4 + 2 * DATA_BYTES;        // cmd+addr+data
    localparam int NW          = $clog2(NIB_TOTAL);
    localparam int WW          = $clog2(SLOT_WORDS);
    localparam int MW          = $clog2(FRAME_WORDS);
    localparam int DW          = (DIV <= 1) ? 1 : $clog2(DIV);

    // Bit-selects on parenthesised expressions are not legal, so every sized
    // constant the comparisons need gets a name.
    localparam [NW-1:0]  N_LAST  = NW'(NIB_TOTAL - 1);
    localparam [NW-1:0]  N_FOUR  = NW'(4);
    localparam [NW-1:0]  N_THREE = NW'(3);
    localparam [NW-1:0]  N_ZERO  = NW'(0);
    localparam [NW-1:0]  N_ONE   = NW'(1);
    localparam [NW-1:0]  N_TWO   = NW'(2);
    localparam [WW-1:0]  W_LAST  = WW'(SLOT_WORDS - 1);
    localparam [MW-1:0]  M_LAST  = MW'(FRAME_WORDS - 1);
    localparam [MW-1:0]  M_TTXL  = MW'(6);
    localparam [MW-1:0]  M_TTXH  = MW'(7);
    localparam [DW-1:0]  D_LAST  = DW'(DIV - 1);

    localparam [1:0] S_IDLE = 2'd0, S_XFER = 2'd1, S_TAIL = 2'd2;

    reg [1:0]      state;
    reg [DW-1:0]   dcnt;
    reg [NW-1:0]   nidx;           // nibble index within the transaction
    reg [15:0]     cur;            // word currently being shifted
    reg [WW-1:0]   fetch_idx;      // word the read port is presenting
    reg [MW-1:0]   wmod;           // fetch_idx mod FRAME_WORDS
    reg [31:0]     t_tx_lat;
    reg [AW-1:0]   base;

    assign dq_oe = 1'b1;

    // Next nibble, and whether it starts a new slot word.
    wire [NW-1:0] nn       = nidx + NW'(1);
    wire [NW-1:0] nsub_w   = (nn - N_FOUR) & N_THREE;
    wire          new_word = (nn >= N_FOUR) && (nsub_w == '0);

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

    function automatic [3:0] hdr_nib(input [NW-1:0] n);
        if      (n == N_ZERO) hdr_nib = CMD_BYTE[7:4];
        else if (n == N_ONE)  hdr_nib = CMD_BYTE[3:0];
        else if (n == N_TWO)  hdr_nib = ADDR_BYTE[7:4];
        else                  hdr_nib = ADDR_BYTE[3:0];
    endfunction

    wire tock = (dcnt == D_LAST);

    always @(posedge clk) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            dcnt         <= '0;
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

            case (state)
                S_IDLE: begin
                    cs_n  <= 1'b1;
                    sclk  <= 1'b0;
                    rd_en <= 1'b0;
                    // slot_release is registered, so on the cycle it is high
                    // the FIFO has not yet advanced its read pointer and
                    // slot_rd still names the slot just sent. Starting here
                    // would transmit that slot a second time.
                    if (slot_valid && !slot_release && esp_ready) begin
                        state     <= S_XFER;
                        cs_n      <= 1'b0;
                        dcnt      <= '0;
                        nidx      <= '0;
                        dq_out    <= hdr_nib(N_ZERO);
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

                S_XFER: begin
                    if (!tock) begin
                        dcnt <= dcnt + 1'b1;
                    end else begin
                        dcnt <= '0;
                        sclk <= ~sclk;
                        if (sclk) begin            // falling edge ends a nibble
                            if (nidx == N_LAST) begin
                                state <= S_TAIL;
                                rd_en <= 1'b0;
                            end else begin
                                nidx <= nn;
                                if (nn < N_FOUR) begin
                                    dq_out <= hdr_nib(nn);
                                end else if (new_word) begin
                                    cur    <= nxt_word;
                                    dq_out <= nxt_word[7:4];
                                    if (wmod == M_TTXL) t_tx_lat <= tick;
                                    wmod <= (wmod == M_LAST) ? '0 : wmod + 1'b1;
                                    if (fetch_idx != W_LAST) begin
                                        fetch_idx <= fetch_idx + 1'b1;
                                        rd_addr   <= base + AW'(fetch_idx)
                                                          + AW'(1);
                                    end
                                end else begin
                                    dq_out <= nib_of(cur, nsub_w[1:0]);
                                end
                            end
                        end
                    end
                end

                default: begin                     // S_TAIL: cs hold time
                    sclk <= 1'b0;
                    if (!tock) begin
                        dcnt <= dcnt + 1'b1;
                    end else begin
                        dcnt         <= '0;
                        state        <= S_IDLE;
                        cs_n         <= 1'b1;
                        slot_release <= 1'b1;
                        dgram_sent   <= 1'b1;
                        dgrams       <= dgrams + 32'd1;
                    end
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
