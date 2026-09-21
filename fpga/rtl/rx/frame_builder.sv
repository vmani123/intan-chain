// ============================================================================
// frame_builder.sv -- channel stream in, finished datagrams in slot memory out.
// ----------------------------------------------------------------------------
// Takes the stream that spi_master and pattern_gen both produce and lays out
// CONTRACTS.md sections 1 and 2 directly in the slot RAM, as 16-bit little
// endian words:
//
//   frame    = 10 header words + N_CH payload words        (138 words)
//   datagram = FRAMES_PER_DGRAM frames                     (690 words)
//
// The ESP appends the 12-byte trailer, so nothing is reserved for it here.
//
// WHY THE PAYLOAD IS INDEXED BY `ch` AND NOT BY AN ARRIVAL COUNTER
//   spi_master emits channels 0, 32, 64, 96, then 1, 33, 65, 97, ... because
//   that is the order the RHD2164's double-data-rate MISO produces them. An
//   arrival counter would therefore write a permuted frame that still has the
//   right length and the right header, which the host cannot distinguish from
//   real data. Writing to slot_base + frame*138 + 10 + ch makes the order of
//   arrival irrelevant.
//
// WHY t_tx IS LEFT AT ZERO
//   t_tx must be the moment the frame actually leaves the FPGA, which is not
//   known until the ESP accepts the transfer. qspi_master substitutes the live
//   tick for words 6 and 7 as it shifts the slot out, so t_tx - t_cap measures
//   the real queueing time, including the wait for the ESP.
//
// BACK PRESSURE
//   `stall` means the FIFO has no free slot. The decision is taken once, at the
//   first word of a datagram, and then held: dropping a whole datagram is
//   recoverable (the host sees a gap in `seq`), whereas abandoning one halfway
//   leaves a slot that parses as valid and is quietly wrong. `seq` still
//   advances across a dropped datagram, so the loss is visible on the host.
//
// `busy` is high while the 10 header words are being written. The single write
// port is shared with the payload, so the top level must not start a new sample
// while it is high.
// ============================================================================

`default_nettype none

module frame_builder #(
    parameter int N_CH             = 128,
    parameter int HDR_WORDS        = 10,
    parameter int FRAMES_PER_DGRAM = 5,
    parameter int NSLOT            = 4,
    parameter int SW               = $clog2(NSLOT),
    parameter int AW               = $clog2(NSLOT * FRAMES_PER_DGRAM
                                            * (HDR_WORDS + N_CH)),
    parameter logic [15:0] MAGIC   = 16'hA55A
) (
    input  wire                      clk,
    input  wire                      rst_n,
    input  wire                      clear,        // zero frames and seq

    // channel stream
    input  wire                      valid,
    input  wire [$clog2(N_CH)-1:0]   ch,
    input  wire signed [15:0]        value,
    input  wire                      sample_done,

    input  wire [7:0]                flags,
    input  wire [7:0]                frame_type,
    input  wire [31:0]               tick,         // free-running time base

    input  wire                      stall,        // no free slot
    input  wire [SW-1:0]             slot_wr,

    output reg                       wr_en,
    output reg  [AW-1:0]             wr_addr,
    output reg  [15:0]               wr_data,

    output reg                       commit,       // slot_wr is now complete
    output reg                       overrun,      // a datagram was dropped
    output reg  [31:0]               frames,
    output wire                      busy
);

    localparam int FRAME_WORDS = HDR_WORDS + N_CH;
    localparam int SLOT_WORDS  = FRAMES_PER_DGRAM * FRAME_WORDS;
    localparam int FW          = $clog2(FRAMES_PER_DGRAM + 1);

    localparam [15:0] N_CH_W    = 16'(N_CH);
    localparam [15:0] PAYLEN_W  = 16'(N_CH * 2);
    localparam [FW-1:0] LAST_FR = FW'(FRAMES_PER_DGRAM - 1);
    localparam [3:0]  HDR_LAST  = 4'(HDR_WORDS - 1);

    reg [31:0]   seq;
    reg [31:0]   t_cap;
    reg [FW-1:0] frame_idx;
    reg [AW-1:0] frame_off;      // frame_idx * FRAME_WORDS, as an accumulator
    reg          armed;          // a datagram is in progress
    reg          dropping;       // ... and it is being discarded
    reg          hdr_run;
    reg [3:0]    hdr_cnt;

    assign busy = hdr_run;

    wire [AW-1:0] slot_base = slot_wr * SLOT_WORDS;
    wire [AW-1:0] pay_addr  = slot_base + frame_off + AW'(HDR_WORDS) + AW'(ch);
    wire [AW-1:0] hdr_addr  = slot_base + frame_off + AW'(hdr_cnt);

    // Decided at the first word of a datagram and held for all of it.
    wire drop_now = armed ? dropping : stall;

    function automatic [15:0] hdr_word(input [3:0] k);
        case (k)
            4'd0:    hdr_word = MAGIC;
            4'd1:    hdr_word = {flags, frame_type};   // little-endian: type low
            4'd2:    hdr_word = seq[15:0];
            4'd3:    hdr_word = seq[31:16];
            4'd4:    hdr_word = t_cap[15:0];
            4'd5:    hdr_word = t_cap[31:16];
            4'd6:    hdr_word = 16'd0;                 // t_tx, patched on egress
            4'd7:    hdr_word = 16'd0;
            4'd8:    hdr_word = N_CH_W;
            default: hdr_word = PAYLEN_W;
        endcase
    endfunction

    always @(posedge clk) begin
        if (!rst_n) begin
            wr_en     <= 1'b0;
            wr_addr   <= '0;
            wr_data   <= '0;
            commit    <= 1'b0;
            overrun   <= 1'b0;
            frames    <= 32'd0;
            seq       <= 32'd0;
            t_cap     <= 32'd0;
            frame_idx <= '0;
            frame_off <= '0;
            armed     <= 1'b0;
            dropping  <= 1'b0;
            hdr_run   <= 1'b0;
            hdr_cnt   <= 4'd0;
        end else begin
            wr_en   <= 1'b0;
            commit  <= 1'b0;
            overrun <= 1'b0;

            if (hdr_run) begin
                wr_en   <= 1'b1;
                wr_addr <= hdr_addr;
                wr_data <= hdr_word(hdr_cnt);
                if (hdr_cnt == HDR_LAST) begin
                    hdr_run <= 1'b0;
                    frames  <= frames + 32'd1;
                    seq     <= seq + 32'd1;
                    if (frame_idx == LAST_FR) begin
                        commit    <= 1'b1;
                        frame_idx <= '0;
                        frame_off <= '0;
                        armed     <= 1'b0;
                    end else begin
                        frame_idx <= frame_idx + 1'b1;
                        frame_off <= frame_off + AW'(FRAME_WORDS);
                    end
                end else begin
                    hdr_cnt <= hdr_cnt + 1'b1;
                end
            end else begin
                if (valid || sample_done) begin
                    if (!armed) begin
                        armed    <= 1'b1;
                        dropping <= stall;
                    end
                end

                if (valid && !drop_now) begin
                    wr_en   <= 1'b1;
                    wr_addr <= pay_addr;
                    wr_data <= value;
                end

                // sample_done arrives on the same clock as the last payload
                // word, so the header phase starts on the clock after it.
                if (sample_done) begin
                    if (!drop_now) begin
                        t_cap   <= tick;
                        hdr_run <= 1'b1;
                        hdr_cnt <= 4'd0;
                    end else begin
                        seq <= seq + 32'd1;        // keep the gap visible
                        if (frame_idx == LAST_FR) begin
                            overrun   <= 1'b1;
                            frame_idx <= '0;
                            frame_off <= '0;
                            armed     <= 1'b0;
                        end else begin
                            frame_idx <= frame_idx + 1'b1;
                            frame_off <= frame_off + AW'(FRAME_WORDS);
                        end
                    end
                end
            end

            if (clear) begin
                frames <= 32'd0;
                seq    <= 32'd0;
            end
        end
    end

endmodule

`default_nettype wire
