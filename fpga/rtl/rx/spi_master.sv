// ============================================================================
// spi_master.sv -- RHD2164 command sequencer and double-data-rate capture.
// ----------------------------------------------------------------------------
// Two jobs share one bus engine:
//
//   SWEEP  (`start`)   one sample: 35 SPI commands (32 CONVERT + 3 AUX), the
//                      resulting 128 channel values emitted as a stream, then
//                      idle until the next start.
//   LIST   (`diag_go`) the diagnostic list: commands 0..DIAG_LEN-1 from the
//                      command memory, back to back with sweep timing, each
//                      cycle's decoded words and raw MISO window written to the
//                      result memory by cycle index. Nothing is emitted, so the
//                      frame builder never sees a list.
//
// STRUCTURE, and why it is a counter rather than Intan's 2300 unrolled states:
//
//   Rhythm's main.v spells out every one of the 80 fast-clock ticks in a command
//   as a named state, because each state also services DACs, ADCs and USB. None
//   of that exists here, so the same timing is three counters: which command we
//   are in, which SPI tick within it, and which fast clock within the tick.
//
//   Per command, tick 0..83 at OS=4 ticks per SCLK bit (CONTRACTS.md section 6):
//     tick <  64   16 SCLK bits. MOSI is set up on sub-tick 0, SCLK is high on
//                  sub-ticks 2 and 3, giving two ticks of setup before the edge.
//     tick >= 65   CS high. 19 ticks = 198 ns, over the RHD2164's 154 ns minimum.
//   MISO is recorded on every tick from 2 to 79 into a 78-tap window, which runs
//   past CS rising on purpose (see miso_phase_select.sv): 78 taps is what lets
//   all sixteen PHASE_SELECT values 0..15 read a whole word. The four channel
//   words are emitted on ticks 80..83, after the window has closed.
//
// SPI_DIV, THE TICK ENABLE:
//
//   One SPI tick is SPI_DIV+1 fast clocks. Only the inside of a running cycle
//   -- bus timing, capture, emits -- waits for the enable; the idle state looks
//   at `start` and `diag_go` on every clock, so a strobe can never fall between
//   two enables and be lost. The prescaler restarts with every sweep or list,
//   so the first tick always comes one clock after the start. Output strobes
//   (valid, sample_done and the rest) are cleared on every clock, so they stay
//   one fast clock wide whatever SPI_DIV is. At SPI_DIV = 0 the enable is
//   always on and the timing is exactly that of the undivided master.
//
// THE TWO-COMMAND PIPELINE:
//
//   The RHD2164 returns a command's result two CS cycles later. Rather than
//   buffering, the result latched during command c is simply attributed to
//   command c-2 -- the same trick as Rhythm's `channel_MISO` counter. Because a
//   sweep is 35 commands and only the first 32 are CONVERTs, every CONVERT
//   result lands inside the same sweep (CONVERT(31) issued at command 31 comes
//   back at command 33), so no state crosses the idle gap between samples and
//   the very first sweep after reset is already correct. The AUX slot READ(59)
//   in cycle 32 comes back in cycle 34, which is what the link check reads.
//
//   A list is NOT re-aligned: cycle i is stored as cycle i and holds the result
//   of command i-2. The host knows the rule and asks for two trailing dummies.
//
// THE HOLDING REGISTER:
//
//   The raw window (2 x 78 taps) and its four decoded words are copied into the
//   diag_wr_* / link_words output registers on tick 80, the first tick after
//   the window closes, and only then written to memory (next clock) or
//   compared. The next cycle starts overwriting the taps at its tick 2, six
//   ticks after the copy, so neither the write nor the compare ever reads a
//   tap the capture is changing, whatever SPI_DIV is.
//
// CHANNEL ORDER: chip 0 module A, chip 0 module B, chip 1 module A, chip 1
// module B, as CONTRACTS.md section 1 specifies. Module A is the word selector
// miso_phase_select calls `word_sdr`, module B `word_ddr`.
// ============================================================================

`default_nettype none

module spi_master #(
    parameter int BITS       = 16,
    parameter int OS         = 4,     // SPI ticks per SCLK bit
    parameter int TAIL       = 20,    // ticks after the last SCLK bit
    parameter int N_CMD      = 35,    // 32 CONVERT + 3 AUX
    parameter int N_CONVERT  = 32,
    parameter int N_CH       = 128,
    parameter int DIAG_DEPTH = 64,
    // Capture taps per CS cycle. A parameter only so the caller can size the
    // raw-window ports; the value is fixed by BITS, OS and the 4-bit phase
    // (TAPS_NEEDED below) and checked when the design is elaborated.
    parameter int TAPS       = 78
) (
    input  wire                     clk,
    input  wire                     rst_n,

    input  wire                     start,          // begin one sample sweep
    input  wire [3:0]               phase_select,   // 0..15, in SPI ticks
    input  wire [3:0]               spi_div,        // one SPI tick = spi_div+1 clocks

    // Diagnostic list. `diag_go` is honoured when idle; one that arrives during
    // a sweep is held and played as soon as the sweep ends, and one that
    // arrives during a list is ignored.
    input  wire                     diag_go,
    input  wire [$clog2(DIAG_DEPTH):0] diag_len,    // 1..DIAG_DEPTH, latched at go
    output wire [$clog2(DIAG_DEPTH)-1:0] diag_cmd_addr,
    input  wire [BITS-1:0]          diag_cmd,       // command memory, read asynchronously
    output reg                      diag_wr_en,     // one clock per list cycle
    output reg  [$clog2(DIAG_DEPTH)-1:0] diag_wr_addr,
    output reg  [63:0]              diag_wr_res,    // {c1 B, c1 A, c0 B, c0 A}
    output reg  [TAPS-1:0]          diag_wr_raw0,   // bit k = tap k of miso0
    output reg  [TAPS-1:0]          diag_wr_raw1,
    output wire                     diag_busy,      // held or playing
    output reg                      diag_done,      // one clock, after the last write

    // Link check: the four words of sweep cycle 34, the result of READ(59)
    output reg                      link_stb,
    output reg  [63:0]              link_words,     // {c1 B, c1 A, c0 B, c0 A}

    output reg                      cycle_stb,      // one clock per CS cycle issued

    // SPI bus (single-ended; LVDS buffering, if ever needed, belongs in the top)
    output reg                      cs_n,
    output reg                      sclk,
    output reg                      mosi,
    input  wire                     miso0,          // chip 0
    input  wire                     miso1,          // chip 1

    // Channel stream (sweeps only)
    output reg                      valid,
    output reg  [$clog2(N_CH)-1:0]  ch,
    output reg  signed [15:0]       value,
    output reg                      sample_done,
    output wire                     busy            // sweep, list, or a list held
);

    localparam int SCLK_TICKS   = BITS * OS;                 // 64
    localparam int CMD_TICKS    = SCLK_TICKS + TAIL;         // 84
    localparam int CS_HIGH_TICK = SCLK_TICKS + 1;            // 65
    localparam int PHASE_MAX    = 15;                        // 4-bit phase_select
    localparam int TAPS_NEEDED  = PHASE_MAX + OS/2 + OS*(BITS-1) + 1;   // 78
    localparam int CAP_FIRST    = 2;                         // first tick recorded
    localparam int CAP_LAST     = CAP_FIRST + TAPS - 1;      // 79
    localparam int EMIT_TICK    = CAP_LAST + 1;              // 80: 4 emits, cycle ends
    localparam int CHW          = $clog2(N_CH);
    localparam int TW           = $clog2(CMD_TICKS);
    localparam int DAW          = $clog2(DIAG_DEPTH);
    localparam int CIW          = (DAW > $clog2(N_CMD)) ? DAW : $clog2(N_CMD);
    localparam int SW           = $clog2(OS);

    // Tick constants as sized values. A bit-select on a parenthesised
    // expression is not legal, so name each one instead of casting inline.
    localparam [TW-1:0] T_FIRST    = '0;
    localparam [TW-1:0] T_SCLK_END = TW'(SCLK_TICKS);
    localparam [TW-1:0] T_CS_HIGH  = TW'(CS_HIGH_TICK);
    localparam [TW-1:0] T_CAP_LO   = TW'(CAP_FIRST);
    localparam [TW-1:0] T_CAP_HI   = TW'(CAP_LAST);
    localparam [TW-1:0] T_EMIT_A   = TW'(EMIT_TICK);
    localparam [TW-1:0] T_EMIT_B   = TW'(EMIT_TICK + 1);
    localparam [TW-1:0] T_EMIT_C   = TW'(EMIT_TICK + 2);
    localparam [TW-1:0] T_EMIT_D   = TW'(EMIT_TICK + 3);
    localparam [TW-1:0] T_LAST     = TW'(CMD_TICKS - 1);

    localparam [CIW-1:0] C_SWEEP_LAST = CIW'(N_CMD - 1);     // cycle 34: link check
    localparam [CIW-1:0] C_DONE       = CIW'(N_CONVERT + 1); // cycle 33: last CONVERT result

    // synthesis translate_off
    initial begin
        if (TAPS != TAPS_NEEDED)
            $fatal(1, "spi_master: TAPS=%0d, phase 0..%0d needs %0d", TAPS, PHASE_MAX, TAPS_NEEDED);
        if (EMIT_TICK + 3 != CMD_TICKS - 1)
            $fatal(1, "spi_master: the four emits must fill ticks %0d..%0d", EMIT_TICK, CMD_TICKS - 1);
    end
    // synthesis translate_on

    // ------------------------------------------------------------------
    // Sweep command ROM. CONVERT(c) = 00 cccccc 00000000, READ(r) = 11 rrrrrr
    // 00000000. The AUX slots are fixed (CONTRACTS.md section 6): READ(59), the
    // module A/B marker the link check compares, then READ(40) 'I' and READ(63)
    // chip ID, so even an idle sweep carries a liveness signal a scope can see.
    // ------------------------------------------------------------------
    function automatic [BITS-1:0] command_for(input [CIW-1:0] idx);
        case (idx)
            CIW'(N_CONVERT):     command_for = {2'b11, 6'd59, 8'h00};
            CIW'(N_CONVERT + 1): command_for = {2'b11, 6'd40, 8'h00};
            CIW'(N_CONVERT + 2): command_for = {2'b11, 6'd63, 8'h00};
            default:             command_for = {2'b00, 6'(idx), 8'h00};
        endcase
    endfunction

    reg [CIW-1:0]  cmd_idx;
    reg [CIW-1:0]  last_idx;       // N_CMD-1 for a sweep, DIAG_LEN-1 for a list
    reg [TW-1:0]   tick;
    reg [3:0]      div_cnt;        // fast clocks left before the next SPI tick
    reg            running;
    reg            list_mode;
    reg            diag_pend;
    reg [BITS-1:0] shifter;

    reg [TAPS-1:0] in4x0, in4x1;

    wire [BITS-1:0] sdr0, ddr0, sdr1, ddr1;

    miso_phase_select #(.BITS(BITS), .OS(OS), .TAPS(TAPS)) u_sel0 (
        .in4x(in4x0), .phase(phase_select), .word_sdr(sdr0), .word_ddr(ddr0));

    miso_phase_select #(.BITS(BITS), .OS(OS), .TAPS(TAPS)) u_sel1 (
        .in4x(in4x1), .phase(phase_select), .word_sdr(sdr1), .word_ddr(ddr1));

    wire ce = (div_cnt == 4'd0);

    assign busy      = running || diag_pend;
    assign diag_busy = (running && list_mode) || diag_pend;

    // Results are attributed to the command issued two cycles earlier. Sweep
    // commands 2..33 therefore carry CONVERT results for channels 0..31.
    wire                 emit_window = running && !list_mode && (cmd_idx >= 2)
                                       && (cmd_idx <= C_DONE);
    wire [CHW-1:0]       base_ch     = CHW'(cmd_idx - 2);

    wire [CIW-1:0] next_cmd = cmd_idx + 1'b1;
    wire [$clog2(BITS)-1:0] bit_idx = tick[SW +: $clog2(BITS)];
    wire [SW-1:0]           sub     = tick[SW-1:0];

    // The list memory is read asynchronously, so the address only has to be
    // right on the clock that loads the shifter: entry 0 while idle, the next
    // entry while a cycle runs.
    assign diag_cmd_addr = running ? DAW'(next_cmd) : '0;

    always @(posedge clk) begin
        if (!rst_n) begin
            cs_n         <= 1'b1;
            sclk         <= 1'b0;
            mosi         <= 1'b0;
            cmd_idx      <= '0;
            last_idx     <= '0;
            tick         <= '0;
            div_cnt      <= '0;
            running      <= 1'b0;
            list_mode    <= 1'b0;
            diag_pend    <= 1'b0;
            shifter      <= '0;
            in4x0        <= '0;
            in4x1        <= '0;
            valid        <= 1'b0;
            ch           <= '0;
            value        <= '0;
            sample_done  <= 1'b0;
            cycle_stb    <= 1'b0;
            link_stb     <= 1'b0;
            link_words   <= '0;
            diag_wr_en   <= 1'b0;
            diag_wr_addr <= '0;
            diag_wr_res  <= '0;
            diag_wr_raw0 <= '0;
            diag_wr_raw1 <= '0;
            diag_done    <= 1'b0;
        end else begin
            valid       <= 1'b0;
            sample_done <= 1'b0;
            cycle_stb   <= 1'b0;
            link_stb    <= 1'b0;
            diag_wr_en  <= 1'b0;
            diag_done   <= 1'b0;

            if (!running) begin
                cs_n    <= 1'b1;
                sclk    <= 1'b0;
                tick    <= '0;
                div_cnt <= '0;      // first tick on the next clock
                cmd_idx <= '0;
                if (diag_go || diag_pend) begin
                    running   <= 1'b1;
                    list_mode <= 1'b1;
                    diag_pend <= 1'b0;
                    last_idx  <= CIW'(diag_len - 1'b1);
                    shifter   <= diag_cmd;              // entry 0
                end else if (start) begin
                    running   <= 1'b1;
                    list_mode <= 1'b0;
                    last_idx  <= C_SWEEP_LAST;
                    shifter   <= command_for('0);
                end
            end else begin
                if (diag_go && !list_mode) diag_pend <= 1'b1;

                if (!ce) begin
                    div_cnt <= div_cnt - 1'b1;
                end else begin
                    div_cnt <= spi_div;

                    // ---- bus timing ----
                    cs_n <= (tick >= T_CS_HIGH);

                    if (tick < T_SCLK_END) begin
                        if (sub == '0) mosi <= shifter[BITS-1-bit_idx];
                        sclk <= (sub >= SW'(OS/2));
                    end else begin
                        sclk <= 1'b0;
                        mosi <= 1'b0;
                    end

                    if (tick == T_FIRST) cycle_stb <= 1'b1;

                    // ---- oversampled capture ----
                    if (tick >= T_CAP_LO && tick <= T_CAP_HI) begin
                        in4x0[tick - T_CAP_LO] <= miso0;
                        in4x1[tick - T_CAP_LO] <= miso1;
                    end

                    // ---- window closed: hold it, for the list or the link check ----
                    if (tick == T_EMIT_A) begin
                        if (list_mode) begin
                            diag_wr_en   <= 1'b1;
                            diag_wr_addr <= DAW'(cmd_idx);
                            diag_wr_res  <= {ddr1, sdr1, ddr0, sdr0};
                            diag_wr_raw0 <= in4x0;
                            diag_wr_raw1 <= in4x1;
                        end else if (cmd_idx == C_SWEEP_LAST) begin
                            link_stb     <= 1'b1;
                            link_words   <= {ddr1, sdr1, ddr0, sdr0};
                        end
                    end

                    // ---- emit the four channel words this command produced ----
                    if (emit_window) begin
                        case (tick)
                            T_EMIT_A: begin
                                valid <= 1'b1; ch <= base_ch;              value <= sdr0;
                            end
                            T_EMIT_B: begin
                                valid <= 1'b1; ch <= base_ch + CHW'(32);   value <= ddr0;
                            end
                            T_EMIT_C: begin
                                valid <= 1'b1; ch <= base_ch + CHW'(64);   value <= sdr1;
                            end
                            T_EMIT_D: begin
                                valid <= 1'b1; ch <= base_ch + CHW'(96);   value <= ddr1;
                                // Last CONVERT result of the sweep completes the sample.
                                if (cmd_idx == C_DONE) sample_done <= 1'b1;
                            end
                            default: ;
                        endcase
                    end

                    // ---- advance ----
                    if (tick == T_LAST) begin
                        tick <= '0;
                        if (cmd_idx == last_idx) begin
                            running <= 1'b0;
                            if (list_mode) diag_done <= 1'b1;
                        end else begin
                            cmd_idx <= next_cmd;
                            shifter <= list_mode ? diag_cmd : command_for(next_cmd);
                        end
                    end else begin
                        tick <= tick + 1'b1;
                    end
                end
            end
        end
    end

endmodule

`default_nettype wire
