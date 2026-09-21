// ============================================================================
// spi_master.sv -- RHD2164 command sequencer and double-data-rate capture.
// ----------------------------------------------------------------------------
// Runs one sample: 35 SPI commands (32 CONVERT + 3 AUX), then idles until the
// next start strobe. Emits the resulting 128 channel values as a stream.
//
// STRUCTURE, and why it is a counter rather than Intan's 2300 unrolled states:
//
//   Rhythm's main.v spells out every one of the 80 fast-clock ticks in a command
//   as a named state, because each state also services DACs, ADCs and USB. None
//   of that exists here, so the same timing is two counters: which command we
//   are in, and which tick within it.
//
//   Per command, tick 0..79 at OS=4 ticks per SCLK bit:
//     tick <  64   16 SCLK bits. MOSI is set up on sub-tick 0, SCLK is high on
//                  sub-ticks 2 and 3, giving two ticks of setup before the edge.
//     tick >= 65   CS high. 15 ticks = 156 ns, over the RHD2164's 154 ns minimum.
//   MISO is recorded on every tick from 2 to 75 into a 74-tap window, which runs
//   past CS rising on purpose (see miso_phase_select.sv).
//
// THE TWO-COMMAND PIPELINE:
//
//   The RHD2164 returns a command's result two CS cycles later. Rather than
//   buffering, the result latched during command c is simply attributed to
//   command c-2 -- the same trick as Rhythm's `channel_MISO` counter. Because a
//   sweep is 35 commands and only the first 32 are CONVERTs, every CONVERT
//   result lands inside the same sweep (CONVERT(31) issued at command 31 comes
//   back at command 33), so no state crosses the idle gap between samples and
//   the very first sweep after reset is already correct.
//
// CHANNEL ORDER: chip 0 module A, chip 0 module B, chip 1 module A, chip 1
// module B, as CONTRACTS.md section 1 specifies. Module A is the rising-edge
// half of the DDR stream, module B the falling-edge half.
// ============================================================================

`default_nettype none

module spi_master #(
    parameter int BITS      = 16,
    parameter int OS        = 4,     // fast-clock ticks per SCLK bit
    parameter int TAIL      = 16,    // ticks after the last SCLK bit
    parameter int N_CMD     = 35,    // 32 CONVERT + 3 AUX
    parameter int N_CONVERT = 32,
    parameter int N_CH      = 128
) (
    input  wire                     clk,
    input  wire                     rst_n,

    input  wire                     start,          // begin one sample sweep
    input  wire [3:0]               phase_select,

    // SPI bus (single-ended; LVDS buffering, if ever needed, belongs in the top)
    output reg                      cs_n,
    output reg                      sclk,
    output reg                      mosi,
    input  wire                     miso0,          // chip 0
    input  wire                     miso1,          // chip 1

    // Channel stream
    output reg                      valid,
    output reg  [$clog2(N_CH)-1:0]  ch,
    output reg  signed [15:0]       value,
    output reg                      sample_done,
    output wire                     busy
);

    localparam int SCLK_TICKS   = BITS * OS;                 // 64
    localparam int CMD_TICKS    = SCLK_TICKS + TAIL;         // 80
    localparam int CS_HIGH_TICK = SCLK_TICKS + 1;            // 65
    localparam int TAPS         = SCLK_TICKS + OS*2 + 2;     // 74
    localparam int CAP_FIRST    = 2;                         // first tick recorded
    localparam int CAP_LAST     = CAP_FIRST + TAPS - 1;      // 75
    localparam int EMIT_TICK    = CAP_LAST + 1;              // 76: 4 emits, 4 ticks left
    localparam int CHW          = $clog2(N_CH);
    localparam int TW           = $clog2(CMD_TICKS);

    // Tick constants as sized values. A bit-select on a parenthesised
    // expression is not legal, so name each one instead of casting inline.
    localparam [TW-1:0] T_SCLK_END = TW'(SCLK_TICKS);
    localparam [TW-1:0] T_CS_HIGH  = TW'(CS_HIGH_TICK);
    localparam [TW-1:0] T_CAP_LO   = TW'(CAP_FIRST);
    localparam [TW-1:0] T_CAP_HI   = TW'(CAP_LAST);
    localparam [TW-1:0] T_EMIT_A   = TW'(EMIT_TICK);
    localparam [TW-1:0] T_EMIT_B   = TW'(EMIT_TICK + 1);
    localparam [TW-1:0] T_EMIT_C   = TW'(EMIT_TICK + 2);
    localparam [TW-1:0] T_EMIT_D   = TW'(EMIT_TICK + 3);
    localparam [TW-1:0] T_LAST     = TW'(CMD_TICKS - 1);

    // ------------------------------------------------------------------
    // Command ROM. CONVERT(c) = 00 cccccc 00000000, READ(r) = 11 rrrrrr 00000000.
    // The three AUX slots read the ROM identity registers (40..42 spell "INT"),
    // so even an idle sweep carries a liveness signal a scope or the closed-loop
    // testbench can recognise.
    // ------------------------------------------------------------------
    function automatic [BITS-1:0] command_for(input [5:0] idx);
        command_for = (idx < N_CONVERT[5:0]) ? {2'b00, idx, 8'h00}
                                             : {2'b11, (6'd40 + (idx - N_CONVERT[5:0])), 8'h00};
    endfunction

    reg [$clog2(N_CMD)-1:0] cmd_idx;
    reg [TW-1:0] tick;
    reg                     running;
    reg [BITS-1:0]          shifter;

    reg [TAPS-1:0] in4x0, in4x1;

    wire [BITS-1:0] sdr0, ddr0, sdr1, ddr1;

    miso_phase_select #(.BITS(BITS), .OS(OS), .TAPS(TAPS)) u_sel0 (
        .in4x(in4x0), .phase(phase_select), .word_sdr(sdr0), .word_ddr(ddr0));

    miso_phase_select #(.BITS(BITS), .OS(OS), .TAPS(TAPS)) u_sel1 (
        .in4x(in4x1), .phase(phase_select), .word_sdr(sdr1), .word_ddr(ddr1));

    assign busy = running;

    // Results are attributed to the command issued two cycles earlier. Commands
    // 2..33 therefore carry CONVERT results for channels 0..31.
    wire                 emit_window = running && (cmd_idx >= 2) && (cmd_idx <= (N_CONVERT + 1));
    wire [CHW-1:0]       base_ch     = CHW'(cmd_idx - 2);

    wire [5:0] next_cmd = 6'(cmd_idx) + 6'd1;
    wire [$clog2(BITS)-1:0] bit_idx = tick[TW-1:$clog2(OS)];
    wire [$clog2(OS)-1:0]   sub     = tick[$clog2(OS)-1:0];

    always @(posedge clk) begin
        if (!rst_n) begin
            cs_n        <= 1'b1;
            sclk        <= 1'b0;
            mosi        <= 1'b0;
            cmd_idx     <= '0;
            tick        <= '0;
            running     <= 1'b0;
            shifter     <= '0;
            in4x0       <= '0;
            in4x1       <= '0;
            valid       <= 1'b0;
            ch          <= '0;
            value       <= '0;
            sample_done <= 1'b0;
        end else begin
            valid       <= 1'b0;
            sample_done <= 1'b0;

            if (!running) begin
                cs_n <= 1'b1;
                sclk <= 1'b0;
                if (start) begin
                    running <= 1'b1;
                    cmd_idx <= '0;
                    tick    <= '0;
                    shifter <= command_for(6'd0);
                end
            end else begin
                // ---- bus timing ----
                cs_n <= (tick >= T_CS_HIGH);

                if (tick < T_SCLK_END) begin
                    if (sub == '0) mosi <= shifter[BITS-1-bit_idx];
                    sclk <= (sub >= (OS/2));
                end else begin
                    sclk <= 1'b0;
                    mosi <= 1'b0;
                end

                // ---- oversampled capture ----
                if (tick >= T_CAP_LO && tick <= T_CAP_HI) begin
                    in4x0[tick - T_CAP_LO] <= miso0;
                    in4x1[tick - T_CAP_LO] <= miso1;
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
                            if (cmd_idx == (N_CONVERT + 1)) sample_done <= 1'b1;
                        end
                        default: ;
                    endcase
                end

                // ---- advance ----
                if (tick == T_LAST) begin
                    tick <= '0;
                    if (cmd_idx == (N_CMD-1)) begin
                        running <= 1'b0;
                    end else begin
                        cmd_idx <= cmd_idx + 1'b1;
                        shifter <= command_for(next_cmd);
                    end
                end else begin
                    tick <= tick + 1'b1;
                end
            end
        end
    end

endmodule

`default_nettype wire
