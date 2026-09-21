// ============================================================================
// pattern_gen.sv -- the test pattern, CONTRACTS.md section 3.
// ----------------------------------------------------------------------------
// Emits one sample of N_CH channels, one channel per clock, on a start strobe.
//
// This module and host/chain/pattern.py must agree bit for bit: the host
// regenerates the same values to score accuracy, so a disagreement here does not
// look like a pattern bug, it looks like the whole link is corrupting data.
//
// ALIGNMENT: emit, then advance. Sample 0 reports the seed itself. The state is
// read combinationally and the advance is clocked, which is why this shape is
// the natural one in RTL -- and why the contract pins it rather than leaving
// each implementation to pick.
//
// Two modes, both driven from the same LFSR state so only one array is needed:
//   CHECK     value = the LFSR state. Maximally sensitive to single-bit errors,
//             and incompressible, which is the point.
//   REALISTIC triangle wave with a per-channel phase offset plus +/-8 of LFSR
//             noise. Smooth, so the codec sees a plausible ratio.
// ============================================================================

`default_nettype none

module pattern_gen #(
    parameter int N_CH = 128
) (
    input  wire                      clk,
    input  wire                      rst_n,

    input  wire                      start,      // begin one sample
    input  wire                      realistic,  // 0 = CHECK, 1 = REALISTIC

    output reg                       valid,      // value/ch are live this cycle
    output reg  [$clog2(N_CH)-1:0]   ch,
    output reg  signed [15:0]        value,
    output reg                       done,       // pulses after the last channel
    output wire                      busy
);

    localparam int CHW = $clog2(N_CH);

    // x^16 + x^14 + x^13 + x^11, Fibonacci form. 0xB400 is the standard mask
    // for this tap set with a right shift; it is maximal length (period 65535).
    function automatic [15:0] lfsr_next(input [15:0] s);
        lfsr_next = (s >> 1) ^ (s[0] ? 16'hB400 : 16'h0000);
    endfunction

    // One LFSR state per channel. Small enough to infer as distributed RAM;
    // only one entry is read and one written per clock.
    reg [15:0] state [0:N_CH-1];

    reg [CHW-1:0]  idx;
    reg            running;
    reg [31:0]     sample_index;   // only the low 8 bits matter (triangle phase)
    integer        i;

    assign busy = running;

    wire [15:0] s_now = state[idx];

    // Triangle: phase = (sample_index + 8*ch) mod 256, peak-to-peak ~16 k.
    wire [7:0]  phase = sample_index[7:0] + {idx[4:0], 3'b000};
    wire signed [16:0] tri_raw = phase[7]
        ? ((17'sd255 - $signed({1'b0, phase})) <<< 7) - 17'sd8192
        : ( $signed({1'b0, phase})             <<< 7) - 17'sd8192;

    // Noise: top nibble of the LFSR state, recentred to -8..+7.
    wire signed [4:0] noise = $signed({1'b0, s_now[15:12]}) - 5'sd8;

    always @(posedge clk) begin
        if (!rst_n) begin
            for (i = 0; i < N_CH; i = i + 1)
                state[i] <= i[15:0] + 16'd1;   // seed(ch) = ch + 1; never 0
            idx          <= '0;
            running      <= 1'b0;
            valid        <= 1'b0;
            done         <= 1'b0;
            ch           <= '0;
            value        <= '0;
            sample_index <= '0;
        end else begin
            valid <= 1'b0;
            done  <= 1'b0;

            if (start && !running) begin
                running <= 1'b1;
                idx     <= '0;
            end else if (running) begin
                ch    <= idx;
                valid <= 1'b1;
                value <= realistic ? (tri_raw[15:0] + {{11{noise[4]}}, noise})
                                   : s_now;

                state[idx] <= lfsr_next(s_now);   // emit first, then advance

                if (idx == CHW'(N_CH - 1)) begin
                    running      <= 1'b0;
                    done         <= 1'b1;
                    sample_index <= sample_index + 1'b1;
                end else begin
                    idx <= idx + 1'b1;
                end
            end
        end
    end

endmodule

`default_nettype wire
