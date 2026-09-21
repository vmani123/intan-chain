// ============================================================================
// tb_spi_master.sv -- closed loop: our receiver against the real emulator RTL.
// ----------------------------------------------------------------------------
// spi_master drives two unmodified rhd2164_emulator cores (vendored under
// fpga/upstream/) and the resulting 128-channel stream is checked against the
// same .mem files the emulator plays back. Nothing here is a model of the chip
// written by the same hand as the receiver, which is the point: the stimulus is
// a separately-verified implementation of the datasheet.
//
// WHAT IS ACTUALLY BEING PROVEN
//   - the command sequencer's CS/SCLK/MOSI timing is accepted by the chip
//   - the two-command result pipeline is attributed correctly
//   - the 4x-oversample + phase-select recovery reads the right bits
//   - module A and module B of the DDR stream land in the right channels
//   - chip 0 and chip 1 are not swapped
//   - at least one phase setting works, which is what the bench eye scan relies on
//
// HOW THE EXPECTED VALUE IS DERIVED
//   Each emulator memory holds 256 time samples x 32 channels, addressed
//   sample*32 + channel, and a playback pointer advances once per CONVERT(0).
//   Rather than re-deriving exactly which sample the pointer lands on -- a real
//   ambiguity in the emulator between its combinational address and its
//   registered read -- this testbench finds the offset from channel 0 and then
//   requires ALL 128 channels to agree with that single offset, and requires the
//   offset to advance by exactly one per sweep. An off-by-one in playback is
//   then harmless, while any error in de-interleaving, mapping or bit recovery
//   still fails, because those would break the agreement.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_spi_master;

    localparam int N_CH        = 128;
    localparam int MEM_SAMPLES = 256;
    localparam int DEPTH       = MEM_SAMPLES * 32;
    localparam int N_SWEEP     = 4;

    integer errors = 0;
    integer checks = 0;

    task automatic chk(input cond, input [8*56-1:0] name);
        begin
            checks = checks + 1;
            if (cond) $display("  pass  %0s", name);
            else begin errors = errors + 1; $display("  FAIL  %0s", name); end
        end
    endtask

    // ---- clocks ----
    // 96 MHz receiver fast clock (SCLK = 24 MHz at OS=4), and the emulator's own
    // 400 MHz oversampling clock. Two unrelated domains, exactly as on the bench.
    reg clk96 = 1'b0;   always #5.2083 clk96 = ~clk96;
    reg clk400 = 1'b0;  always #1.25   clk400 = ~clk400;

    reg rst_n = 1'b0;

    // ---- DUT ----
    reg        start = 1'b0;
    reg  [3:0] phase = 4'd0;
    wire       cs_n, sclk, mosi;
    wire       miso0, miso1;
    wire       valid, sample_done, busy;
    wire [6:0] ch;
    wire signed [15:0] value;

    spi_master #(.N_CH(N_CH)) dut (
        .clk(clk96), .rst_n(rst_n),
        .start(start), .phase_select(phase),
        .cs_n(cs_n), .sclk(sclk), .mosi(mosi),
        .miso0(miso0), .miso1(miso1),
        .valid(valid), .ch(ch), .value(value),
        .sample_done(sample_done), .busy(busy)
    );

    // ---- stimulus: two real emulator chips on a shared bus ----
    rhd2164_emulator #(
        .MEM_A_FILE("fpga/upstream/rhd2164-emulator-mem/chip0_A.mem"),
        .MEM_B_FILE("fpga/upstream/rhd2164-emulator-mem/chip0_B.mem")
    ) chip0 (.clk(clk400), .rst_n(rst_n), .cs(cs_n), .sclk(sclk), .mosi(mosi), .miso(miso0));

    rhd2164_emulator #(
        .MEM_A_FILE("fpga/upstream/rhd2164-emulator-mem/chip1_A.mem"),
        .MEM_B_FILE("fpga/upstream/rhd2164-emulator-mem/chip1_B.mem")
    ) chip1 (.clk(clk400), .rst_n(rst_n), .cs(cs_n), .sclk(sclk), .mosi(mosi), .miso(miso1));

    // ---- the same memories, for checking ----
    reg [15:0] m0a [0:DEPTH-1];
    reg [15:0] m0b [0:DEPTH-1];
    reg [15:0] m1a [0:DEPTH-1];
    reg [15:0] m1b [0:DEPTH-1];

    function automatic [15:0] mem_at(input integer c, input integer sptr);
        integer a;
        begin
            a = sptr * 32 + (c % 32);
            if      (c < 32)  mem_at = m0a[a];
            else if (c < 64)  mem_at = m0b[a];
            else if (c < 96)  mem_at = m1a[a];
            else              mem_at = m1b[a];
        end
    endfunction

    // ---- capture one sweep ----
    reg [15:0] got [0:N_CH-1];
    reg        seen [0:N_CH-1];
    integer    n_seen;

    always @(posedge clk96) begin
        if (valid) begin
            got[ch]  <= value;
            seen[ch] <= 1'b1;
        end
    end

    integer i, sw, off, cand, agree, best_off;
    integer first_off, consistent_sweeps, working_phases;
    reg     sweep_ok;

    task automatic run_sweep(output integer found_off, output integer n_agree);
        integer c, o, a, best;
        begin
            for (c = 0; c < N_CH; c = c + 1) seen[c] = 1'b0;
            n_seen = 0;

            // start is ignored while the sequencer is mid-sweep, and
            // sample_done fires at command 33 of 35, so the previous sweep is
            // still running when it arrives. Wait for busy to fall or the next
            // start is silently dropped and the capture buffer is re-read
            // unchanged -- which reads as "the playback pointer stalled".
            begin : wait_idle
                integer g0;
                g0 = 0;
                while (g0 < 5000) begin
                    @(posedge clk96); #0.1;
                    if (!busy) disable wait_idle;
                    g0 = g0 + 1;
                end
                errors = errors + 1;
                $display("  FAIL  sequencer never went idle before a new sweep");
            end

            @(negedge clk96); start = 1'b1;
            @(negedge clk96); start = 1'b0;

            begin : wait_done
                integer guard;
                guard = 0;
                while (guard < 5000) begin
                    @(posedge clk96); #0.1;
                    if (sample_done) disable wait_done;
                    guard = guard + 1;
                end
                errors = errors + 1;
                $display("  FAIL  no sample_done within 5000 cycles");
            end
            // Let the last few emits settle.
            repeat (8) @(posedge clk96);

            // Find the playback offset that channel 0 agrees with, then measure
            // how many of the 128 channels agree with that same offset.
            found_off = -1;
            best      = 0;
            for (o = 0; o < MEM_SAMPLES; o = o + 1) begin
                if (got[0] === m0a[o*32]) begin
                    a = 0;
                    for (c = 0; c < N_CH; c = c + 1)
                        if (got[c] === mem_at(c, o)) a = a + 1;
                    if (a > best) begin best = a; found_off = o; end
                end
            end
            n_agree = best;
        end
    endtask

    integer ph, agr, o1;

    initial begin
        $dumpfile("fpga/sim/tb_spi_master.vcd");
        $dumpvars(0, tb_spi_master);

        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip0_A.mem", m0a);
        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip0_B.mem", m0b);
        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip1_A.mem", m1a);
        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip1_B.mem", m1b);
        chk(m0a[0] !== 16'hxxxx, "emulator memories loaded");
        chk(m0a[0] !== m1a[0],   "chip0 and chip1 memories differ");

        $display("\n=== phase scan: which MISO sampling phases recover the data ===");
        rst_n = 1'b0;
        repeat (20) @(negedge clk96);
        rst_n = 1'b1;
        repeat (20) @(negedge clk96);

        working_phases = 0;
        best_off = -1;
        for (ph = 0; ph <= 11; ph = ph + 1) begin
            phase = ph[3:0];
            run_sweep(o1, agr);
            $display("    phase %2d: %3d/%0d channels agree (offset %0d)", ph, agr, N_CH, o1);
            if (agr == N_CH) begin
                working_phases = working_phases + 1;
                if (best_off < 0) best_off = ph;
            end
        end
        chk(working_phases > 0, "at least one phase recovers every channel");
        if (best_off >= 0)
            $display("    -> using phase %0d for the remaining checks", best_off);

        $display("\n=== sustained capture at the working phase ===");
        phase = best_off[3:0];
        consistent_sweeps = 0;
        first_off = -1;
        for (sw = 0; sw < N_SWEEP; sw = sw + 1) begin
            run_sweep(o1, agr);
            sweep_ok = (agr == N_CH);
            if (!sweep_ok)
                $display("    sweep %0d: only %0d/%0d channels agree", sw, agr, N_CH);
            // The playback pointer must advance exactly one sample per sweep.
            if (first_off < 0) first_off = o1;
            else if (o1 !== ((first_off + sw) % MEM_SAMPLES)) begin
                sweep_ok = 1'b0;
                $display("    sweep %0d: playback offset %0d, expected %0d",
                         sw, o1, (first_off + sw) % MEM_SAMPLES);
            end
            if (sweep_ok) consistent_sweeps = consistent_sweeps + 1;
        end
        chk(consistent_sweeps == N_SWEEP, "every sweep reads all 128 channels correctly");

        // A wrong module-A/module-B split would still give 128 "agreeing"
        // channels if both halves happened to hold the same data, so prove the
        // halves are actually different data.
        chk(m0a[first_off*32] !== m0b[first_off*32], "module A and B data differ (split is meaningful)");

        $display("\n=== %0d error(s) over %0d checks ===", errors, checks);
        if (errors == 0) $display("ALL CHECKS PASSED");
        else             $display("THERE WERE FAILURES");
        $finish;
    end

    initial begin
        #20000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule

`default_nettype wire
