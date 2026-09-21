// ============================================================================
// tb_pattern_gen.sv -- pattern_gen against the host's golden vectors.
//
// The vectors are produced by tools/gen_pattern_vectors.py from
// host/chain/pattern.py, so this testbench compares the RTL against the exact
// code the PC will use to score accuracy at the bench. If they ever disagree,
// this fails in simulation rather than looking like a corrupted link later.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_pattern_gen;

    localparam int N_CH      = 128;
    localparam int N_SAMPLES = 8;

    integer errors = 0;
    integer checks = 0;

    reg clk = 1'b0;
    always #5 clk = ~clk;          // 100 MHz, rate is irrelevant here

    reg  rst_n     = 1'b0;
    reg  start     = 1'b0;
    reg  realistic = 1'b0;

    wire                     valid, done, busy;
    wire [$clog2(N_CH)-1:0]  ch;
    wire signed [15:0]       value;

    pattern_gen #(.N_CH(N_CH)) dut (
        .clk(clk), .rst_n(rst_n),
        .start(start), .realistic(realistic),
        .valid(valid), .ch(ch), .value(value), .done(done), .busy(busy)
    );

    reg [15:0] golden [0:N_SAMPLES*N_CH-1];

    integer s, seen, expect_idx;
    reg [15:0] got;

    task automatic chk(input cond, input [8*48-1:0] name);
        begin
            checks = checks + 1;
            if (cond) $display("  pass  %0s", name);
            else begin errors = errors + 1; $display("  FAIL  %0s", name); end
        end
    endtask

    // Drive on the negedge and sample a delta after the posedge. Driving and
    // sampling on the same edge is a race against the DUT's own non-blocking
    // updates, and it reads stale values.
    integer dones;

    task automatic run_mode(input use_realistic, input [8*16-1:0] label);
        integer mism;
        begin
            mism  = 0;
            dones = 0;
            realistic = use_realistic;
            @(negedge clk); rst_n = 1'b0;
            repeat (3) @(negedge clk);
            rst_n = 1'b1;
            @(negedge clk);

            for (s = 0; s < N_SAMPLES; s = s + 1) begin
                @(negedge clk); start = 1'b1;
                @(negedge clk); start = 1'b0;

                seen = 0;
                while (seen < N_CH) begin
                    @(posedge clk); #1;
                    if (valid) begin
                        expect_idx = s * N_CH + ch;
                        got = value;
                        if (got !== golden[expect_idx]) begin
                            mism = mism + 1;
                            if (mism <= 5)
                                $display("    %0s sample %0d ch %0d: rtl %04h, host %04h",
                                         label, s, ch, got, golden[expect_idx]);
                        end
                        seen = seen + 1;
                    end
                    if (done) dones = dones + 1;
                end
                // done may land on the same cycle as the last value or the next
                if (dones < (s + 1)) begin
                    @(posedge clk); #1;
                    if (done) dones = dones + 1;
                end
            end
            chk(mism == 0,          {label, " matches host golden vectors"});
            chk(dones == N_SAMPLES, {label, " one done strobe per sample"});
        end
    endtask

    integer j;

    initial begin
        $dumpfile("fpga/sim/tb_pattern_gen.vcd");
        $dumpvars(0, tb_pattern_gen);

        $display("\n=== pattern_gen vs host golden vectors ===");

        $readmemh("fpga/sim/vectors/pattern_check.mem", golden);
        // A missing vector file leaves the array as X and every compare fails
        // for the wrong reason, so say so plainly instead.
        chk(golden[0] === 16'h0001, "check vectors loaded (ch0 s0 == 1)");
        run_mode(1'b0, "CHECK          ");

        // Spot values from CONTRACTS.md, so a passing compare cannot just mean
        // "RTL and host are wrong in the same way as the vector file".
        chk(golden[1]         === 16'h0002, "spec: ch1 sample0 == 2");
        chk(golden[N_CH]      === 16'hB400, "spec: ch0 sample1 == 0xB400");
        chk(golden[2*N_CH]    === 16'h5A00, "spec: ch0 sample2 == 0x5A00");

        $readmemh("fpga/sim/vectors/pattern_realistic.mem", golden);
        chk(golden[0] !== 16'h0001, "realistic vectors loaded");
        run_mode(1'b1, "REALISTIC      ");

        // busy must fall between samples, or a caller cannot tell when it is
        // safe to start the next one.
        chk(busy === 1'b0, "busy low when idle");

        $display("\n=== %0d error(s) over %0d checks ===", errors, checks);
        if (errors == 0) $display("ALL CHECKS PASSED");
        else             $display("THERE WERE FAILURES");
        $finish;
    end

    initial begin
        #5000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule

`default_nettype wire
