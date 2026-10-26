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
//   - the command sequencer's CS/SCLK/MOSI timing is accepted by the chip, and
//     is what CONTRACTS.md section 6 says, tick for tick, at SPI_DIV 0, 1 and 3
//   - the sweep sends CONVERT(0..31) then READ(59), READ(40), READ(63)
//   - the two-command result pipeline is attributed correctly
//   - the 4x-oversample + phase-select recovery reads the right bits
//   - module A and module B of the DDR stream land in the right channels
//   - chip 0 and chip 1 are not swapped
//   - the link check sees 0035 / 003a / 0035 / 003a once per sweep at a good
//     phase, and something else at a bad one
//   - the diagnostic list reads the ROM ("INTAN", 53/58, 1, 64, 4), echoes a
//     WRITE as {FF, d} and reads the written register back, on both chips
//   - the raw taps it stores, decoded with the host's formula from section 6,
//     give exactly the words the RTL decoded, at every phase
//   - a list never emits a channel word, and a `go` that lands mid-sweep (on a
//     clock the SPI_DIV enable is off, at SPI_DIV 3) is held, not lost
//   - the phase windows from the list and from real frames agree and are
//     contiguous; at SPI_DIV 1 and 3 they are at least two phases wide (at
//     SPI_DIV 0 the 400 MHz emulator leaves one clean phase -- see the scan)
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
    localparam int TAPS        = 78;
    localparam int CMD_TICKS   = 84;

    integer errors = 0;
    integer checks = 0;

    task automatic chk(input cond, input [8*64-1:0] name);
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
    reg        start   = 1'b0;
    reg  [3:0] phase   = 4'd0;
    reg  [3:0] spi_div = 4'd0;
    reg        diag_go = 1'b0;
    reg  [6:0] diag_len = 7'd1;
    wire [5:0] diag_cmd_addr, diag_wr_addr;
    wire [15:0] diag_cmd;
    wire       diag_wr_en, diag_busy, diag_done;
    wire [63:0] diag_wr_res, link_words;
    wire [TAPS-1:0] diag_wr_raw0, diag_wr_raw1;
    wire       link_stb, cycle_stb;
    wire       cs_n, sclk, mosi;
    wire       miso0, miso1;
    wire       valid, sample_done, busy;
    wire [6:0] ch;
    wire signed [15:0] value;

    spi_master #(.N_CH(N_CH)) dut (
        .clk(clk96), .rst_n(rst_n),
        .start(start), .phase_select(phase), .spi_div(spi_div),
        .diag_go(diag_go), .diag_len(diag_len),
        .diag_cmd_addr(diag_cmd_addr), .diag_cmd(diag_cmd),
        .diag_wr_en(diag_wr_en), .diag_wr_addr(diag_wr_addr),
        .diag_wr_res(diag_wr_res), .diag_wr_raw0(diag_wr_raw0), .diag_wr_raw1(diag_wr_raw1),
        .diag_busy(diag_busy), .diag_done(diag_done),
        .link_stb(link_stb), .link_words(link_words),
        .cycle_stb(cycle_stb),
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

    // ---- the command list: a plain array standing in for diag_mem ----
    reg [15:0] list [0:63];
    assign diag_cmd = list[diag_cmd_addr];

    function automatic [15:0] READ(input integer r);
        READ = {2'b11, r[5:0], 8'h00};
    endfunction
    function automatic [15:0] WRITE(input integer r, input integer d);
        WRITE = {2'b10, r[5:0], d[7:0]};
    endfunction

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

    // ---- monitors ----
    reg [15:0] got  [0:N_CH-1];
    reg [63:0] r_res  [0:63];
    reg [TAPS-1:0] r_raw0 [0:63];
    reg [TAPS-1:0] r_raw1 [0:63];
    integer n_valid = 0, n_done = 0, n_cycle = 0, n_link = 0, n_wr = 0, n_ddone = 0;
    integer valid_in_list = 0;
    reg [63:0] last_link = 64'd0;

    always @(posedge clk96) begin
        if (valid) begin
            got[ch] = value;
            n_valid = n_valid + 1;
            if (dut.list_mode) valid_in_list = valid_in_list + 1;
        end
        if (sample_done) begin
            n_done = n_done + 1;
            if (dut.list_mode) valid_in_list = valid_in_list + 1;
        end
        if (cycle_stb) n_cycle = n_cycle + 1;
        if (link_stb)  begin n_link = n_link + 1; last_link = link_words; end
        if (diag_done) n_ddone = n_ddone + 1;
        if (diag_wr_en) begin
            n_wr = n_wr + 1;
            r_res[diag_wr_addr]  = diag_wr_res;
            r_raw0[diag_wr_addr] = diag_wr_raw0;
            r_raw1[diag_wr_addr] = diag_wr_raw1;
        end
    end

    // Bus timing, in fast clocks, straight off the pins.
    integer cyc = 0;
    always @(posedge clk96) cyc = cyc + 1;

    integer c_csf = -1, c_csr = -1, c_sr = -1, c_sf = -1, rises = 0;
    integer cs_lo_min, cs_lo_max, cs_hi_min, sk_hi_min, sk_hi_max, sk_lo_min, sk_lo_max;
    integer rise_bad, first_rise_min, first_rise_max;
    reg [15:0] mosi_sh;
    reg [15:0] mosi_log [0:63];
    integer mosi_n;

    task automatic bus_clear;
        begin
            cs_lo_min = 1 << 30; cs_lo_max = 0; cs_hi_min = 1 << 30;
            sk_hi_min = 1 << 30; sk_hi_max = 0; sk_lo_min = 1 << 30; sk_lo_max = 0;
            rise_bad = 0; first_rise_min = 1 << 30; first_rise_max = 0;
            mosi_n = 0;
            c_csr = -1;
        end
    endtask

    always @(negedge cs_n) begin
        if (c_csr >= 0 && cyc - c_csr < cs_hi_min) cs_hi_min = cyc - c_csr;
        c_csf = cyc;
        c_sf  = -1;
        rises = 0;
    end
    always @(posedge cs_n) begin
        if (c_csf >= 0) begin
            if (cyc - c_csf < cs_lo_min) cs_lo_min = cyc - c_csf;
            if (cyc - c_csf > cs_lo_max) cs_lo_max = cyc - c_csf;
        end
        if (rises != 16) rise_bad = rise_bad + 1;
        if (mosi_n < 64) mosi_log[mosi_n] = mosi_sh;
        mosi_n = mosi_n + 1;
        c_csr = cyc;
    end
    always @(posedge sclk) if (!cs_n) begin
        if (rises == 0) begin
            if (cyc - c_csf < first_rise_min) first_rise_min = cyc - c_csf;
            if (cyc - c_csf > first_rise_max) first_rise_max = cyc - c_csf;
        end
        if (c_sf >= 0) begin
            if (cyc - c_sf < sk_lo_min) sk_lo_min = cyc - c_sf;
            if (cyc - c_sf > sk_lo_max) sk_lo_max = cyc - c_sf;
        end
        mosi_sh = {mosi_sh[14:0], mosi};     // the chip's view of the command
        rises = rises + 1;
        c_sr  = cyc;
    end
    always @(negedge sclk) if (!cs_n) begin
        if (cyc - c_sr < sk_hi_min) sk_hi_min = cyc - c_sr;
        if (cyc - c_sr > sk_hi_max) sk_hi_max = cyc - c_sr;
        c_sf = cyc;
    end

    // ---- sequencing helpers ----
    task automatic wait_idle;
        integer g;
        begin : w
            for (g = 0; g < 200000; g = g + 1) begin
                @(posedge clk96); #0.1;
                if (!busy) disable w;
            end
            errors = errors + 1;
            $display("  FAIL  sequencer never went idle");
        end
    endtask

    // start is ignored while the sequencer is mid-sweep, and sample_done fires
    // at command 33 of 35, so the previous sweep is still running when it
    // arrives. Wait for busy to fall or the next start is silently dropped and
    // the capture buffer re-read unchanged -- which reads as "the playback
    // pointer stalled".
    task automatic run_sweep(output integer found_off, output integer n_agree);
        integer c, o, a, best, g;
        begin
            wait_idle();
            n_valid = 0; n_done = 0; n_cycle = 0; n_link = 0;
            for (c = 0; c < N_CH; c = c + 1) got[c] = 16'hxxxx;
            @(negedge clk96); start = 1'b1;
            @(negedge clk96); start = 1'b0;
            wait_idle();
            repeat (4) @(posedge clk96);

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

    task automatic run_list(input integer n);
        begin
            wait_idle();
            n_valid = 0; n_done = 0; n_cycle = 0; n_wr = 0; n_ddone = 0;
            valid_in_list = 0;
            diag_len = n[6:0];
            @(negedge clk96); diag_go = 1'b1;
            @(negedge clk96); diag_go = 1'b0;
            wait_idle();
            repeat (4) @(posedge clk96);
        end
    endtask

    // The host's decode, CONTRACTS.md section 6, written independently of
    // miso_phase_select: module A bit (15-j) = tap[p + 4j], module B tap[p+2+4j].
    function automatic [15:0] host_dec(input [TAPS-1:0] raw, input integer p,
                                       input integer modb);
        integer j;
        begin
            for (j = 0; j < 16; j = j + 1)
                host_dec[15-j] = raw[p + 2*modb + 4*j];
        end
    endfunction

    // Every stored cycle, decoded from its raw taps, must equal what the RTL
    // decoded at the same phase. Returns the number of mismatching words.
    function automatic integer raw_mismatch(input integer n, input integer p);
        integer i, bad;
        begin
            bad = 0;
            for (i = 0; i < n; i = i + 1) begin
                if (host_dec(r_raw0[i], p, 0) !== r_res[i][15:0])  bad = bad + 1;
                if (host_dec(r_raw0[i], p, 1) !== r_res[i][31:16]) bad = bad + 1;
                if (host_dec(r_raw1[i], p, 0) !== r_res[i][47:32]) bad = bad + 1;
                if (host_dec(r_raw1[i], p, 1) !== r_res[i][63:48]) bad = bad + 1;
            end
            raw_mismatch = bad;
        end
    endfunction

    // The ROM list: READ(40..44), READ(59..63), then two READ(63) dummies so
    // every result comes back. Cycle i carries command i-2's result.
    localparam int ROM_LEN = 12;
    task automatic load_rom_list;
        integer k;
        begin
            for (k = 0; k < 5; k = k + 1) list[k]     = READ(40 + k);
            for (k = 0; k < 5; k = k + 1) list[5 + k] = READ(59 + k);
            list[10] = READ(63);
            list[11] = READ(63);
        end
    endtask

    // Expected ROM byte for list cycle i (2..11) on module A / B. Register 60,
    // the die revision, is not checked: it is not a fixed number on silicon.
    function automatic [15:0] rom_exp(input integer i, input integer modb);
        case (i)
            2: rom_exp = 16'h0049;  3: rom_exp = 16'h004E;  4: rom_exp = 16'h0054;
            5: rom_exp = 16'h0041;  6: rom_exp = 16'h004E;
            7: rom_exp = modb ? 16'h003A : 16'h0035;
            9: rom_exp = 16'h0001; 10: rom_exp = 16'h0040; 11: rom_exp = 16'h0004;
            default: rom_exp = 16'hxxxx;
        endcase
    endfunction

    // Number of wrong ROM words over both chips and both modules.
    function automatic integer rom_bad();
        integer i, bad;
        begin
            bad = 0;
            for (i = 2; i < ROM_LEN; i = i + 1) if (i != 8) begin
                if (r_res[i][15:0]  !== rom_exp(i, 0)) bad = bad + 1;
                if (r_res[i][31:16] !== rom_exp(i, 1)) bad = bad + 1;
                if (r_res[i][47:32] !== rom_exp(i, 0)) bad = bad + 1;
                if (r_res[i][63:48] !== rom_exp(i, 1)) bad = bad + 1;
            end
            rom_bad = bad;
        end
    endfunction

    // Scan PHASE_SELECT 0..15 with the ROM list; returns the passing set as a
    // bit mask and checks the raw/decoded agreement at every phase on the way.
    task automatic list_scan(output [15:0] mask);
        integer p, rawbad;
        begin
            load_rom_list();
            mask   = 16'd0;
            rawbad = 0;
            for (p = 0; p < 16; p = p + 1) begin
                phase = p[3:0];
                run_list(ROM_LEN);
                if (rom_bad() == 0) mask[p] = 1'b1;
                rawbad = rawbad + raw_mismatch(ROM_LEN, p);
            end
            chk(rawbad == 0, "host decode of raw taps == DIAG_RES, all 16 phases");
        end
    endtask

    // Window helpers: lowest set bit, width, and whether the set is one run.
    function automatic integer win_lo(input [15:0] m);
        integer p;
        begin
            win_lo = -1;
            for (p = 15; p >= 0; p = p - 1) if (m[p]) win_lo = p;
        end
    endfunction
    function automatic integer win_n(input [15:0] m);
        integer p;
        begin
            win_n = 0;
            for (p = 0; p < 16; p = p + 1) if (m[p]) win_n = win_n + 1;
        end
    endfunction
    function automatic integer win_contig(input [15:0] m);
        integer lo, n;
        begin
            lo = win_lo(m);
            n  = win_n(m);
            win_contig = (n > 0) && (m == (((16'd1 << n) - 16'd1) << lo));
        end
    endfunction

    // Bus timing for one sweep at the current SPI_DIV, against section 6.
    task automatic check_bus(input integer d);
        integer k, o1, agr, cmdbad;
        reg [8*64-1:0] nm;
        begin
            bus_clear();
            run_sweep(o1, agr);
            $display("    SPI_DIV %0d: CS low %0d..%0d, CS high >= %0d, SCLK high %0d..%0d low %0d..%0d, first edge %0d (clocks)",
                     d, cs_lo_min, cs_lo_max, cs_hi_min, sk_hi_min, sk_hi_max,
                     sk_lo_min, sk_lo_max, first_rise_min);
            chk(cs_lo_min == 65*(d+1) && cs_lo_max == 65*(d+1), "CS low for 65 ticks");
            chk(cs_hi_min == 19*(d+1), "CS high 19 ticks between cycles");
            chk(sk_hi_min == 2*(d+1) && sk_hi_max == 2*(d+1) &&
                sk_lo_min == 2*(d+1) && sk_lo_max == 2*(d+1),
                "SCLK 2 ticks high, 2 low: 24 MHz / (SPI_DIV+1)");
            chk(first_rise_min == 2*(d+1) && first_rise_max == 2*(d+1),
                "first SCLK edge 2 ticks after CS falls");
            chk(rise_bad == 0, "16 SCLK edges in every CS cycle");
            chk(n_cycle == 35 && mosi_n == 35, "35 CS cycles per sweep");
            cmdbad = 0;
            for (k = 0; k < 32; k = k + 1)
                if (mosi_log[k] !== {2'b00, k[5:0], 8'h00}) cmdbad = cmdbad + 1;
            chk(cmdbad == 0, "cycles 0..31 send CONVERT(0..31)");
            chk(mosi_log[32] === READ(59) && mosi_log[33] === READ(40) &&
                mosi_log[34] === READ(63), "AUX slots send READ(59), READ(40), READ(63)");
            chk(n_valid == N_CH && n_done == 1,
                "128 one-clock valid pulses and one sample_done per sweep");
            chk(n_link == 1, "one link-check strobe per sweep");
        end
    endtask

    integer i, sw, first_off, consistent_sweeps, d, mid;
    integer ph, agr, o1, n_good, wbad;
    reg [15:0] fmask, lmask;
    reg [15:0] good0;
    reg        sweep_ok;
    integer    divs [0:2];

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("fpga/sim/tb_spi_master.vcd");
            $dumpvars(0, tb_spi_master);
        end

        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip0_A.mem", m0a);
        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip0_B.mem", m0b);
        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip1_A.mem", m1a);
        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip1_B.mem", m1b);
        chk(m0a[0] !== 16'hxxxx, "emulator memories loaded");
        chk(m0a[0] !== m1a[0],   "chip0 and chip1 memories differ");
        for (i = 0; i < 64; i = i + 1) list[i] = READ(63);

        rst_n = 1'b0;
        repeat (20) @(negedge clk96);
        rst_n = 1'b1;
        repeat (20) @(negedge clk96);

        // =================================================================
        $display("\n=== bus timing and sweep commands, SPI_DIV = 0 ===");
        check_bus(0);

        // =================================================================
        $display("\n=== phase scan with real frames: which phases recover the data ===");
        fmask = 16'd0;
        for (ph = 0; ph < 16; ph = ph + 1) begin
            phase = ph[3:0];
            run_sweep(o1, agr);
            $display("    phase %2d: %3d/%0d channels agree (offset %0d), link %016h",
                     ph, agr, N_CH, o1, last_link);
            if (agr == N_CH) fmask[ph] = 1'b1;
            // The link check must pass exactly where the frames are clean.
            if ((agr == N_CH) != (last_link === 64'h003A_0035_003A_0035)) begin
                errors = errors + 1;
                $display("  FAIL  phase %0d: link words disagree with the frame check", ph);
            end
        end
        chk(win_n(fmask) > 0, "at least one phase recovers every channel");
        chk(win_contig(fmask), "frame phase window contiguous");
        mid = win_lo(fmask) + (win_n(fmask) - 1) / 2;
        $display("    -> frame window %0d..%0d, using phase %0d",
                 win_lo(fmask), win_lo(fmask) + win_n(fmask) - 1, mid);

        $display("\n=== sustained capture at the working phase ===");
        phase = mid[3:0];
        consistent_sweeps = 0;
        first_off = -1;
        for (sw = 0; sw < N_SWEEP; sw = sw + 1) begin
            run_sweep(o1, agr);
            sweep_ok = (agr == N_CH) && (n_link == 1) &&
                       (last_link === 64'h003A_0035_003A_0035);
            if (!sweep_ok)
                $display("    sweep %0d: only %0d/%0d channels agree, link %016h",
                         sw, agr, N_CH, last_link);
            // The playback pointer must advance exactly one sample per sweep.
            if (first_off < 0) first_off = o1;
            else if (o1 !== ((first_off + sw) % MEM_SAMPLES)) begin
                sweep_ok = 1'b0;
                $display("    sweep %0d: playback offset %0d, expected %0d",
                         sw, o1, (first_off + sw) % MEM_SAMPLES);
            end
            if (sweep_ok) consistent_sweeps = consistent_sweeps + 1;
        end
        chk(consistent_sweeps == N_SWEEP, "every sweep: 128 channels right, link words 0035/003a");

        // A wrong module-A/module-B split would still give 128 "agreeing"
        // channels if both halves happened to hold the same data, so prove the
        // halves are actually different data.
        chk(m0a[first_off*32] !== m0b[first_off*32], "module A and B data differ (split is meaningful)");

        phase = (mid + 8) % 16;
        run_sweep(o1, agr);
        chk(n_link == 1 && last_link !== 64'h003A_0035_003A_0035,
            "link words wrong at a phase outside the window");

        // =================================================================
        $display("\n=== diagnostic list: ROM, both chips, at phase %0d ===", mid);
        phase = mid[3:0];
        load_rom_list();
        run_list(ROM_LEN);
        chk(n_ddone == 1, "diag_done pulses once at the end of the list");
        chk(n_cycle == ROM_LEN && n_wr == ROM_LEN, "one CS cycle and one result write per command");
        chk(valid_in_list == 0 && n_valid == 0 && n_done == 0,
            "nothing reaches the frame builder during a list");
        $display("    chip0 A: %04h %04h %04h %04h %04h | 59=%04h 60=%04h 61=%04h 62=%04h 63=%04h",
                 r_res[2][15:0], r_res[3][15:0], r_res[4][15:0], r_res[5][15:0], r_res[6][15:0],
                 r_res[7][15:0], r_res[8][15:0], r_res[9][15:0], r_res[10][15:0], r_res[11][15:0]);
        chk(r_res[2][15:0] == 16'h0049 && r_res[3][15:0] == 16'h004E &&
            r_res[4][15:0] == 16'h0054 && r_res[5][15:0] == 16'h0041 &&
            r_res[6][15:0] == 16'h004E, "chip 0 module A reads \"INTAN\" from 40..44");
        chk(r_res[2][47:32] == 16'h0049 && r_res[3][47:32] == 16'h004E &&
            r_res[4][47:32] == 16'h0054 && r_res[5][47:32] == 16'h0041 &&
            r_res[6][47:32] == 16'h004E, "chip 1 module A reads \"INTAN\" from 40..44");
        chk(r_res[7][15:0] == 16'd53 && r_res[7][47:32] == 16'd53, "module A reg 59 = 53 on both chips");
        chk(r_res[7][31:16] == 16'd58 && r_res[7][63:48] == 16'd58, "module B reg 59 = 58 on both chips");
        chk(r_res[9][15:0] == 16'd1 && r_res[10][15:0] == 16'd64 && r_res[11][15:0] == 16'd4 &&
            r_res[9][47:32] == 16'd1 && r_res[10][47:32] == 16'd64 && r_res[11][47:32] == 16'd4,
            "module A regs 61, 62, 63 = 1, 64, 4 on both chips");
        chk(rom_bad() == 0, "every ROM word right on both modules of both chips");
        chk(raw_mismatch(ROM_LEN, mid) == 0, "host decode of the raw taps == DIAG_RES");

        $display("\n=== diagnostic list: WRITE echo and read-back ===");
        list[0] = WRITE(12, 8'hA5);
        list[1] = WRITE(13, 8'h3C);
        list[2] = READ(12);
        list[3] = READ(13);
        list[4] = READ(63);
        list[5] = READ(63);
        run_list(6);
        wbad = 0;
        for (i = 0; i < 4; i = i + 1) begin
            if (r_res[2][16*i +: 16] !== 16'hFFA5) wbad = wbad + 1;
            if (r_res[3][16*i +: 16] !== 16'hFF3C) wbad = wbad + 1;
        end
        chk(wbad == 0, "WRITE(r, d) echoes {FF, d} on every lane");
        wbad = 0;
        for (i = 0; i < 4; i = i + 1) begin
            if (r_res[4][16*i +: 16] !== 16'h00A5) wbad = wbad + 1;
            if (r_res[5][16*i +: 16] !== 16'h003C) wbad = wbad + 1;
        end
        chk(wbad == 0, "READ of the written registers returns {00, d} on every lane");
        chk(r_res[0][15:0] === 16'h0004 && r_res[1][15:0] === 16'h0004,
            "cycles 0, 1 hold the previous list's results: no re-alignment");

        // =================================================================
        // The same scan at three tick lengths. Each phase step is one SPI
        // tick, so with the emulator's fixed delay the window moves earlier
        // as the tick grows.
        divs[0] = 0; divs[1] = 1; divs[2] = 3;
        for (i = 0; i < 3; i = i + 1) begin
            d = divs[i];
            spi_div = d[3:0];
            $display("\n=== SPI_DIV = %0d ===", d);
            if (d != 0) check_bus(d);
            list_scan(lmask);
            $display("    list phase window: %016b (bit = phase), %0d..%0d",
                     lmask, win_lo(lmask), win_lo(lmask) + win_n(lmask) - 1);
            // At SPI_DIV 0 a half-bit lasts two ticks (20.8 ns) and the 400 MHz
            // emulator spends 10-13 ns of it in its input synchroniser, with
            // 2.5 ns of jitter because its clock is unrelated to ours. That
            // leaves one tick boundary that always lands inside the eye and a
            // neighbour that lands on the edge about half the time (the
            // original 80-tick master measured the same). From SPI_DIV 1 a
            // tick is at least 20.8 ns, the synchroniser delay fits inside one
            // tick, and both tick boundaries of each half-bit land in the eye.
            if (d == 0)
                chk(win_contig(lmask) && win_n(lmask) >= 1, "ROM phase window contiguous, >= 1 wide");
            else
                chk(win_contig(lmask) && win_n(lmask) >= 2, "ROM phase window contiguous, >= 2 wide");
            if (d == 0) begin
                good0 = lmask;
                chk(lmask == fmask, "list window == frame window at SPI_DIV 0");
            end else begin
                // Frames at the middle of this divider's window.
                phase = win_lo(lmask) + (win_n(lmask) - 1) / 2;
                n_good = 0;
                for (sw = 0; sw < 2; sw = sw + 1) begin
                    run_sweep(o1, agr);
                    if (agr == N_CH && last_link === 64'h003A_0035_003A_0035) n_good = n_good + 1;
                end
                chk(n_good == 2, "frames bit-exact and link check clean in this window");
            end
        end

        // =================================================================
        $display("\n=== a go that lands mid-sweep is held, not lost (SPI_DIV = 3) ===");
        load_rom_list();
        phase = win_lo(lmask) + (win_n(lmask) - 1) / 2;
        for (i = 0; i < 64; i = i + 1) r_res[i] = 64'd0;
        wait_idle();
        n_valid = 0; n_done = 0; n_wr = 0; n_ddone = 0; valid_in_list = 0;
        diag_len = ROM_LEN;
        @(negedge clk96); start = 1'b1;
        @(negedge clk96); start = 1'b0;
        repeat (1000) @(negedge clk96);            // inside cycle 2
        while (dut.div_cnt == 0) @(negedge clk96);
        chk(dut.div_cnt != 0 && dut.running, "go arrives mid-sweep, tick enable off");
        diag_go = 1'b1;
        @(negedge clk96); diag_go = 1'b0;
        #0.1;
        chk(diag_busy === 1'b1 && busy === 1'b1, "diag_busy rises at once");
        wait_idle();
        repeat (4) @(posedge clk96);
        chk(n_valid == N_CH && n_done == 1, "the sweep in flight finished untouched");
        chk(n_wr == ROM_LEN && n_ddone == 1 && valid_in_list == 0, "then the held list played");
        chk(rom_bad() == 0, "and read the ROM correctly");

        spi_div = 4'd0;

        $display("\n=== %0d error(s) over %0d checks ===", errors, checks);
        if (errors == 0) $display("ALL CHECKS PASSED");
        else             $display("THERE WERE FAILURES");
        $finish;
    end

    initial begin
        #40_000_000;
        $display("TIMEOUT");
        $display("THERE WERE FAILURES");
        $finish;
    end

endmodule

`default_nettype wire
