// ============================================================================
// tb_spi_tap.sv -- the passive tap against a Rhythm-like master and two real
// emulator cores, listening mid-cable.
// ----------------------------------------------------------------------------
//   master (this TB) --D1--> [tap point] --D2--> chip0, chip1 (rhd2164_emulator)
//          <--D1-- MISO --------<--D2--
//
// The master runs Intan's controller timing (Rhythm / RHX, per the RHX source):
// a data clock of 2800 x the sample rate, 80 data clocks per CS cycle, CS low
// for 66 of them, one data clock from CS low to the first SCLK pulse and one
// from the last falling edge to CS high, SCLK = 4 data clocks, MOSI changing
// two data clocks before each rising edge. 35 commands a sweep: CONVERT(0..31)
// and three auxiliary commands that change from sweep to sweep (WRITEs,
// READ(40..44), READ(59..63), CONVERT(32..34), CONVERT(48/49), one CALIBRATE).
// It is asynchronous to the tap's 96 MHz clock, and the chips run their own
// 400 MHz clock, so MISO timing (tMISO ~ 10-13 ns) is the emulator's own.
//
// The master decodes MISO itself, sampling SAMP_NS after each of its own
// edges (round trip plus margin). That is the reference: the tap's frames
// must equal it channel for channel, sweep for sweep. The reference is in
// turn checked against the emulator's playback memory and against the known
// ROM answers, so a wrong reference cannot make a wrong tap pass.
//
//  (a) 2 kS/s, TAP_DELAY 8, listening switched on in the middle of sweep 0:
//      the first frame is sweep 1, every whole sweep is one frame, equal to
//      the reference. A CALIBRATE in sweep 1 swallows the next 9 commands; the
//      known-answer checks skip them and TAP_ERR stays 0. An SCLK glitch on
//      the tap's wire only (the chips never see it) in cycle 10 of sweep 2:
//      TAP_FRAMING 1, that frame is TAP_PARTIAL with slots 8 and 10 (the
//      glitched cycle's result and command) zero on all four lanes and every
//      other channel still right. TAP_CHECKS equals an independent count.
//  (b) the same, clean, after a clear: TAP_SCLK_MIN is the true half period.
//  (c) 5 kS/s: frames equal the reference, TAP_SCLK_MIN again, no errors.
//  (d) TAP_DELAY scans at 2 and 5 kS/s, one value per CS cycle of READ(59):
//      the clean values form one contiguous window about a half period wide
//      that contains the reset value 8; values outside it count errors.
//  (e) a frame builder that stays busy: two sweeps wait in the buffers, three
//      more are owed as zero frames with TAP_PARTIAL, the next ones are lost
//      and pulse `lost`, and once it frees up the frames leave in sweep order
//      and data resumes. Nothing is ever streamed while it is busy.
//
// Only spi_tap is under test here; tb_tap_arty_s7_25 runs it inside the whole
// board, through frame_builder and QSPI.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_spi_tap;

    localparam real CLK_HALF = 1.0e9 / (2.0 * 96.0e6);
    localparam real D1       = 5.0;         // master -> tap point, ns
    localparam real D2       = 15.0;        // tap point -> chips, ns
    localparam real SAMP_NS  = 90.0;        // master's MISO sample, after its own edge
    localparam real FCLK_NS  = 2.0 * CLK_HALF;

    localparam int MAXC  = 1024;            // CS cycles logged
    localparam int MAXF  = 32;              // frames captured
    localparam int MEM_SAMPLES = 256;
    localparam int MEM_DEPTH   = MEM_SAMPLES * 32;

    integer errors = 0;
    integer checks = 0;

    task automatic chk(input cond, input [8*80-1:0] name);
        begin
            checks = checks + 1;
            if (cond) $display("  pass  %0s", name);
            else begin errors = errors + 1; $display("  FAIL  %0s", name); end
        end
    endtask

    // ---------------- clocks, DUT controls ----------------
    reg clk = 1'b0;     always #(CLK_HALF) clk = ~clk;
    reg clk400 = 1'b0;  always #1.25 clk400 = ~clk400;

    reg        rst_n      = 1'b0;
    reg        chip_rst_n = 1'b0;
    reg        en         = 1'b0;
    reg        clear      = 1'b0;
    reg  [5:0] delay      = 6'd8;
    reg        force_busy = 1'b0;
    reg  [3:0] busy_cnt   = 4'd0;
    wire       fb_busy    = force_busy || (busy_cnt != 4'd0);

    // ---------------- the cable ----------------
    reg  m_cs = 1'b1, m_sclk = 1'b0, m_mosi = 1'b0;     // at the master
    reg  glitch = 1'b0;                                 // on the tap's SCLK wire only
    wire t_cs, t_sclk_w, t_mosi, t_m0, t_m1;            // at the tap point
    wire c_cs, c_sclk, c_mosi, c_m0, c_m1;              // at the chips
    wire m_m0, m_m1;                                    // MISO back at the master

    assign #(D1) t_cs     = m_cs;
    assign #(D1) t_sclk_w = m_sclk;
    assign #(D1) t_mosi   = m_mosi;
    assign #(D2) c_cs     = t_cs;
    assign #(D2) c_sclk   = t_sclk_w;
    assign #(D2) c_mosi   = t_mosi;
    assign #(D2) t_m0     = c_m0;
    assign #(D2) t_m1     = c_m1;
    assign #(D1) m_m0     = t_m0;
    assign #(D1) m_m1     = t_m1;

    wire t_sclk = t_sclk_w | glitch;

    rhd2164_emulator #(
        .MEM_A_FILE("fpga/upstream/rhd2164-emulator-mem/chip0_A.mem"),
        .MEM_B_FILE("fpga/upstream/rhd2164-emulator-mem/chip0_B.mem")
    ) chip0 (.clk(clk400), .rst_n(chip_rst_n),
             .cs(c_cs), .sclk(c_sclk), .mosi(c_mosi), .miso(c_m0));

    rhd2164_emulator #(
        .MEM_A_FILE("fpga/upstream/rhd2164-emulator-mem/chip1_A.mem"),
        .MEM_B_FILE("fpga/upstream/rhd2164-emulator-mem/chip1_B.mem")
    ) chip1 (.clk(clk400), .rst_n(chip_rst_n),
             .cs(c_cs), .sclk(c_sclk), .mosi(c_mosi), .miso(c_m1));

    // ---------------- the tap ----------------
    wire        valid, sample_done, partial, lost, good;
    wire [6:0]  ch;
    wire [15:0] value;
    wire [31:0] t_cap, stat, cycles, framing, sweeps, partials, tchecks;
    wire [31:0] err0, err1, sclk_min;
    reg  [31:0] tick = 32'd0;
    always @(posedge clk) tick <= tick + 32'd1;

    // LED stretch shortened from 44 ms to 170 us so both of its states fit in
    // the run.
    spi_tap #(.GOOD_W(14)) dut (
        .clk(clk), .rst_n(rst_n), .en(en), .clear(clear), .delay(delay),
        .cs_n_i(t_cs), .sclk_i(t_sclk), .mosi_i(t_mosi),
        .miso0_i(t_m0), .miso1_i(t_m1),
        .tick(tick), .fb_busy(fb_busy),
        .valid(valid), .ch(ch), .value(value), .sample_done(sample_done),
        .partial(partial), .t_cap(t_cap), .lost(lost),
        .stat(stat), .cycles(cycles), .framing(framing), .sweeps(sweeps),
        .partials(partials), .checks(tchecks), .err0(err0), .err1(err1),
        .sclk_min(sclk_min), .good(good));

    // frame_builder's header phase: busy for 10 clocks after each sample_done
    always @(posedge clk) begin
        if (sample_done)        busy_cnt <= 4'd10;
        else if (busy_cnt != 0) busy_cnt <= busy_cnt - 4'd1;
    end

    // ---------------- the master, and its own decode (the reference) ----------------
    real       tq = 178.571;                // data clock, ns
    integer    ncyc = 0;                    // CS cycles finished
    reg [15:0] log_cmd [0:MAXC-1];
    reg [15:0] log_w   [0:4*MAXC-1];        // lane l of cycle i at 4*i + l: A0 B0 A1 B1
    reg        log_gl  [0:MAXC-1];          // the tap's SCLK was glitched in this cycle
    reg [15:0] ra0, rb0, ra1, rb1;

    always @(posedge m_sclk) if (!m_cs) begin
        #(SAMP_NS);
        ra0 = {ra0[14:0], m_m0};
        ra1 = {ra1[14:0], m_m1};
    end
    always @(negedge m_sclk) if (!m_cs) begin
        #(SAMP_NS);
        rb0 = {rb0[14:0], m_m0};
        rb1 = {rb1[14:0], m_m1};
    end

    // One CS cycle, 80 data clocks. gl >= 0 puts a 40 ns pulse on the tap's
    // SCLK wire in the low phase before rising edge gl.
    task automatic send(input [15:0] cmd, input integer gl);
        integer j;
        begin
            m_cs = 1'b0;                                        // tick 0
            #(tq);
            for (j = 0; j < 16; j = j + 1) begin
                m_mosi = cmd[15-j];                             // tick 4j+1
                if (j == gl) begin
                    #(tq - 20.0); glitch = 1'b1;
                    #(40.0);      glitch = 1'b0;
                    #(tq - 20.0);
                end else begin
                    #(2.0 * tq);
                end
                m_sclk = 1'b1;                                  // tick 4j+3: rising
                #(2.0 * tq);
                m_sclk = 1'b0;                                  // tick 4j+5: falling
            end
            #(tq); m_cs = 1'b1; m_mosi = 1'b0;                  // tick 66
            #(14.0 * tq);                                       // tick 80
            log_cmd[ncyc]       = cmd;
            log_w[4*ncyc + 0]   = ra0;
            log_w[4*ncyc + 1]   = rb0;
            log_w[4*ncyc + 2]   = ra1;
            log_w[4*ncyc + 3]   = rb1;
            log_gl[ncyc]        = (gl >= 0);
            ncyc = ncyc + 1;
        end
    endtask

    function automatic [15:0] CONVERT(input integer c); CONVERT = {2'b00, c[5:0], 8'h00}; endfunction
    function automatic [15:0] READ(input integer r);    READ    = {2'b11, r[5:0], 8'h00}; endfunction
    function automatic [15:0] WRITE(input integer r, input integer d);
        WRITE = {2'b10, r[5:0], d[7:0]};
    endfunction
    localparam logic [15:0] CALIBRATE = 16'h5500;

    // Three auxiliary slots, each cycling through its own list, as Rhythm's
    // AuxCmd1..3 do. Registers 0..2 and 5..7 only, so nothing the emulator
    // models changes underneath (register 4 holds the two's-complement bit).
    function automatic [15:0] aux(input integer s, input integer k);
        integer m;
        begin
            m = s % 8;
            case (k)
                0: case (m)
                       0: aux = WRITE(0, 8'hDE);  1: aux = READ(40);  2: aux = READ(41);
                       3: aux = READ(42);         4: aux = READ(43);  5: aux = READ(44);
                       6: aux = WRITE(5, s);      default: aux = CONVERT(48);
                   endcase
                1: case (m)
                       0: aux = READ(59);   1: aux = READ(61);   2: aux = READ(62);
                       3: aux = READ(63);   4: aux = CONVERT(32); 5: aux = CONVERT(33);
                       6: aux = CONVERT(34); default: aux = CONVERT(49);
                   endcase
                default: case (m)
                       0: aux = WRITE(6, 8'h80 + s); 1: aux = READ(60);
                       2: aux = WRITE(7, s);         3: aux = READ(59);
                       4: aux = WRITE(1, 8'h55);     5: aux = READ(63);
                       6: aux = WRITE(2, s ^ 8'hFF); default: aux = READ(12);
                   endcase
            endcase
        end
    endfunction

    integer sweep_base [0:15];

    // One sweep: CONVERT(0..31) + 3 aux. `en_at` switches the tap's assembly on
    // before that cycle; `gl_cyc` glitches that cycle; `cal` replaces aux 0.
    task automatic run_sweep(input integer s, input integer en_at, input integer gl_cyc,
                             input integer cal);
        integer c;
        reg [15:0] cmd;
        begin
            sweep_base[s] = ncyc;
            for (c = 0; c < 35; c = c + 1) begin
                if (c == en_at) en = 1'b1;
                if (c < 32)                cmd = CONVERT(c);
                else if (c == 32 && cal)   cmd = CALIBRATE;
                else                       cmd = aux(s, c - 32);
                send(cmd, (c == gl_cyc) ? 3 : -1);
            end
        end
    endtask

    // ---------------- what the tap should have done, from the master's log ----------------
    function automatic logic okc(input integer i);      // the tap heard cycle i cleanly
        okc = (i >= 0) && !log_gl[i];
    endfunction

    function automatic logic is_cal(input [15:0] c);
        is_cal = (c == 16'h5500) || (c == 16'h6A00);
    endfunction

    function automatic logic is_ka(input [15:0] c);
        is_ka = (c[15:14] == 2'b10) ||
                (c[15:14] == 2'b11 && ((c[13:8] >= 6'd40 && c[13:8] <= 6'd44) ||
                                       c[13:8] == 6'd59 || c[13:8] == 6'd61 ||
                                       c[13:8] == 6'd62 || c[13:8] == 6'd63));
    endfunction

    // CONTRACTS.md section 7 table, written out again independently of the RTL.
    function automatic [15:0] ka_a(input [15:0] c);
        if (c[15:14] == 2'b10) ka_a = {8'hFF, c[7:0]};
        else case (c[13:8])
            6'd40: ka_a = 16'h0049;  6'd41: ka_a = 16'h004E;  6'd42: ka_a = 16'h0054;
            6'd43: ka_a = 16'h0041;  6'd44: ka_a = 16'h004E;  6'd59: ka_a = 16'h0035;
            6'd61: ka_a = 16'h0001;  6'd62: ka_a = 16'h0040;  default: ka_a = 16'h0004;
        endcase
    endfunction

    // Results the tap must have compared among cycles [from, to): both cycles
    // heard, a known-answer command, and no CALIBRATE / CLEAR / unheard cycle
    // in the 11 before (the RTL treats an unheard cycle as a possible CALIBRATE).
    function automatic integer want_checks(input integer from, input integer to);
        integer i, k, n;
        logic blank;
        begin
            n = 0;
            for (i = from; i < to; i = i + 1)
                if (i >= 2 && okc(i) && okc(i - 2) && is_ka(log_cmd[i - 2])) begin
                    blank = 1'b0;
                    for (k = 1; k <= 11; k = k + 1)
                        if (!okc(i - k) || is_cal(log_cmd[i - k])) blank = 1'b1;
                    if (!blank) n = n + 1;
                end
            want_checks = n;
        end
    endfunction

    // The reference must itself be right: every known answer outside a
    // calibration window, as the master decoded it.
    function automatic integer ref_ka_bad(input integer from, input integer to);
        integer i, k, n;
        logic blank;
        begin
            n = 0;
            for (i = from; i < to; i = i + 1)
                if (i >= 2 && is_ka(log_cmd[i - 2])) begin
                    blank = 1'b0;
                    for (k = 1; k <= 11; k = k + 1)
                        if (i - k >= 0 && is_cal(log_cmd[i - k])) blank = 1'b1;
                    if (!blank) begin
                        if (log_w[4*i + 0] !== ka_a(log_cmd[i - 2])) n = n + 1;
                        if (log_w[4*i + 2] !== ka_a(log_cmd[i - 2])) n = n + 1;
                        if (log_cmd[i - 2] == READ(59) &&
                            (log_w[4*i + 1] !== 16'h003A || log_w[4*i + 3] !== 16'h003A))
                            n = n + 1;
                    end
                end
            ref_ka_bad = n;
        end
    endfunction

    // ---------------- the tap's frames, as frame_builder would take them ----------------
    integer    nfr = 0, prot_bad = 0;
    reg [15:0] fr_val  [0:MAXF*128-1];
    reg        fr_part [0:MAXF-1];
    reg [15:0] cur     [0:127];
    reg [127:0] cur_got = 128'd0;
    integer    k_ch;

    always @(posedge clk) begin
        if ((valid || sample_done) && fb_busy) begin
            prot_bad = prot_bad + 1;
            $display("    protocol: stream while frame_builder busy at %0t", $time);
        end
        if (valid) begin
            if (cur_got[ch]) prot_bad = prot_bad + 1;
            cur_got[ch] = 1'b1;
            cur[ch]     = value;
        end
        if (sample_done) begin
            if (!valid || ch != 7'd127 || cur_got !== {128{1'b1}}) begin
                prot_bad = prot_bad + 1;
                $display("    protocol: frame %0d ended without all 128 channels once", nfr);
            end
            if (nfr < MAXF) begin
                for (k_ch = 0; k_ch < 128; k_ch = k_ch + 1) fr_val[nfr*128 + k_ch] = cur[k_ch];
                fr_part[nfr] = partial;
            end
            nfr = nfr + 1;
            cur_got = 128'd0;
        end
    end

    // LD5 must go dark after the glitch.
    reg good_after_glitch = 1'b1;
    always @(negedge glitch) begin
        #(20_000.0);
        good_after_glitch = good;
    end

    integer lost_n = 0;
    integer lost_at [0:15];
    always @(posedge clk) if (lost) begin
        if (lost_n < 16) lost_at[lost_n] = ncyc;
        lost_n = lost_n + 1;
    end

    // Frame f against sweep s: slot c is expected iff the tap heard both the
    // cycle that sent CONVERT(c) and the one that returned it.
    task automatic cmp_sweep(input integer f, input integer s, output integer bad,
                             output logic want_part);
        integer c2, lane, ic, ir;
        logic   ok;
        reg [15:0] want;
        begin
            bad = 0;
            want_part = 1'b0;
            for (k_ch = 0; k_ch < 128; k_ch = k_ch + 1) begin
                c2   = k_ch % 32;
                lane = k_ch / 32;
                ic   = sweep_base[s] + c2;
                ir   = ic + 2;
                ok   = okc(ic) && okc(ir);
                want = ok ? log_w[4*ir + lane] : 16'd0;
                if (!ok) want_part = 1'b1;
                if (fr_val[f*128 + k_ch] !== want) begin
                    bad = bad + 1;
                    if (bad <= 4)
                        $display("    frame %0d (sweep %0d) ch %0d: tap %04h want %04h",
                                 f, s, k_ch, fr_val[f*128 + k_ch], want);
                end
            end
            if (fr_part[f] !== want_part) bad = bad + 1;
        end
    endtask

    // The reference against the emulator's playback memory (tb_spi_master's
    // method): one offset must explain all 128 channels of the sweep.
    reg [15:0] m0a [0:MEM_DEPTH-1];
    reg [15:0] m0b [0:MEM_DEPTH-1];
    reg [15:0] m1a [0:MEM_DEPTH-1];
    reg [15:0] m1b [0:MEM_DEPTH-1];

    function automatic integer ref_agree(input integer s);
        integer p, c3, a, best, ir;
        reg [15:0] want;
        begin
            best = 0;
            for (p = 0; p < MEM_SAMPLES; p = p + 1) begin
                a = 0;
                for (c3 = 0; c3 < 32; c3 = c3 + 1) begin
                    ir = sweep_base[s] + c3 + 2;
                    if (log_w[4*ir + 0] === m0a[p*32 + c3]) a = a + 1;
                    if (log_w[4*ir + 1] === m0b[p*32 + c3]) a = a + 1;
                    if (log_w[4*ir + 2] === m1a[p*32 + c3]) a = a + 1;
                    if (log_w[4*ir + 3] === m1b[p*32 + c3]) a = a + 1;
                end
                if (a > best) best = a;
            end
            ref_agree = best;
        end
    endfunction

    // ---------------- helpers ----------------
    integer clear_cyc = 0;

    task automatic do_clear;
        begin
            @(negedge clk); clear = 1'b1;
            @(negedge clk); clear = 1'b0;
            clear_cyc = ncyc;
        end
    endtask

    task automatic wait_clocks(input integer n);
        repeat (n) @(posedge clk);
    endtask

    // TAP_DELAY scan: one READ(59) per cycle, the delay stepped while CS is
    // high (the previous cycle has long closed, the next has not reached the
    // delay line). good[d] = that cycle's result compared clean on all lanes.
    reg [63:0] good_d;

    task automatic scan(input string what, output integer lo, output integer hi,
                        output integer runs);
        integer d, c0, e0, e1;
        string row;
        begin
            good_d = 64'd0;
            delay  = 6'd8;
            for (d = 0; d < 3; d = d + 1) send(READ(59), -1);   // lead-in
            for (d = 0; d < 64; d = d + 1) begin
                delay = d[5:0];
                c0 = tchecks; e0 = err0; e1 = err1;
                send(READ(59), -1);
                good_d[d] = (tchecks == c0 + 1) && (err0 == e0) && (err1 == e1);
            end
            delay = 6'd8;
            send(READ(59), -1);
            send(READ(59), -1);
            row = "";
            lo = -1; hi = -1; runs = 0;
            for (d = 0; d < 64; d = d + 1) begin
                row = {row, good_d[d] ? "#" : "."};
                if (good_d[d] && (d == 0 || !good_d[d-1])) runs = runs + 1;
                if (good_d[d] && lo < 0) lo = d;
                if (good_d[d]) hi = d;
            end
            $display("    %0s TAP_DELAY 0..63: %0s", what, row);
        end
    endtask

    // ---------------- the run ----------------
    integer f, s, bad, nb, lo, hi, runs, first_cyc, c_before, half_x100;
    logic   wp;
    reg [31:0] v;

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("fpga/sim/tb_spi_tap.vcd");
            $dumpvars(0, tb_spi_tap);
        end
        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip0_A.mem", m0a);
        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip0_B.mem", m0b);
        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip1_A.mem", m1a);
        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip1_B.mem", m1b);

        #200;
        rst_n = 1'b1;
        chip_rst_n = 1'b1;
        #1000;

        chk(sclk_min === 32'hFFFF_FFFF && cycles === 0 && stat[16] === 1'b0,
            "after reset: TAP_SCLK_MIN ffffffff, no cycles, not running");

        // =================================================================
        $display("\n=== (a) 2 kS/s, TAP_DELAY 8, switched on mid-sweep; CALIBRATE; a glitch ===");
        tq = 1.0e9 / (2800.0 * 2000.0);
        run_sweep(0, 10, -1, 0);                // listening from cycle 10 of sweep 0
        run_sweep(1, -1, -1, 1);                // CALIBRATE in aux slot 0
        run_sweep(2, -1, 10, 0);                // SCLK glitch on the tap's wire, cycle 10
        wait_clocks(400);

        chk(stat[16] === 1'b1 && stat[12:8] === 5'h1F,
            "TAP_STAT: running, all five lines seen toggling");
        chk(nfr == 2, "two frames: sweeps 1 and 2 (sweep 0 was joined mid-way)");
        cmp_sweep(0, 1, bad, wp);
        chk(bad == 0 && !wp, "frame 0 = sweep 1, all 128 channels, not partial");
        cmp_sweep(1, 2, bad, wp);
        chk(bad == 0 && wp, "frame 1 = sweep 2, TAP_PARTIAL, slots 8 and 10 zero, rest right");
        nb = 0;
        for (k_ch = 0; k_ch < 4; k_ch = k_ch + 1)
            if (fr_val[128 + 8 + 32*k_ch] !== 0 || fr_val[128 + 10 + 32*k_ch] !== 0) nb = nb + 1;
        chk(nb == 0 && log_w[4*(sweep_base[2] + 10) + 0] != 0,
            "the lost slots read 0 on all four lanes (and were not 0 on the wire)");
        chk(cycles == 3*35 && framing == 1, "TAP_CYCLES 105, TAP_FRAMING 1 (the glitched cycle)");
        chk(sweeps == 2 && partials == 1, "TAP_SWEEPS 2, TAP_PARTIAL 1");
        $display("    TAP_CHECKS %0d (independent count %0d)", tchecks, want_checks(0, ncyc));
        chk(tchecks == want_checks(0, ncyc) && tchecks >= 4,
            "TAP_CHECKS = every known answer outside the CALIBRATE and glitch windows");
        chk(err0 == 0 && err1 == 0, "TAP_ERR0/1 zero at TAP_DELAY 8, CALIBRATE included");
        // READ(61) went out right after the CALIBRATE, so the chip swallowed it:
        // its result (cycle 0 of sweep 2) is not 0001 on the wire, and only the
        // blanking window kept it out of TAP_ERR.
        chk(log_cmd[sweep_base[1] + 33] == READ(61) &&
            log_w[4*sweep_base[2] + 0] !== 16'h0001,
            "the swallowed READ(61) came back wrong on the wire, and was not counted");
        chk(sclk_min < 8, "the glitch shows as a short TAP_SCLK_MIN");
        chk(good_after_glitch === 1'b0, "good (LD5) dark just after the framing error");
        chk(ref_ka_bad(0, ncyc) == 0, "reference: the master decoded every known answer right");

        // =================================================================
        $display("\n=== (b) 2 kS/s clean, after clear_counters ===");
        do_clear();
        chk(cycles == 0 && framing == 0 && sweeps == 0 && partials == 0 && tchecks == 0 &&
            err0 == 0 && err1 == 0 && sclk_min === 32'hFFFF_FFFF && stat[12:8] == 0,
            "clear: counters 0, TAP_SCLK_MIN ffffffff, toggles cleared");
        c_before = ncyc;
        run_sweep(3, -1, -1, 0);
        wait_clocks(400);
        chk(nfr == 3, "one more frame");
        cmp_sweep(2, 3, bad, wp);
        chk(bad == 0 && !wp, "frame 2 = sweep 3 exactly");
        chk(ref_agree(3) == 128, "reference: sweep 3 matches the emulator memory on all 128");
        half_x100 = $rtoi(200.0 * tq / FCLK_NS);
        $display("    TAP_SCLK_MIN %0d clocks, true half period %0d.%02d",
                 sclk_min, half_x100 / 100, half_x100 % 100);
        chk(sclk_min * 100 >= half_x100 - 100 && sclk_min * 100 <= half_x100 + 100,
            "TAP_SCLK_MIN within a clock of the true half period (2 kS/s)");
        chk(cycles == 35 && framing == 0 && sweeps == 1 && partials == 0,
            "TAP_CYCLES 35, no framing errors, one clean sweep");
        chk(tchecks == want_checks(clear_cyc, ncyc) && err0 == 0 && err1 == 0,
            "known answers all compared and clean");
        chk(good === 1'b1, "good (LD5) lit again after a clean stretch");

        // =================================================================
        $display("\n=== (c) 5 kS/s ===");
        do_clear();
        tq = 1.0e9 / (2800.0 * 5000.0);
        for (s = 4; s < 7; s = s + 1) run_sweep(s, -1, -1, 0);
        wait_clocks(400);
        chk(nfr == 6, "three more frames, one per sweep");
        bad = 0;
        for (s = 4; s < 7; s = s + 1) begin
            cmp_sweep(s - 1, s, nb, wp);
            bad = bad + nb + wp;
        end
        chk(bad == 0, "frames 3..5 = sweeps 4..6 exactly, none partial");
        chk(ref_agree(5) == 128, "reference: sweep 5 matches the emulator memory");
        half_x100 = $rtoi(200.0 * tq / FCLK_NS);
        $display("    TAP_SCLK_MIN %0d clocks, true half period %0d.%02d",
                 sclk_min, half_x100 / 100, half_x100 % 100);
        chk(sclk_min * 100 >= half_x100 - 100 && sclk_min * 100 <= half_x100 + 100,
            "TAP_SCLK_MIN within a clock of the true half period (5 kS/s)");
        chk(framing == 0 && sweeps == 3 && partials == 0 && err0 == 0 && err1 == 0,
            "no framing errors, three clean sweeps, no known-answer errors");
        chk(tchecks == want_checks(clear_cyc, ncyc) && tchecks > 0,
            "TAP_CHECKS matches the independent count");
        chk(ref_ka_bad(clear_cyc, ncyc) == 0, "reference known answers right at 5 kS/s");
        chk(prot_bad == 0, "no stream while busy, every frame exactly 128 channels");

        // =================================================================
        $display("\n=== (d) TAP_DELAY scans ===");
        en = 1'b0;                              // the decoder runs anyway
        tq = 1.0e9 / (2800.0 * 2000.0);
        scan("2 kS/s", lo, hi, runs);
        half_x100 = $rtoi(200.0 * tq / FCLK_NS);
        chk(runs == 1, "2 kS/s: the clean TAP_DELAY values are one contiguous window");
        chk(lo <= 8 && hi >= 8, "2 kS/s: the reset value 8 is inside it");
        chk((hi - lo + 1) * 100 >= half_x100 - 300 && (hi - lo + 1) * 100 <= half_x100 + 200,
            "2 kS/s: about one half period wide");
        chk(!good_d[0] && !good_d[63], "2 kS/s: TAP_DELAY 0 and 63 count errors");

        tq = 1.0e9 / (2800.0 * 5000.0);
        scan("5 kS/s", lo, hi, runs);
        half_x100 = $rtoi(200.0 * tq / FCLK_NS);
        chk(runs == 1, "5 kS/s: one contiguous window");
        chk(lo <= 8 && hi >= 8, "5 kS/s: the reset value 8 is inside it");
        chk((hi - lo + 1) * 100 >= half_x100 - 300 && (hi - lo + 1) * 100 <= half_x100 + 200,
            "5 kS/s: about one half period wide");
        chk(!good_d[0] && !good_d[40], "5 kS/s: TAP_DELAY 0 and 40 count errors");
        chk(nfr == 6, "no frames while assembly is off");

        // =================================================================
        $display("\n=== (e) frame builder held busy: buffered, owed, lost, in order ===");
        do_clear();
        force_busy = 1'b1;
        en = 1'b1;
        first_cyc = ncyc;
        // Ten short sweeps: CONVERT(0), CONVERT(31), two fillers. Each one
        // opens with slot 0's result and closes with slot 31's.
        for (s = 0; s < 10; s = s + 1) begin
            if (s == 7) force_busy = 1'b0;      // freed before sweep 7 starts
            send(CONVERT(0), -1); send(CONVERT(31), -1);
            send(READ(63), -1);   send(READ(63), -1);
        end
        wait_clocks(2000);
        $display("    frames %0d, lost %0d (at cycles %0d, %0d)",
                 nfr - 6, lost_n, lost_at[0] - first_cyc, lost_at[1] - first_cyc);
        chk(lost_n == 2 && lost_at[0] == first_cyc + 4*5 + 3 && lost_at[1] == first_cyc + 4*6 + 3,
            "sweeps 5 and 6 lost (2 buffered + 3 owed already), each pulses `lost` once");
        chk(nfr - 6 == 8 && sweeps == 8 && partials == 8,
            "8 frames out of 10 sweeps; TAP_SWEEPS 8, TAP_PARTIAL 8 (all partial)");
        // Expected order: sweeps 0, 1 with data, 2, 3, 4 as zero frames, 7, 8, 9 with data.
        bad = 0;
        for (f = 0; f < 8; f = f + 1) begin
            s = (f < 5) ? f : f + 2;
            for (k_ch = 0; k_ch < 128; k_ch = k_ch + 1) begin
                if (f >= 2 && f <= 4)
                    v[15:0] = 16'd0;
                else if (k_ch % 32 == 0)
                    v[15:0] = log_w[4*(first_cyc + 4*s + 2) + k_ch/32];
                else if (k_ch % 32 == 31)
                    v[15:0] = log_w[4*(first_cyc + 4*s + 3) + k_ch/32];
                else
                    v[15:0] = 16'd0;
                if (fr_val[(6 + f)*128 + k_ch] !== v[15:0]) bad = bad + 1;
            end
            if (fr_part[6 + f] !== 1'b1) bad = bad + 1;
        end
        chk(bad == 0, "frames in sweep order: data, data, zero x3, data x3, all TAP_PARTIAL");
        chk(prot_bad == 0, "still nothing streamed while frame_builder was busy");

        $display("\n=== %0d error(s) over %0d checks ===", errors, checks);
        if (errors == 0) $display("ALL CHECKS PASSED");
        else             $display("THERE WERE FAILURES");
        $finish;
    end

    initial begin
        #60_000_000;
        $display("TIMEOUT");
        $display("THERE WERE FAILURES");
        $finish;
    end

endmodule

`default_nettype wire
