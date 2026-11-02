// ============================================================================
// tb_tap_arty_s7_25.sv -- the tap bitstream, from the pins out.
// ----------------------------------------------------------------------------
// Runs tap_arty_s7_25_top itself -- MMCM, resets, IBUFDS, pad flops, the
// on-die emulator -- with unisim_stubs.sv for the vendor primitives, and the
// ESP as qspi_hd_monitor on the JC pins. All control goes over the UART.
// tb_spi_tap covers the tap's decode against an Intan-timed master in depth;
// this is the proof that the board and the rest of the receiver carry it.
//
//  (a) which bitstream: CAPS has the emulator (bit8) and the tap (bit9);
//      TAP_DELAY resets to 8 and saturates at 63; TAP_SCLK_MIN starts at
//      ffffffff; TAP_STAT says the tap is not running.
//  (b) the pins: each LVDS pair, driven alone, lights its own TAP_STAT level
//      bit, so CS, SCLK, MOSI, MISO1 and MISO2 reach the tap on the right
//      pairs with the right polarity.
//  (c) the self-test (MODE bit3 + bit4, SPI_DIV 5): spi_master sweeps the
//      on-die emulator, the tap hears that bus, and over QSPI every frame
//      carries flags TAP (and not TAP_PARTIAL), seq runs without a gap, and
//      all 128 channels equal the emulator's playback memory at one offset
//      that advances by exactly one per frame. LD5 lit while it runs.
//  (d) the counters agree with the receiver's own: TAP_CYCLES = SPI_CYCLES,
//      TAP_SWEEPS = FRAMES, known answers 3 per sweep with no errors,
//      TAP_SCLK_MIN = the 12-clock half period, OVERRUNS 0.
//  (e) clear_counters zeroes the TAP counters and resets TAP_SCLK_MIN.
//  (f) the bench recipe from CONTRACTS.md section 7 -- SPI_DIV 15 and the reset
//      SAMPLE_PERIOD and TAP_DELAY -- runs clean: Rhythm-like 32-clock half
//      period, sweeps framed whole, no known-answer errors.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_tap_arty_s7_25;

    localparam int N_CH        = 128;
    localparam int FPD         = 5;
    localparam int HDR_BYTES   = 20;
    localparam int FRAME_BYTES = HDR_BYTES + N_CH * 2;      // 276
    localparam int DATA_BYTES  = FPD * FRAME_BYTES;         // 1380

    localparam int  QSPI_DIV = 6;
    localparam int  BAUD     = 6_000_000;                   // divisor 16 at 96 MHz
    localparam real BITP     = 1.0e9 / BAUD;

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

    // ---------------- the board ----------------
    reg  clk_100mhz = 1'b0;  always #5.0 clk_100mhz = ~clk_100mhz;
    reg  rst_btn     = 1'b1;
    reg  uart_txd_in = 1'b1;
    reg  esp_ready   = 1'b1;

    // The cable, single-ended here and split into pairs below.
    reg  x_cs_n = 1'b1, x_sclk = 1'b0, x_mosi = 1'b0, x_miso0 = 1'b0, x_miso1 = 1'b0;

    wire qspi_cs_n, qspi_sclk;
    wire [3:0] qspi_dq;
    wire uart_rxd_out;
    wire [3:0] led;

    tap_arty_s7_25_top #(
        .QSPI_DIV(QSPI_DIV), .BAUD(BAUD),
        .MEM_C0A("fpga/upstream/rhd2164-emulator-mem/chip0_A.mem"),
        .MEM_C0B("fpga/upstream/rhd2164-emulator-mem/chip0_B.mem"),
        .MEM_C1A("fpga/upstream/rhd2164-emulator-mem/chip1_A.mem"),
        .MEM_C1B("fpga/upstream/rhd2164-emulator-mem/chip1_B.mem")
    ) dut (
        .clk_100mhz(clk_100mhz), .rst_btn(rst_btn),
        .tap_cs_p   (x_cs_n),  .tap_cs_n   (~x_cs_n),
        .tap_sclk_p (x_sclk),  .tap_sclk_n (~x_sclk),
        .tap_mosi_p (x_mosi),  .tap_mosi_n (~x_mosi),
        .tap_miso0_p(x_miso0), .tap_miso0_n(~x_miso0),
        .tap_miso1_p(x_miso1), .tap_miso1_n(~x_miso1),
        .qspi_cs_n(qspi_cs_n), .qspi_sclk(qspi_sclk), .qspi_dq(qspi_dq),
        .esp_ready(esp_ready),
        .uart_txd_in(uart_txd_in), .uart_rxd_out(uart_rxd_out),
        .led(led)
    );

    // LD5's stretch shortened from 44 ms to 170 us so it can go dark and come
    // back inside the run. Simulation only; the bitstream keeps the default.
    defparam dut.u_rx.g_tap.u_tap.GOOD_W = 14;

    // ---------------- the ESP ----------------
    reg mon_en = 1'b0;
    qspi_hd_monitor #(.EXP_BYTES(DATA_BYTES)) mon (
        .en(mon_en), .cs_n(qspi_cs_n), .sclk(qspi_sclk), .dq(qspi_dq)
    );

    integer dg_consumed = 0;

    task automatic wait_dgram(output integer ok);
        integer guard;
        begin
            ok = 0;
            for (guard = 0; guard < 400_000 && !ok; guard = guard + 1) begin
                if (mon.dg_seen != dg_consumed) begin
                    dg_consumed = mon.dg_seen;
                    ok = 1;
                end else begin
                    #10;
                end
            end
            if (!ok) $display("  FAIL  no datagram arrived within 4 ms");
        end
    endtask

    function automatic [15:0] g16(input integer off);
        g16 = {mon.dgcopy[off+1], mon.dgcopy[off]};
    endfunction

    function automatic [31:0] g32(input integer off);
        g32 = {mon.dgcopy[off+3], mon.dgcopy[off+2], mon.dgcopy[off+1], mon.dgcopy[off]};
    endfunction

    // ---------------- UART, on the real pins ----------------
    task automatic uart_put(input [7:0] b);
        integer i;
        begin
            uart_txd_in = 1'b0; #(BITP);
            for (i = 0; i < 8; i = i + 1) begin
                uart_txd_in = b[i]; #(BITP);
            end
            uart_txd_in = 1'b1; #(BITP);
        end
    endtask

    task automatic uart_puts(input string s);
        integer i;
        for (i = 0; i < s.len(); i = i + 1) uart_put(s[i]);
    endtask

    task automatic uart_get(output [7:0] b);
        integer i;
        begin
            @(negedge uart_rxd_out);
            #(BITP * 1.5);
            for (i = 0; i < 8; i = i + 1) begin
                b[i] = uart_rxd_out;
                #(BITP);
            end
        end
    endtask

    task automatic uart_getline(output string s);
        logic [7:0] b;
        logic       fin;
        begin
            s = ""; fin = 1'b0;
            while (!fin) begin
                uart_get(b);
                if (b == 8'h0A) fin = 1'b1;
                else s = {s, string'(b)};
            end
        end
    endtask

    string reply;

    task automatic cmd(input string c);
        fork
            uart_puts(c);
            uart_getline(reply);
        join
    endtask

    task automatic wr(input [7:0] a, input [31:0] v);
        string s;
        begin
            $sformat(s, "W %02x %08x\n", a, v);
            cmd(s);
            if (reply != "ok") begin
                errors = errors + 1;
                $display("  FAIL  [W %02x] -- board replied [%0s]", a, reply);
            end
        end
    endtask

    function automatic [31:0] hex32(input string s, input integer from);
        integer i;
        logic [7:0] c;
        begin
            hex32 = 32'd0;
            for (i = from; i < s.len(); i = i + 1) begin
                c = s[i];
                if (c >= "0" && c <= "9")      hex32 = (hex32 << 4) | (c - "0");
                else if (c >= "a" && c <= "f") hex32 = (hex32 << 4) | (c - 8'h57);
            end
        end
    endfunction

    task automatic rd(input [7:0] a, output [31:0] v);
        string s;
        begin
            $sformat(s, "R %02x\n", a);
            cmd(s);
            v = hex32(reply, 3);
        end
    endtask

    // ---------------- the emulator's playback memory ----------------
    reg [15:0] m0a [0:MEM_DEPTH-1];
    reg [15:0] m0b [0:MEM_DEPTH-1];
    reg [15:0] m1a [0:MEM_DEPTH-1];
    reg [15:0] m1b [0:MEM_DEPTH-1];

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

    // tb_spi_master's method: the offset channel 0 agrees with, and how many of
    // the 128 channels agree with that one offset.
    task automatic frame_offset(input integer f, output integer off, output integer agree);
        integer o, c, p, a, best;
        begin
            o    = f * FRAME_BYTES;
            off  = -1;
            best = 0;
            for (p = 0; p < MEM_SAMPLES; p = p + 1) begin
                if (g16(o + HDR_BYTES) === m0a[p*32]) begin
                    a = 0;
                    for (c = 0; c < N_CH; c = c + 1)
                        if (g16(o + HDR_BYTES + 2*c) === mem_at(c, p)) a = a + 1;
                    if (a > best) begin best = a; off = p; end
                end
            end
            agree = best;
        end
    endtask

    // ---------------- the run ----------------
    integer d, f, ok, k, off, agree, first_off, exp_seq, nfr;
    integer flag_bad, seq_bad, agree_bad, off_bad, ttx_bad;
    reg [31:0] v, v2, sweeps, frames, spi_cyc, link_chk;
    reg [31:0] f_seq, f_tcap, f_ttx;

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("fpga/sim/tb_tap_arty_s7_25.vcd");
            $dumpvars(0, tb_tap_arty_s7_25);
        end
        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip0_A.mem", m0a);
        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip0_B.mem", m0b);
        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip1_A.mem", m1a);
        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip1_B.mem", m1b);

        #200 rst_btn = 1'b0;
        #3000;
        mon_en = 1'b1;

        // =================================================================
        $display("\n=== (a) which bitstream ===");
        rd(8'h1f, v);
        chk(v === 32'h1C10_0002, "ID 1c100002 over the board's UART pins");
        rd(8'h1e, v);
        chk(v === 32'h004E_0306, "CAPS: 78 taps, tap (bit9), emulator (bit8), QSPI_DIV 6");
        rd(8'h05, v);
        chk(v === 32'd8, "TAP_DELAY resets to 8");
        wr(8'h05, 32'h50);
        rd(8'h05, v);
        chk(v === 32'd63, "TAP_DELAY write 0x50 saturates at 63");
        wr(8'h05, 32'd8);
        rd(8'h38, v);
        chk(v === 32'hFFFF_FFFF, "TAP_SCLK_MIN ffffffff until an SCLK period is heard");
        rd(8'h30, v);
        chk(v[16] === 1'b0 && v[4:0] === 5'b00001, "TAP_STAT: not running, idle pins (CS high)");

        // =================================================================
        $display("\n=== (b) each LVDS pair reaches its own TAP_STAT bit ===");
        nfr = 0;
        for (k = 0; k < 5; k = k + 1) begin
            {x_miso1, x_miso0, x_mosi, x_sclk, x_cs_n} = 5'(1 << k);
            #500;
            rd(8'h30, v);
            if (v[4:0] !== 5'(1 << k)) begin
                nfr = nfr + 1;
                $display("    line %0d alone high: TAP_STAT[4:0] = %05b", k, v[4:0]);
            end
        end
        {x_miso1, x_miso0, x_mosi, x_sclk, x_cs_n} = 5'b00001;
        #500;
        chk(nfr == 0, "CS, SCLK, MOSI, MISO1, MISO2: each pair lights only its bit");
        // CS went low and high around one SCLK pulse: a framing error.
        chk(led[3] === 1'b0, "LD5 dark: the wiggle was a CS cycle without 16 + 16 edges");
        rd(8'h32, v);
        chk(v == 1, "TAP_FRAMING 1 for it");
        rd(8'h30, v);
        chk(v[12:8] === 5'h1F, "TAP_STAT: all five seen toggling (sticky)");

        // =================================================================
        $display("\n=== (c) self-test: the tap hears spi_master and the on-die emulator ===");
        wr(8'h04, 32'd5);                                   // SCLK 4 MHz, half period 12 clocks
        wr(8'h01, 32'd18000);                               // 5.33 kS/s, over 35 x 84 x 6
        wr(8'h03, 32'h18);                                  // loopback + tap
        wr(8'h00, 32'h2);                                   // clear counters
        rd(8'h30, v);
        chk(v[16] === 1'b0 && v[12:8] === 5'h0, "TAP_STAT after clear: idle, toggles cleared");
        wr(8'h00, 32'h1);

        flag_bad = 0; seq_bad = 0; agree_bad = 0; off_bad = 0; ttx_bad = 0;
        first_off = -1; exp_seq = 0;
        for (d = 0; d < 2; d = d + 1) begin
            wait_dgram(ok);
            if (!ok) errors = errors + 1;
            for (f = 0; f < FPD; f = f + 1) begin
                if (g16(f*FRAME_BYTES) !== 16'hA55A || mon.dgcopy[f*FRAME_BYTES + 2] !== 8'd0)
                    flag_bad = flag_bad + 1;
                if (mon.dgcopy[f*FRAME_BYTES + 3] !== 8'h04) begin
                    flag_bad = flag_bad + 1;
                    $display("    dgram %0d frame %0d: flags %02h", d, f,
                             mon.dgcopy[f*FRAME_BYTES + 3]);
                end
                f_seq  = g32(f*FRAME_BYTES + 4);
                f_tcap = g32(f*FRAME_BYTES + 8);
                f_ttx  = g32(f*FRAME_BYTES + 12);
                if (f_seq !== exp_seq[31:0]) seq_bad = seq_bad + 1;
                exp_seq = f_seq + 1;
                if (f_ttx == 0 || f_ttx < f_tcap) ttx_bad = ttx_bad + 1;
                frame_offset(f, off, agree);
                if (agree != N_CH) begin
                    agree_bad = agree_bad + 1;
                    if (agree_bad <= 3)
                        $display("    dgram %0d frame %0d: only %0d/128 agree", d, f, agree);
                end else if (first_off < 0) begin
                    first_off = off;
                end else if (off != (first_off + d*FPD + f) % MEM_SAMPLES) begin
                    off_bad = off_bad + 1;
                end
            end
        end
        chk(led[3] === 1'b1 && led[0] === 1'b1, "LD5 (tap hears clean cycles) and LD2 (run) lit");
        rd(8'h30, v);
        chk(v[16] === 1'b1 && v[12:8] === 5'h1F, "TAP_STAT: running, all five internal lines move");
        wr(8'h00, 32'h0);
        #600_000;                                           // the last sweep and its frame drain

        chk(flag_bad == 0, "every frame: magic, RAW16, flags = TAP only");
        chk(seq_bad == 0,  "seq starts at 0 and runs without a gap");
        chk(ttx_bad == 0,  "t_tx nonzero and never earlier than t_cap");
        chk(agree_bad == 0, "every frame matches all 128 emulator channels at one offset");
        chk(off_bad == 0,  "the playback offset advances exactly one per frame: one frame per sweep");
        chk(mon.bad_len == 0 && mon.proto_err == 0, "QSPI: every segment 1380 bytes, no protocol error");

        // =================================================================
        $display("\n=== (d) the tap's counters against the receiver's own ===");
        rd(8'h33, sweeps);  rd(8'h10, frames);
        rd(8'h31, v);       rd(8'h15, spi_cyc);
        rd(8'h16, link_chk);
        $display("    TAP_SWEEPS %0d, FRAMES %0d, TAP_CYCLES %0d, SPI_CYCLES %0d, LINK_CHECKS %0d",
                 sweeps, frames, v, spi_cyc, link_chk);
        chk(sweeps == frames && sweeps >= 10, "TAP_SWEEPS = FRAMES: every frame built was a tap sweep");
        chk(v == spi_cyc && v == 35 * link_chk, "TAP_CYCLES = SPI_CYCLES = 35 per sweep");
        chk(sweeps == link_chk || sweeps + 1 == link_chk,
            "one tap sweep per spi_master sweep (the last may end with `run`)");
        rd(8'h32, v);  rd(8'h34, v2);
        chk(v == 0 && v2 == 0, "TAP_FRAMING 0, TAP_PARTIAL 0");
        rd(8'h35, v);
        // READ(59), READ(40), READ(63) in every sweep; the first sweep's two
        // late results land in the reset's blanking window.
        chk(v == 3 * link_chk - 2, "TAP_CHECKS: the 3 known answers of every sweep");
        rd(8'h36, v);  rd(8'h37, v2);
        chk(v == 0 && v2 == 0, "TAP_ERR0/1 zero at the reset TAP_DELAY");
        rd(8'h38, v);
        chk(v == 12, "TAP_SCLK_MIN = 12 clocks, the half period at SPI_DIV 5");
        rd(8'h13, v);
        chk(v == 0, "OVERRUNS 0");

        // =================================================================
        $display("\n=== (e) clear_counters ===");
        wr(8'h00, 32'h2);
        k = 0;
        for (d = 8'h31; d <= 8'h37; d = d + 1) begin
            rd(d[7:0], v);
            if (v !== 0) k = k + 1;
        end
        rd(8'h38, v);
        chk(k == 0 && v === 32'hFFFF_FFFF, "TAP_CYCLES..TAP_ERR1 zero, TAP_SCLK_MIN ffffffff");
        rd(8'h30, v);
        chk(v[12:8] === 5'h0, "TAP_STAT toggles cleared");

        // =================================================================
        $display("\n=== (f) the bench recipe: SPI_DIV 15, reset SAMPLE_PERIOD and TAP_DELAY ===");
        wr(8'h04, 32'd15);
        wr(8'h01, 32'd48000);
        wr(8'h00, 32'h1);
        #1_100_000;                                         // two sweeps at 2 kS/s
        wr(8'h00, 32'h0);
        #600_000;
        rd(8'h33, sweeps);  rd(8'h34, v2);
        rd(8'h16, link_chk);
        $display("    TAP_SWEEPS %0d, LINK_CHECKS %0d", sweeps, link_chk);
        chk(sweeps >= 1 && v2 == 0, "sweeps framed whole, none partial");
        rd(8'h35, v);  rd(8'h36, v2);  rd(8'h37, frames);
        chk(v == 3 * link_chk && v2 == 0 && frames == 0,
            "every known answer compared, none wrong, at TAP_DELAY 8");
        rd(8'h38, v);
        chk(v == 32, "TAP_SCLK_MIN 32: SCLK 1.5 MHz, Rhythm's at 2.1 kS/s");

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
