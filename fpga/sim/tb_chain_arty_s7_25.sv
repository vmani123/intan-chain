// ============================================================================
// tb_chain_arty_s7_25.sv -- the single-board bench, from the pins out.
// ----------------------------------------------------------------------------
// Runs chain_arty_s7_25_top itself -- MMCM, resets, both clock domains, pad
// registers -- with unisim_stubs.sv standing in for the three vendor
// primitives. The jumper wires are modelled here, outside the design, with a
// couple of nanoseconds of delay each way:
//
//   JA1 -> JB1 cs_n   JA2 -> JB2 sclk   JA3 -> JB3 mosi
//   JB4 -> JA4 miso0  JB7 -> JA7 miso1
//
// and the ESP is qspi_hd_monitor on the JC pins, always ready. Everything is
// driven over the UART, as at the bench. QSPI_DIV is 6, the first-light
// bitstream's divider. tb_rx_top already covers the receiver at DIV 2; what
// this adds is the board wrapper: that each port goes where the pin table
// says, that the divider reaches the QSPI master, and that the emulator on
// its own 192 MHz clock answers the receiver through the jumpers.
//
//  (a) internal pattern over JC: bytes match the host's CHECK vectors and
//      QSPI SCLK is 96 / 12 = 8 MHz.
//  (b) real SPI through the jumpers: scan PHASE_SELECT 0..11, print which
//      phases recover all 128 channels, then run at the middle of the widest
//      clean run and demand that every frame matches the emulator's memory
//      with the playback pointer advancing one sample per frame.
//  (c) the counters and LEDs agree with what the pins showed.
//
// The phase window printed here has no pad or cable delay in it beyond the
// modelled 2 ns, so it predicts the bench only roughly. The bench answer comes
// from `python3 -m chain.sweep --phase-scan`.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_chain_arty_s7_25;

    localparam int N_CH        = 128;
    localparam int FPD         = 5;
    localparam int HDR_BYTES   = 20;
    localparam int FRAME_BYTES = HDR_BYTES + N_CH * 2;      // 276
    localparam int DATA_BYTES  = FPD * FRAME_BYTES;         // 1380

    localparam int QSPI_DIV = 6;
    localparam int BAUD     = 6_000_000;                    // divisor 16 at 96 MHz
    localparam real BITP    = 1.0e9 / BAUD;
    localparam real JUMPER_NS = 2.0;

    localparam int MEM_SAMPLES  = 256;
    localparam int MEM_DEPTH    = MEM_SAMPLES * 32;
    localparam int GOLD_SAMPLES = 8;

    integer errors = 0;
    integer checks = 0;

    task automatic chk(input cond, input [8*64-1:0] name);
        begin
            checks = checks + 1;
            if (cond) $display("  pass  %0s", name);
            else begin errors = errors + 1; $display("  FAIL  %0s", name); end
        end
    endtask

    // ---------------- the board ----------------
    reg  clk_100mhz = 1'b0;  always #5.0 clk_100mhz = ~clk_100mhz;
    reg  rst_btn    = 1'b1;
    reg  uart_txd_in = 1'b1;
    reg  esp_ready  = 1'b1;

    wire spi_cs_n, spi_sclk, spi_mosi, spi_miso0, spi_miso1;
    wire emu_cs_n, emu_sclk, emu_mosi, emu_miso0, emu_miso1;
    wire qspi_cs_n, qspi_sclk;
    wire [3:0] qspi_dq;
    wire uart_rxd_out;
    wire [3:0] led;

    chain_arty_s7_25_top #(
        .QSPI_DIV(QSPI_DIV), .BAUD(BAUD),
        .MEM_C0A("fpga/upstream/rhd2164-emulator-mem/chip0_A.mem"),
        .MEM_C0B("fpga/upstream/rhd2164-emulator-mem/chip0_B.mem"),
        .MEM_C1A("fpga/upstream/rhd2164-emulator-mem/chip1_A.mem"),
        .MEM_C1B("fpga/upstream/rhd2164-emulator-mem/chip1_B.mem")
    ) dut (
        .clk_100mhz(clk_100mhz), .rst_btn(rst_btn),
        .spi_cs_n(spi_cs_n), .spi_sclk(spi_sclk), .spi_mosi(spi_mosi),
        .spi_miso0(spi_miso0), .spi_miso1(spi_miso1),
        .emu_cs_n(emu_cs_n), .emu_sclk(emu_sclk), .emu_mosi(emu_mosi),
        .emu_miso0(emu_miso0), .emu_miso1(emu_miso1),
        .qspi_cs_n(qspi_cs_n), .qspi_sclk(qspi_sclk), .qspi_dq(qspi_dq),
        .esp_ready(esp_ready),
        .uart_txd_in(uart_txd_in), .uart_rxd_out(uart_rxd_out),
        .led(led)
    );

    // ---------------- the jumper wires ----------------
    assign #(JUMPER_NS) emu_cs_n  = spi_cs_n;
    assign #(JUMPER_NS) emu_sclk  = spi_sclk;
    assign #(JUMPER_NS) emu_mosi  = spi_mosi;
    assign #(JUMPER_NS) spi_miso0 = emu_miso0;
    assign #(JUMPER_NS) spi_miso1 = emu_miso1;

    // ---------------- the ESP ----------------
    reg mon_en = 1'b0;
    qspi_hd_monitor #(.EXP_BYTES(DATA_BYTES)) mon (
        .en(mon_en), .cs_n(qspi_cs_n), .sclk(qspi_sclk), .dq(qspi_dq)
    );

    // QSPI SCLK period, measured between rising edges inside a transaction.
    real t_rise = -1.0, sclk_min = 1.0e9, sclk_max = 0.0;
    always @(negedge qspi_cs_n) t_rise = -1.0;
    always @(posedge qspi_sclk) if (!qspi_cs_n) begin
        if (t_rise >= 0.0) begin
            if ($realtime - t_rise < sclk_min) sclk_min = $realtime - t_rise;
            if ($realtime - t_rise > sclk_max) sclk_max = $realtime - t_rise;
        end
        t_rise = $realtime;
    end

    integer dg_consumed = 0;

    task automatic wait_dgram(output integer ok);
        integer guard;
        begin
            ok = 0;
            for (guard = 0; guard < 2_000_000 && !ok; guard = guard + 1) begin
                if (mon.dg_seen != dg_consumed) begin
                    dg_consumed = mon.dg_seen;
                    ok = 1;
                end else begin
                    #10;
                end
            end
            if (!ok) $display("  FAIL  no datagram arrived within 20 ms");
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

    task automatic cmd_ok(input string c);
        begin
            cmd(c);
            if (reply != "ok") begin
                errors = errors + 1;
                $display("  FAIL  [%0s] -- board replied [%0s]", c, reply);
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

    task automatic reg_rd(input string addr, output [31:0] v);
        begin
            cmd({"R ", addr, "\n"});
            v = hex32(reply, 3);
        end
    endtask

    // ---------------- golden data ----------------
    reg [15:0] gchk [0:GOLD_SAMPLES*N_CH-1];
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

    // Find the playback offset channel 0 agrees with, then count how many of
    // the 128 channels agree with that one offset.
    task automatic spi_frame_offset(input integer f, output integer off,
                                    output integer agree);
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
    integer d, f, ok, off, agree, first_off, p;
    integer hdr_bad, pay_bad, off_bad, agree_bad;
    integer good [0:11];
    integer best_lo, best_len, run_lo, run_len, phase;
    reg [31:0] v, seq;
    string s;

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("fpga/sim/tb_chain_arty_s7_25.vcd");
            $dumpvars(0, tb_chain_arty_s7_25);
        end
        $readmemh("fpga/sim/vectors/pattern_check.mem", gchk);
        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip0_A.mem", m0a);
        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip0_B.mem", m0b);
        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip1_A.mem", m1a);
        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip1_B.mem", m1b);

        // BTN0 held, then released: the MMCM locks and both domains come out
        // of reset on their own clocks.
        #200 rst_btn = 1'b0;
        #3000;
        mon_en = 1'b1;

        // =================================================================
        $display("\n=== (a) internal pattern over Pmod JC, QSPI_DIV = %0d ===", QSPI_DIV);
        reg_rd("1f", v);
        chk(v === 32'h1C10_0001, "ID register reads back over the board's UART pins");

        // 8000 clocks per sample = 12 kS/s: one datagram per 40000 clocks,
        // comfortably more than the ~35000 one takes at 8 MHz.
        cmd_ok("W 01 00001f40\n");
        cmd_ok("W 03 00000004\n");
        cmd_ok("W 00 00000001\n");
        hdr_bad = 0; pay_bad = 0;
        for (d = 0; d < 2; d = d + 1) begin
            wait_dgram(ok);
            if (!ok) errors = errors + 1;
            for (f = 0; f < FPD; f = f + 1) begin
                if (g16(f*FRAME_BYTES) !== 16'hA55A)      hdr_bad = hdr_bad + 1;
                if (g16(f*FRAME_BYTES + 16) !== 16'd128)  hdr_bad = hdr_bad + 1;
                seq = g32(f*FRAME_BYTES + 4);
                if (seq < GOLD_SAMPLES)
                    for (p = 0; p < N_CH; p = p + 1)
                        if (g16(f*FRAME_BYTES + HDR_BYTES + 2*p) !== gchk[seq*N_CH + p])
                            pay_bad = pay_bad + 1;
            end
        end
        cmd_ok("W 00 00000000\n");

        chk(hdr_bad == 0, "frame headers intact on the JC pins");
        chk(pay_bad == 0, "payload matches the host's CHECK vectors");
        chk(mon.bad_len == 0 && mon.proto_err == 0 && mon.missing_done == 0,
            "ESP slave-HD protocol clean: 1380-byte segments, WR_DONE each");
        $display("    QSPI SCLK period %0.1f .. %0.1f ns", sclk_min, sclk_max);
        chk(sclk_min > 124.0 && sclk_max < 126.0,
            "QSPI_DIV reaches the master: SCLK = 8 MHz");

        // =================================================================
        $display("\n=== (b) real SPI through the JA <-> JB jumpers ===");
        cmd_ok("W 03 00000000\n");
        for (phase = 0; phase < 12; phase = phase + 1) begin
            $sformat(s, "W 02 %08x\n", phase);
            cmd_ok(s);
            cmd_ok("W 00 00000001\n");
            wait_dgram(ok);                    // may hold frames from the last phase
            wait_dgram(ok);
            good[phase] = ok;
            for (f = 0; f < FPD && good[phase]; f = f + 1) begin
                spi_frame_offset(f, off, agree);
                if (agree != N_CH) good[phase] = 0;
            end
            cmd_ok("W 00 00000000\n");
            $display("    PHASE_SELECT %2d: %0s", phase,
                     good[phase] ? "all 128 channels" : "errors");
        end

        // The middle of the widest run of clean phases.
        best_lo = -1; best_len = 0; run_len = 0; run_lo = 0;
        for (phase = 0; phase < 12; phase = phase + 1) begin
            if (good[phase]) begin
                if (run_len == 0) run_lo = phase;
                run_len = run_len + 1;
                if (run_len > best_len) begin best_len = run_len; best_lo = run_lo; end
            end else begin
                run_len = 0;
            end
        end
        chk(best_len > 0, "some PHASE_SELECT recovers all 128 channels");
        phase = (best_len > 0) ? best_lo + (best_len - 1) / 2 : 0;
        $display("    running at PHASE_SELECT %0d (clean window %0d..%0d)",
                 phase, best_lo, best_lo + best_len - 1);

        $sformat(s, "W 02 %08x\n", phase);
        cmd_ok(s);
        cmd_ok("W 00 00000002\n");             // clear counters
        cmd_ok("W 00 00000001\n");
        wait_dgram(ok);                        // may hold frames from before the run
        agree_bad = 0; off_bad = 0; first_off = -1;
        for (d = 0; d < 3; d = d + 1) begin
            wait_dgram(ok);
            if (!ok) errors = errors + 1;
            for (f = 0; f < FPD; f = f + 1) begin
                spi_frame_offset(f, off, agree);
                if (agree != N_CH) agree_bad = agree_bad + 1;
                else if (first_off < 0) first_off = off;
                else if (off != (first_off + d*FPD + f) % MEM_SAMPLES) off_bad = off_bad + 1;
            end
        end
        chk(agree_bad == 0, "every frame matches all 128 emulator channels");
        chk(off_bad == 0,   "the playback pointer advances one sample per frame");
        chk(led[3] === 1'b1, "LD5 lit: the emulator sees chip select on JB");
        chk(led[0] === 1'b1, "LD2 lit: the receiver is running");
        cmd_ok("W 00 00000000\n");

        // =================================================================
        $display("\n=== (c) counters ===");
        #500_000;                              // let the last datagram out
        reg_rd("13", v);
        chk(v === 32'd0, "OVERRUNS zero at 12 kS/s with QSPI at 8 MHz");
        reg_rd("14", v);
        chk(v === 32'd0, "STALLS zero with the ESP always ready");
        reg_rd("11", v);
        chk(v > 32'd0, "DGRAMS counted the run");
        chk(mon.proto_err == 0 && mon.orphan_done == 0 && mon.missing_done == 0,
            "no malformed QSPI transaction in the whole run");

        $display("\n=== %0d error(s) over %0d checks ===", errors, checks);
        if (errors == 0) $display("ALL CHECKS PASSED");
        else             $display("THERE WERE FAILURES");
        $finish;
    end

    initial begin
        #200_000_000;
        $display("TIMEOUT");
        $display("THERE WERE FAILURES");
        $finish;
    end

endmodule

`default_nettype wire
