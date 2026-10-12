// ============================================================================
// tb_rx_top.sv -- the whole receiver, driven the way the bench will drive it.
// ----------------------------------------------------------------------------
// Nothing here pokes an internal signal. The board is configured over the real
// serial wires, exactly as CONTRACTS.md section 4 specifies, and every check is
// made on bytes recovered from the QSPI pins by qspi_hd_monitor, which latches
// them where the ESP32-S3's slave-HD hardware would: command and address on one
// line, 8 dummy cycles, data on four, and nothing delivered until a WR_DONE
// closes the segment. If the FPGA and the host ever disagree about the frame
// layout, or the FPGA and the ESP about the bus protocol, this is where it
// shows up.
//
// FOUR THINGS ARE BEING PROVEN
//
//  (a) INTERNAL PATTERN. mode[2] = 1, no SPI involved. Payload is compared
//      against fpga/sim/vectors/pattern_check.mem, which tools/gen_pattern_
//      vectors.py produced from host/chain/pattern.py -- so a pass means the
//      FPGA and the host's scoring code agree bit for bit, which is the whole
//      point of the CHECK pattern. Run again with mode[0] = 1 to prove the
//      REALISTIC pattern and the flags bit that selects it travel together.
//
//  (b) REAL SPI. mode[2] = 0 against two unmodified rhd2164_emulator cores.
//      The playback offset is found from channel 0 and then all 128 channels
//      must agree with that one offset, and the offset must advance by exactly
//      one per frame -- the technique tb_spi_master.sv uses, for the same
//      reason: an off-by-one in the emulator's playback pointer is harmless,
//      any error in de-interleaving or mapping is not.
//
//  (c) BACK PRESSURE. With esp_ready held low, STALLS must climb -- that is the
//      number that tells the host the ESP and not the radio was the limit --
//      and once the four slots are full OVERRUNS must climb too.
//
//  (d) NOTHING IS EVER TORN. Every datagram, in every phase, must carry magic
//      at all five frame offsets and be exactly 5 x 276 bytes long.
//
// The UART divisor is shrunk to 16 so the configuration traffic does not
// dominate the run; FRAMES_PER_DGRAM stays at the real value of 5 throughout.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_rx_top;

    localparam int N_CH        = 128;
    localparam int FPD         = 5;
    localparam int HDR_BYTES   = 20;
    localparam int FRAME_BYTES = HDR_BYTES + N_CH * 2;      // 276
    localparam int DATA_BYTES  = FPD * FRAME_BYTES;         // 1380

    localparam int CLK_HZ  = 96_000_000;
    localparam int BAUD    = 6_000_000;                     // divisor 16
    localparam real HALF   = 1.0e9 / (2.0 * CLK_HZ);
    localparam real BITP   = 1.0e9 / BAUD;

    localparam int MEM_SAMPLES = 256;
    localparam int MEM_DEPTH   = MEM_SAMPLES * 32;
    localparam int GOLD_SAMPLES = 8;

    localparam int WORKING_PHASE = 2;   // from tb_spi_master's phase scan

    integer errors = 0;
    integer checks = 0;

    task automatic chk(input cond, input [8*64-1:0] name);
        begin
            checks = checks + 1;
            if (cond) $display("  pass  %0s", name);
            else begin errors = errors + 1; $display("  FAIL  %0s", name); end
        end
    endtask

    // ---------------- clocks and reset ----------------
    reg clk    = 1'b0;  always #(HALF) clk = ~clk;
    reg clk400 = 1'b0;  always #1.25   clk400 = ~clk400;

    reg rst_n     = 1'b0;
    reg uart_rx_i = 1'b1;
    reg esp_ready = 1'b1;

    wire uart_tx_o;
    wire spi_cs_n, spi_sclk, spi_mosi;
    wire miso0, miso1;
    wire qspi_cs_n, qspi_sclk, qspi_dq_oe;
    wire [3:0] qspi_dq_out;
    wire [2:0] led;

    rx_top #(.CLK_HZ(CLK_HZ), .BAUD(BAUD), .FRAMES_PER_DGRAM(FPD)) dut (
        .clk(clk), .rst_n(rst_n),
        .uart_rx_i(uart_rx_i), .uart_tx_o(uart_tx_o),
        .spi_cs_n(spi_cs_n), .spi_sclk(spi_sclk), .spi_mosi(spi_mosi),
        .miso0(miso0), .miso1(miso1),
        .esp_ready(esp_ready),
        .qspi_cs_n(qspi_cs_n), .qspi_sclk(qspi_sclk),
        .qspi_dq_out(qspi_dq_out), .qspi_dq_oe(qspi_dq_oe),
        .led(led)
    );

    rhd2164_emulator #(
        .MEM_A_FILE("fpga/upstream/rhd2164-emulator-mem/chip0_A.mem"),
        .MEM_B_FILE("fpga/upstream/rhd2164-emulator-mem/chip0_B.mem")
    ) chip0 (.clk(clk400), .rst_n(rst_n), .cs(spi_cs_n), .sclk(spi_sclk),
             .mosi(spi_mosi), .miso(miso0));

    rhd2164_emulator #(
        .MEM_A_FILE("fpga/upstream/rhd2164-emulator-mem/chip1_A.mem"),
        .MEM_B_FILE("fpga/upstream/rhd2164-emulator-mem/chip1_B.mem")
    ) chip1 (.clk(clk400), .rst_n(rst_n), .cs(spi_cs_n), .sclk(spi_sclk),
             .mosi(spi_mosi), .miso(miso1));

    // ---------------- QSPI monitor: the ESP32-S3's view of the bus ----------------
    integer   dg_consumed = 0;
    integer   mon_base   = 0;
    // Off across a reset, so a transaction cut short by the reset is not
    // mistaken for a datagram.
    reg       mon_en     = 1'b0;

    qspi_hd_monitor #(.EXP_BYTES(DATA_BYTES)) mon (
        .en(mon_en), .cs_n(qspi_cs_n), .sclk(qspi_sclk), .dq(qspi_dq_out)
    );

    task automatic wait_dgram(output integer ok);
        integer guard;
        begin
            guard = 0;
            ok    = 0;
            while (guard < 400000) begin
                if (mon.dg_seen != dg_consumed) begin
                    dg_consumed = mon.dg_seen;
                    ok = 1;
                    guard = 400000;
                end else begin
                    @(posedge clk);
                    guard = guard + 1;
                end
            end
            if (!ok) $display("  FAIL  no datagram arrived within 400000 clocks");
        end
    endtask

    function automatic [15:0] g16(input integer off);
        g16 = {mon.dgcopy[off+1], mon.dgcopy[off]};
    endfunction

    function automatic [31:0] g32(input integer off);
        g32 = {mon.dgcopy[off+3], mon.dgcopy[off+2], mon.dgcopy[off+1], mon.dgcopy[off]};
    endfunction

    // ---------------- UART, on the real wires ----------------
    task automatic uart_put(input [7:0] b);
        integer i;
        begin
            uart_rx_i = 1'b0; #(BITP);
            for (i = 0; i < 8; i = i + 1) begin
                uart_rx_i = b[i]; #(BITP);      // LSB first
            end
            uart_rx_i = 1'b1; #(BITP);
        end
    endtask

    task automatic uart_puts(input string s);
        integer i;
        for (i = 0; i < s.len(); i = i + 1) uart_put(s[i]);
    endtask

    task automatic uart_get(output [7:0] b);
        integer i;
        begin
            @(negedge uart_tx_o);
            #(BITP * 1.5);
            for (i = 0; i < 8; i = i + 1) begin
                b[i] = uart_tx_o;
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

    // The reply can start before the command's last stop bit is finished, so
    // the receiver has to be armed while the command is still going out.
    string reply;

    task automatic cmd(input string c);
        begin
            fork
                uart_puts(c);
                uart_getline(reply);
            join
        end
    endtask

    task automatic cmd_ok(input string c, input [8*40-1:0] name);
        begin
            cmd(c);
            if (reply != "ok") begin
                errors = errors + 1;
                $display("  FAIL  %0s -- board replied [%0s]", name, reply);
            end
        end
    endtask

    // Indexing a module-scope string crashes Icarus 12, so the reply is always
    // passed in as an argument and indexed there.
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

    // "R aa" replies "aa=vvvvvvvv"; the three leading characters are dropped.
    task automatic reg_rd(input string addr, output [31:0] v);
        begin
            cmd({"R ", addr, "\n"});
            v = hex32(reply, 3);
        end
    endtask

    // ---------------- golden data ----------------
    reg [15:0] gchk [0:GOLD_SAMPLES*N_CH-1];
    reg [15:0] grea [0:GOLD_SAMPLES*N_CH-1];
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

    // ---------------- per-frame checks ----------------
    integer hdr_bad = 0, torn = 0, ttx_bad = 0, seq_bad = 0, pay_bad = 0;
    integer exp_seq = 0, have_seq = 0;

    reg [31:0] f_seq, f_tcap, f_ttx;

    task automatic check_header(input integer f, input [7:0] exp_flags);
        integer o;
        begin
            o = f * FRAME_BYTES;
            if (g16(o) !== 16'hA55A) begin
                torn    = torn + 1;
                hdr_bad = hdr_bad + 1;
            end
            if (mon.dgcopy[o+2] !== 8'd0)      hdr_bad = hdr_bad + 1;  // RAW16
            if (mon.dgcopy[o+3] !== exp_flags) hdr_bad = hdr_bad + 1;
            if (g16(o+16)   !== 16'd128)     hdr_bad = hdr_bad + 1;  // n_ch
            if (g16(o+18)   !== 16'd256)     hdr_bad = hdr_bad + 1;  // payload_len

            f_seq  = g32(o + 4);
            f_tcap = g32(o + 8);
            f_ttx  = g32(o + 12);

            if (!have_seq) begin
                exp_seq  = f_seq;
                have_seq = 1;
            end
            if (f_seq !== exp_seq[31:0]) begin
                seq_bad = seq_bad + 1;
                if (seq_bad <= 4)
                    $display("    seq gap: got %0d want %0d", f_seq, exp_seq);
            end
            exp_seq = f_seq + 1;

            if (f_ttx == 32'd0)  ttx_bad = ttx_bad + 1;
            if (f_ttx <  f_tcap) ttx_bad = ttx_bad + 1;
        end
    endtask

    task automatic check_pattern_payload(input integer f, input integer realistic);
        integer o, c, si;
        reg [15:0] got, want;
        begin
            o  = f * FRAME_BYTES;
            si = f_seq;                       // seq == pattern sample index
            if (si < GOLD_SAMPLES) begin
                for (c = 0; c < N_CH; c = c + 1) begin
                    got  = g16(o + HDR_BYTES + 2*c);
                    want = realistic ? grea[si*N_CH + c] : gchk[si*N_CH + c];
                    if (got !== want) begin
                        pay_bad = pay_bad + 1;
                        if (pay_bad <= 5)
                            $display("    sample %0d ch %0d: fpga %04h host %04h",
                                     si, c, got, want);
                    end
                end
            end
        end
    endtask

    // Same technique as tb_spi_master: find the offset channel 0 agrees with,
    // then demand that all 128 channels agree with that one offset.
    task automatic spi_frame_offset(input integer f, output integer off,
                                    output integer agree);
        integer o, c, p, a, best;
        reg [15:0] got;
        begin
            o    = f * FRAME_BYTES;
            off  = -1;
            best = 0;
            for (p = 0; p < MEM_SAMPLES; p = p + 1) begin
                if (g16(o + HDR_BYTES) === m0a[p*32]) begin
                    a = 0;
                    for (c = 0; c < N_CH; c = c + 1) begin
                        got = g16(o + HDR_BYTES + 2*c);
                        if (got === mem_at(c, p)) a = a + 1;
                    end
                    if (a > best) begin best = a; off = p; end
                end
            end
            agree = best;
        end
    endtask

    // ---------------- helpers ----------------
    task automatic hard_reset;
        begin
            mon_en = 1'b0;
            @(negedge clk); rst_n = 1'b0;
            repeat (20) @(negedge clk);
            rst_n = 1'b1;
            repeat (20) @(negedge clk);
            dg_consumed = mon.dg_seen;
            mon_base    = mon.dg_seen;
            mon_en      = 1'b1;
            have_seq = 0;
            hdr_bad = 0; ttx_bad = 0; seq_bad = 0; pay_bad = 0;
        end
    endtask

    integer d, f, ok, off, agree, first_off, off_bad, agree_bad, base_dg, ndg;
    reg [31:0] v;

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("fpga/sim/tb_rx_top.vcd");
            $dumpvars(0, tb_rx_top);
        end
        $readmemh("fpga/sim/vectors/pattern_check.mem", gchk);
        $readmemh("fpga/sim/vectors/pattern_realistic.mem", grea);
        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip0_A.mem", m0a);
        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip0_B.mem", m0b);
        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip1_A.mem", m1a);
        $readmemh("fpga/upstream/rhd2164-emulator-mem/chip1_B.mem", m1b);
        chk(gchk[0] === 16'h0001, "host golden vectors loaded");
        chk(m0a[0] !== 16'hxxxx,  "emulator memories loaded");

        // =================================================================
        $display("\n=== (a) internal pattern, CHECK mode, over QSPI ===");
        hard_reset();
        reg_rd("1f", v);
        chk(v === 32'h1C10_0001, "ID register reads back over the UART");

        cmd_ok("W 01 00000bb8\n", "set SAMPLE_PERIOD = 3000");
        cmd_ok("W 03 00000004\n", "set MODE = internal pattern, CHECK");
        cmd_ok("W 00 00000001\n", "set CTRL run");

        base_dg = mon.dg_seen;
        for (d = 0; d < 2; d = d + 1) begin
            wait_dgram(ok);
            if (!ok) errors = errors + 1;
            for (f = 0; f < FPD; f = f + 1) begin
                check_header(f, 8'h00);
                check_pattern_payload(f, 0);
            end
        end
        cmd_ok("W 00 00000000\n", "clear CTRL run");

        chk(mon.wr_cmd   === 8'h23, "QSPI write is WRDMA 0x23 (cmd+addr 1 line, data 4)");
        chk(mon.wr_addr  === 8'h00, "QSPI address byte is 0x00");
        chk(mon.done_cmd === 8'h27, "every segment is closed by WR_DONE 0x27");
        chk(mon.missing_done == 0 && mon.orphan_done == 0,
            "exactly one WR_DONE per WRDMA segment");
        chk(mon.proto_err == 0,     "no malformed transaction on the QSPI bus");
        chk(mon.bad_len == 0,       "every segment is exactly 5 x 276 bytes");
        chk(mon.min_gap_ns >= 10000.0, "CS idles >= 10 us after WR_DONE (ESP re-arm)");
        chk(hdr_bad == 0,        "magic, type, flags, n_ch, payload_len correct");
        chk(seq_bad == 0,        "seq increments by one with no gaps");
        chk(ttx_bad == 0,        "t_tx nonzero and never earlier than t_cap");
        chk(pay_bad == 0,        "payload matches the host's CHECK vectors");
        chk(mon.dg_seen - base_dg >= 2, "datagrams keep coming");

        repeat (20000) @(posedge clk);          // let the pipe drain
        reg_rd("11", v);
        ndg = mon.dg_seen - mon_base;
        chk(v === ndg[31:0], "DGRAMS register agrees with the pin monitor");
        reg_rd("13", v);
        chk(v === 32'd0, "OVERRUNS zero when the rate is inside the link");
        reg_rd("14", v);
        chk(v === 32'd0, "STALLS zero while esp_ready is held high");
        reg_rd("12", v);
        chk(v >= 32'd1 && v <= 32'd4, "FIFO_HIGH inside 1..NSLOT");

        // =================================================================
        $display("\n=== (a2) internal pattern, REALISTIC mode ===");
        hard_reset();
        cmd_ok("W 01 00000bb8\n", "set SAMPLE_PERIOD = 3000");
        cmd_ok("W 03 00000005\n", "set MODE = internal pattern, REALISTIC");
        cmd_ok("W 00 00000001\n", "set CTRL run");

        wait_dgram(ok);
        if (!ok) errors = errors + 1;
        for (f = 0; f < FPD; f = f + 1) begin
            check_header(f, 8'h01);             // flags bit0 follows mode bit0
            check_pattern_payload(f, 1);
        end
        cmd_ok("W 00 00000000\n", "clear CTRL run");

        chk(hdr_bad == 0, "REALISTIC flag appears in the frame header");
        chk(seq_bad == 0, "seq still gapless in REALISTIC mode");
        chk(pay_bad == 0, "payload matches the host's REALISTIC vectors");

        // =================================================================
        $display("\n=== (b) real SPI against two rhd2164 emulator cores ===");
        hard_reset();
        cmd_ok("W 02 00000002\n", "set PHASE_SELECT = 2");
        cmd_ok("W 01 00000bb8\n", "set SAMPLE_PERIOD = 3000");
        cmd_ok("W 03 00000000\n", "set MODE = external SPI");
        cmd_ok("W 00 00000001\n", "set CTRL run");

        off_bad   = 0;
        agree_bad = 0;
        first_off = -1;
        for (d = 0; d < 3; d = d + 1) begin
            wait_dgram(ok);
            if (!ok) errors = errors + 1;
            for (f = 0; f < FPD; f = f + 1) begin
                check_header(f, 8'h00);
                // The first datagram after a mode change can still hold frames
                // built before it, so its payload is not comparable.
                if (d > 0) begin
                    spi_frame_offset(f, off, agree);
                    if (agree != N_CH) begin
                        agree_bad = agree_bad + 1;
                        if (agree_bad <= 3)
                            $display("    dgram %0d frame %0d: only %0d/128 agree",
                                     d, f, agree);
                    end else if (first_off < 0) begin
                        first_off = off;
                    end else if (off != ((first_off + (d-1)*FPD + f) % MEM_SAMPLES)) begin
                        off_bad = off_bad + 1;
                        if (off_bad <= 3)
                            $display("    dgram %0d frame %0d: offset %0d, expected %0d",
                                     d, f, off,
                                     (first_off + (d-1)*FPD + f) % MEM_SAMPLES);
                    end
                end
            end
        end
        cmd_ok("W 00 00000000\n", "clear CTRL run");

        chk(hdr_bad   == 0, "SPI-mode headers correct");
        chk(seq_bad   == 0, "SPI-mode seq gapless");
        chk(ttx_bad   == 0, "SPI-mode t_tx nonzero and >= t_cap");
        chk(agree_bad == 0, "every SPI frame matches all 128 emulator channels");
        chk(off_bad   == 0, "the playback pointer advances one sample per frame");

        // =================================================================
        // A sweep takes 2800 clocks and a datagram slot drains in about 2440 per
        // frame, so at a 2000-clock period the source is the only thing that
        // cannot keep up: every other start is refused. That isolates the
        // missed-start counter from a FIFO overflow, which would also raise
        // OVERRUNS but would show up as a gap in seq and a full FIFO.
        $display("\n=== (d) sample period shorter than one SPI sweep ===");
        hard_reset();
        cmd_ok("W 02 00000002\n", "set PHASE_SELECT = 2");
        cmd_ok("W 01 000007d0\n", "set SAMPLE_PERIOD = 2000");
        cmd_ok("W 03 00000002\n", "set MODE = external SPI, codec flag on");
        cmd_ok("W 00 00000001\n", "set CTRL run");

        wait_dgram(ok);
        if (!ok) errors = errors + 1;
        for (f = 0; f < FPD; f = f + 1) check_header(f, 8'h02);
        cmd_ok("W 00 00000000\n", "clear CTRL run");

        chk(hdr_bad == 0, "mode bit1 appears as flags bit1 (codec enabled)");
        chk(seq_bad == 0, "no frame lost, so nothing was dropped by the FIFO");
        reg_rd("13", v);
        chk(v > 32'd0, "OVERRUNS counts starts refused while the source is busy");
        reg_rd("12", v);
        chk(v < 32'd4, "FIFO_HIGH below full, so the buffer was not the limit");

        // =================================================================
        $display("\n=== (c) esp_ready held low: stalls, then overruns ===");
        hard_reset();
        cmd_ok("W 01 0000012c\n", "set SAMPLE_PERIOD = 300");
        cmd_ok("W 03 00000004\n", "set MODE = internal pattern, CHECK");
        @(negedge clk); esp_ready = 1'b0;
        cmd_ok("W 00 00000001\n", "set CTRL run");

        repeat (40000) @(posedge clk);
        reg_rd("14", v);
        chk(v > 32'd0, "STALLS climbs while esp_ready is low");
        reg_rd("12", v);
        chk(v === 32'd4, "FIFO_HIGH reaches all four slots");
        reg_rd("13", v);
        chk(v > 32'd0, "OVERRUNS climbs once the FIFO is full");
        reg_rd("11", v);
        chk(v === 32'd0, "no datagram was sent while esp_ready was low");

        // Stop the source BEFORE letting the backlog out, so exactly the four
        // buffered datagrams are in flight and nothing new is being built.
        cmd_ok("W 00 00000000\n", "clear CTRL run");
        repeat (2000) @(posedge clk);
        base_dg = mon.dg_seen;
        hdr_bad = 0; torn = 0;
        @(negedge clk); esp_ready = 1'b1;
        for (d = 0; d < 4; d = d + 1) begin
            wait_dgram(ok);
            if (!ok) errors = errors + 1;
            for (f = 0; f < FPD; f = f + 1) begin
                if (g16(f * FRAME_BYTES) !== 16'hA55A) torn = torn + 1;
                if (g16(f * FRAME_BYTES + 16) !== 16'd128) hdr_bad = hdr_bad + 1;
            end
        end
        chk(mon.dg_seen - base_dg == 4, "the four buffered datagrams all come out");
        chk(torn == 0,    "no datagram is torn: magic at all five frame offsets");
        chk(hdr_bad == 0, "buffered datagrams still carry n_ch = 128");
        chk(mon.bad_len == 0,   "every segment in the whole run was 1380 bytes");
        chk(mon.proto_err == 0, "no malformed QSPI transaction in the whole run");

        // Counters must be clearable, or a sweep cannot start from a clean slate.
        repeat (20000) @(posedge clk);          // drain before measuring
        cmd_ok("W 00 00000002\n", "write CTRL clear_counters");
        repeat (50) @(posedge clk);
        reg_rd("10", v); chk(v === 32'd0, "clear zeroes FRAMES");
        reg_rd("11", v); chk(v === 32'd0, "clear zeroes DGRAMS");
        reg_rd("12", v); chk(v === 32'd0, "clear zeroes FIFO_HIGH");
        reg_rd("13", v); chk(v === 32'd0, "clear zeroes OVERRUNS");
        reg_rd("14", v); chk(v === 32'd0, "clear zeroes STALLS");

        chk(qspi_dq_oe === 1'b1, "dq_oe asserted: the link is master-driven");

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
