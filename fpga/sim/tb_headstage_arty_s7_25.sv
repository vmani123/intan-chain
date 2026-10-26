// ============================================================================
// tb_headstage_arty_s7_25.sv -- the headstage bitstream, from the pins out.
// ----------------------------------------------------------------------------
// Runs headstage_arty_s7_25_top itself -- MMCM, resets, pad flops, the
// on-die emulator -- with unisim_stubs.sv for the vendor primitives. The
// "real" chips are two more emulator cores on their own 400 MHz clock, behind
// a modelled converter board that adds CONV_NS each way on Pmod JA. All
// control goes over the UART, as it will at the bench.
//
//  (a) ID and CAPS: this is the headstage build and it has the loopback.
//  (b) self-test: MODE bit3 on, a diagnostic list of ROM reads answers
//      "INTAN" / 53 / 58 / 64 / 4 from the on-die emulator at some phase,
//      and Pmod JA stays idle the whole time (the real chips see nothing).
//  (c) the real bus: MODE bit3 off, the same list now goes out on JA and the
//      chips behind the converter answer it; the phase window moves by the
//      converter delay.
//  (d) streaming at the middle of that window: the link check counts sweeps
//      with no errors on any lane, LD5 lights, MISO_STAT saw both levels.
//  (e) a MISO wire off (pulled low, as the XDC's PULLDOWN does): LINK_ERR1
//      climbs, chip 0 stays clean, LD5 goes dark, MISO_STAT never sees miso1
//      high -- the signature the bring-up script looks for.
//
// Phases are scored from the raw taps with the decode formula in CONTRACTS.md
// section 6, so one capture covers all sixteen, exactly as the host does it.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_headstage_arty_s7_25;

    localparam int  QSPI_DIV = 6;
    localparam int  BAUD     = 6_000_000;                   // divisor 16 at 96 MHz
    localparam real BITP     = 1.0e9 / BAUD;
    localparam real CONV_NS  = 8.0;                         // converter + cable, each way
    localparam int  NLIST    = 12;                          // 10 ROM reads + 2 flushers

    integer errors = 0;
    integer checks = 0;

    task automatic chk(input cond, input [8*72-1:0] name);
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
    reg  esp_ready   = 1'b0;                                // no ESP: as at first light

    wire spi_cs_n, spi_sclk, spi_mosi, spi_miso0, spi_miso1;
    wire qspi_cs_n, qspi_sclk;
    wire [3:0] qspi_dq;
    wire uart_rxd_out;
    wire [3:0] led;

    headstage_arty_s7_25_top #(
        .QSPI_DIV(QSPI_DIV), .BAUD(BAUD),
        .MEM_C0A("fpga/upstream/rhd2164-emulator-mem/chip0_A.mem"),
        .MEM_C0B("fpga/upstream/rhd2164-emulator-mem/chip0_B.mem"),
        .MEM_C1A("fpga/upstream/rhd2164-emulator-mem/chip1_A.mem"),
        .MEM_C1B("fpga/upstream/rhd2164-emulator-mem/chip1_B.mem")
    ) dut (
        .clk_100mhz(clk_100mhz), .rst_btn(rst_btn),
        .spi_cs_n(spi_cs_n), .spi_sclk(spi_sclk), .spi_mosi(spi_mosi),
        .spi_miso0(spi_miso0), .spi_miso1(spi_miso1),
        .qspi_cs_n(qspi_cs_n), .qspi_sclk(qspi_sclk), .qspi_dq(qspi_dq),
        .esp_ready(esp_ready),
        .uart_txd_in(uart_txd_in), .uart_rxd_out(uart_rxd_out),
        .led(led)
    );

    // ---------------- the converter board and the two chips ----------------
    reg chip_clk = 1'b0;  always #1.25 chip_clk = ~chip_clk;   // 400 MHz
    reg chip_rst_n = 1'b0;
    wire c_cs_n, c_sclk, c_mosi, c_miso0, c_miso1;

    assign #(CONV_NS) c_cs_n = spi_cs_n;
    assign #(CONV_NS) c_sclk = spi_sclk;
    assign #(CONV_NS) c_mosi = spi_mosi;

    rhd2164_emulator #(
        .MEM_A_FILE("fpga/upstream/rhd2164-emulator-mem/chip0_A.mem"),
        .MEM_B_FILE("fpga/upstream/rhd2164-emulator-mem/chip0_B.mem")
    ) chip0 (.clk(chip_clk), .rst_n(chip_rst_n),
             .cs(c_cs_n), .sclk(c_sclk), .mosi(c_mosi), .miso(c_miso0));

    rhd2164_emulator #(
        .MEM_A_FILE("fpga/upstream/rhd2164-emulator-mem/chip1_A.mem"),
        .MEM_B_FILE("fpga/upstream/rhd2164-emulator-mem/chip1_B.mem")
    ) chip1 (.clk(chip_clk), .rst_n(chip_rst_n),
             .cs(c_cs_n), .sclk(c_sclk), .mosi(c_mosi), .miso(c_miso1));

    reg miso1_off = 1'b0;                                   // (e): the wire comes off
    assign #(CONV_NS) spi_miso0 = c_miso0;
    assign #(CONV_NS) spi_miso1 = miso1_off ? 1'b0 : c_miso1;

    // Every chip-select the real chips see.
    integer ja_cs = 0;
    always @(negedge spi_cs_n) ja_cs = ja_cs + 1;

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

    task automatic wr(input [7:0] a, input [31:0] v);
        string s;
        begin
            $sformat(s, "W %02x %08x\n", a, v);
            cmd_ok(s);
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

    // ---------------- the ROM list, played and scored ----------------
    // Command i's result arrives in cycle i + 2 (CONTRACTS.md section 6).
    reg [15:0] list    [0:NLIST-1];
    reg [7:0]  want_a  [0:9];
    reg [7:0]  want_b  [0:9];
    reg [77:0] raw     [0:1][0:NLIST-1];                    // [chip][cycle]

    task automatic play_list;
        integer i, guard;
        reg [31:0] v, r0, r1, r2;
        begin
            wr(8'h20, 0);
            for (i = 0; i < NLIST; i = i + 1) wr(8'h21, {16'd0, list[i]});
            wr(8'h22, NLIST);
            wr(8'h23, 1);
            v = 32'h1;
            for (guard = 0; guard < 50 && v[0]; guard = guard + 1) rd(8'h23, v);
            chk(v[1:0] == 2'b10 && !v[2], "diagnostic list ran to completion");
            for (i = 0; i < NLIST; i = i + 1) begin
                wr(8'h20, i);
                rd(8'h26, r0); rd(8'h27, r1); rd(8'h28, r2);
                raw[0][i] = {r2[13:0], r1, r0};
                rd(8'h29, r0); rd(8'h2a, r1); rd(8'h2b, r2);
                raw[1][i] = {r2[13:0], r1, r0};
            end
        end
    endtask

    function automatic [15:0] dec(input [77:0] t, input integer p, input integer b_half);
        integer j;
        begin
            dec = 16'd0;
            for (j = 0; j < 16; j = j + 1)
                dec[15-j] = t[p + (b_half ? 2 : 0) + 4*j];
        end
    endfunction

    // Phases at which every ROM word on both chips and both modules is right.
    task automatic score(output reg [15:0] good);
        integer p, i, c;
        reg ok;
        begin
            good = 16'd0;
            for (p = 0; p < 16; p = p + 1) begin
                ok = 1'b1;
                for (c = 0; c < 2; c = c + 1)
                    for (i = 0; i < 10; i = i + 1) begin
                        if (dec(raw[c][i+2], p, 0) !== {8'h00, want_a[i]}) ok = 1'b0;
                        if (dec(raw[c][i+2], p, 1) !== {8'h00, want_b[i]}) ok = 1'b0;
                    end
                good[p] = ok;
            end
        end
    endtask

    function automatic integer middle(input [15:0] good);
        integer p, lo, len, best_lo, best_len;
        begin
            best_lo = -1; best_len = 0; len = 0; lo = 0;
            for (p = 0; p < 16; p = p + 1) begin
                if (good[p]) begin
                    if (len == 0) lo = p;
                    len = len + 1;
                    if (len > best_len) begin best_len = len; best_lo = lo; end
                end else len = 0;
            end
            middle = (best_len > 0) ? best_lo + (best_len - 1) / 2 : -1;
        end
    endfunction

    task automatic show(input string what, input [15:0] good);
        integer p;
        string row;
        begin
            row = "";
            for (p = 0; p < 16; p = p + 1) row = {row, good[p] ? "#" : "."};
            $display("    %0s phases 0..15: %0s", what, row);
        end
    endtask

    // ---------------- the run ----------------
    integer i, ph_loop, ph_ext, cs_before;
    reg [15:0] good;
    reg [31:0] v, v2, chk0, chk1;

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("fpga/sim/tb_headstage_arty_s7_25.vcd");
            $dumpvars(0, tb_headstage_arty_s7_25);
        end

        for (i = 0; i < 5; i = i + 1) list[i] = {2'b11, 6'(40 + i), 8'h00};
        for (i = 0; i < 5; i = i + 1) list[5 + i] = {2'b11, 6'(59 + i), 8'h00};
        list[10] = 16'hFF00;                                // READ(63) flushers
        list[11] = 16'hFF00;
        want_a[0] = 8'h49; want_a[1] = 8'h4E; want_a[2] = 8'h54;
        want_a[3] = 8'h41; want_a[4] = 8'h4E; want_a[5] = 8'h35;
        want_a[6] = 8'h01; want_a[7] = 8'h01; want_a[8] = 8'h40; want_a[9] = 8'h04;
        for (i = 0; i < 10; i = i + 1) want_b[i] = want_a[i];
        want_b[5] = 8'h3A;

        #200 rst_btn = 1'b0;
        chip_rst_n = 1'b1;
        #3000;

        // =================================================================
        $display("\n=== (a) which bitstream ===");
        rd(8'h1f, v);
        chk(v === 32'h1C10_0002, "ID 1c100002 over the board's UART pins");
        rd(8'h1e, v);
        chk(v[8] === 1'b1, "CAPS: on-die emulator present");
        chk(v[23:16] == 8'd78 && v[7:0] == QSPI_DIV, "CAPS: 78 taps, QSPI_DIV reaches rx_top");

        // =================================================================
        $display("\n=== (b) self-test through the on-die emulator ===");
        wr(8'h03, 32'h8);
        rd(8'h1b, v);
        chk(v[8] === 1'b1, "MISO_STAT bit8: loopback active");
        cs_before = ja_cs;
        play_list();
        chk(ja_cs == cs_before, "Pmod JA stays idle while looped back");
        score(good);
        show("loopback", good);
        ph_loop = middle(good);
        chk(ph_loop >= 0, "the on-die emulator answers the ROM list at some phase");

        // FPGA decode agrees with the formula at the phase it ran with.
        wr(8'h02, ph_loop);
        play_list();
        wr(8'h20, 7);                                       // READ(59)'s result
        rd(8'h24, v);  rd(8'h25, v2);
        chk(v == 32'h003A_0035 && v2 == 32'h003A_0035,
            "DIAG_RES0/1 at the chosen phase: 0035 / 003a on both chips");

        // =================================================================
        $display("\n=== (c) the real bus through the converter (%0.1f ns each way) ===", CONV_NS);
        wr(8'h03, 32'h0);
        cs_before = ja_cs;
        play_list();
        chk(ja_cs - cs_before == NLIST, "one chip select on JA per list entry");
        score(good);
        show("external", good);
        ph_ext = middle(good);
        chk(ph_ext >= 0, "the chips behind the converter answer the ROM list");
        chk(ph_ext >= ph_loop, "the window sits no earlier than the loopback's");

        // =================================================================
        $display("\n=== (d) streaming at PHASE_SELECT %0d, no ESP ===", ph_ext);
        wr(8'h02, ph_ext);
        wr(8'h01, 32'd9600);                                // 10 kS/s
        wr(8'h00, 32'h2);                                   // clear counters
        wr(8'h00, 32'h1);
        #2_000_000;
        rd(8'h16, chk0);
        chk(chk0 > 32'd10, "LINK_CHECKS climbs with the sweeps");
        rd(8'h17, v);  rd(8'h18, v2);
        chk(v == 0 && v2 == 0, "LINK_ERR0 and LINK_ERR1 stay zero");
        rd(8'h19, v);
        chk(v == 32'h003A_0035, "LINK_LAST0 = 0035 / 003a");
        chk(led[3] === 1'b1 && led[0] === 1'b1, "LD5 (link good) and LD2 (run) lit");
        rd(8'h1b, v);
        chk(v[3:0] == 4'hF, "MISO_STAT: both lines seen low and high");
        rd(8'h11, v);
        chk(v == 0, "DGRAMS zero with no ESP (esp_ready low)");

        // =================================================================
        $display("\n=== (e) chip 1's MISO wire comes off ===");
        miso1_off = 1'b1;
        wr(8'h00, 32'h3);                                   // clear, keep running
        #2_000_000;
        chk(led[3] === 1'b0, "LD5 dark while running");
        wr(8'h00, 32'h0);                                   // stop, so the counts hold still
        #100_000;
        rd(8'h16, chk1);
        rd(8'h17, v);  rd(8'h18, v2);
        chk(chk1 > 32'd10 && v == 0, "chip 0 still clean");
        chk(v2[15:0] == chk1[15:0] && v2[31:16] == chk1[15:0],
            "LINK_ERR1 counts every sweep on both modules");
        rd(8'h1a, v);
        chk(v == 32'h0, "LINK_LAST1 reads 0000 / 0000: a line held low");
        rd(8'h1b, v);
        chk(v[2] === 1'b1 && v[3] === 1'b0, "MISO_STAT: miso1 seen low, never high");

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
