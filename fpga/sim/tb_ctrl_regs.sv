`timescale 1ns/1ps
//
// Self-checking testbench for the UART control interface.
//
// Everything goes through the real serial wires: each command is bit-banged
// onto rx at the true bit period and each reply is sampled off tx the same way.
// Poking ctrl_regs' ports directly would pass even if the bit timing were
// wrong, which is the failure this whole block exists to avoid.
//
// Most checks run on a harness with a small CLK/BAUD ratio to keep the run
// short; one check runs at the real 96 MHz / 115200 (divisor 833) so the
// divisor arithmetic itself is exercised.
//
// The diagnostic command list lives outside ctrl_regs (diag_mem), so the
// harness carries a 64-entry array wired the same way: written on
// diag_cmd_we at diag_ptr, read back at diag_ptr. That is enough to prove the
// write lands at the pointer it was aimed at and the pointer steps after it.
//
module ctrl_harness #(
    parameter int CLK_HZ = 96_000_000,
    parameter int BAUD   = 115200
) (
    input  wire         clk,
    input  wire         rst,
    input  wire         rx,
    output wire         tx,
    input  wire  [31:0] frames,
    input  wire  [31:0] dgrams,
    input  wire  [31:0] fifo_high,
    input  wire  [31:0] overruns,
    input  wire  [31:0] stalls,
    input  wire  [31:0] base,           // read-only 0x15..0x1E read base + addr
    input  wire  [2:0]  diag_status,
    input  wire  [63:0] diag_res,
    input  wire  [77:0] diag_raw0,
    input  wire  [77:0] diag_raw1,
    output wire  [31:0] ctrl,
    output wire  [31:0] sample_period,
    output wire  [31:0] phase_select,
    output wire  [31:0] mode,
    output wire  [31:0] spi_div,
    output wire  [5:0]  diag_ptr,
    output wire  [6:0]  diag_len,
    output wire         diag_cmd_we,
    output wire         diag_go_stb,
    output wire         clear_counters_stb
);
    wire [7:0] rx_data, tx_data;
    wire       rx_valid, tx_send, tx_ready;
    wire [15:0] diag_cmd_wdata;

    reg  [15:0] cmd_mem [0:63];
    reg  [15:0] cmd_rd = 16'd0;
    integer k;
    initial for (k = 0; k < 64; k = k + 1) cmd_mem[k] = 16'd0;
    always @(posedge clk) begin
        if (diag_cmd_we) cmd_mem[diag_ptr] <= diag_cmd_wdata;
        cmd_rd <= cmd_mem[diag_ptr];
    end

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_rx (
        .clk(clk), .rst(rst), .rx(rx), .data(rx_data), .valid(rx_valid));

    ctrl_regs u_regs (
        .clk(clk), .rst(rst),
        .rx_data(rx_data), .rx_valid(rx_valid),
        .tx_data(tx_data), .tx_send(tx_send), .tx_ready(tx_ready),
        .ctrl(ctrl), .sample_period(sample_period),
        .phase_select(phase_select), .mode(mode), .spi_div(spi_div),
        .clear_counters_stb(clear_counters_stb),
        .diag_ptr(diag_ptr), .diag_len(diag_len),
        .diag_cmd_we(diag_cmd_we), .diag_cmd_wdata(diag_cmd_wdata),
        .diag_go_stb(diag_go_stb),
        .frames(frames), .dgrams(dgrams), .fifo_high(fifo_high),
        .overruns(overruns), .stalls(stalls),
        .spi_cycles(base + 32'h15), .link_checks(base + 32'h16),
        .link_err0(base + 32'h17), .link_err1(base + 32'h18),
        .link_last0(base + 32'h19), .link_last1(base + 32'h1A),
        .miso_stat(base + 32'h1B), .miso_edges0(base + 32'h1C),
        .miso_edges1(base + 32'h1D), .caps(base + 32'h1E),
        .diag_status(diag_status), .diag_cmd_rd(cmd_rd),
        .diag_res0(diag_res[31:0]), .diag_res1(diag_res[63:32]),
        .diag_raw0(diag_raw0), .diag_raw1(diag_raw1));

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_tx (
        .clk(clk), .rst(rst), .data(tx_data), .send(tx_send),
        .busy(), .ready(tx_ready), .tx(tx));
endmodule


module tb_ctrl_regs;

    // sel 0: short ratio, used for the bulk of the checks.
    // sel 1: the real numbers from CONTRACTS.md section 5.
    localparam int  FCLK = 40_000_000, FBAUD = 5_000_000;
    localparam int  RCLK = 96_000_000, RBAUD = 115_200;
    localparam real FHALF = 1.0e9 / (2.0 * FCLK);
    localparam real RHALF = 1.0e9 / (2.0 * RCLK);
    localparam real FBIT  = 1.0e9 / FBAUD;
    localparam real RBIT  = 1.0e9 / RBAUD;

    logic clk0 = 1'b0, clk1 = 1'b0;
    logic rst0 = 1'b1, rst1 = 1'b1;
    logic rx0  = 1'b1, rx1  = 1'b1;
    wire  tx0, tx1;

    always #(FHALF) clk0 = ~clk0;
    always #(RHALF) clk1 = ~clk1;

    logic [31:0] frames = 32'h0000_1234, dgrams   = 32'h0000_5678;
    logic [31:0] fifo_h = 32'h0000_0009, overruns = 32'h0000_000a;
    logic [31:0] stalls = 32'hfeed_face;

    logic [31:0] base   = 32'h5000_0000;
    logic [2:0]  dstat  = 3'b101;
    logic [63:0] dres   = 64'h1111_2222_3333_4444;
    // Top bits all set, so the zero padding above tap 77 is visible.
    logic [77:0] draw0  = {14'h3fff, 32'h89ab_cdef, 32'h0123_4567};
    logic [77:0] draw1  = {14'h2aaa, 32'h7654_3210, 32'hfedc_ba98};

    wire [31:0] ctrl0, sper0, phase0, mode0, sdiv0;
    wire [5:0]  dptr0;
    wire [6:0]  dlen0;
    wire        ccs0, dwe0, dgo0;

    ctrl_harness #(.CLK_HZ(FCLK), .BAUD(FBAUD)) h0 (
        .clk(clk0), .rst(rst0), .rx(rx0), .tx(tx0),
        .frames(frames), .dgrams(dgrams), .fifo_high(fifo_h),
        .overruns(overruns), .stalls(stalls),
        .base(base), .diag_status(dstat), .diag_res(dres),
        .diag_raw0(draw0), .diag_raw1(draw1),
        .ctrl(ctrl0), .sample_period(sper0), .phase_select(phase0),
        .mode(mode0), .spi_div(sdiv0), .diag_ptr(dptr0), .diag_len(dlen0),
        .diag_cmd_we(dwe0), .diag_go_stb(dgo0), .clear_counters_stb(ccs0));

    ctrl_harness #(.CLK_HZ(RCLK), .BAUD(RBAUD)) h1 (
        .clk(clk1), .rst(rst1), .rx(rx1), .tx(tx1),
        .frames(32'd0), .dgrams(32'd0), .fifo_high(32'd0),
        .overruns(32'd0), .stalls(32'd0),
        .base(32'd0), .diag_status(3'd0), .diag_res(64'd0),
        .diag_raw0(78'd0), .diag_raw1(78'd0),
        .ctrl(), .sample_period(), .phase_select(), .mode(), .spi_div(),
        .diag_ptr(), .diag_len(), .diag_cmd_we(), .diag_go_stb(),
        .clear_counters_stb());

    // Counts every clk0 cycle in which a strobe is asserted, so a pulse that
    // is two cycles long is caught as loudly as one that never fires.
    int stb_cycles = 0, go_cycles = 0, we_cycles = 0;
    always @(posedge clk0) if (!rst0 && ccs0) stb_cycles = stb_cycles + 1;
    always @(posedge clk0) if (!rst0 && dgo0) go_cycles  = go_cycles + 1;
    always @(posedge clk0) if (!rst0 && dwe0) we_cycles  = we_cycles + 1;

    int errors = 0;

    // Deliberate host-side baud error, applied to the bytes the testbench
    // sends. A receiver that resynchronises per byte survives this; one that
    // free-runs off a single edge does not.
    real tx_skew = 1.0;

    task automatic chk(input logic ok, input string name);
        if (ok) $display("pass: %0s", name);
        else begin
            $display("FAIL: %0s", name);
            errors = errors + 1;
        end
    endtask

    task automatic uart_put(input int sel, input logic [7:0] b);
        real bp;
        int  i;
        bp = ((sel == 0) ? FBIT : RBIT) * tx_skew;
        if (sel == 0) rx0 = 1'b0; else rx1 = 1'b0;      // start
        #(bp);
        for (i = 0; i < 8; i = i + 1) begin
            if (sel == 0) rx0 = b[i]; else rx1 = b[i];  // LSB first
            #(bp);
        end
        if (sel == 0) rx0 = 1'b1; else rx1 = 1'b1;      // stop
        #(bp);
    endtask

    task automatic uart_puts(input int sel, input string s);
        int i;
        for (i = 0; i < s.len(); i = i + 1) uart_put(sel, s[i]);
    endtask

    task automatic uart_get(input int sel, output logic [7:0] b);
        real bp;
        int  i;
        bp = (sel == 0) ? FBIT : RBIT;
        if (sel == 0) @(negedge tx0); else @(negedge tx1);
        #(bp * 1.5);                                    // centre of bit 0
        for (i = 0; i < 8; i = i + 1) begin
            if (sel == 0) b[i] = tx0; else b[i] = tx1;
            #(bp);
        end
    endtask

    task automatic uart_getline(input int sel, output string s);
        logic [7:0] b;
        logic       done;
        s = "";
        done = 1'b0;
        while (!done) begin
            uart_get(sel, b);
            if (b == 8'h0A) done = 1'b1;
            else s = {s, string'(b)};
        end
    endtask

    // The reply can start before the last stop bit of the command has finished,
    // so the receiver has to be armed while the command is still going out.
    task automatic cmd_chk(input int sel, input string cmd, input string want,
                           input string name);
        string got;
        fork
            uart_puts(sel, cmd);
            uart_getline(sel, got);
        join
        if (got == want) $display("pass: %0s", name);
        else begin
            $display("FAIL: %0s -- sent [%0s] want [%0s] got [%0s]",
                     name, cmd, want, got);
            errors = errors + 1;
        end
    endtask

    int    stb_before;
    string cr;

    initial begin
        cr = string'(8'h0d);            // Icarus does not decode the \r escape

        repeat (8) @(posedge clk0);
        rst0 = 1'b0;
        repeat (8) @(posedge clk1);
        rst1 = 1'b0;
        repeat (8) @(posedge clk0);

        // Real 96 MHz / 115200 ratio: proves the divisor arithmetic.
        cmd_chk(1, "R 1f\n", "1f=1c100002", "ID at real 96MHz/115200");

        tx_skew = 1.02;
        cmd_chk(1, "R 1f\n", "1f=1c100002", "real ratio, host baud +2%");
        tx_skew = 0.98;
        cmd_chk(1, "R 01\n", "01=0000bb80", "real ratio, host baud -2%");
        tx_skew = 1.0;

        cmd_chk(0, "R 1f\n", "1f=1c100002", "ID read");

        // Reset defaults.
        cmd_chk(0, "R 00\n", "00=00000000", "CTRL reset value");
        cmd_chk(0, "R 01\n", "01=0000bb80", "SAMPLE_PERIOD reset = 48000");
        cmd_chk(0, "R 02\n", "02=00000000", "PHASE_SELECT reset = 0");
        cmd_chk(0, "R 03\n", "03=00000000", "MODE reset = 0");

        // Write then read back.
        cmd_chk(0, "W 01 0000c350\n", "ok", "write SAMPLE_PERIOD");
        cmd_chk(0, "R 01\n", "01=0000c350", "read back SAMPLE_PERIOD");
        chk(sper0 == 32'h0000c350, "sample_period port follows write");

        cmd_chk(0, "W 02 00000007\n", "ok", "write PHASE_SELECT");
        cmd_chk(0, "R 02\n", "02=00000007", "read back PHASE_SELECT");

        cmd_chk(0, "W 03 00000005\n", "ok", "write MODE");
        cmd_chk(0, "R 03\n", "03=00000005", "read back MODE");
        chk(mode0 == 32'h00000005, "mode port follows write");

        // Saturation: the 78-tap window covers phases 0..15.
        cmd_chk(0, "W 02 0000000f\n", "ok", "write PHASE_SELECT = 15");
        cmd_chk(0, "R 02\n", "02=0000000f", "PHASE_SELECT 15 is in range");
        cmd_chk(0, "W 02 000000ff\n", "ok", "write PHASE_SELECT = 0xff");
        cmd_chk(0, "R 02\n", "02=0000000f", "PHASE_SELECT saturates at 15");
        chk(phase0 == 32'd15, "phase_select port saturates at 15");
        cmd_chk(0, "W 02 80000000\n", "ok", "write PHASE_SELECT = 0x80000000");
        cmd_chk(0, "R 02\n", "02=0000000f", "PHASE_SELECT compares all 32 bits");

        // SPI_DIV: 0..15, saturating.
        cmd_chk(0, "R 04\n", "04=00000000", "SPI_DIV reset = 0");
        cmd_chk(0, "W 04 00000003\n", "ok", "write SPI_DIV = 3");
        cmd_chk(0, "R 04\n", "04=00000003", "read back SPI_DIV");
        chk(sdiv0 == 32'd3, "spi_div port follows write");
        cmd_chk(0, "W 04 00000010\n", "ok", "write SPI_DIV = 16");
        cmd_chk(0, "R 04\n", "04=0000000f", "SPI_DIV saturates at 15");
        cmd_chk(0, "W 04 00000000\n", "ok", "write SPI_DIV = 0");

        // Diagnostic list: pointer, write-and-step, length, go.
        cmd_chk(0, "R 20\n", "20=00000000", "DIAG_PTR reset = 0");
        cmd_chk(0, "R 22\n", "22=00000001", "DIAG_LEN reset = 1");
        cmd_chk(0, "W 21 deadc000\n", "ok", "write DIAG_CMD entry 0");
        cmd_chk(0, "R 20\n", "20=00000001", "DIAG_CMD write steps DIAG_PTR");
        cmd_chk(0, "W 21 0000fb00\n", "ok", "write DIAG_CMD entry 1");
        cmd_chk(0, "W 21 0000ff00\n", "ok", "write DIAG_CMD entry 2");
        chk(we_cycles == 3, "diag_cmd_we one clock per DIAG_CMD write");
        chk(h0.cmd_mem[0] == 16'hc000 && h0.cmd_mem[1] == 16'hfb00 &&
            h0.cmd_mem[2] == 16'hff00 && h0.cmd_mem[3] == 16'h0000,
            "each entry lands at the pointer before the step");
        cmd_chk(0, "R 21\n", "21=00000000", "DIAG_CMD read at entry 3, no step");
        cmd_chk(0, "R 20\n", "20=00000003", "DIAG_CMD read leaves DIAG_PTR alone");
        cmd_chk(0, "W 20 00000001\n", "ok", "write DIAG_PTR = 1");
        cmd_chk(0, "R 21\n", "21=0000fb00", "DIAG_CMD reads entry 1");
        cmd_chk(0, "W 20 00000000\n", "ok", "write DIAG_PTR = 0");
        cmd_chk(0, "R 21\n", "21=0000c000", "DIAG_CMD keeps only bits 15:0");
        cmd_chk(0, "W 20 00000100\n", "ok", "write DIAG_PTR = 0x100");
        cmd_chk(0, "R 20\n", "20=0000003f", "DIAG_PTR saturates at 63");
        cmd_chk(0, "W 21 00001234\n", "ok", "write DIAG_CMD entry 63");
        cmd_chk(0, "R 20\n", "20=00000000", "DIAG_PTR steps from 63 to 0");
        chk(h0.cmd_mem[63] == 16'h1234 && h0.cmd_mem[0] == 16'hc000,
            "entry 63 written, entry 0 untouched");
        cmd_chk(0, "W 22 00000000\n", "ok", "write DIAG_LEN = 0");
        cmd_chk(0, "R 22\n", "22=00000001", "DIAG_LEN 0 reads back as 1");
        chk(dlen0 == 7'd1, "diag_len port is 1 after a 0 write");
        cmd_chk(0, "W 22 00000040\n", "ok", "write DIAG_LEN = 64");
        cmd_chk(0, "R 22\n", "22=00000040", "DIAG_LEN 64 is in range");
        cmd_chk(0, "W 22 ffffffff\n", "ok", "write DIAG_LEN = 0xffffffff");
        cmd_chk(0, "R 22\n", "22=00000040", "DIAG_LEN saturates at 64");
        cmd_chk(0, "W 22 0000000c\n", "ok", "write DIAG_LEN = 12");
        chk(dlen0 == 7'd12, "diag_len port follows write");
        chk(go_cycles == 0, "diag_go_stb idle before DIAG_CTRL write");
        cmd_chk(0, "W 23 00000000\n", "ok", "write DIAG_CTRL = 0");
        chk(go_cycles == 0, "DIAG_CTRL bit0 clear: no go");
        cmd_chk(0, "W 23 00000001\n", "ok", "write DIAG_CTRL go");
        chk(go_cycles == 1, "diag_go_stb pulsed one cycle");
        cmd_chk(0, "R 23\n", "23=00000005", "DIAG_CTRL reads the status inputs");

        // Read-only diagnostics come straight from the input ports.
        cmd_chk(0, "R 15\n", "15=50000015", "SPI_CYCLES");
        cmd_chk(0, "R 16\n", "16=50000016", "LINK_CHECKS");
        cmd_chk(0, "R 17\n", "17=50000017", "LINK_ERR0");
        cmd_chk(0, "R 18\n", "18=50000018", "LINK_ERR1");
        cmd_chk(0, "R 19\n", "19=50000019", "LINK_LAST0");
        cmd_chk(0, "R 1a\n", "1a=5000001a", "LINK_LAST1");
        cmd_chk(0, "R 1b\n", "1b=5000001b", "MISO_STAT");
        cmd_chk(0, "R 1c\n", "1c=5000001c", "MISO_EDGES0");
        cmd_chk(0, "R 1d\n", "1d=5000001d", "MISO_EDGES1");
        cmd_chk(0, "R 1e\n", "1e=5000001e", "CAPS");
        cmd_chk(0, "R 24\n", "24=33334444", "DIAG_RES0 = res[31:0]");
        cmd_chk(0, "R 25\n", "25=11112222", "DIAG_RES1 = res[63:32]");
        cmd_chk(0, "R 26\n", "26=01234567", "DIAG_RAW0_0 = taps 0..31");
        cmd_chk(0, "R 27\n", "27=89abcdef", "DIAG_RAW0_1 = taps 32..63");
        cmd_chk(0, "R 28\n", "28=00003fff", "DIAG_RAW0_2 = taps 64..77, zero above");
        cmd_chk(0, "R 29\n", "29=fedcba98", "DIAG_RAW1_0");
        cmd_chk(0, "R 2a\n", "2a=76543210", "DIAG_RAW1_1");
        cmd_chk(0, "R 2B\n", "2b=00002aaa", "DIAG_RAW1_2");

        // Rejections.
        cmd_chk(0, "W 10 00000001\n", "err", "write to read-only FRAMES");
        cmd_chk(0, "W 15 00000001\n", "err", "write to read-only SPI_CYCLES");
        cmd_chk(0, "W 1e 00000001\n", "err", "write to read-only CAPS");
        cmd_chk(0, "W 1f 00000001\n", "err", "write to read-only ID");
        cmd_chk(0, "W 24 00000001\n", "err", "write to read-only DIAG_RES0");
        cmd_chk(0, "W 2b 00000001\n", "err", "write to read-only DIAG_RAW1_2");
        chk(we_cycles == 4 && go_cycles == 1, "rejected writes strobe nothing");
        cmd_chk(0, "R 05\n", "err", "read unknown address 05");
        cmd_chk(0, "W 05 00000001\n", "err", "write unknown address 05");
        cmd_chk(0, "R 0f\n", "err", "read unknown address 0f");
        cmd_chk(0, "R 2c\n", "err", "read unknown address 2c");
        cmd_chk(0, "W 2c 00000001\n", "err", "write unknown address 2c");
        cmd_chk(0, "R ff\n", "err", "read unknown address ff");
        cmd_chk(0, "hello\n", "err", "garbage line");
        cmd_chk(0, "R\n", "err", "truncated read command");
        cmd_chk(0, "W 01 12\n", "err", "short write value");
        cmd_chk(0, "R zz\n", "err", "non-hex address");
        cmd_chk(0, "\n", "err", "empty line");
        cmd_chk(0, "W 01 000000000000000000000\n", "err", "over-long line");
        cmd_chk(0, "R 01\n", "01=0000c350", "still alive after over-long line");

        // Terminal habits: CRLF and the shift key.
        cmd_chk(0, {"R 1f", cr, "\n"}, "1f=1c100002", "CRLF line ending");
        cmd_chk(0, {"W 03 00000002", cr, "\n"}, "ok", "CRLF on a write");
        cmd_chk(0, "R 1F\n", "1f=1c100002", "uppercase address");
        cmd_chk(0, "W 01 DEADBEEF\n", "ok", "uppercase hex write");
        cmd_chk(0, "R 01\n", "01=deadbeef", "uppercase write read back lowercase");

        // Read-only counters come straight from the input ports.
        cmd_chk(0, "R 10\n", "10=00001234", "FRAMES");
        cmd_chk(0, "R 11\n", "11=00005678", "DGRAMS");
        cmd_chk(0, "R 12\n", "12=00000009", "FIFO_HIGH");
        cmd_chk(0, "R 13\n", "13=0000000a", "OVERRUNS");
        cmd_chk(0, "R 14\n", "14=feedface", "STALLS");

        // CTRL bit1 is a self-clearing strobe.
        chk(stb_cycles == 0, "clear_counters_stb idle before CTRL write");
        stb_before = stb_cycles;
        cmd_chk(0, "W 00 00000003\n", "ok", "write CTRL run + clear_counters");
        chk(stb_cycles - stb_before == 1, "clear_counters_stb pulsed one cycle");
        chk(ctrl0 == 32'h00000001, "ctrl port holds run, drops clear_counters");
        cmd_chk(0, "R 00\n", "00=00000001", "CTRL reads back without bit1");
        chk(stb_cycles == 1, "clear_counters_stb did not fire again");

        if (errors == 0) begin
            $display("ALL CHECKS PASSED");
            $finish;
        end else begin
            $display("%0d CHECK(S) FAILED", errors);
            $fatal(1, "tb_ctrl_regs failed");
        end
    end

    initial begin
        #30_000_000;                    // 30 ms; a full run takes about 3 ms
        $display("FAIL: timeout");
        $fatal(1, "tb_ctrl_regs timed out");
    end

endmodule
