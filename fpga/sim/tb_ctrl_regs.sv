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
    output wire  [31:0] ctrl,
    output wire  [31:0] sample_period,
    output wire  [31:0] phase_select,
    output wire  [31:0] mode,
    output wire         clear_counters_stb
);
    wire [7:0] rx_data, tx_data;
    wire       rx_valid, tx_send, tx_ready;

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_rx (
        .clk(clk), .rst(rst), .rx(rx), .data(rx_data), .valid(rx_valid));

    ctrl_regs u_regs (
        .clk(clk), .rst(rst),
        .rx_data(rx_data), .rx_valid(rx_valid),
        .tx_data(tx_data), .tx_send(tx_send), .tx_ready(tx_ready),
        .ctrl(ctrl), .sample_period(sample_period),
        .phase_select(phase_select), .mode(mode),
        .clear_counters_stb(clear_counters_stb),
        .frames(frames), .dgrams(dgrams), .fifo_high(fifo_high),
        .overruns(overruns), .stalls(stalls));

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

    wire [31:0] ctrl0, sper0, phase0, mode0;
    wire        ccs0;

    ctrl_harness #(.CLK_HZ(FCLK), .BAUD(FBAUD)) h0 (
        .clk(clk0), .rst(rst0), .rx(rx0), .tx(tx0),
        .frames(frames), .dgrams(dgrams), .fifo_high(fifo_h),
        .overruns(overruns), .stalls(stalls),
        .ctrl(ctrl0), .sample_period(sper0), .phase_select(phase0),
        .mode(mode0), .clear_counters_stb(ccs0));

    ctrl_harness #(.CLK_HZ(RCLK), .BAUD(RBAUD)) h1 (
        .clk(clk1), .rst(rst1), .rx(rx1), .tx(tx1),
        .frames(32'd0), .dgrams(32'd0), .fifo_high(32'd0),
        .overruns(32'd0), .stalls(32'd0),
        .ctrl(), .sample_period(), .phase_select(), .mode(),
        .clear_counters_stb());

    // Counts every clk0 cycle in which the strobe is asserted, so a pulse that
    // is two cycles long is caught as loudly as one that never fires.
    int stb_cycles = 0;
    always @(posedge clk0) if (!rst0 && ccs0) stb_cycles = stb_cycles + 1;

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
        cmd_chk(1, "R 1f\n", "1f=1c100001", "ID at real 96MHz/115200");

        tx_skew = 1.02;
        cmd_chk(1, "R 1f\n", "1f=1c100001", "real ratio, host baud +2%");
        tx_skew = 0.98;
        cmd_chk(1, "R 01\n", "01=0000bb80", "real ratio, host baud -2%");
        tx_skew = 1.0;

        cmd_chk(0, "R 1f\n", "1f=1c100001", "ID read");

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

        // Saturation: only 12 MISO phase taps exist.
        cmd_chk(0, "W 02 000000ff\n", "ok", "write PHASE_SELECT = 0xff");
        cmd_chk(0, "R 02\n", "02=0000000b", "PHASE_SELECT saturates at 11");
        chk(phase0 == 32'd11, "phase_select port saturates at 11");

        // Rejections.
        cmd_chk(0, "W 10 00000001\n", "err", "write to read-only FRAMES");
        cmd_chk(0, "W 1f 00000001\n", "err", "write to read-only ID");
        cmd_chk(0, "R 20\n", "err", "read unknown address 20");
        cmd_chk(0, "R 04\n", "err", "read unknown address 04");
        cmd_chk(0, "W 04 00000001\n", "err", "write unknown address 04");
        cmd_chk(0, "hello\n", "err", "garbage line");
        cmd_chk(0, "R\n", "err", "truncated read command");
        cmd_chk(0, "W 01 12\n", "err", "short write value");
        cmd_chk(0, "R zz\n", "err", "non-hex address");
        cmd_chk(0, "\n", "err", "empty line");
        cmd_chk(0, "W 01 000000000000000000000\n", "err", "over-long line");
        cmd_chk(0, "R 01\n", "01=0000c350", "still alive after over-long line");

        // Terminal habits: CRLF and the shift key.
        cmd_chk(0, {"R 1f", cr, "\n"}, "1f=1c100001", "CRLF line ending");
        cmd_chk(0, {"W 03 00000002", cr, "\n"}, "ok", "CRLF on a write");
        cmd_chk(0, "R 1F\n", "1f=1c100001", "uppercase address");
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
