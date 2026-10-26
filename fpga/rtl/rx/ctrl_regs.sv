`default_nettype none
//
// Text command parser and control register file (CONTRACTS.md section 4).
//
//   "R aa\n"          -> "aa=vvvvvvvv\n"
//   "W aa vvvvvvvv\n" -> "ok\n"
//   anything else     -> "err\n"
//
// Text rather than binary so the board can be driven by hand from a terminal
// before any host tool exists. That is also why input hex is case insensitive
// and a bare CR is dropped: terminals send CRLF and people use the shift key.
// Output is always lowercase, so the host side can compare strings exactly.
//
// A finished line is executed one clock after its line feed: the parsed
// address and value are registered first, and the read multiplexer -- some
// thirty 32-bit sources, most of them diagnostic -- works from those registers
// rather than from the hex decoders. One clock is invisible next to a UART
// character, and it keeps the decode and the mux out of one timing path.
//
// Writable registers are 00-04 and 20-23. Writes anywhere else, read-only or
// unknown, answer "err", as do reads of unknown addresses.
//
module ctrl_regs #(
    parameter int CAP_TAPS = 78         // raw taps per CS cycle, at most 96
) (
    input  wire         clk,
    input  wire         rst,            // synchronous, active high

    input  wire  [7:0]  rx_data,        // from uart_rx
    input  wire         rx_valid,
    output logic [7:0]  tx_data,        // to uart_tx
    output logic        tx_send,
    input  wire         tx_ready,

    output logic [31:0] ctrl,           // 0x00
    output logic [31:0] sample_period,  // 0x01
    output logic [31:0] phase_select,   // 0x02, 0..15
    output logic [31:0] mode,           // 0x03
    output logic [31:0] spi_div,        // 0x04, 0..15
    output logic        clear_counters_stb,

    // Diagnostic list (0x20-0x23). The command memory itself lives outside;
    // a DIAG_CMD write pulses diag_cmd_we for one clock with diag_ptr still
    // pointing at the entry, and the pointer steps on the following clock.
    output logic [5:0]  diag_ptr,       // 0x20, 0..63
    output logic [6:0]  diag_len,       // 0x22, 1..64
    output logic        diag_cmd_we,
    output logic [15:0] diag_cmd_wdata,
    output logic        diag_go_stb,    // 0x23 bit0, self-clearing

    input  wire  [31:0] frames,         // 0x10
    input  wire  [31:0] dgrams,         // 0x11
    input  wire  [31:0] fifo_high,      // 0x12
    input  wire  [31:0] overruns,       // 0x13
    input  wire  [31:0] stalls,         // 0x14
    input  wire  [31:0] spi_cycles,     // 0x15
    input  wire  [31:0] link_checks,    // 0x16
    input  wire  [31:0] link_err0,      // 0x17
    input  wire  [31:0] link_err1,      // 0x18
    input  wire  [31:0] link_last0,     // 0x19
    input  wire  [31:0] link_last1,     // 0x1A
    input  wire  [31:0] miso_stat,      // 0x1B
    input  wire  [31:0] miso_edges0,    // 0x1C
    input  wire  [31:0] miso_edges1,    // 0x1D
    input  wire  [31:0] caps,           // 0x1E

    input  wire  [2:0]  diag_status,    // 0x23 read: {rejected, done, busy}
    input  wire  [15:0] diag_cmd_rd,    // 0x21 read: list entry at diag_ptr
    input  wire  [31:0] diag_res0,      // 0x24
    input  wire  [31:0] diag_res1,      // 0x25
    input  wire  [CAP_TAPS-1:0] diag_raw0,  // 0x26-0x28
    input  wire  [CAP_TAPS-1:0] diag_raw1   // 0x29-0x2B
);

    localparam int LINE_MAX   = 16;     // longest legal line is 13 bytes
    localparam int PHASE_MAX  = 15;     // 78 capture taps cover phases 0..15
    localparam int SPIDIV_MAX = 15;
    localparam int DIAG_MAX   = 64;     // command list entries
    localparam logic [31:0] ID = 32'h1C10_0002;

    function automatic logic is_hex(input logic [7:0] c);
        is_hex = (c >= "0" && c <= "9") || (c >= "a" && c <= "f")
                                        || (c >= "A" && c <= "F");
    endfunction
    // "| 8'h20" folds 'A'-'F' onto 'a'-'f', so input case does not matter.
    function automatic logic [3:0] hex_val(input logic [7:0] c);
        hex_val = (c <= "9") ? (c - "0") : ((c | 8'h20) - 8'h57);
    endfunction
    // Byte k of the line buffer. Taken as an argument rather than read from the
    // module scope so Icarus builds a whole-vector sensitivity list for it.
    function automatic logic [7:0] bat(input logic [LINE_MAX*8-1:0] b,
                                       input integer k);
        bat = b[8*k +: 8];
    endfunction
    function automatic logic [7:0] hex_chr(input logic [3:0] n);
        hex_chr = (n < 4'd10) ? (8'h30 + n) : (8'h61 + n - 8'd10);
    endfunction

    // Packed rather than an unpacked array of bytes purely so the parser below
    // is a set of plain part selects.
    logic [LINE_MAX*8-1:0] lbuf;
    logic [4:0]            lcnt;
    logic                  ltoolong;    // reply err instead of overflowing
    wire  [6:0]            lpos = {lcnt[3:0], 3'b000};

    logic [7:0]  addr;
    logic [31:0] wval;
    logic        fmt_r, fmt_w;
    integer      i, j;              // one loop variable per process, never shared

    always_comb begin
        addr = {hex_val(bat(lbuf, 2)), hex_val(bat(lbuf, 3))};
        wval = {hex_val(bat(lbuf, 5)),  hex_val(bat(lbuf, 6)),
                hex_val(bat(lbuf, 7)),  hex_val(bat(lbuf, 8)),
                hex_val(bat(lbuf, 9)),  hex_val(bat(lbuf, 10)),
                hex_val(bat(lbuf, 11)), hex_val(bat(lbuf, 12))};
        fmt_r = (lcnt == 5'd4)
                && (bat(lbuf, 0) == "R" || bat(lbuf, 0) == "r")
                && (bat(lbuf, 1) == " ")
                && is_hex(bat(lbuf, 2)) && is_hex(bat(lbuf, 3));
        fmt_w = (lcnt == 5'd13)
                && (bat(lbuf, 0) == "W" || bat(lbuf, 0) == "w")
                && (bat(lbuf, 1) == " ")
                && is_hex(bat(lbuf, 2)) && is_hex(bat(lbuf, 3))
                && (bat(lbuf, 4) == " ");
        for (i = 5; i < 13; i = i + 1)
            if (!is_hex(bat(lbuf, i))) fmt_w = 1'b0;
    end

    // The parsed line, registered on its line feed.
    logic [7:0]  addr_q;
    logic [31:0] wval_q;
    logic        op_r, op_w;

    // The raw windows as 32-bit words, zero-padded above the last tap. Named
    // wires rather than part selects inside the mux, which Icarus would make
    // sensitive to the whole vector.
    wire [95:0] raw0_pad = 96'(diag_raw0);
    wire [95:0] raw1_pad = 96'(diag_raw1);
    wire [31:0] raw0_w0 = raw0_pad[31:0], raw0_w1 = raw0_pad[63:32], raw0_w2 = raw0_pad[95:64];
    wire [31:0] raw1_w0 = raw1_pad[31:0], raw1_w1 = raw1_pad[63:32], raw1_w2 = raw1_pad[95:64];

    logic [31:0] rdata;
    logic        raddr_ok;

    always_comb begin
        raddr_ok = 1'b1;
        case (addr_q)
            8'h00:   rdata = ctrl;      // bit1 is never stored, so never reads back
            8'h01:   rdata = sample_period;
            8'h02:   rdata = phase_select;
            8'h03:   rdata = mode;
            8'h04:   rdata = spi_div;
            8'h10:   rdata = frames;
            8'h11:   rdata = dgrams;
            8'h12:   rdata = fifo_high;
            8'h13:   rdata = overruns;
            8'h14:   rdata = stalls;
            8'h15:   rdata = spi_cycles;
            8'h16:   rdata = link_checks;
            8'h17:   rdata = link_err0;
            8'h18:   rdata = link_err1;
            8'h19:   rdata = link_last0;
            8'h1A:   rdata = link_last1;
            8'h1B:   rdata = miso_stat;
            8'h1C:   rdata = miso_edges0;
            8'h1D:   rdata = miso_edges1;
            8'h1E:   rdata = caps;
            8'h1F:   rdata = ID;
            8'h20:   rdata = {26'd0, diag_ptr};
            8'h21:   rdata = {16'd0, diag_cmd_rd};
            8'h22:   rdata = {25'd0, diag_len};
            8'h23:   rdata = {29'd0, diag_status};
            8'h24:   rdata = diag_res0;
            8'h25:   rdata = diag_res1;
            8'h26:   rdata = raw0_w0;
            8'h27:   rdata = raw0_w1;
            8'h28:   rdata = raw0_w2;
            8'h29:   rdata = raw1_w0;
            8'h2A:   rdata = raw1_w1;
            8'h2B:   rdata = raw1_w2;
            default: begin rdata = 32'd0; raddr_ok = 1'b0; end
        endcase
    end

    // Read-only (0x10-0x1F, 0x24-0x2B) and unknown addresses both fail here.
    wire waddr_ok = (addr_q <= 8'h04) || (addr_q >= 8'h20 && addr_q <= 8'h23);
    wire eol      = rx_valid && (rx_data == 8'h0A);

    logic [7:0] resp [0:11];            // "aa=vvvvvvvv\n" is the longest reply
    logic [3:0] rlen;
    logic [3:0] tidx;

    localparam logic [1:0] T_IDLE = 2'd0, T_EXEC = 2'd1, T_SEND = 2'd2, T_WAIT = 2'd3;
    logic [1:0] tstate;

    always_ff @(posedge clk) begin
        clear_counters_stb <= 1'b0;
        diag_go_stb        <= 1'b0;
        diag_cmd_we        <= 1'b0;
        tx_send            <= 1'b0;
        // The DIAG_CMD write lands at the old pointer on this clock; step it now.
        if (diag_cmd_we) diag_ptr <= diag_ptr + 1'b1;
        if (rst) begin
            ctrl          <= 32'd0;
            sample_period <= 32'd48000;   // 2 kS/s at 96 MHz
            phase_select  <= 32'd0;
            mode          <= 32'd0;
            spi_div       <= 32'd0;
            diag_ptr      <= 6'd0;
            diag_len      <= 7'd1;
            lcnt          <= 5'd0;
            ltoolong      <= 1'b0;
            tstate        <= T_IDLE;
            tidx          <= 4'd0;
            rlen          <= 4'd0;
            op_r          <= 1'b0;
            op_w          <= 1'b0;
        end else if (tstate == T_IDLE) begin
            if (eol) begin
                op_r     <= !ltoolong && fmt_r;
                op_w     <= !ltoolong && fmt_w;
                addr_q   <= addr;
                wval_q   <= wval;
                lcnt     <= 5'd0;
                ltoolong <= 1'b0;
                tstate   <= T_EXEC;
            end else if (rx_valid && rx_data != 8'h0D) begin  // drop CR of CRLF
                if (lcnt < LINE_MAX) begin
                    lbuf[lpos +: 8] <= rx_data;
                    lcnt            <= lcnt + 1'b1;
                end else begin
                    ltoolong <= 1'b1;
                end
            end
        end else if (tstate == T_EXEC) begin
            tidx   <= 4'd0;
            tstate <= T_SEND;
            if (op_r && raddr_ok) begin
                resp[0] <= hex_chr(addr_q[7:4]);
                resp[1] <= hex_chr(addr_q[3:0]);
                resp[2] <= "=";
                for (j = 0; j < 8; j = j + 1)
                    resp[3+j] <= hex_chr(rdata[31-4*j -: 4]);
                resp[11] <= 8'h0A;
                rlen     <= 4'd12;
            end else if (op_w && waddr_ok) begin
                case (addr_q)
                    8'h00: begin
                        ctrl               <= wval_q & ~32'd2;
                        clear_counters_stb <= wval_q[1];
                    end
                    8'h01: sample_period <= wval_q;
                    // Saturate: a larger tap index would read out of range.
                    8'h02: phase_select  <= (wval_q > PHASE_MAX)  ? PHASE_MAX  : wval_q;
                    8'h03: mode          <= wval_q;
                    8'h04: spi_div       <= (wval_q > SPIDIV_MAX) ? SPIDIV_MAX : wval_q;
                    8'h20: diag_ptr      <= (wval_q > DIAG_MAX - 1) ? 6'(DIAG_MAX - 1)
                                                                    : wval_q[5:0];
                    8'h21: begin
                        diag_cmd_we    <= 1'b1;
                        diag_cmd_wdata <= wval_q[15:0];
                    end
                    // 0 would be an empty list; it plays one command instead.
                    8'h22: diag_len      <= (wval_q > DIAG_MAX) ? 7'(DIAG_MAX)
                                          : (wval_q == 0)       ? 7'd1
                                          :                       wval_q[6:0];
                    8'h23: diag_go_stb   <= wval_q[0];
                    default: ;
                endcase
                resp[0] <= "o"; resp[1] <= "k"; resp[2] <= 8'h0A;
                rlen    <= 4'd3;
            end else begin
                resp[0] <= "e"; resp[1] <= "r"; resp[2] <= "r";
                resp[3] <= 8'h0A;
                rlen    <= 4'd4;
            end
        end else if (tstate == T_SEND) begin
            if (tx_ready) begin
                tx_data <= resp[tidx];
                tx_send <= 1'b1;
                tstate  <= T_WAIT;
            end
        end else begin              // T_WAIT: hold until uart_tx has taken it
            if (!tx_ready) begin
                tidx   <= tidx + 1'b1;
                tstate <= (tidx == rlen - 1'b1) ? T_IDLE : T_SEND;
            end
        end
    end

endmodule
`default_nettype wire
