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
module ctrl_regs (
    input  wire         clk,
    input  wire         rst,            // synchronous, active high

    input  wire  [7:0]  rx_data,        // from uart_rx
    input  wire         rx_valid,
    output logic [7:0]  tx_data,        // to uart_tx
    output logic        tx_send,
    input  wire         tx_ready,

    output logic [31:0] ctrl,           // 0x00
    output logic [31:0] sample_period,  // 0x01
    output logic [31:0] phase_select,   // 0x02
    output logic [31:0] mode,           // 0x03
    output logic        clear_counters_stb,

    input  wire  [31:0] frames,         // 0x10
    input  wire  [31:0] dgrams,         // 0x11
    input  wire  [31:0] fifo_high,      // 0x12
    input  wire  [31:0] overruns,       // 0x13
    input  wire  [31:0] stalls          // 0x14
);

    localparam int LINE_MAX  = 16;      // longest legal line is 13 bytes
    localparam int PHASE_MAX = 11;      // the MISO phase selector has 12 taps

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

    logic [31:0] rdata;
    logic        raddr_ok;

    always_comb begin
        raddr_ok = 1'b1;
        case (addr)
            8'h00:   rdata = ctrl;      // bit1 is never stored, so never reads back
            8'h01:   rdata = sample_period;
            8'h02:   rdata = phase_select;
            8'h03:   rdata = mode;
            8'h10:   rdata = frames;
            8'h11:   rdata = dgrams;
            8'h12:   rdata = fifo_high;
            8'h13:   rdata = overruns;
            8'h14:   rdata = stalls;
            8'h1F:   rdata = 32'h1C10_0001;
            default: begin rdata = 32'd0; raddr_ok = 1'b0; end
        endcase
    end

    // Read-only (>= 0x10) and unknown addresses both fail the same test.
    wire waddr_ok = (addr <= 8'h03);
    wire eol      = rx_valid && (rx_data == 8'h0A);

    logic [7:0] resp [0:11];            // "aa=vvvvvvvv\n" is the longest reply
    logic [3:0] rlen;
    logic [3:0] tidx;

    localparam logic [1:0] T_IDLE = 2'd0, T_SEND = 2'd1, T_WAIT = 2'd2;
    logic [1:0] tstate;

    always_ff @(posedge clk) begin
        clear_counters_stb <= 1'b0;
        tx_send            <= 1'b0;
        if (rst) begin
            ctrl          <= 32'd0;
            sample_period <= 32'd48000;   // 2 kS/s at 96 MHz
            phase_select  <= 32'd0;
            mode          <= 32'd0;
            lcnt          <= 5'd0;
            ltoolong      <= 1'b0;
            tstate        <= T_IDLE;
            tidx          <= 4'd0;
            rlen          <= 4'd0;
        end else if (tstate == T_IDLE) begin
            if (eol) begin
                lcnt     <= 5'd0;
                ltoolong <= 1'b0;
                tidx     <= 4'd0;
                tstate   <= T_SEND;
                if (!ltoolong && fmt_r && raddr_ok) begin
                    resp[0] <= hex_chr(addr[7:4]);
                    resp[1] <= hex_chr(addr[3:0]);
                    resp[2] <= "=";
                    for (j = 0; j < 8; j = j + 1)
                        resp[3+j] <= hex_chr(rdata[31-4*j -: 4]);
                    resp[11] <= 8'h0A;
                    rlen     <= 4'd12;
                end else if (!ltoolong && fmt_w && waddr_ok) begin
                    case (addr)
                        8'h00: begin
                            ctrl               <= wval & ~32'd2;
                            clear_counters_stb <= wval[1];
                        end
                        8'h01: sample_period <= wval;
                        // Saturate: a larger tap index would read out of range.
                        8'h02: phase_select  <= (wval > PHASE_MAX) ? PHASE_MAX : wval;
                        8'h03: mode          <= wval;
                        default: ;
                    endcase
                    resp[0] <= "o"; resp[1] <= "k"; resp[2] <= 8'h0A;
                    rlen    <= 4'd3;
                end else begin
                    resp[0] <= "e"; resp[1] <= "r"; resp[2] <= "r";
                    resp[3] <= 8'h0A;
                    rlen    <= 4'd4;
                end
            end else if (rx_valid && rx_data != 8'h0D) begin  // drop CR of CRLF
                if (lcnt < LINE_MAX) begin
                    lbuf[lpos +: 8] <= rx_data;
                    lcnt            <= lcnt + 1'b1;
                end else begin
                    ltoolong <= 1'b1;
                end
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
