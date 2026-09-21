`default_nettype none
//
// 8N1 UART receiver, no parity, no flow control.
//
// The bit timer is restarted from the falling edge of every start bit rather
// than free-running off the first edge of the stream. Over a long cable the
// host and board clocks differ by enough that a free-running divider walks off
// the bit centres after a few hundred characters; the symptom is occasional
// wrong bytes, which looks like line noise and is very hard to chase. Per-byte
// resynchronisation bounds the accumulated error to one character.
//
module uart_rx #(
    parameter int CLK_HZ = 96_000_000,
    parameter int BAUD   = 115200
) (
    input  wire        clk,
    input  wire        rst,        // synchronous, active high
    input  wire        rx,         // asynchronous serial input
    output logic [7:0] data,
    output logic       valid       // one clk pulse per accepted byte
);

    localparam int DIV   = CLK_HZ / BAUD;   // clk ticks per bit
    localparam int HALF  = DIV / 2;         // start-bit centre
    localparam int CNT_W = (DIV <= 2) ? 1 : $clog2(DIV);

    localparam logic [1:0] S_IDLE = 2'd0, S_START = 2'd1,
                           S_DATA = 2'd2, S_STOP  = 2'd3;

    logic [1:0]       sync;   // 2FF synchroniser: rx crosses into clk domain
    logic [1:0]       state;
    logic [CNT_W-1:0] cnt;
    logic [2:0]       bit_idx;
    logic [7:0]       shreg;

    wire rx_s   = sync[1];
    wire tick   = (cnt == CNT_W'(DIV - 1));
    wire tick_h = (cnt == CNT_W'(HALF - 1));

    always_ff @(posedge clk) begin
        sync  <= {sync[0], rx};
        valid <= 1'b0;
        cnt   <= cnt + 1'b1;
        if (rst) begin
            sync  <= 2'b11;         // idle line is high; do not fake a start bit
            state <= S_IDLE;
            cnt   <= '0;
        end else begin
            case (state)
                S_IDLE: begin
                    cnt <= '0;
                    if (!rx_s) state <= S_START;
                end
                S_START: if (tick_h) begin
                    cnt   <= '0;
                    // Still low at the centre, so it was a start bit and not a
                    // glitch; anything shorter is rejected here.
                    state <= rx_s ? S_IDLE : S_DATA;
                    bit_idx <= 3'd0;
                end
                S_DATA: if (tick) begin
                    cnt     <= '0;
                    shreg   <= {rx_s, shreg[7:1]};  // LSB first on the wire
                    bit_idx <= bit_idx + 1'b1;
                    if (bit_idx == 3'd7) state <= S_STOP;
                end
                S_STOP: if (tick) begin
                    cnt   <= '0;
                    state <= S_IDLE;
                    // Framing error (stop bit low) drops the byte rather than
                    // handing the parser a value it cannot tell is corrupt.
                    if (rx_s) begin
                        data  <= shreg;
                        valid <= 1'b1;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
`default_nettype wire
