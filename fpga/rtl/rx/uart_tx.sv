`default_nettype none
//
// 8N1 UART transmitter, no parity, no flow control.
//
// Handshake: assert send for one cycle while ready is high. busy stays high
// from the cycle after send until the stop bit has been held for a full bit
// time, so a caller that waits for ready cannot truncate the stop bit.
//
module uart_tx #(
    parameter int CLK_HZ = 96_000_000,
    parameter int BAUD   = 115200
) (
    input  wire        clk,
    input  wire        rst,        // synchronous, active high
    input  wire  [7:0] data,
    input  wire        send,       // accepted only while ready
    output logic       busy,
    output wire        ready,
    output logic       tx
);

    localparam int DIV   = CLK_HZ / BAUD;
    localparam int CNT_W = (DIV <= 2) ? 1 : $clog2(DIV);

    logic [CNT_W-1:0] cnt;
    logic [3:0]       bit_idx;
    logic [9:0]       shreg;      // {stop, data[7:0], start}

    assign ready = ~busy;

    always_ff @(posedge clk) begin
        if (rst) begin
            busy    <= 1'b0;
            tx      <= 1'b1;       // idle high
            cnt     <= '0;
            bit_idx <= '0;
        end else if (!busy) begin
            tx  <= 1'b1;
            cnt <= '0;
            if (send) begin
                shreg   <= {1'b1, data, 1'b0};
                bit_idx <= '0;
                busy    <= 1'b1;
                tx      <= 1'b0;   // start bit goes out immediately
            end
        end else begin
            cnt <= cnt + 1'b1;
            if (cnt == CNT_W'(DIV - 1)) begin
                cnt     <= '0;
                shreg   <= {1'b1, shreg[9:1]};
                tx      <= shreg[1];
                bit_idx <= bit_idx + 1'b1;
                if (bit_idx == 4'd9) begin
                    busy <= 1'b0;  // stop bit has had its full bit time
                    tx   <= 1'b1;
                end
            end
        end
    end

endmodule
`default_nettype wire
