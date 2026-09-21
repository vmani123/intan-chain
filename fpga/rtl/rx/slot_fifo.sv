// ============================================================================
// slot_fifo.sv -- which datagram slots hold a finished datagram.
// ----------------------------------------------------------------------------
// The datagrams themselves live in dgram_mem; this is only the bookkeeping, so
// it is a pair of pointers and a count. Its whole job is to keep the builder
// and the shifter out of the same slot, and to measure how close the buffer
// came to overflowing -- high_water is what tells the bench whether four slots
// is the right number.
//
// The release port cannot be called `release`: that is a Verilog keyword.
// ============================================================================

`default_nettype none

module slot_fifo #(
    parameter int NSLOT = 4,
    parameter int SW    = $clog2(NSLOT)
) (
    input  wire            clk,
    input  wire            rst_n,
    input  wire            clear,          // zero the counters, not the state

    // write side (frame_builder)
    output wire [SW-1:0]   wr_slot,
    output wire            full,
    input  wire            commit,

    // read side (qspi_master)
    output wire [SW-1:0]   rd_slot,
    output wire            empty,
    input  wire            rd_release,

    output reg  [31:0]     high_water,
    output reg  [31:0]     overruns
);

    reg  [SW-1:0] wptr, rptr;
    reg  [SW:0]   count;                   // 0..NSLOT, so one bit wider

    assign wr_slot = wptr;
    assign rd_slot = rptr;
    assign full    = (count == (SW+1)'(NSLOT));
    assign empty   = (count == '0);

    wire do_push = commit     && !full;
    wire do_pop  = rd_release && !empty;

    reg [SW:0] cnt_next;
    always @* begin
        cnt_next = count;
        if (do_push) cnt_next = cnt_next + 1'b1;
        if (do_pop)  cnt_next = cnt_next - 1'b1;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            wptr       <= '0;
            rptr       <= '0;
            count      <= '0;
            high_water <= 32'd0;
            overruns   <= 32'd0;
        end else begin
            if (do_push) wptr <= wptr + 1'b1;
            if (do_pop)  rptr <= rptr + 1'b1;
            count <= cnt_next;

            // A commit that arrives with no free slot must never happen: the
            // builder watches `full` and drops the datagram before it starts.
            // Counted anyway, because if it ever does happen the alternative is
            // a silently overwritten slot.
            if (commit && full) overruns <= overruns + 32'd1;

            if (32'(cnt_next) > high_water) high_water <= 32'(cnt_next);

            if (clear) begin
                high_water <= 32'd0;
                overruns   <= 32'd0;
            end
        end
    end

endmodule

`default_nettype wire
