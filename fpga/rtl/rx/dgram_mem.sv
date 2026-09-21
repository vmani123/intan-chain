// ============================================================================
// dgram_mem.sv -- the datagram slot store: NSLOT x SLOT_WORDS 16-bit words.
// ----------------------------------------------------------------------------
// Simple dual port, one writer (frame_builder) and one reader (qspi_master),
// written as a plain reg array with a registered read so that it both simulates
// in Icarus and infers a block RAM in Vivado. No vendor primitive, because a
// primitive here would mean the whole receiver could no longer be simulated.
//
// Read data appears one clock after rd_en, and holds until the next read.
// ============================================================================

`default_nettype none

module dgram_mem #(
    parameter int NSLOT      = 4,
    parameter int SLOT_WORDS = 690,
    parameter int DW         = 16,
    parameter int AW         = $clog2(NSLOT * SLOT_WORDS)
) (
    input  wire            clk,

    input  wire            wr_en,
    input  wire [AW-1:0]   wr_addr,
    input  wire [DW-1:0]   wr_data,

    input  wire            rd_en,
    input  wire [AW-1:0]   rd_addr,
    output reg  [DW-1:0]   rd_data
);

    localparam int DEPTH = NSLOT * SLOT_WORDS;

    reg [DW-1:0] mem [0:DEPTH-1];

    // Write-first or read-first does not matter: the FIFO guarantees the writer
    // and the reader are never in the same slot, so the two ports never collide.
    always @(posedge clk) begin
        if (wr_en) mem[wr_addr] <= wr_data;
        if (rd_en) rd_data      <= mem[rd_addr];
    end

endmodule

`default_nettype wire
