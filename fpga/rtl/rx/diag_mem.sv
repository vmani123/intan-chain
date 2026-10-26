// ============================================================================
// diag_mem.sv -- the diagnostic command list and its results (CONTRACTS.md
// sections 4 and 6).
// ----------------------------------------------------------------------------
//   command list   DEPTH x 16    written and read by the host at DIAG_PTR,
//                                read by spi_master as it plays the list
//   results        DEPTH x 220   written by spi_master once per list cycle,
//                                read by the host at DIAG_PTR: four decoded
//                                words (64 bits) and both raw windows (2 x 78)
//
// WHY A SEPARATE MODULE, AND WHY DISTRIBUTED RAM
//   Keeping the storage out of spi_master leaves the sequencer a pure bus
//   engine with a narrow memory interface, and keeps the ~15 kbit of state in
//   one place where its inference can be checked. Sixty-four entries is far
//   too shallow for block RAM to be worth a tile, so both memories are LUT RAM:
//   the command list is a dual-port RAM64 (one read/write port at DIAG_PTR, one
//   asynchronous read port for the sequencer), the results a simple dual-port
//   one (write at the cycle index, read at DIAG_PTR).
//
// READ TIMING
//   The sequencer's port is asynchronous so spi_master can load its shifter on
//   the same clock it decides to start, without a pipeline bubble. The host's
//   reads are registered: DIAG_PTR changes at most once per UART line, so one
//   clock of latency costs nothing and cuts the path from the RAM into
//   ctrl_regs' wide read multiplexer.
//
// Results are zero at configuration (initial block -> LUT INIT) so a read
// before the first list returns 0 rather than X in simulation.
// ============================================================================

`default_nettype none

module diag_mem #(
    parameter int DEPTH = 64,
    parameter int TAPS  = 78
) (
    input  wire                       clk,

    // host side, all at DIAG_PTR
    input  wire [$clog2(DEPTH)-1:0]   ptr,
    input  wire                       cmd_we,
    input  wire [15:0]                cmd_wdata,
    output reg  [15:0]                cmd_rd,
    output reg  [63:0]                res_rd,     // {c1 B, c1 A, c0 B, c0 A}
    output reg  [TAPS-1:0]            raw0_rd,
    output reg  [TAPS-1:0]            raw1_rd,

    // sequencer side
    input  wire [$clog2(DEPTH)-1:0]   seq_addr,
    output wire [15:0]                seq_cmd,    // asynchronous
    input  wire                       res_we,
    input  wire [$clog2(DEPTH)-1:0]   res_waddr,
    input  wire [63:0]                res_wdata,
    input  wire [TAPS-1:0]            raw0_wdata,
    input  wire [TAPS-1:0]            raw1_wdata
);

    localparam int RW = 64 + 2 * TAPS;

    (* ram_style = "distributed" *) reg [15:0]   cmd_mem [0:DEPTH-1];
    (* ram_style = "distributed" *) reg [RW-1:0] res_mem [0:DEPTH-1];

    integer i;
    initial begin
        for (i = 0; i < DEPTH; i = i + 1) begin
            cmd_mem[i] = 16'd0;
            res_mem[i] = '0;
        end
    end

    always @(posedge clk) begin
        if (cmd_we) cmd_mem[ptr] <= cmd_wdata;
        if (res_we) res_mem[res_waddr] <= {raw1_wdata, raw0_wdata, res_wdata};
    end

    assign seq_cmd = cmd_mem[seq_addr];

    wire [RW-1:0] res_at_ptr = res_mem[ptr];

    always @(posedge clk) begin
        cmd_rd  <= cmd_mem[ptr];
        res_rd  <= res_at_ptr[63:0];
        raw0_rd <= res_at_ptr[64 +: TAPS];
        raw1_rd <= res_at_ptr[64 + TAPS +: TAPS];
    end

endmodule

`default_nettype wire
