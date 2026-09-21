// ============================================================================
// register_file.sv
// ----------------------------------------------------------------------------
// RHD2164 emulator — on-chip register bank (one instance per emulated chip).
//
// Holds the 22 RAM registers (0..21) and supplies the read-only ROM registers
// (40..44 = "INTAN", 59 = A/B marker, 60..63 = identity).  Because the RHD2164
// contains two 32-channel cores (module A = ch0..31, module B = ch32..63) that
// receive identical commands, this bank exposes BOTH the A-stream and B-stream
// read values from a single combinational read port:
//
//   * regs 0..17  : same value on A and B
//   * regs 18..21 : (RHD2164-specific amp-power for ch32..63) read back
//                   correctly ONLY on the B stream; A returns 0x00 (matches the
//                   datasheet quirk)
//   * reg 59      : A returns 0x35, B returns 0x3A (the A/B marker)
//   * other ROM   : identical on A and B
//
// twoscomp (Register 4, bit 6) is exported because it controls the MSB of the
// "MSB-only" results returned by CALIBRATE/CLEAR/invalid commands.
// ============================================================================

`default_nettype none

module register_file (
    input  wire        clk,
    input  wire        rst_n,

    // Write port (driven by the command decoder on a WRITE command)
    input  wire        wr_en,
    input  wire [5:0]  wr_addr,
    input  wire [7:0]  wr_data,

    // Combinational read port
    input  wire [5:0]  rd_addr,
    output reg  [7:0]  rd_data_a,
    output reg  [7:0]  rd_data_b,

    output wire        twoscomp
);

    // ------------------------------------------------------------------
    // RAM storage (regs 0..21). Reset to 0 for deterministic simulation
    // (real silicon powers up indeterminate — see docs/SPEC.md §9).
    // ------------------------------------------------------------------
    reg [7:0] ram [0:21];
    integer i;

    always @(posedge clk) begin
        if (!rst_n) begin
            for (i = 0; i < 22; i = i + 1)
                ram[i] <= 8'h00;
        end else if (wr_en && (wr_addr <= 6'd21)) begin
            ram[wr_addr] <= wr_data;
        end
    end

    assign twoscomp = ram[4][6];

    // ------------------------------------------------------------------
    // ROM contents. Only reg 59 differs between the A and B streams.
    // ------------------------------------------------------------------
    function [7:0] rom_val(input [5:0] a, input is_b);
        begin
            case (a)
                6'd40:   rom_val = 8'h49;             // 'I'
                6'd41:   rom_val = 8'h4E;             // 'N'
                6'd42:   rom_val = 8'h54;             // 'T'
                6'd43:   rom_val = 8'h41;             // 'A'
                6'd44:   rom_val = 8'h4E;             // 'N'
                6'd59:   rom_val = is_b ? 8'h3A : 8'h35; // A/B marker (58 / 53)
                6'd60:   rom_val = 8'h01;             // die revision
                6'd61:   rom_val = 8'h01;             // unipolar amplifiers
                6'd62:   rom_val = 8'h40;             // number of amps = 64
                6'd63:   rom_val = 8'h04;             // chip ID = 4 (RHD2164)
                default: rom_val = 8'h00;
            endcase
        end
    endfunction

    // ------------------------------------------------------------------
    // Combinational read mux.
    // ------------------------------------------------------------------
    always @(*) begin
        if (rd_addr <= 6'd17) begin
            rd_data_a = ram[rd_addr];
            rd_data_b = ram[rd_addr];
        end else if (rd_addr <= 6'd21) begin
            rd_data_a = 8'h00;          // regs 18..21 read correct only on B
            rd_data_b = ram[rd_addr];
        end else begin
            rd_data_a = rom_val(rd_addr, 1'b0);
            rd_data_b = rom_val(rd_addr, 1'b1);
        end
    end

endmodule

`default_nettype wire
