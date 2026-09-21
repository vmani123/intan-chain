// ============================================================================
// rhd2164_emulator.sv
// ----------------------------------------------------------------------------
// One emulated RHD2164 chip. Single-ended SPI interface (the differential
// IBUFDS/OBUFDS buffers live in rhd2164_top.sv) so this core is fully
// simulatable without Xilinx primitives.
//
// Contains:
//   * spi_frontend     — input capture + edge strobes
//   * command_decoder  — command decode + 2-command result pipeline
//   * register_file    — RAM/ROM registers (A and B stream read values)
//   * two channel BRAMs (module A ch0..31, module B ch32..63), $readmemh-init
//   * ddr_miso         — DDR merge onto the serial MISO line
//
// The two 32x16 channel memories are the emulator's "ADC" data source. Each is
// initialized from a .mem file at build time so the host can validate that
// every channel returns a known, distinct 16-bit value.
// ============================================================================

`default_nettype none

module rhd2164_emulator #(
    parameter MEM_A_FILE   = "chip0_A.mem",  // module A: channels 0..31
    parameter MEM_B_FILE   = "chip0_B.mem",  // module B: channels 32..63
    parameter MEM_SAMPLES  = 256             // time-varying playback depth (samples)
) (
    input  wire clk,        // fast oversampling clock (~400 MHz)
    input  wire rst_n,

    // Single-ended SPI (shared CS/SCLK/MOSI; dedicated MISO)
    input  wire cs,         // active-low chip select
    input  wire sclk,       // serial clock, CPOL=0
    input  wire mosi,       // command in, MSB first
    output wire miso        // serial data out (DDR-merged A/B)
);

    // ---- front-end <-> decoder/ddr ----
    wire [15:0] cmd_word;
    wire        cmd_valid;
    wire        sclk_rising, sclk_falling, cs_rising, cs_falling;

    spi_frontend u_frontend (
        .clk          (clk),
        .rst_n        (rst_n),
        .cs_in        (cs),
        .sclk_in      (sclk),
        .mosi_in      (mosi),
        .cs_q         (),              // levels unused at this level
        .sclk_q       (),
        .sclk_rising  (sclk_rising),
        .sclk_falling (sclk_falling),
        .cs_rising    (cs_rising),
        .cs_falling   (cs_falling),
        .cmd_word     (cmd_word),
        .cmd_valid    (cmd_valid),
        .bit_index    ()
    );

    // ---- decoder <-> register file ----
    wire        reg_wr_en;
    wire [5:0]  reg_wr_addr, reg_rd_addr;
    wire [7:0]  reg_wr_data, reg_rd_data_a, reg_rd_data_b;
    wire        twoscomp;

    // ---- decoder <-> channel BRAM ----
    wire [4:0]  bram_addr;
    reg  [15:0] bram_data_a, bram_data_b;

    // ---- decoder -> ddr ----
    wire [15:0] result_a, result_b;

    command_decoder u_decoder (
        .clk           (clk),
        .rst_n         (rst_n),
        .cmd_word      (cmd_word),
        .cmd_valid     (cmd_valid),
        .cs_rising     (cs_rising),
        .bram_addr     (bram_addr),
        .bram_data_a   (bram_data_a),
        .bram_data_b   (bram_data_b),
        .reg_wr_en     (reg_wr_en),
        .reg_wr_addr   (reg_wr_addr),
        .reg_wr_data   (reg_wr_data),
        .reg_rd_addr   (reg_rd_addr),
        .reg_rd_data_a (reg_rd_data_a),
        .reg_rd_data_b (reg_rd_data_b),
        .twoscomp      (twoscomp),
        .result_a      (result_a),
        .result_b      (result_b)
    );

    register_file u_regs (
        .clk       (clk),
        .rst_n     (rst_n),
        .wr_en     (reg_wr_en),
        .wr_addr   (reg_wr_addr),
        .wr_data   (reg_wr_data),
        .rd_addr   (reg_rd_addr),
        .rd_data_a (reg_rd_data_a),
        .rd_data_b (reg_rd_data_b),
        .twoscomp  (twoscomp)
    );

    // ------------------------------------------------------------------
    // Channel data memories (inferred block RAM, synchronous read).
    //
    // Time-varying playback: each memory holds MEM_SAMPLES time samples x 32
    // channels, stored sample-major (word address = sample*32 + channel). A
    // free-running sample pointer advances once per channel sweep so the chip
    // replays a real signal instead of a constant per channel.
    //
    // Advance trigger: the host sends one CONVERT(0..31) sweep per sample
    // period (STM32 firmware: CONVERT_COMMANDS_PER_SEQUENCE = 32, starting at
    // CONVERT(0)). So we bump the pointer on every CONVERT with channel field
    // 0. This is derived purely from the command stream -- the SPI front-end,
    // command decoder, and DDR datapath are all unchanged.
    // ------------------------------------------------------------------
    localparam integer SAMP_BITS = (MEM_SAMPLES <= 1) ? 1 : $clog2(MEM_SAMPLES);
    localparam integer DEPTH     = MEM_SAMPLES * 32;

    (* ram_style = "block" *) reg [15:0] mem_a [0:DEPTH-1];
    (* ram_style = "block" *) reg [15:0] mem_b [0:DEPTH-1];

    initial begin
        $readmemh(MEM_A_FILE, mem_a);
        $readmemh(MEM_B_FILE, mem_b);
    end

    // CONVERT(0) detector (opcode 00, channel field == 0), from the raw command.
    wire conv_ch0 = cmd_valid && (cmd_word[15:14] == 2'b00) && (cmd_word[13:8] == 6'd0);

    reg [SAMP_BITS-1:0] sample_ptr;
    always @(posedge clk) begin
        if (!rst_n)
            sample_ptr <= MEM_SAMPLES[SAMP_BITS-1:0] - 1'b1;  // first CONVERT(0) wraps to 0
        else if (conv_ch0)
            sample_ptr <= (sample_ptr == MEM_SAMPLES[SAMP_BITS-1:0] - 1'b1)
                          ? {SAMP_BITS{1'b0}} : sample_ptr + 1'b1;
    end

    // Full read address = sample_ptr*32 + channel (bram_addr is the channel).
    wire [SAMP_BITS+4:0] rd_addr = (sample_ptr * 6'd32) + bram_addr;

    always @(posedge clk) begin
        bram_data_a <= mem_a[rd_addr];
        bram_data_b <= mem_b[rd_addr];
    end

    // ------------------------------------------------------------------
    // DDR MISO merge.
    // ------------------------------------------------------------------
    ddr_miso u_ddr (
        .clk          (clk),
        .rst_n        (rst_n),
        .cs_falling   (cs_falling),
        .sclk_rising  (sclk_rising),
        .sclk_falling (sclk_falling),
        .result_a     (result_a),
        .result_b     (result_b),
        .miso_out     (miso)
    );

endmodule

`default_nettype wire
