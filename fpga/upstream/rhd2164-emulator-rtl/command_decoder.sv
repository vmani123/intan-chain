// ============================================================================
// command_decoder.sv
// ----------------------------------------------------------------------------
// RHD2164 emulator — command decode, response generation, and the 2-command
// pipeline that the RHD2000 protocol requires (the result of a command is
// transmitted two CS cycles later).
//
// One instance per emulated chip. It drives:
//   * the register file write/read ports
//   * the channel-data BRAM address (for CONVERT)
// and produces result_a / result_b — the 16-bit words the DDR MISO stage must
// shift out during the CURRENT CS cycle (already pipeline-delayed by 2).
//
// Command map (top 2 bits of cmd_word):
//   00 -> CONVERT(C)         result = ADC/BRAM value for channel C
//   01 -> 0x5500 CALIBRATE / 0x6A00 CLEAR / else invalid -> MSB-only result
//   10 -> WRITE(R,D)         result = {0xFF, D}
//   11 -> READ(R)            result = {0x00, reg[R]}
//
// "MSB-only" result = {~twoscomp, 15'b0}: MSB is 1 unless two's-complement
// output mode is enabled (Register 4 bit 6).
//
// CALIBRATE busy window: the 9 commands following a CALIBRATE are ignored (not
// executed) and return MSB-only results.
//
// CONVERT(63) auto-increments an internal MUX pointer through amplifier
// channels 0..31 (per module) and converts that channel.
// ============================================================================

`default_nettype none

module command_decoder (
    input  wire        clk,
    input  wire        rst_n,

    // From SPI front-end
    input  wire [15:0] cmd_word,
    input  wire        cmd_valid,    // 1-cycle pulse at end of a 16-bit command
    input  wire        cs_rising,    // 1-cycle pulse at end of a CS cycle

    // Channel-data BRAM (addressed by this module; data returns next cycle)
    output reg  [4:0]  bram_addr,
    input  wire [15:0] bram_data_a,  // module A channel value (ch 0..31)
    input  wire [15:0] bram_data_b,  // module B channel value (ch 32..63)

    // Register file
    output wire        reg_wr_en,
    output wire [5:0]  reg_wr_addr,
    output wire [7:0]  reg_wr_data,
    output wire [5:0]  reg_rd_addr,
    input  wire [7:0]  reg_rd_data_a,
    input  wire [7:0]  reg_rd_data_b,
    input  wire        twoscomp,

    // Pipelined results to shift out THIS CS cycle
    output wire [15:0] result_a,
    output wire [15:0] result_b
);

    localparam [1:0] OP_CONVERT = 2'b00;
    localparam [1:0] OP_CALCMD  = 2'b01;
    localparam [1:0] OP_WRITE   = 2'b10;
    localparam [1:0] OP_READ    = 2'b11;

    // ------------------------------------------------------------------
    // Latched command + sequencing state, updated once per command.
    // ------------------------------------------------------------------
    reg  [15:0] cmd_r;       // latched command (drives combinational result)
    reg         ignored_r;   // latched "in dummy window" status for cmd_r
    reg  [3:0]  calib_cnt;   // >0 => inside the post-CALIBRATE dummy window
    reg  [4:0]  mux_ptr;     // CONVERT(63) auto-increment pointer (0..31)

    wire        ignored = (calib_cnt != 4'd0);  // current cmd in dummy window

    // Next MUX pointer (wrap at 31 -> 0; amplifier array is 32 per module)
    wire [4:0]  ptr_next = (mux_ptr == 5'd31) ? 5'd0 : (mux_ptr + 5'd1);

    always @(posedge clk) begin
        if (!rst_n) begin
            cmd_r     <= 16'h0000;
            ignored_r <= 1'b0;
            calib_cnt <= 4'd0;
            mux_ptr   <= 5'd0;
            bram_addr <= 5'd0;
        end else if (cmd_valid) begin
            cmd_r     <= cmd_word;
            ignored_r <= ignored;   // capture status BEFORE the decrement below

            if (ignored) begin
                // Command is a CALIBRATE dummy: ignore, just count down.
                calib_cnt <= calib_cnt - 4'd1;
            end else begin
                // CONVERT: select channel and update the auto-increment pointer.
                if (cmd_word[15:14] == OP_CONVERT) begin
                    if (cmd_word[13:8] == 6'd63) begin
                        mux_ptr   <= ptr_next;
                        bram_addr <= ptr_next;
                    end else begin
                        mux_ptr   <= cmd_word[12:8];
                        bram_addr <= cmd_word[12:8];
                    end
                end
                // CALIBRATE arms the 9-command dummy window.
                if (cmd_word == 16'h5500)
                    calib_cnt <= 4'd9;
            end
        end
    end

    // ------------------------------------------------------------------
    // Register-file drive (combinational; pulses for exactly one clk while
    // cmd_valid is asserted on a non-ignored WRITE).
    // ------------------------------------------------------------------
    assign reg_wr_en   = cmd_valid && (cmd_word[15:14] == OP_WRITE) && !ignored;
    assign reg_wr_addr = cmd_word[13:8];
    assign reg_wr_data = cmd_word[7:0];
    assign reg_rd_addr = cmd_r[13:8];      // for READ result (from latched cmd)

    // ------------------------------------------------------------------
    // Combinational result for the just-latched command.
    // ------------------------------------------------------------------
    wire [15:0] msb_only = {~twoscomp, 15'b0};

    reg [15:0] res_new_a, res_new_b;
    always @(*) begin
        if (ignored_r) begin       // use the LATCHED status for the latched cmd
            res_new_a = msb_only;
            res_new_b = msb_only;
        end else begin
            case (cmd_r[15:14])
                OP_CONVERT: begin
                    res_new_a = bram_data_a;
                    res_new_b = bram_data_b;
                end
                OP_WRITE: begin
                    res_new_a = {8'hFF, cmd_r[7:0]};
                    res_new_b = {8'hFF, cmd_r[7:0]};
                end
                OP_READ: begin
                    res_new_a = {8'h00, reg_rd_data_a};
                    res_new_b = {8'h00, reg_rd_data_b};
                end
                default: begin // OP_CALCMD: CALIBRATE / CLEAR / invalid
                    res_new_a = msb_only;
                    res_new_b = msb_only;
                end
            endcase
        end
    end

    // ------------------------------------------------------------------
    // 2-command pipeline. Advance once per CS cycle (at cs_rising). The value
    // presented during CS cycle k is the result of the command from cycle k-2.
    // ------------------------------------------------------------------
    reg [15:0] res_dly_a, res_dly_b;   // result of command from 1 cycle ago
    reg [15:0] res_out_a, res_out_b;   // result presented this cycle (k-2)

    always @(posedge clk) begin
        if (!rst_n) begin
            res_dly_a <= 16'h0000;
            res_dly_b <= 16'h0000;
            res_out_a <= 16'h0000;
            res_out_b <= 16'h0000;
        end else if (cs_rising) begin
            res_out_a <= res_dly_a;
            res_out_b <= res_dly_b;
            res_dly_a <= res_new_a;
            res_dly_b <= res_new_b;
        end
    end

    assign result_a = res_out_a;
    assign result_b = res_out_b;

endmodule

`default_nettype wire
