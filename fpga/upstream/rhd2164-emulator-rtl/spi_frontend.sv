// ============================================================================
// spi_frontend.sv
// ----------------------------------------------------------------------------
// RHD2164 emulator — SPI input capture + edge detection.
//
// The real RHD2164 is purely asynchronous (it has no internal clock; the ADC
// clock is derived from SCLK).  On the FPGA we oversample the three SPI input
// signals (CS, SCLK, MOSI) with a fast fabric clock and reconstruct the bus
// activity as 1-cycle edge strobes in that domain.
//
// IMPORTANT: the differential receivers (IBUFDS) for CS/SCLK/MOSI live at the
// top level and are shared by BOTH emulated chips, so this module takes the
// already single-ended signals.  That keeps the module instantiable twice
// without contending for the same input pins.
//
// Outputs:
//   - cmd_word   : last fully-received 16-bit command (MSB first)
//   - cmd_valid  : 1-cycle pulse in clk domain when the 16th SCLK rising edge
//                  of a word has been captured
//   - *_rising / *_falling : 1-cycle edge strobes for SCLK and CS, used by the
//                  DDR MISO output stage to launch data on the correct edges
//   - sclk_q / cs_q : synchronized signal levels
//   - bit_index  : index of the SCLK rising edge within the current word (0..15)
// ============================================================================

`default_nettype none

module spi_frontend #(
    parameter int SYNC_STAGES = 3   // metastability hardening FF stages
) (
    input  wire        clk,         // fast oversampling clock (e.g. ~400 MHz)
    input  wire        rst_n,       // active-low synchronous reset

    // Single-ended SPI inputs (post-IBUFDS, shared across chips)
    input  wire        cs_in,       // active-low chip select
    input  wire        sclk_in,     // serial clock, CPOL=0 (idle low)
    input  wire        mosi_in,     // command data, MSB first

    // Synchronized levels
    output reg         cs_q,        // 1 = idle/high, 0 = selected/low
    output reg         sclk_q,

    // Edge strobes (1 clk cycle each)
    output reg         sclk_rising,
    output reg         sclk_falling,
    output reg         cs_rising,    // end of a transfer
    output reg         cs_falling,   // start of a transfer (ADC sample point)

    // Captured command
    output reg  [15:0] cmd_word,
    output reg         cmd_valid,
    output reg  [4:0]  bit_index     // 0..15 index of next bit to capture
);

    // ------------------------------------------------------------------
    // Metastability synchronizers for the three asynchronous inputs.
    // ------------------------------------------------------------------
    reg [SYNC_STAGES-1:0] cs_sync, sclk_sync, mosi_sync;

    always @(posedge clk) begin
        if (!rst_n) begin
            cs_sync   <= {SYNC_STAGES{1'b1}};  // CS idle high
            sclk_sync <= '0;
            mosi_sync <= '0;
        end else begin
            cs_sync   <= {cs_sync[SYNC_STAGES-2:0],   cs_in};
            sclk_sync <= {sclk_sync[SYNC_STAGES-2:0], sclk_in};
            mosi_sync <= {mosi_sync[SYNC_STAGES-2:0], mosi_in};
        end
    end

    wire cs_curr   = cs_sync[SYNC_STAGES-1];
    wire sclk_curr = sclk_sync[SYNC_STAGES-1];
    wire mosi_curr = mosi_sync[SYNC_STAGES-1];

    // Previous-cycle copies for edge detection
    reg cs_prev, sclk_prev;

    // ------------------------------------------------------------------
    // Edge detection + command shift register.
    // ------------------------------------------------------------------
    reg [15:0] shift_reg;

    always @(posedge clk) begin
        if (!rst_n) begin
            cs_q         <= 1'b1;
            sclk_q       <= 1'b0;
            cs_prev      <= 1'b1;
            sclk_prev    <= 1'b0;
            sclk_rising  <= 1'b0;
            sclk_falling <= 1'b0;
            cs_rising    <= 1'b0;
            cs_falling   <= 1'b0;
            cmd_word     <= 16'h0000;
            cmd_valid    <= 1'b0;
            bit_index    <= 5'd0;
            shift_reg    <= 16'h0000;
        end else begin
            // Register synchronized levels and history
            cs_q      <= cs_curr;
            sclk_q    <= sclk_curr;
            cs_prev   <= cs_curr;
            sclk_prev <= sclk_curr;

            // Default: strobes low (they are 1-cycle pulses)
            sclk_rising  <= 1'b0;
            sclk_falling <= 1'b0;
            cs_rising    <= 1'b0;
            cs_falling   <= 1'b0;
            cmd_valid    <= 1'b0;

            // ---- CS edges ----
            if (cs_prev && !cs_curr) begin
                // Falling edge of CS: start of a 16-bit transfer.
                cs_falling <= 1'b1;
                bit_index  <= 5'd0;
                shift_reg  <= 16'h0000;
            end
            if (!cs_prev && cs_curr) begin
                // Rising edge of CS: end of transfer.
                cs_rising <= 1'b1;
            end

            // ---- SCLK edges (only meaningful while CS is low) ----
            if (!cs_curr) begin
                // Rising edge: chip samples MOSI here, MSB first.
                if (!sclk_prev && sclk_curr) begin
                    sclk_rising <= 1'b1;
                    shift_reg   <= {shift_reg[14:0], mosi_curr};
                    if (bit_index == 5'd15) begin
                        // 16th bit captured: command complete.
                        cmd_word  <= {shift_reg[14:0], mosi_curr};
                        cmd_valid <= 1'b1;
                        bit_index <= 5'd0;
                    end else begin
                        bit_index <= bit_index + 5'd1;
                    end
                end
                // Falling edge.
                if (sclk_prev && !sclk_curr) begin
                    sclk_falling <= 1'b1;
                end
            end
        end
    end

endmodule

`default_nettype wire
