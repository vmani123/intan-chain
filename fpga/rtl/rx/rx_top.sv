// ============================================================================
// rx_top.sv -- the Arty S7-50 receiver: SPI or pattern in, QSPI datagrams out.
// ----------------------------------------------------------------------------
//   uart_rx -> ctrl_regs -> uart_tx                         control (section 4)
//   spi_master / pattern_gen -> frame_builder -> dgram_mem -> qspi_master  data
//   slot_fifo                                               back pressure
//
// `clk` is a port, not an MMCM output: the board top owns the clock tree, and
// putting an MMCM here would make the whole receiver unsimulatable in Icarus.
//
// THE TIME BASE
//   One free-running counter at the system clock feeds both t_cap (latched by
//   frame_builder when a sample finishes) and t_tx (substituted by qspi_master
//   as the frame goes out). Because it is literally the same counter, the host
//   gets t_tx - t_cap with no clock-domain guesswork.
//
// THE SAMPLE TIMER, AND WHY A MISSED START IS AN OVERRUN
//   Both sources ignore `start` while busy, so a period shorter than one sweep
//   would be silently absorbed and an impossible sample rate would look like it
//   worked. Instead the timer refuses to issue a start into a busy source and
//   counts the missed start, so the host sees OVERRUNS climb and reads the run
//   as source-limited. frame_builder's own busy is included because it shares
//   the slot RAM write port with the payload.
// ============================================================================

`default_nettype none

module rx_top #(
    parameter int CLK_HZ           = 96_000_000,
    parameter int BAUD             = 115_200,
    parameter int N_CH             = 128,
    parameter int HDR_WORDS        = 10,
    parameter int FRAMES_PER_DGRAM = 5,
    parameter int NSLOT            = 4,
    parameter int QSPI_DIV         = 2,
    parameter logic [7:0] QSPI_CMD = 8'h23
) (
    input  wire        clk,            // 96 MHz, buffered by the board top
    input  wire        rst_n,

    // control
    input  wire        uart_rx_i,
    output wire        uart_tx_o,

    // RHD2164 SPI
    output wire        spi_cs_n,
    output wire        spi_sclk,
    output wire        spi_mosi,
    input  wire        miso0,
    input  wire        miso1,

    // QSPI egress to the ESP32-S3
    input  wire        esp_ready,
    output wire        qspi_cs_n,
    output wire        qspi_sclk,
    output wire [3:0]  qspi_dq_out,
    output wire        qspi_dq_oe,

    output wire [2:0]  led
);

    localparam int CHW         = $clog2(N_CH);
    localparam int SW          = $clog2(NSLOT);
    localparam int FRAME_WORDS = HDR_WORDS + N_CH;
    localparam int SLOT_WORDS  = FRAMES_PER_DGRAM * FRAME_WORDS;
    localparam int AW          = $clog2(NSLOT * SLOT_WORDS);

    wire rst = ~rst_n;                 // ctrl_regs and the UARTs reset high

    // ---------------- time base ----------------
    reg [31:0] tick;
    always @(posedge clk) begin
        if (!rst_n) tick <= 32'd0;
        else        tick <= tick + 32'd1;
    end

    // ---------------- control interface ----------------
    wire [7:0]  urx_data, utx_data;
    wire        urx_valid, utx_send, utx_ready;
    wire [31:0] ctrl, sample_period, phase_select, mode;
    wire        clear_stb;

    wire [31:0] fb_frames, q_dgrams, q_stalls, fifo_high, fifo_overruns;
    reg  [31:0] ov_reg;
    wire [31:0] overruns_total = ov_reg + fifo_overruns;

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_urx (
        .clk(clk), .rst(rst), .rx(uart_rx_i),
        .data(urx_data), .valid(urx_valid));

    ctrl_regs u_ctrl (
        .clk(clk), .rst(rst),
        .rx_data(urx_data), .rx_valid(urx_valid),
        .tx_data(utx_data), .tx_send(utx_send), .tx_ready(utx_ready),
        .ctrl(ctrl), .sample_period(sample_period),
        .phase_select(phase_select), .mode(mode),
        .clear_counters_stb(clear_stb),
        .frames(fb_frames), .dgrams(q_dgrams), .fifo_high(fifo_high),
        .overruns(overruns_total), .stalls(q_stalls));

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_utx (
        .clk(clk), .rst(rst), .data(utx_data), .send(utx_send),
        .busy(), .ready(utx_ready), .tx(uart_tx_o));

    wire run       = ctrl[0];
    wire use_pat   = mode[2];
    wire [7:0] flags_w = {6'd0, mode[1], mode[0]};

    // ---------------- sources ----------------
    reg         start;
    wire        sm_valid, sm_done, sm_busy;
    wire [CHW-1:0] sm_ch;
    wire signed [15:0] sm_value;

    spi_master #(.N_CH(N_CH)) u_spi (
        .clk(clk), .rst_n(rst_n),
        .start(start && !use_pat), .phase_select(phase_select[3:0]),
        .cs_n(spi_cs_n), .sclk(spi_sclk), .mosi(spi_mosi),
        .miso0(miso0), .miso1(miso1),
        .valid(sm_valid), .ch(sm_ch), .value(sm_value),
        .sample_done(sm_done), .busy(sm_busy));

    wire        pg_valid, pg_done, pg_busy;
    wire [CHW-1:0] pg_ch;
    wire signed [15:0] pg_value;

    pattern_gen #(.N_CH(N_CH)) u_pat (
        .clk(clk), .rst_n(rst_n),
        .start(start && use_pat), .realistic(mode[0]),
        .valid(pg_valid), .ch(pg_ch), .value(pg_value),
        .done(pg_done), .busy(pg_busy));

    wire               src_valid = use_pat ? pg_valid : sm_valid;
    wire [CHW-1:0]     src_ch    = use_pat ? pg_ch    : sm_ch;
    wire signed [15:0] src_value = use_pat ? pg_value : sm_value;
    wire               src_done  = use_pat ? pg_done  : sm_done;
    wire               src_busy  = use_pat ? pg_busy  : sm_busy;

    // ---------------- datagram path ----------------
    wire [SW-1:0] wr_slot, rd_slot;
    wire          fifo_full, fifo_empty, commit, slot_release;
    wire          fb_wr_en, fb_overrun, fb_busy;
    wire [AW-1:0] fb_wr_addr, q_rd_addr;
    wire [15:0]   fb_wr_data, q_rd_data;
    wire          q_rd_en, dgram_sent;

    frame_builder #(
        .N_CH(N_CH), .HDR_WORDS(HDR_WORDS),
        .FRAMES_PER_DGRAM(FRAMES_PER_DGRAM), .NSLOT(NSLOT)
    ) u_fb (
        .clk(clk), .rst_n(rst_n), .clear(clear_stb),
        .valid(src_valid), .ch(src_ch), .value(src_value),
        .sample_done(src_done),
        .flags(flags_w), .frame_type(8'd0), .tick(tick),
        .stall(fifo_full), .slot_wr(wr_slot),
        .wr_en(fb_wr_en), .wr_addr(fb_wr_addr), .wr_data(fb_wr_data),
        .commit(commit), .overrun(fb_overrun), .frames(fb_frames),
        .busy(fb_busy));

    dgram_mem #(.NSLOT(NSLOT), .SLOT_WORDS(SLOT_WORDS)) u_mem (
        .clk(clk),
        .wr_en(fb_wr_en), .wr_addr(fb_wr_addr), .wr_data(fb_wr_data),
        .rd_en(q_rd_en),  .rd_addr(q_rd_addr),  .rd_data(q_rd_data));

    slot_fifo #(.NSLOT(NSLOT)) u_fifo (
        .clk(clk), .rst_n(rst_n), .clear(clear_stb),
        .wr_slot(wr_slot), .full(fifo_full), .commit(commit),
        .rd_slot(rd_slot), .empty(fifo_empty), .rd_release(slot_release),
        .high_water(fifo_high), .overruns(fifo_overruns));

    qspi_master #(
        .N_CH(N_CH), .HDR_WORDS(HDR_WORDS),
        .FRAMES_PER_DGRAM(FRAMES_PER_DGRAM), .NSLOT(NSLOT),
        .DIV(QSPI_DIV), .CMD_BYTE(QSPI_CMD)
    ) u_qspi (
        .clk(clk), .rst_n(rst_n), .clear(clear_stb),
        .esp_ready(esp_ready),
        .rd_en(q_rd_en), .rd_addr(q_rd_addr), .rd_data(q_rd_data),
        .slot_rd(rd_slot), .slot_valid(!fifo_empty),
        .slot_release(slot_release),
        .tick(tick),
        .cs_n(qspi_cs_n), .sclk(qspi_sclk),
        .dq_out(qspi_dq_out), .dq_oe(qspi_dq_oe),
        .dgrams(q_dgrams), .stalls(q_stalls), .dgram_sent(dgram_sent));

    // ---------------- sample timer ----------------
    reg [31:0] tmr;

    always @(posedge clk) begin
        if (!rst_n) begin
            tmr    <= 32'd0;
            start  <= 1'b0;
            ov_reg <= 32'd0;
        end else begin
            start <= 1'b0;
            if (!run) begin
                tmr <= 32'd0;
            end else if ((tmr + 32'd1) >= sample_period) begin
                tmr <= 32'd0;
                if (!src_busy && !fb_busy) start  <= 1'b1;
                else                       ov_reg <= ov_reg + 32'd1;
            end else begin
                tmr <= tmr + 32'd1;
            end

            if (fb_overrun) ov_reg <= ov_reg + 32'd1;
            if (clear_stb)  ov_reg <= 32'd0;
        end
    end

    // ---------------- LEDs ----------------
    // Same idea as the emulator's board top: one for the run bit, one that
    // always blinks so a dead clock is obvious, one that shows traffic.
    reg [21:0] act;
    always @(posedge clk) begin
        if (!rst_n)          act <= 22'd0;
        else if (dgram_sent) act <= {22{1'b1}};
        else if (act != 0)   act <= act - 22'd1;
    end

    assign led = {|act, tick[23], run};

endmodule

`default_nettype wire
