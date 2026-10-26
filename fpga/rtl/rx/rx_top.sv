// ============================================================================
// rx_top.sv -- the Arty S7-50 receiver: SPI or pattern in, QSPI datagrams out.
// ----------------------------------------------------------------------------
//   uart_rx -> ctrl_regs -> uart_tx                         control (section 4)
//   spi_master / pattern_gen -> frame_builder -> dgram_mem -> qspi_master  data
//   slot_fifo                                               back pressure
//   diag_mem, link check, MISO monitor                      bring-up (section 6)
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
//
// BRING-UP DIAGNOSTICS, AND WHY THEY NEED NO ESP
//   A real headstage has to be proven alive before any frame is worth looking
//   at, so everything here is readable over the UART alone:
//   - the diagnostic list (diag_mem + spi_master) sends arbitrary commands and
//     returns each cycle's decoded words and raw MISO taps. A `go` while `run`
//     is set is refused, so a list can never interleave with a sweep.
//   - the link check compares every sweep's READ(59) result with the ROM
//     marker, per chip and module, and counts the mismatches.
//   - the MISO monitor says whether each MISO line moves at all while CS is
//     low. It is the one consumer of MISO that is not timed by the master, so
//     it gets a two-flop synchroniser; the capture taps in spi_master do not,
//     because their phase relative to SCLK is the whole point of them.
//   `mode_o` is MODE as written, so a board top can steer the SPI pins (bit3,
//   loopback to an on-die emulator) without a second control path. HAS_EMU
//   tells the host whether that loopback exists in this build.
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
    parameter logic [7:0] QSPI_CMD = 8'h23,
    parameter bit HAS_EMU          = 1'b0    // board top has an on-die emulator (MODE bit3)
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

    output wire [2:0]  led,
    output wire [31:0] mode_o,         // MODE register, for SPI routing in the board top
    output wire        link_good       // last link check passed on all four lanes
);

    localparam int CHW         = $clog2(N_CH);
    localparam int SW          = $clog2(NSLOT);
    localparam int FRAME_WORDS = HDR_WORDS + N_CH;
    localparam int SLOT_WORDS  = FRAMES_PER_DGRAM * FRAME_WORDS;
    localparam int AW          = $clog2(NSLOT * SLOT_WORDS);
    localparam int CAP_TAPS    = 78;   // CONTRACTS.md section 5
    localparam int DIAG_DEPTH  = 64;
    localparam int DAW         = $clog2(DIAG_DEPTH);

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
    wire [31:0] ctrl, sample_period, phase_select, mode, spi_div;
    wire        clear_stb;

    wire [31:0] fb_frames, q_dgrams, q_stalls, fifo_high, fifo_overruns;
    reg  [31:0] ov_reg;
    wire [31:0] overruns_total = ov_reg + fifo_overruns;

    // diagnostic list, host side
    wire [DAW-1:0]      diag_ptr;
    wire [DAW:0]        diag_len;
    wire                diag_cmd_we, diag_go_req;
    wire [15:0]         diag_cmd_wdata, diag_cmd_rd;
    wire [63:0]         diag_res_rd;
    wire [CAP_TAPS-1:0] diag_raw0_rd, diag_raw1_rd;
    reg                 diag_done_f, diag_rejected;
    wire                sm_diag_busy;

    // link check and MISO monitor counters (declared here, kept further down)
    reg  [31:0] spi_cycles, link_checks, link_last0, link_last1;
    reg  [15:0] lerr0a, lerr0b, lerr1a, lerr1b;
    reg  [31:0] miso_edges0, miso_edges1;
    reg  [3:0]  miso_seen;             // {m1 high, m1 low, m0 high, m0 low}
    wire [1:0]  miso_now;
    wire        loopback = mode[3] && HAS_EMU;

    wire [31:0] miso_stat = {23'd0, loopback, 2'b00, miso_now, miso_seen};
    wire [31:0] caps      = {8'd0, 8'(CAP_TAPS), 7'd0, HAS_EMU, 8'(QSPI_DIV)};

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_urx (
        .clk(clk), .rst(rst), .rx(uart_rx_i),
        .data(urx_data), .valid(urx_valid));

    ctrl_regs #(.CAP_TAPS(CAP_TAPS)) u_ctrl (
        .clk(clk), .rst(rst),
        .rx_data(urx_data), .rx_valid(urx_valid),
        .tx_data(utx_data), .tx_send(utx_send), .tx_ready(utx_ready),
        .ctrl(ctrl), .sample_period(sample_period),
        .phase_select(phase_select), .mode(mode), .spi_div(spi_div),
        .clear_counters_stb(clear_stb),
        .diag_ptr(diag_ptr), .diag_len(diag_len),
        .diag_cmd_we(diag_cmd_we), .diag_cmd_wdata(diag_cmd_wdata),
        .diag_go_stb(diag_go_req),
        .frames(fb_frames), .dgrams(q_dgrams), .fifo_high(fifo_high),
        .overruns(overruns_total), .stalls(q_stalls),
        .spi_cycles(spi_cycles), .link_checks(link_checks),
        .link_err0({lerr0b, lerr0a}), .link_err1({lerr1b, lerr1a}),
        .link_last0(link_last0), .link_last1(link_last1),
        .miso_stat(miso_stat),
        .miso_edges0(miso_edges0), .miso_edges1(miso_edges1),
        .caps(caps),
        .diag_status({diag_rejected, diag_done_f, sm_diag_busy}),
        .diag_cmd_rd(diag_cmd_rd),
        .diag_res0(diag_res_rd[31:0]), .diag_res1(diag_res_rd[63:32]),
        .diag_raw0(diag_raw0_rd), .diag_raw1(diag_raw1_rd));

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_utx (
        .clk(clk), .rst(rst), .data(utx_data), .send(utx_send),
        .busy(), .ready(utx_ready), .tx(uart_tx_o));

    wire run       = ctrl[0];
    wire use_pat   = mode[2];
    wire [7:0] flags_w = {6'd0, mode[1], mode[0]};

    assign mode_o = mode;

    // ---------------- sources ----------------
    reg         start;
    wire        sm_valid, sm_done, sm_busy;
    wire [CHW-1:0] sm_ch;
    wire signed [15:0] sm_value;

    // A list while `run` is set would steal sweep slots and leave the host
    // reading frames with holes in them, so that `go` is refused outright.
    // One that arrives while a list is already playing is ignored, status
    // and all, which is also what spi_master does with it.
    wire diag_go = diag_go_req && !run && !sm_diag_busy;

    wire                sm_cycle, sm_link_stb, sm_diag_done;
    wire [63:0]         sm_link_words;
    wire [DAW-1:0]      sm_cmd_addr, sm_wr_addr;
    wire [15:0]         sm_cmd;
    wire                sm_wr_en;
    wire [63:0]         sm_wr_res;
    wire [CAP_TAPS-1:0] sm_wr_raw0, sm_wr_raw1;

    spi_master #(.N_CH(N_CH), .DIAG_DEPTH(DIAG_DEPTH), .TAPS(CAP_TAPS)) u_spi (
        .clk(clk), .rst_n(rst_n),
        .start(start && !use_pat), .phase_select(phase_select[3:0]),
        .spi_div(spi_div[3:0]),
        .diag_go(diag_go), .diag_len(diag_len),
        .diag_cmd_addr(sm_cmd_addr), .diag_cmd(sm_cmd),
        .diag_wr_en(sm_wr_en), .diag_wr_addr(sm_wr_addr),
        .diag_wr_res(sm_wr_res), .diag_wr_raw0(sm_wr_raw0), .diag_wr_raw1(sm_wr_raw1),
        .diag_busy(sm_diag_busy), .diag_done(sm_diag_done),
        .link_stb(sm_link_stb), .link_words(sm_link_words),
        .cycle_stb(sm_cycle),
        .cs_n(spi_cs_n), .sclk(spi_sclk), .mosi(spi_mosi),
        .miso0(miso0), .miso1(miso1),
        .valid(sm_valid), .ch(sm_ch), .value(sm_value),
        .sample_done(sm_done), .busy(sm_busy));

    diag_mem #(.DEPTH(DIAG_DEPTH), .TAPS(CAP_TAPS)) u_diag (
        .clk(clk),
        .ptr(diag_ptr), .cmd_we(diag_cmd_we), .cmd_wdata(diag_cmd_wdata),
        .cmd_rd(diag_cmd_rd), .res_rd(diag_res_rd),
        .raw0_rd(diag_raw0_rd), .raw1_rd(diag_raw1_rd),
        .seq_addr(sm_cmd_addr), .seq_cmd(sm_cmd),
        .res_we(sm_wr_en), .res_waddr(sm_wr_addr), .res_wdata(sm_wr_res),
        .raw0_wdata(sm_wr_raw0), .raw1_wdata(sm_wr_raw1));

    // DIAG_CTRL status: done means "the list started by the last go finished",
    // so every go that is not ignored clears it, refused or not.
    always @(posedge clk) begin
        if (!rst_n) begin
            diag_done_f   <= 1'b0;
            diag_rejected <= 1'b0;
        end else begin
            if (diag_go_req && !sm_diag_busy) begin
                diag_rejected <= run;
                diag_done_f   <= 1'b0;
            end
            if (sm_diag_done) diag_done_f <= 1'b1;
        end
    end

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

    // ---------------- link check (CONTRACTS.md section 6) ----------------
    // spi_master holds sweep cycle 34's words in link_words when it strobes, so
    // the compare reads registers. Each lane's error count saturates rather
    // than wraps: a wrapped count would read as a healthy link.
    localparam logic [15:0] MARK_A = 16'h0035, MARK_B = 16'h003A;

    wire ok0a = (sm_link_words[15:0]  == MARK_A);
    wire ok0b = (sm_link_words[31:16] == MARK_B);
    wire ok1a = (sm_link_words[47:32] == MARK_A);
    wire ok1b = (sm_link_words[63:48] == MARK_B);
    reg  link_good_r;

    always @(posedge clk) begin
        if (!rst_n || clear_stb) begin
            spi_cycles  <= 32'd0;
            link_checks <= 32'd0;
            link_last0  <= 32'd0;
            link_last1  <= 32'd0;
            lerr0a      <= 16'd0;
            lerr0b      <= 16'd0;
            lerr1a      <= 16'd0;
            lerr1b      <= 16'd0;
            link_good_r <= 1'b0;
        end else begin
            if (sm_cycle) spi_cycles <= spi_cycles + 32'd1;
            if (sm_link_stb) begin
                link_checks <= link_checks + 32'd1;
                link_last0  <= sm_link_words[31:0];
                link_last1  <= sm_link_words[63:32];
                link_good_r <= ok0a && ok0b && ok1a && ok1b;
                if (!ok0a && lerr0a != 16'hFFFF) lerr0a <= lerr0a + 16'd1;
                if (!ok0b && lerr0b != 16'hFFFF) lerr0b <= lerr0b + 16'd1;
                if (!ok1a && lerr1a != 16'hFFFF) lerr1a <= lerr1a + 16'd1;
                if (!ok1b && lerr1b != 16'hFFFF) lerr1b <= lerr1b + 16'd1;
            end
        end
    end

    assign link_good = link_good_r;

    // ---------------- MISO monitor ----------------
    // The MISO pins are asynchronous to this clock, so they are synchronised
    // before anything counts them. CS is delayed by the same two clocks, so
    // "while CS was low" means the same stretch of the MISO waveform.
    (* ASYNC_REG = "TRUE" *) reg [1:0] m0_sync, m1_sync;
    reg [1:0] m_prev;
    reg [1:0] cs_dly;
    wire      cs_low = !cs_dly[1];

    assign miso_now = {m1_sync[1], m0_sync[1]};

    always @(posedge clk) begin
        m0_sync <= {m0_sync[0], miso0};
        m1_sync <= {m1_sync[0], miso1};
        m_prev  <= miso_now;
        cs_dly  <= rst_n ? {cs_dly[0], spi_cs_n} : 2'b11;
        if (!rst_n || clear_stb) begin
            miso_seen   <= 4'd0;
            miso_edges0 <= 32'd0;
            miso_edges1 <= 32'd0;
        end else if (cs_low) begin
            if (!miso_now[0]) miso_seen[0] <= 1'b1;
            if ( miso_now[0]) miso_seen[1] <= 1'b1;
            if (!miso_now[1]) miso_seen[2] <= 1'b1;
            if ( miso_now[1]) miso_seen[3] <= 1'b1;
            if (miso_now[0] != m_prev[0]) miso_edges0 <= miso_edges0 + 32'd1;
            if (miso_now[1] != m_prev[1]) miso_edges1 <= miso_edges1 + 32'd1;
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
