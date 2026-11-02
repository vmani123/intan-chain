// ============================================================================
// spi_tap.sv -- a passive listener on somebody else's RHD2164 SPI bus.
// ----------------------------------------------------------------------------
// CONTRACTS.md section 7. Another master (Intan's RHD recording controller)
// drives CS, SCLK and MOSI; two RHD2164s answer on MISO1 / MISO2. This module
// never drives anything. It decodes every CS cycle it hears, attributes each
// result to the command two cycles earlier, assembles CONVERT(0..31) results
// into 128-channel sweeps and streams each sweep to frame_builder exactly the
// way spi_master does (valid / ch / value, then sample_done), with the frame
// flags TAP and TAP_PARTIAL.
//
//   pins -> 2-flop sync -> CS/SCLK/MOSI delay line -> edge counter + shifters
//        -> per-cycle close: framing, history, known answers, sweep assembly
//        -> double buffer -> streamer -> frame_builder
//
// WHY CS, SCLK AND MOSI ARE DELAYED AND MISO IS NOT
//   The chip launches A bit (15-j) at SCLK rising edge j and B bit (15-j) at
//   falling edge j (ddr_miso.sv, datasheet p.10). At the tap that bit shows up
//   one round trip (tap -> chip -> tap) plus tMISO after the edge, and lasts
//   half an SCLK period. Delaying the clock view by TAP_DELAY clocks and
//   sampling the undelayed MISO at each delayed edge reads every bit TAP_DELAY
//   clocks after the edge that launched it, so one number covers cable length,
//   chip speed and SCLK rate. All five lines pass identical two-flop
//   synchronisers, and MISO gets two pipeline flops to match the delay line's
//   own two (the SRL stage and its output register), so the offset between
//   the edge and the sample is exactly TAP_DELAY clocks in the synchronised
//   domain. The delay line is a 64-deep shift register with no reset, read at
//   a runtime index -- the dynamic-SRL template, so Vivado can build each line
//   from two SRLC32Es and a MUXF7 instead of 64 flops.
//
// FRAMING, AND WHAT "NOT HEARD" MEANS
//   Within one delayed CS-low period the rising and falling SCLK edges are
//   counted. Anything other than exactly 16 and 16 is a framing error: the
//   cycle counts in TAP_FRAMING and both its command and its result are
//   treated as not heard. A cycle whose CS fall was not seen (the first one
//   after reset) is not heard either, and is not counted at all. Edges that
//   coincide with the CS edges in the same clock are counted with the cycle.
//
// THE TWO-CYCLE PIPELINE
//   The result heard in cycle i belongs to the command heard in cycle i-2, so
//   the last two commands are kept with a heard flag each. A result is
//   attributable only if its own cycle and its command's cycle were both
//   heard. A spurious CS pulse that splits one real cycle into two costs two
//   unattributable results and nothing else: attribution only ever looks two
//   cycles back, so it re-aligns by itself.
//
// FROM RESULTS TO SWEEPS (only while `en`)
//   For each attributable result of CONVERT(c), c < 32:
//     - CONVERT(0), or a slot already heard in the open sweep, ends the open
//       sweep early (TAP_PARTIAL: slot 31 never arrived) and starts a new one.
//       Treating a repeated slot as a boundary is what keeps one frame per
//       sweep when CONVERT(0)'s own result was lost to a framing error and the
//       previous sweep's CONVERT(31) was lost too; a Rhythm-style master never
//       repeats a slot inside a sweep, so the two readings of "heard twice"
//       agree for it.
//     - With no sweep open, CONVERT(0) opens one. So does any other slot once
//       the first CONVERT(0) has been seen (its own CONVERT(0) was lost), so a
//       sweep is never skipped. Before the first CONVERT(0) result everything
//       is dropped: the first frame is the first whole sweep.
//     - CONVERT(31)'s result completes the sweep. TAP_PARTIAL is set unless
//       all 32 slots were heard, and missing slots read 0.
//   CONVERT(c >= 32), READ, WRITE, CALIBRATE and CLEAR results are not
//   channels and never touch a sweep.
//
// THE HAND-OFF, AND WHY NO SWEEP IS EVER SILENTLY LOST
//   Two sweep buffers (64 x 64 bits, one word per slot holding all four lanes)
//   alternate: one fills while the other streams. frame_builder takes one word
//   per clock and is busy for 10 clocks after each sample_done, so a sweep
//   leaves in about 145 clocks, and the streamer never starts while it is busy
//   and never stops halfway (unless `en` drops, which ends the run). A sweep
//   from an RHD2164-legal master takes at least 35 CS cycles of >= 76 clocks,
//   so in practice a buffer is always free.
//   If one is not (a burst of very short sweeps), the new sweep is still
//   tracked but its values are discarded, and when it completes it is owed as
//   a frame of zeros with TAP_PARTIAL set. Owed frames are streamed after the
//   buffered sweeps that came before them, and no later sweep takes a buffer
//   while one is owed, so frames always leave in sweep order and `seq` still
//   counts sweeps. Only when OWED_MAX zero frames are already owed is a sweep
//   lost; that raises `lost` for one clock and rx_top counts it in OVERRUNS.
//
// KNOWN ANSWERS (CONTRACTS.md section 7 table)
//   Checked for every attributable result whose command is a WRITE or a ROM
//   READ with a fixed answer, unless a CALIBRATE or CLEAR -- or a cycle that
//   was not heard, since that could have been one -- came in the previous 11
//   cycles (the 9 commands a calibration swallows, plus the 2-cycle pipeline).
//   Error counts saturate per lane at ffff, like LINK_ERR.
//
// WHAT RUNS WHEN
//   The decoder, framing, known-answer checks, TAP_SCLK_MIN and TAP_STAT run
//   whenever the tap exists, whatever MODE says, so the host can scan
//   TAP_DELAY against TAP_ERR* before streaming a single frame. Sweep assembly
//   and streaming run only while `en` (tap selected and `run` set); dropping
//   `en` discards the open sweep and anything not yet streamed, as stopping
//   the controller mid-sweep would.
//
// LIMITS
//   - B0, the last bit of each word, is launched at the last falling edge and
//     a real chip may release MISO soon after CS rises. Rhythm raises CS one
//     data clock after that edge (71 ns at 5 kS/s, 179 ns at 2 kS/s), so at
//     5 kS/s TAP_DELAY 8 (83 ns) samples B0 after CS has risen at the tap.
//     The vendored emulator holds MISO through CS high, so simulation cannot
//     show this; if B lanes fail at 5 kS/s on hardware, lower TAP_DELAY.
//   - Changing TAP_DELAY while a cycle is in flight jumps the delayed view
//     and costs that cycle (a framing error, a partial frame).
//   - Self-test against the on-die emulator needs a slow SCLK: the round trip
//     through the emulator is ~3 clocks, so SPI_DIV must leave a half period
//     well above TAP_DELAY (SPI_DIV 15: 32 clocks, Rhythm at 2.1 kS/s).
// ============================================================================

`default_nettype none

module spi_tap #(
    parameter int GOOD_W = 22           // LED stretch: 2^22 clocks = 44 ms
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        en,              // tap is the frame source and `run` is set
    input  wire        clear,           // clear_counters
    input  wire [5:0]  delay,           // TAP_DELAY, 0..63 fast clocks

    // The overheard bus, asynchronous. cs_n is the line as wired (active low).
    input  wire        cs_n_i,
    input  wire        sclk_i,
    input  wire        mosi_i,
    input  wire        miso0_i,         // cable MISO1: chip 0, channels 0..63
    input  wire        miso1_i,         // cable MISO2: chip 1, channels 64..127

    // Channel stream, the same protocol as spi_master's
    input  wire [31:0] tick,            // time base, for t_cap
    input  wire        fb_busy,         // frame_builder writing a header
    output reg         valid,
    output reg  [6:0]  ch,
    output reg  [15:0] value,
    output reg         sample_done,
    output reg         partial,         // flags bit3 for the frame being streamed;
    output reg  [31:0] t_cap,           // both held until the next frame starts
    output reg         lost,            // one clock: a sweep that could not be framed

    // Registers 0x30-0x38
    output wire [31:0] stat,
    output reg  [31:0] cycles,
    output reg  [31:0] framing,
    output reg  [31:0] sweeps,
    output reg  [31:0] partials,
    output reg  [31:0] checks,
    output wire [31:0] err0,            // {chip 0 B, chip 0 A}
    output wire [31:0] err1,            // {chip 1 B, chip 1 A}
    output reg  [31:0] sclk_min,
    output wire        good             // clean cycles heard lately, no recent error
);

    localparam logic [1:0] OWED_MAX = 2'd3;
    localparam logic [3:0] CAL_BLANK = 4'd11;

    function automatic [31:0] sat32(input [31:0] x);
        sat32 = (x == 32'hFFFF_FFFF) ? x : x + 32'd1;
    endfunction
    function automatic [15:0] sat16(input [15:0] x);
        sat16 = (x == 16'hFFFF) ? x : x + 16'd1;
    endfunction
    function automatic [15:0] lane(input [63:0] w, input [1:0] k);
        case (k)
            2'd0:    lane = w[15:0];
            2'd1:    lane = w[31:16];
            2'd2:    lane = w[47:32];
            default: lane = w[63:48];
        endcase
    endfunction

    // ------------------------------------------------------------------
    // Synchronisers. No reset: they only ever hold the pins' recent past.
    // ------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) reg [1:0] sy_cs   = 2'b11;
    (* ASYNC_REG = "TRUE" *) reg [1:0] sy_sclk = 2'b00;
    (* ASYNC_REG = "TRUE" *) reg [1:0] sy_mosi = 2'b00;
    (* ASYNC_REG = "TRUE" *) reg [1:0] sy_m0   = 2'b00;
    (* ASYNC_REG = "TRUE" *) reg [1:0] sy_m1   = 2'b00;

    always @(posedge clk) begin
        sy_cs   <= {sy_cs[0],   cs_n_i};
        sy_sclk <= {sy_sclk[0], sclk_i};
        sy_mosi <= {sy_mosi[0], mosi_i};
        sy_m0   <= {sy_m0[0],   miso0_i};
        sy_m1   <= {sy_m1[0],   miso1_i};
    end

    wire cs_s   = sy_cs[1];
    wire sclk_s = sy_sclk[1];
    wire mosi_s = sy_mosi[1];
    wire m0_s   = sy_m0[1];
    wire m1_s   = sy_m1[1];

    // ------------------------------------------------------------------
    // Delay line. dl[k] is the line k+1 clocks ago; the output register adds
    // one more, so x_d = x_s delayed by delay + 2. MISO gets the same 2.
    // ------------------------------------------------------------------
    reg [63:0] dl_cs   = {64{1'b1}};
    reg [63:0] dl_sclk = 64'd0;
    reg [63:0] dl_mosi = 64'd0;
    reg        cs_d    = 1'b1;
    reg        sclk_d  = 1'b0;
    reg        mosi_d  = 1'b0;
    reg [1:0]  m0_p    = 2'b00;
    reg [1:0]  m1_p    = 2'b00;

    always @(posedge clk) begin
        dl_cs   <= {dl_cs[62:0],   cs_s};
        dl_sclk <= {dl_sclk[62:0], sclk_s};
        dl_mosi <= {dl_mosi[62:0], mosi_s};
        cs_d    <= dl_cs[delay];
        sclk_d  <= dl_sclk[delay];
        mosi_d  <= dl_mosi[delay];
        m0_p    <= {m0_p[0], m0_s};
        m1_p    <= {m1_p[0], m1_s};
    end

    wire m0_d = m0_p[1];
    wire m1_d = m1_p[1];

    // ------------------------------------------------------------------
    // One CS cycle: count edges, shift bits, close on the clock after CS rises
    // ------------------------------------------------------------------
    reg cs_q   = 1'b1;
    reg sclk_q = 1'b0;

    wire in_cyc  = !cs_d || !cs_q;          // CS low now, or until this clock
    wire cs_fall =  cs_q && !cs_d;
    wire cs_rise = !cs_q &&  cs_d;
    wire s_rise  = in_cyc &&  sclk_d && !sclk_q;
    wire s_fall  = in_cyc && !sclk_d &&  sclk_q;

    reg [4:0]  n_rise, n_fall;              // saturate at 31
    reg        seen_fall;                   // this CS-low period began while listening
    reg        close;                       // counts and words below are final
    reg [15:0] sh_cmd = 16'd0, sh_a0 = 16'd0, sh_b0 = 16'd0, sh_a1 = 16'd0, sh_b1 = 16'd0;

    wire [4:0] nr0 = cs_fall ? 5'd0 : n_rise;
    wire [4:0] nf0 = cs_fall ? 5'd0 : n_fall;

    always @(posedge clk) begin
        if (!rst_n) begin
            cs_q      <= 1'b1;
            sclk_q    <= 1'b0;
            n_rise    <= 5'd0;
            n_fall    <= 5'd0;
            seen_fall <= 1'b0;
            close     <= 1'b0;
        end else begin
            cs_q   <= cs_d;
            sclk_q <= sclk_d;
            close  <= cs_rise;
            n_rise <= (s_rise && nr0 != 5'd31) ? nr0 + 5'd1 : nr0;
            n_fall <= (s_fall && nf0 != 5'd31) ? nf0 + 5'd1 : nf0;
            if (close)   seen_fall <= 1'b0;
            if (cs_fall) seen_fall <= 1'b1;
        end
        // MSB first: after 16 edges bit 15 is the one taken at edge 0.
        if (s_rise) begin
            sh_cmd <= {sh_cmd[14:0], mosi_d};
            sh_a0  <= {sh_a0[14:0],  m0_d};
            sh_a1  <= {sh_a1[14:0],  m1_d};
        end
        if (s_fall) begin
            sh_b0  <= {sh_b0[14:0],  m0_d};
            sh_b1  <= {sh_b1[14:0],  m1_d};
        end
    end

    wire heard  = close && seen_fall;
    wire ok_now = heard && (n_rise == 5'd16) && (n_fall == 5'd16);

    // ------------------------------------------------------------------
    // The last two commands, and the CALIBRATE / CLEAR blanking window
    // ------------------------------------------------------------------
    reg [15:0] h1_cmd = 16'd0, h2_cmd = 16'd0;
    reg        h1_ok, h2_ok;
    reg [3:0]  cal_cnt;                     // >0: a CALIBRATE may still be running

    always @(posedge clk) begin
        if (!rst_n) begin
            h1_ok   <= 1'b0;
            h2_ok   <= 1'b0;
            cal_cnt <= CAL_BLANK;           // nothing before reset was heard
        end else if (close) begin
            h2_cmd <= h1_cmd;
            h2_ok  <= h1_ok;
            h1_cmd <= sh_cmd;
            h1_ok  <= ok_now;
            if (!ok_now || sh_cmd == 16'h5500 || sh_cmd == 16'h6A00)
                cal_cnt <= CAL_BLANK;
            else if (cal_cnt != 4'd0)
                cal_cnt <= cal_cnt - 4'd1;
        end
    end

    wire attrib = ok_now && h2_ok;          // this result's command is known

    // ------------------------------------------------------------------
    // Known answers for the command two cycles back
    // ------------------------------------------------------------------
    reg        ka, ka_b;
    reg [15:0] exp_a, exp_b;

    always @* begin
        ka    = 1'b0;
        ka_b  = 1'b0;
        exp_a = 16'd0;
        exp_b = 16'd0;
        if (h2_cmd[15:14] == 2'b10) begin               // WRITE(r, d) -> FF, d
            ka    = 1'b1;
            exp_a = {8'hFF, h2_cmd[7:0]};
        end else if (h2_cmd[15:14] == 2'b11) begin      // READ(r)
            case (h2_cmd[13:8])
                6'd40: begin ka = 1'b1; exp_a = 16'h0049; end   // 'I'
                6'd41: begin ka = 1'b1; exp_a = 16'h004E; end   // 'N'
                6'd42: begin ka = 1'b1; exp_a = 16'h0054; end   // 'T'
                6'd43: begin ka = 1'b1; exp_a = 16'h0041; end   // 'A'
                6'd44: begin ka = 1'b1; exp_a = 16'h004E; end   // 'N'
                6'd59: begin ka = 1'b1; ka_b = 1'b1;
                             exp_a = 16'h0035; exp_b = 16'h003A; end
                6'd61: begin ka = 1'b1; exp_a = 16'h0001; end
                6'd62: begin ka = 1'b1; exp_a = 16'h0040; end
                6'd63: begin ka = 1'b1; exp_a = 16'h0004; end
                default: ;
            endcase
        end
    end

    wire do_check = attrib && ka && (cal_cnt == 4'd0);
    wire bad0a    = sh_a0 != exp_a;
    wire bad0b    = ka_b && (sh_b0 != exp_b);
    wire bad1a    = sh_a1 != exp_a;
    wire bad1b    = ka_b && (sh_b1 != exp_b);
    wire ka_fail  = do_check && (bad0a || bad0b || bad1a || bad1b);

    // ------------------------------------------------------------------
    // Sweep assembly: what this close does (combinational, valid on `close`)
    // ------------------------------------------------------------------
    reg        open, synced, homeless, cur, wbuf, rbuf;
    reg [31:0] heard_m;                     // slots heard in the open sweep
    reg [1:0]  full;                        // buffer queued or streaming
    reg [31:0] bmask0, bmask1;              // slots stored, per buffer
    reg [31:0] btcap0, btcap1;              // tick at hand-off, per buffer
    reg [1:0]  owed;                        // zero frames owed

    wire [4:0] c2   = h2_cmd[12:8];
    wire       conv = en && attrib && (h2_cmd[15:13] == 3'b000);   // CONVERT(c), c < 32

    reg        a_hand, a_hand_home, a_hand_buf;
    reg [31:0] a_hand_mask, a_heard;
    reg        a_open, a_synced, a_home, a_cur, a_wbuf, a_fill;

    always @* begin
        a_hand      = 1'b0;
        a_hand_home = homeless;
        a_hand_buf  = cur;
        a_hand_mask = heard_m;
        a_heard     = heard_m;
        a_open      = open;
        a_synced    = synced;
        a_home      = homeless;
        a_cur       = cur;
        a_wbuf      = wbuf;
        a_fill      = 1'b0;
        if (conv) begin
            // CONVERT(0), or a slot heard twice: the open sweep ends early.
            if (open && (c2 == 5'd0 || heard_m[c2])) begin
                a_hand = 1'b1;
                a_open = 1'b0;
            end
            // Nothing open: start a sweep, once the first CONVERT(0) is in.
            if (!a_open && (c2 == 5'd0 || synced)) begin
                a_open   = 1'b1;
                a_synced = 1'b1;
                a_heard  = 32'd0;
                // A buffer only if one is free and no zero frame is owed (or
                // about to be), so frames keep sweep order.
                a_home = full[wbuf] || (owed != 2'd0) || (a_hand && homeless);
                if (!a_home) begin
                    a_cur  = wbuf;
                    a_wbuf = !wbuf;
                end
            end
            if (a_open) begin
                a_heard[c2] = 1'b1;
                a_fill      = !a_home;
                // CONVERT(31) completes it. Never together with the early end
                // above: an open sweep cannot already hold slot 31.
                if (c2 == 5'd31) begin
                    a_hand      = 1'b1;
                    a_hand_home = a_home;
                    a_hand_buf  = a_cur;
                    a_hand_mask = a_heard;
                    a_open      = 1'b0;
                end
            end
        end
    end

    // synthesis translate_off
    always @(posedge clk)
        if (rst_n && conv && open && c2 == 5'd31 && heard_m[31])
            $fatal(1, "spi_tap: an open sweep already held slot 31");
    // synthesis translate_on

    // ------------------------------------------------------------------
    // Sweep buffers: word {buf, slot} = {chip1 B, chip1 A, chip0 B, chip0 A}
    // ------------------------------------------------------------------
    reg [63:0] mem [0:63];
    reg [63:0] mem_rd;
    reg        mem_we;
    reg [5:0]  mem_wa;
    reg [63:0] mem_wd;

    // Streamer
    localparam logic [1:0] S_IDLE = 2'd0, S_RUN = 2'd1, S_TAIL = 2'd2;
    reg [1:0]  st;
    reg [6:0]  s_cnt;
    reg        s_buf, s_zero;
    reg [31:0] s_mask;
    reg        p1_v;
    reg [6:0]  p1_ch;
    reg [1:0]  tail;

    wire [5:0] mem_ra = {s_buf, s_cnt[4:0]};

    always @(posedge clk) begin
        if (mem_we) mem[mem_wa] <= mem_wd;
        mem_rd <= mem[mem_ra];
    end

    // A write lands one clock after the close; the streamer can be reading the
    // same buffer by then only if the close just handed it over, and it reads
    // slot 31 (the one just written) 32 clocks into the frame at the earliest.
    wire hand_now   = close && a_hand;
    wire [1:0] set_full = {hand_now && !a_hand_home &&  a_hand_buf,
                           hand_now && !a_hand_home && !a_hand_buf};
    wire owe_now    = hand_now && a_hand_home && (owed != OWED_MAX);
    wire lose_now   = hand_now && a_hand_home && (owed == OWED_MAX);

    wire can_start  = (st == S_IDLE) && !fb_busy;
    wire start_buf  = can_start && full[rbuf];
    wire start_zero = can_start && !full[rbuf] && (owed != 2'd0);
    wire end_frame  = p1_v && (p1_ch == 7'd127);
    wire [1:0] clr_full = {end_frame && !s_zero &&  s_buf,
                           end_frame && !s_zero && !s_buf};

    wire [31:0] rb_mask = rbuf ? bmask1 : bmask0;
    wire [31:0] rb_tcap = rbuf ? btcap1 : btcap0;

    always @(posedge clk) begin
        if (!rst_n || !en) begin
            open        <= 1'b0;
            synced      <= 1'b0;
            homeless    <= 1'b0;
            cur         <= 1'b0;
            wbuf        <= 1'b0;
            rbuf        <= 1'b0;
            heard_m     <= 32'd0;
            full        <= 2'b00;
            owed        <= 2'd0;
            mem_we      <= 1'b0;
            st          <= S_IDLE;
            s_cnt       <= 7'd0;
            s_buf       <= 1'b0;
            s_zero      <= 1'b0;
            s_mask      <= 32'd0;
            p1_v        <= 1'b0;
            valid       <= 1'b0;
            sample_done <= 1'b0;
            lost        <= 1'b0;
            if (!rst_n) begin
                partial <= 1'b0;
                t_cap   <= 32'd0;
                ch      <= 7'd0;
                value   <= 16'd0;
                tail    <= 2'd0;
            end
        end else begin
            // ---- assembly ----
            open     <= a_open;
            synced   <= a_synced;
            homeless <= a_home;
            cur      <= a_cur;
            wbuf     <= a_wbuf;
            heard_m  <= a_heard;

            mem_we <= close && a_fill;
            mem_wa <= {a_cur, c2};
            mem_wd <= {sh_b1, sh_a1, sh_b0, sh_a0};

            if (hand_now && !a_hand_home) begin
                if (a_hand_buf) begin bmask1 <= a_hand_mask; btcap1 <= tick; end
                else            begin bmask0 <= a_hand_mask; btcap0 <= tick; end
            end

            full <= (full | set_full) & ~clr_full;
            owed <= owed + {1'b0, owe_now} - {1'b0, start_zero};
            lost <= lose_now;

            // ---- streaming ----
            case (st)
                S_IDLE: if (start_buf || start_zero) begin
                    st      <= S_RUN;
                    s_cnt   <= 7'd0;
                    s_buf   <= rbuf;
                    s_zero  <= start_zero;
                    s_mask  <= start_zero ? 32'd0 : rb_mask;
                    partial <= start_zero ? 1'b1  : ~&rb_mask;
                    t_cap   <= start_zero ? tick  : rb_tcap;
                end
                S_RUN: begin
                    s_cnt <= s_cnt + 7'd1;
                    if (s_cnt == 7'd127) begin
                        st   <= S_TAIL;
                        tail <= 2'd0;
                    end
                end
                default: begin
                    // Three clocks after the last address: sample_done has gone
                    // out and frame_builder's busy is up before S_IDLE looks.
                    tail <= tail + 2'd1;
                    if (tail == 2'd2) st <= S_IDLE;
                end
            endcase

            p1_v        <= (st == S_RUN);
            p1_ch       <= s_cnt;
            valid       <= p1_v;
            ch          <= p1_ch;
            value       <= (p1_v && s_mask[p1_ch[4:0]]) ? lane(mem_rd, p1_ch[6:5]) : 16'd0;
            sample_done <= end_frame;
            if (end_frame && !s_zero) rbuf <= !rbuf;
        end
    end

    // ------------------------------------------------------------------
    // TAP_SCLK_MIN: clocks between successive SCLK edges inside one CS-low
    // ------------------------------------------------------------------
    reg [15:0] run_len;
    reg        edge_seen;
    wire       s_edge    = s_rise || s_fall;
    wire       sclk_cand = s_edge && edge_seen && !cs_fall;

    always @(posedge clk) begin
        run_len <= s_edge ? 16'd1 : ((run_len == 16'hFFFF) ? run_len : run_len + 16'd1);
        if (!rst_n)                  edge_seen <= 1'b0;
        else if (s_edge)             edge_seen <= 1'b1;
        else if (cs_fall || !in_cyc) edge_seen <= 1'b0;
    end

    // ------------------------------------------------------------------
    // Counters and TAP_STAT
    // ------------------------------------------------------------------
    reg [15:0] e0a, e0b, e1a, e1b;
    reg [4:0]  lv_q, tog;
    wire [4:0] lv = {m1_s, m0_s, mosi_s, sclk_s, cs_s};

    assign err0 = {e0b, e0a};
    assign err1 = {e1b, e1a};
    assign stat = {15'd0, en, 3'd0, tog, 3'd0, lv};

    wire frame_out = end_frame && en;       // the sample_done that goes out

    always @(posedge clk) begin
        lv_q <= lv;
        if (!rst_n || clear) begin
            cycles   <= 32'd0;
            framing  <= 32'd0;
            sweeps   <= 32'd0;
            partials <= 32'd0;
            checks   <= 32'd0;
            e0a      <= 16'd0;
            e0b      <= 16'd0;
            e1a      <= 16'd0;
            e1b      <= 16'd0;
            sclk_min <= 32'hFFFF_FFFF;
            tog      <= 5'd0;
        end else begin
            if (heard)            cycles  <= sat32(cycles);
            if (heard && !ok_now) framing <= sat32(framing);
            if (frame_out) begin
                sweeps <= sat32(sweeps);
                if (partial) partials <= sat32(partials);
            end
            if (do_check) begin
                checks <= sat32(checks);
                if (bad0a) e0a <= sat16(e0a);
                if (bad0b) e0b <= sat16(e0b);
                if (bad1a) e1a <= sat16(e1a);
                if (bad1b) e1b <= sat16(e1b);
            end
            if (sclk_cand && {16'd0, run_len} < sclk_min) sclk_min <= {16'd0, run_len};
            tog <= tog | (lv ^ lv_q);
        end
    end

    // ------------------------------------------------------------------
    // LED: a clean cycle lately, and no framing or known-answer error lately
    // ------------------------------------------------------------------
    reg [GOOD_W-1:0] ok_cnt, bad_cnt;

    always @(posedge clk) begin
        if (!rst_n) begin
            ok_cnt  <= '0;
            bad_cnt <= '0;
        end else begin
            if (ok_now && !ka_fail)     ok_cnt  <= '1;
            else if (ok_cnt != '0)      ok_cnt  <= ok_cnt - 1'b1;
            if (heard && (!ok_now || ka_fail)) bad_cnt <= '1;
            else if (bad_cnt != '0)            bad_cnt <= bad_cnt - 1'b1;
        end
    end

    assign good = (ok_cnt != '0) && (bad_cnt == '0);

endmodule

`default_nettype wire
