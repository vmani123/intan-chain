// ============================================================================
// tb_frame_builder.sv -- does a datagram land in memory exactly as the contract
// says, whatever order the channels arrive in?
// ----------------------------------------------------------------------------
// The checks that matter here, and why:
//
//   OUT-OF-ORDER CHANNELS. spi_master emits 0, 32, 64, 96, 1, 33, 65, 97, ...
//   A builder that indexed by an arrival counter would produce a frame with a
//   correct header, a correct length and permuted samples -- which on the host
//   looks like broken analogue, not like a framing bug. So every sample here is
//   fed in a different order, including descending and a stride permutation,
//   and all of them must land at slot_base + frame*138 + 10 + ch.
//
//   DROP WHOLE DATAGRAMS, NOT FRAGMENTS. With `stall` held high the builder
//   must not touch the slot at all, and `seq` must still advance, so the loss
//   shows up on the host as a gap rather than as silence.
//
//   t_tx MUST BE ZERO in memory. qspi_master patches it on the way out; if the
//   builder wrote anything there the patch would be untestable.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_frame_builder;

    localparam int N_CH      = 128;
    localparam int HDR_WORDS = 10;
    localparam int FPD       = 5;
    localparam int NSLOT     = 4;
    localparam int FRAME_W   = HDR_WORDS + N_CH;         // 138
    localparam int SLOT_W    = FPD * FRAME_W;            // 690
    localparam int AW        = $clog2(NSLOT * SLOT_W);   // 12

    integer errors = 0;
    integer checks = 0;

    task automatic chk(input cond, input [8*60-1:0] name);
        begin
            checks = checks + 1;
            if (cond) $display("  pass  %0s", name);
            else begin errors = errors + 1; $display("  FAIL  %0s", name); end
        end
    endtask

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst_n = 1'b0;
    reg clear = 1'b0;

    reg [31:0] tick = 32'h0000_1000;   // not zero, so a stuck t_cap is visible
    always @(posedge clk) tick <= tick + 32'd1;

    reg                     s_valid = 1'b0;
    reg [$clog2(N_CH)-1:0]  s_ch    = '0;
    reg signed [15:0]       s_value = 16'd0;
    reg                     s_done  = 1'b0;
    reg [7:0]               flags   = 8'h03;
    reg [7:0]               ftype   = 8'h00;
    reg                     stall   = 1'b0;
    reg [$clog2(NSLOT)-1:0] slot_wr = '0;

    wire            wr_en, commit, overrun, busy;
    wire [AW-1:0]   wr_addr;
    wire [15:0]     wr_data;
    wire [31:0]     frames;

    frame_builder #(
        .N_CH(N_CH), .HDR_WORDS(HDR_WORDS),
        .FRAMES_PER_DGRAM(FPD), .NSLOT(NSLOT)
    ) dut (
        .clk(clk), .rst_n(rst_n), .clear(clear),
        .valid(s_valid), .ch(s_ch), .value(s_value), .sample_done(s_done),
        .flags(flags), .frame_type(ftype), .tick(tick),
        .stall(stall), .slot_wr(slot_wr),
        .wr_en(wr_en), .wr_addr(wr_addr), .wr_data(wr_data),
        .commit(commit), .overrun(overrun), .frames(frames), .busy(busy)
    );

    reg           rd_en = 1'b0;
    reg [AW-1:0]  rd_addr = '0;
    wire [15:0]   rd_data;

    dgram_mem #(.NSLOT(NSLOT), .SLOT_WORDS(SLOT_W)) mem (
        .clk(clk),
        .wr_en(wr_en), .wr_addr(wr_addr), .wr_data(wr_data),
        .rd_en(rd_en), .rd_addr(rd_addr), .rd_data(rd_data)
    );

    integer commits = 0, overruns = 0, busy_cycles = 0;
    always @(posedge clk) if (rst_n) begin
        if (commit)  commits  = commits + 1;
        if (overrun) overruns = overruns + 1;
        if (busy)    busy_cycles = busy_cycles + 1;
    end

    // ---- stimulus helpers ----------------------------------------------

    // Distinct for every (sample, channel) pair, and not a small number, so a
    // word landing at the wrong address cannot coincidentally match.
    function automatic [15:0] val_of(input integer s, input integer c);
        val_of = 16'(32'hA5A5 + s * 1024 + c * 3);
    endfunction

    // 0 = the order spi_master really emits, 1 = descending, 2 = ascending,
    // 3 = a stride permutation (37 is odd, so it visits every channel once).
    function automatic integer ch_at(input integer order, input integer i);
        case (order)
            0:       ch_at = (i / 4) + 32 * (i % 4);
            1:       ch_at = N_CH - 1 - i;
            2:       ch_at = i;
            default: ch_at = (i * 37) % N_CH;
        endcase
    endfunction

    reg [31:0] exp_tcap [0:63];
    integer    nsamp = 0;

    task automatic feed_sample(input integer s, input integer order);
        integer i, c;
        begin
            for (i = 0; i < N_CH; i = i + 1) begin
                // A bit-select on a function call is not legal in Icarus, so
                // the channel index goes through a plain variable first.
                c = ch_at(order, i);
                @(negedge clk);
                s_valid = 1'b1;
                s_ch    = c[$clog2(N_CH)-1:0];
                s_value = val_of(s, c);
                if (i == N_CH - 1) begin
                    s_done       = 1'b1;
                    // The builder latches `tick` on the edge that follows this
                    // drive, and tick updates on that same edge, so the value
                    // standing here now is the one it will capture.
                    exp_tcap[s]  = tick;
                end
            end
            @(negedge clk);
            s_valid = 1'b0;
            s_done  = 1'b0;
            s_value = 16'd0;
            repeat (14) @(negedge clk);    // 10 header words plus slack
        end
    endtask

    task automatic mem_rd(input integer a, output reg [15:0] d);
        begin
            @(negedge clk);
            rd_en   = 1'b1;
            rd_addr = a[AW-1:0];
            @(posedge clk); #0.1;
            d = rd_data;
            @(negedge clk);
            rd_en = 1'b0;
        end
    endtask

    // ---- checking -------------------------------------------------------
    integer hdr_bad, pay_bad;

    task automatic check_frame(input integer slot, input integer f,
                               input integer s_index, input integer exp_seq,
                               input integer order);
        integer base, c, a;
        reg [15:0] w;
        begin
            base = slot * SLOT_W + f * FRAME_W;
            mem_rd(base + 0, w); if (w !== 16'hA55A)           hdr_bad = hdr_bad + 1;
            mem_rd(base + 1, w); if (w !== {flags, ftype})     hdr_bad = hdr_bad + 1;
            mem_rd(base + 2, w); if (w !== exp_seq[15:0])      hdr_bad = hdr_bad + 1;
            mem_rd(base + 3, w); if (w !== exp_seq[31:16])     hdr_bad = hdr_bad + 1;
            mem_rd(base + 4, w); if (w !== exp_tcap[s_index][15:0])  hdr_bad = hdr_bad + 1;
            mem_rd(base + 5, w); if (w !== exp_tcap[s_index][31:16]) hdr_bad = hdr_bad + 1;
            mem_rd(base + 6, w); if (w !== 16'd0)              hdr_bad = hdr_bad + 1;
            mem_rd(base + 7, w); if (w !== 16'd0)              hdr_bad = hdr_bad + 1;
            mem_rd(base + 8, w); if (w !== 16'd128)            hdr_bad = hdr_bad + 1;
            mem_rd(base + 9, w); if (w !== 16'd256)            hdr_bad = hdr_bad + 1;

            for (c = 0; c < N_CH; c = c + 1) begin
                a = base + HDR_WORDS + c;
                mem_rd(a, w);
                if (w !== val_of(s_index, c)) begin
                    pay_bad = pay_bad + 1;
                    if (pay_bad <= 5)
                        $display("    slot %0d frame %0d ch %0d: got %04h want %04h",
                                 slot, f, c, w, val_of(s_index, c));
                end
            end
            if (order == 0) ;   // order is recorded only for the log below
        end
    endtask

    integer f, s, bcount;
    reg [15:0] w0, w1;

    initial begin
        // A full dump of tb_rx_top is hundreds of megabytes, so both of the new
        // testbenches only dump when asked: vvp <file>.vvp +vcd
        if ($test$plusargs("vcd")) begin
            $dumpfile("fpga/sim/tb_frame_builder.vcd");
            $dumpvars(0, tb_frame_builder);
        end
        $display("\n=== frame_builder: datagram layout in slot memory ===");

        rst_n = 1'b0;
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(negedge clk);

        // -------- datagram A into slot 0, every sample in a different order --
        slot_wr = 2'd0;
        stall   = 1'b0;
        for (s = 0; s < FPD; s = s + 1) feed_sample(s, s % 4);
        nsamp = FPD;

        chk(commits  == 1,  "one commit after 5 frames");
        chk(overruns == 0,  "no overrun while a slot is free");
        chk(frames   == 32'd5, "frames counter reads 5");
        chk(busy_cycles == FPD * HDR_WORDS, "busy is high for exactly 10 header words per frame");

        hdr_bad = 0; pay_bad = 0;
        for (f = 0; f < FPD; f = f + 1) check_frame(0, f, f, f, f % 4);
        chk(hdr_bad == 0, "every header field correct in all 5 frames");
        chk(pay_bad == 0, "every payload word at slot_base + frame*138 + 10 + ch");

        // -------- the same slot must not be touched while stalled -----------
        $display("\n=== overrun: no free slot ===");
        mem_rd(0 * SLOT_W + 0 * FRAME_W + HDR_WORDS + 0, w0);
        stall  = 1'b1;
        bcount = commits;
        for (s = FPD; s < 2*FPD; s = s + 1) feed_sample(s, 0);

        chk(commits  == bcount, "no commit while stalled");
        chk(overruns == 1,      "one overrun pulse per dropped datagram");
        chk(frames   == 32'd5,  "frames counter did not advance on a drop");
        mem_rd(0 * SLOT_W + 0 * FRAME_W + HDR_WORDS + 0, w1);
        chk(w1 === w0, "the committed slot was not overwritten while stalled");

        // -------- and the loss must be visible as a gap in seq --------------
        $display("\n=== seq keeps counting across the dropped datagram ===");
        stall   = 1'b0;
        slot_wr = 2'd1;
        for (s = 2*FPD; s < 3*FPD; s = s + 1) feed_sample(s, 3 - (s % 4));

        chk(commits == bcount + 1, "commit resumes once a slot is free");
        chk(frames  == 32'd10,     "frames counter reads 10");

        hdr_bad = 0; pay_bad = 0;
        for (f = 0; f < FPD; f = f + 1)
            check_frame(1, f, 2*FPD + f, 2*FPD + f, 0);
        chk(hdr_bad == 0, "slot 1 headers correct, seq resumes at 10");
        chk(pay_bad == 0, "slot 1 payload correct");

        // -------- clear --------------------------------------------------
        @(negedge clk); clear = 1'b1;
        @(negedge clk); clear = 1'b0;
        repeat (2) @(negedge clk);
        chk(frames == 32'd0, "clear zeroes the frames counter");

        $display("\n=== %0d error(s) over %0d checks ===", errors, checks);
        if (errors == 0) $display("ALL CHECKS PASSED");
        else             $display("THERE WERE FAILURES");
        $finish;
    end

    initial begin
        #10_000_000;
        $display("TIMEOUT");
        $display("THERE WERE FAILURES");
        $finish;
    end

endmodule

`default_nettype wire
