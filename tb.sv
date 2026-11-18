//`timescale 1ps/1fs
//==============================================================================
// tb_usb2_cdr : single-module self-checking testbench for usb2_cdr_top
//
// ONE clk_sys for every speed (480 MHz).  The oversampling ratio is set by the
// bit rate:   OS_speed = f_clk / bit_rate
//     LS 1.5 Mbps -> 320 x    FS 12 Mbps -> 40 x    (480 MHz)
// rx_serial / dp / dm are driven at exactly that bit rate (+ ppm offset, jitter),
// asynchronous to clk_sys.  Speeds are run one after another on the same DUT
// (reset + speed change between them), then HS is checked to stay idle.
//
// Packet : idle J -> SYNC KJKJKJKK -> random payload (NRZI + bit stuffing)
//          -> EOP (SE0 SE0 J) -> idle J
// Polarity: rx_serial = D+ (FS J = 1, LS J = 0); rx_serial holds during SE0.
//
// Checks  : recovered bit stream == transmitted bits, locked rises during the
//           packet, locked drops after EOP, strobe count ~ bit count,
//           no strobes / lock on an idle line, HS stays idle.
//==============================================================================
/*module tb_usb2_cdr;

    // ------------------------------------------------------------ settings
    parameter real    CLK_MHZ   = 480.0;
    parameter integer RATIO_LS  = 320;     // clk cycles per LS bit  (480/1.5)
    parameter integer RATIO_FS  = 40;      // clk cycles per FS bit  (480/12)
    parameter integer NPKT      = 8;       // packets per speed
    parameter integer PPM_LS    = 15000;   // LS +/-1.5 %
    parameter integer PPM_FS    = 2500;    // FS +/-0.25 %
    parameter integer JIT_PCT   = 5;       // peak edge jitter, % of clk period

    localparam real    CLK_PS   = 1.0e6 / CLK_MHZ;     // 2083.333 ps
    localparam integer MAXB     = 400;
    localparam integer MAX_LEAD = 3;
    localparam [1:0] SYM_J = 2'd0, SYM_K = 2'd1, SYM_SE0 = 2'd2;

    // ----------------------------------------------------------------- DUT
    reg        clk = 1'b0;
    reg        rstn = 1'b0;
    reg        enable = 1'b1;
    reg [1:0]  speed = 2'b00;
    reg        rx_serial = 1'b1, dp = 1'b1, dm = 1'b0;
    wire       rec_clk, rec_data, locked;
    wire [5:0] quality;

    always #(CLK_PS/2.0) clk = ~clk;

    usb2_cdr_top #(.CLK_HZ(480_000_000)) dut (
        .clk_sys(clk), .rstn(rstn), .enable(enable), .speed(speed),
        .rx_serial(rx_serial), .dp(dp), .dm(dm),
        .recovered_bit_clk(rec_clk), .recovered_data(rec_data),
        .locked(locked), .quality(quality));

    // ------------------------------------------------- per-speed variables
    integer        errors = 0;
    integer        seed   = 32'h1234_5678;
    integer        ratio;            // clk cycles per bit of current speed
    integer        ppm_max;
    real           bit_nom;          // nominal bit time (ps)
    real           bit_ps;           // bit time with ppm offset
    real           t_next;           // start time of next bit
    reg [15:0]     tag;
    wire           j_dp = (speed == 2'b00) ? 1'b0 : 1'b1;   // D+ level in J

    task drive_sym(input [1:0] s);
        begin
            case (s)
                SYM_J:   begin dp =  j_dp; dm = ~j_dp; rx_serial =  j_dp; end
                SYM_K:   begin dp = ~j_dp; dm =  j_dp; rx_serial = ~j_dp; end
                default: begin dp = 1'b0;  dm = 1'b0;  end    // SE0: rx holds
            endcase
        end
    endtask

    task wait_until(input real t);
        real d;
        begin
            d = t - $realtime;
            if (d > 0.0) #(d);
        end
    endtask

    task tx_sym(input [1:0] s);
        integer jmax, jr;
        begin
            jmax = (CLK_PS * JIT_PCT) / 100;                  // ps
            jr   = (jmax == 0) ? 0 : ($random(seed) % (jmax + 1));
            wait_until(t_next + 1.0 * jr);
            drive_sym(s);
            t_next = t_next + bit_ps;
        end
    endtask

    // ------------------------------------------------ packet construction
    reg [MAXB-1:0] txlvl;            // 1 = K, index 0 = first bit sent
    integer        ntx, ones;
    reg            cur_k;

    task add_data_bit(input b);      // NRZI: 0 toggles, 1 holds
        begin
            if (!b) cur_k = ~cur_k;
            txlvl[ntx] = cur_k;
            ntx = ntx + 1;
        end
    endtask

    task add_stuffed_bit(input b);
        begin
            add_data_bit(b);
            if (b) begin
                ones = ones + 1;
                if (ones == 6) begin add_data_bit(1'b0); ones = 0; end
            end else ones = 0;
        end
    endtask

    // pattern: 0 random, 1 all zeros, 2 all ones, 3 alternating
    task build_packet(input integer nbits, input integer pattern);
        integer i;
        reg     b;
        begin
            ntx = 0; cur_k = 1'b0; ones = 0; txlvl = {MAXB{1'b0}};
            for (i = 0; i < 7; i = i + 1) add_stuffed_bit(1'b0);   // SYNC
            add_stuffed_bit(1'b1);                                  // KJKJKJKK
            for (i = 0; i < nbits; i = i + 1) begin
                case (pattern)
                    1:       b = 1'b0;
                    2:       b = 1'b1;
                    3:       b = i[0];
                    default: b = $random(seed);
                endcase
                add_stuffed_bit(b);
            end
        end
    endtask

    // ------------------------------------------------------------- monitors
    // recovered_data changes one clk after the strobe: capture then.
    reg             strobe_d = 1'b0;
    reg [MAXB+15:0] rec = 0;
    integer         nrec = 0;
    reg             capture = 1'b0, watch_idle = 1'b0;
    integer         strobes_idle = 0, locked_idle = 0;
    integer         strobes_hs = 0, locked_hs = 0;
    reg             watch_hs = 1'b0;

    always @(posedge clk) begin
        strobe_d <= rec_clk;
        if (strobe_d && capture && nrec < MAXB + 16) begin
            rec[nrec] <= rec_data;
            nrec      <= nrec + 1;
        end
        if (watch_idle) begin
            if (rec_clk) strobes_idle <= strobes_idle + 1;
            if (locked)  locked_idle  <= locked_idle  + 1;
        end
        if (watch_hs) begin
            if (rec_clk) strobes_hs <= strobes_hs + 1;
            if (locked)  locked_hs  <= locked_hs  + 1;
        end
    end

    // ---------------------------------------------------------- one packet
    integer lead, m, off;
    reg     match;

    task send_packet(input integer nbits, input integer pattern, input integer ppm_i);
        integer i, lock_bit;
        real    pre, settle, post;
        begin
            build_packet(nbits, pattern);
            bit_ps = bit_nom * (1.0 + ppm_i / 1.0e6);

            // times: at least 12 / 6 / 4 bits, and at least 80 / 40 / 40 clks
            pre    = (12.0 * bit_nom > 80.0 * CLK_PS) ? 12.0 * bit_nom : 80.0 * CLK_PS;
            settle = (6.0  * bit_nom > 40.0 * CLK_PS) ? 6.0  * bit_nom : 40.0 * CLK_PS;
            post   = (4.0  * bit_nom > 40.0 * CLK_PS) ? 4.0  * bit_nom : 40.0 * CLK_PS;

            // ---- idle J, observe: no strobes / no lock expected
            strobes_idle = 0; locked_idle = 0; watch_idle = 1'b0;
            t_next = $realtime + pre + (($random(seed) & 32'h7fff) % 2083);
            drive_sym(SYM_J);
            wait_until($realtime + settle);
            watch_idle = 1'b1;
            wait_until(t_next - 3.0 * CLK_PS);
            watch_idle = 1'b0;

            // ---- packet
            nrec = 0; rec = 0; capture = 1'b1; lock_bit = -1;
            wait_until(t_next);
            for (i = 0; i < ntx; i = i + 1) begin
                tx_sym(txlvl[i] ? SYM_K : SYM_J);
                if (locked && lock_bit < 0) lock_bit = i;
            end
            tx_sym(SYM_SE0);
            tx_sym(SYM_SE0);
            tx_sym(SYM_J);
            capture = 1'b0;
            wait_until(t_next + post);

            // ---- checks
            if (strobes_idle != 0 || locked_idle != 0) begin
                errors = errors + 1;
                $display("[%0t] ERR %s: idle line shows strobes=%0d locked_cycles=%0d (expected 0)",
                         $time, tag, strobes_idle, locked_idle);
            end
            if (lock_bit < 0) begin
                errors = errors + 1;
                $display("[%0t] ERR %s: locked never asserted during packet", $time, tag);
            end
            if (locked) begin
                errors = errors + 1;
                $display("[%0t] ERR %s: locked still high after EOP", $time, tag);
            end

            match = 1'b0; lead = 0;
            for (off = 0; off <= MAX_LEAD && !match; off = off + 1) begin
                match = 1'b1;
                for (m = 0; m < ntx; m = m + 1)
                    if (rec[off + m] !== (txlvl[m] ^ j_dp)) match = 1'b0;   // rx level: K = ~D+(J)
                if (match) lead = off;
            end
            if (!match) begin
                errors = errors + 1;
                $display("[%0t] ERR %s ppm=%0d: recovered bits do not match (nrec=%0d ntx=%0d)",
                         $time, tag, ppm_i, nrec, ntx);
                $write("  tx : "); for (m = 0; m < ntx; m = m + 1) $write("%0d", txlvl[m] ^ j_dp); $write("\n");
                $write("  rx : "); for (m = 0; m < nrec && m < ntx + 4; m = m + 1) $write("%0d", rec[m]); $write("\n");
            end else if (lead != 0) begin
                errors = errors + 1;
                $display("[%0t] ERR %s ppm=%0d: %0d extra strobe(s) before first SYNC bit",
                         $time, tag, ppm_i, lead);
            end
            if (nrec < ntx || nrec > ntx + 3 + MAX_LEAD) begin
                errors = errors + 1;
                $display("[%0t] ERR %s: strobe count %0d for %0d bits", $time, tag, nrec, ntx);
            end
            $display("[%0t] %s bits=%0d ppm=%0d match=%0b lead=%0d nstrobes=%0d lock_at_bit=%0d",
                     $time, tag, ntx, ppm_i, match, lead, nrec, lock_bit);
        end
    endtask

    // ----------------------------------------------------- one speed sweep
    task run_speed(input [1:0] spd);
        integer p, ppm_i;
        begin
            // reset with the new speed; idle level depends on speed
            rstn  = 1'b0;
            speed = spd;
            if (spd == 2'b00) begin ratio = RATIO_LS; ppm_max = PPM_LS; tag = "LS"; end
            else              begin ratio = RATIO_FS; ppm_max = PPM_FS; tag = "FS"; end
            bit_nom = 1.0 * CLK_PS * ratio;          // bit rate = f_clk / ratio
            bit_ps  = bit_nom;
            #1 drive_sym(SYM_J);
            repeat (8) @(posedge clk);
            rstn = 1'b1;
            repeat (8) @(posedge clk);

            $display("---- %s : %0d clk per bit, ppm range +/-%0d", tag, ratio, ppm_max);
            for (p = 0; p < NPKT; p = p + 1) begin
                case (p)
                    0:       ppm_i = 0;
                    1:       ppm_i =  ppm_max;
                    2:       ppm_i = -ppm_max;
                    default: ppm_i = $random(seed) % (ppm_max + 1);
                endcase
                send_packet(16 + (p * 8) % 64, p % 4, ppm_i);
            end
        end
    endtask

    // ---------------------------------------------------------------- HS
    task run_hs;
        integer n;
        begin
            rstn = 1'b0; speed = 2'b10; #1 drive_sym(SYM_J);
            repeat (8) @(posedge clk);
            rstn = 1'b1;
            strobes_hs = 0; locked_hs = 0; watch_hs = 1'b1;
            for (n = 0; n < 2000; n = n + 1) begin
                @(posedge clk); rx_serial = $random(seed);
            end
            watch_hs = 1'b0;
            if (strobes_hs != 0 || locked_hs != 0) begin
                errors = errors + 1;
                $display("ERR HS: strobes=%0d locked_cycles=%0d, expected 0", strobes_hs, locked_hs);
            end else $display("---- HS idle check OK");
        end
    endtask

    // ---------------------------------------------------------------- main
    initial begin
        run_speed(2'b00);       // LS
        run_speed(2'b01);       // FS
        run_hs;
        $display("--------------------------------------------------");
        if (errors == 0) $display("PASS tb_usb2_cdr");
        else             $display("FAIL tb_usb2_cdr errors=%0d", errors);
        $finish;
    end

    initial begin               // watchdog
        #(64'd10_000_000_000);       // 10 ms
        $display("FAIL tb_usb2_cdr : timeout");
        $finish;
    end


initial begin
    $shm_open("wave.shm");
    $shm_probe("ACTMF");
end


endmodule*/


//`timescale 1ps/1fs
//==============================================================================
// tb_usb2_cdr : single-module self-checking testbench for usb2_cdr_top
//
// ONE clk_sys for every speed (480 MHz).  The oversampling ratio is set by the
// bit rate:   OS_speed = f_clk / bit_rate
//     LS 1.5 Mbps -> 320 x    FS 12 Mbps -> 40 x    (480 MHz)
// rx_serial / dp / dm are driven at exactly that bit rate (+ ppm offset, jitter),
// asynchronous to clk_sys.  Speeds are run one after another on the same DUT
// (reset + speed change between them), then HS is checked to stay idle.
//
// Packet : idle J -> SYNC KJKJKJKK -> random payload (NRZI + bit stuffing)
//          -> EOP (SE0 SE0 J) -> idle J
// Polarity: rx_serial = D+ (FS J = 1, LS J = 0); rx_serial holds during SE0.
//
// Checks  : recovered bit stream == transmitted bits, locked rises during the
//           packet, locked drops after EOP, strobe count ~ bit count,
//           no strobes / lock on an idle line, HS stays idle.
//==============================================================================
module tb_usb2_cdr;

    // ------------------------------------------------------------ settings
    parameter real    CLK_MHZ   = 480.0;
    parameter integer RATIO_LS  = 320;     // clk cycles per LS bit  (480/1.5)
    parameter integer RATIO_FS  = 40;      // clk cycles per FS bit  (480/12)
    parameter integer NPKT      = 8;       // packets per speed
    parameter integer PPM_LS    = 15000;   // LS +/-1.5 %
    parameter integer PPM_FS    = 2500;    // FS +/-0.25 %
    parameter integer JIT_PCT   = 5;       // peak edge jitter, % of clk period

    localparam real    CLK_PS   = 1.0e6 / CLK_MHZ;     // 2083.333 ps
    localparam integer MAXB     = 400;
    localparam integer MAX_LEAD = 3;
    localparam [1:0] SYM_J = 2'd0, SYM_K = 2'd1, SYM_SE0 = 2'd2;

    // ----------------------------------------------------------------- DUT
    reg        clk = 1'b0;
    reg        rstn = 1'b0;
    reg        enable = 1'b1;
    reg [1:0]  speed = 2'b00;
    reg        rx_serial = 1'b0, dp = 1'b0, dm = 1'b1;
    wire       rec_clk, rec_data, locked;
    wire [5:0] quality;

    always #(CLK_PS/2.0) clk = ~clk;

    usb2_cdr_top #(.CLK_HZ(480_000_000)) dut (
        .clk_sys(clk), .rstn(rstn), .enable(enable), .speed(speed),
        .rx_serial(rx_serial), .dp(dp), .dm(dm),
        .recovered_bit_clk(rec_clk), .recovered_data(rec_data),
        .locked(locked), .quality(quality));

    // ------------------------------------------------- per-speed variables
    integer        errors = 0;
    integer        seed   = 32'h1234_5678;
    integer        ratio;            // clk cycles per bit of current speed
    integer        ppm_max;
    real           bit_nom;          // nominal bit time (ps)
    real           bit_ps;           // bit time with ppm offset
    real           t_next;           // start time of next bit
    reg [15:0]     tag;
    wire           j_dp = (speed == 2'b00) ? 1'b0 : 1'b1;   // D+ level in J

    task drive_sym(input [1:0] s);
        begin
            case (s)
                SYM_J:   begin dp =  j_dp; dm = ~j_dp; rx_serial =  j_dp; end
                SYM_K:   begin dp = ~j_dp; dm =  j_dp; rx_serial = ~j_dp; end
                default: begin dp = 1'b0;  dm = 1'b0;  end    // SE0: rx holds
            endcase
        end
    endtask

    task wait_until(input real t);
        real d;
        begin
            d = t - $realtime;
            if (d > 0.0) #(d);
        end
    endtask

    task tx_sym(input [1:0] s);
        integer jmax, jr;
        begin
            jmax = (CLK_PS * JIT_PCT) / 100;                  // ps
            jr   = (jmax == 0) ? 0 : ($random(seed) % (jmax + 1));
            wait_until(t_next + 1.0 * jr);
            drive_sym(s);
            t_next = t_next + bit_ps;
        end
    endtask

    // ------------------------------------------------ packet construction
    reg [MAXB-1:0] txlvl;            // 1 = K, index 0 = first bit sent
    integer        ntx, ones;
    reg            cur_k;

    task add_data_bit(input b);      // NRZI: 0 toggles, 1 holds
        begin
            if (!b) cur_k = ~cur_k;
            txlvl[ntx] = cur_k;
            ntx = ntx + 1;
        end
    endtask

    task add_stuffed_bit(input b);
        begin
            add_data_bit(b);
            if (b) begin
                ones = ones + 1;
                if (ones == 6) begin add_data_bit(1'b0); ones = 0; end
            end else ones = 0;
        end
    endtask

    // pattern: 0 random, 1 all zeros, 2 all ones, 3 alternating
    task build_packet(input integer nbits, input integer pattern);
        integer i;
        reg     b;
        begin
            ntx = 0; cur_k = 1'b0; ones = 0; txlvl = {MAXB{1'b0}};
            for (i = 0; i < 7; i = i + 1) add_stuffed_bit(1'b0);   // SYNC
            add_stuffed_bit(1'b1);                                  // KJKJKJKK
            for (i = 0; i < nbits; i = i + 1) begin
                case (pattern)
                    1:       b = 1'b0;
                    2:       b = 1'b1;
                    3:       b = i[0];
                    default: b = $random(seed);
                endcase
                add_stuffed_bit(b);
            end
        end
    endtask

    // ------------------------------------------------------------- monitors
    // recovered_data changes one clk after the strobe: capture then.
    reg             strobe_d = 1'b0;
    reg [MAXB+15:0] rec = 0;
    integer         nrec = 0;
    reg             capture = 1'b0, watch_idle = 1'b0;
    reg             exp_bit = 1'b0, bit_err = 1'b0;   // debug: add to waves
    integer         strobes_idle = 0, locked_idle = 0;
    integer         strobes_hs = 0, locked_hs = 0;
    reg             watch_hs = 1'b0;

    always @(posedge clk) begin
        strobe_d <= rec_clk;
        if (strobe_d && capture && nrec < MAXB + 16) begin
            rec[nrec] <= rec_data;
            if (nrec < ntx) begin                            // only real packet bits
                exp_bit <= txlvl[nrec] ^ j_dp;               // expected level of this bit
                bit_err <= (rec_data !== (txlvl[nrec] ^ j_dp));
            end else
                bit_err <= 1'b0;                             // extra strobe during EOP
            nrec      <= nrec + 1;
        end
        if (watch_idle) begin
            if (rec_clk) strobes_idle <= strobes_idle + 1;
            if (locked)  locked_idle  <= locked_idle  + 1;
        end
        if (watch_hs) begin
            if (rec_clk) strobes_hs <= strobes_hs + 1;
            if (locked)  locked_hs  <= locked_hs  + 1;
        end
    end

    // ---------------------------------------------------------- one packet
    integer lead, m, off;
    reg     match;

    task send_packet(input integer nbits, input integer pattern, input integer ppm_i);
        integer i, lock_bit;
        real    pre, settle, post;
        begin
            build_packet(nbits, pattern);
            bit_ps = bit_nom * (1.0 + ppm_i / 1.0e6);

            // times: at least 12 / 6 / 4 bits, and at least 80 / 40 / 40 clks
            pre    = (12.0 * bit_nom > 80.0 * CLK_PS) ? 12.0 * bit_nom : 80.0 * CLK_PS;
            settle = (6.0  * bit_nom > 40.0 * CLK_PS) ? 6.0  * bit_nom : 40.0 * CLK_PS;
            post   = (4.0  * bit_nom > 40.0 * CLK_PS) ? 4.0  * bit_nom : 40.0 * CLK_PS;

            // ---- idle J, observe: no strobes / no lock expected
            strobes_idle = 0; locked_idle = 0; watch_idle = 1'b0;
            t_next = $realtime + pre + (($random(seed) & 32'h7fff) % 2083);
            drive_sym(SYM_J);
            wait_until($realtime + settle);
            watch_idle = 1'b1;
            wait_until(t_next - 3.0 * CLK_PS);
            watch_idle = 1'b0;

            // ---- packet
            nrec = 0; rec = 0; capture = 1'b1; lock_bit = -1;
            wait_until(t_next);
            for (i = 0; i < ntx; i = i + 1) begin
                tx_sym(txlvl[i] ? SYM_K : SYM_J);
                if (locked && lock_bit < 0) lock_bit = i;
            end
            tx_sym(SYM_SE0);
            tx_sym(SYM_SE0);
            tx_sym(SYM_J);
            capture = 1'b0;
            wait_until(t_next + post);

            // ---- checks
            if (strobes_idle != 0 || locked_idle != 0) begin
                errors = errors + 1;
                $display("[%0t] ERR %s: idle line shows strobes=%0d locked_cycles=%0d (expected 0)",
                         $time, tag, strobes_idle, locked_idle);
            end
            if (lock_bit < 0) begin
                errors = errors + 1;
                $display("[%0t] ERR %s: locked never asserted during packet", $time, tag);
            end
            if (locked) begin
                errors = errors + 1;
                $display("[%0t] ERR %s: locked still high after EOP", $time, tag);
            end

            match = 1'b0; lead = 0;
            for (off = 0; off <= MAX_LEAD && !match; off = off + 1) begin
                match = 1'b1;
                for (m = 0; m < ntx; m = m + 1)
                    if (rec[off + m] !== (txlvl[m] ^ j_dp)) match = 1'b0;   // rx level: K = ~D+(J)
                if (match) lead = off;
            end
            if (!match) begin
                errors = errors + 1;
                $display("[%0t] ERR %s ppm=%0d: recovered bits do not match (nrec=%0d ntx=%0d)",
                         $time, tag, ppm_i, nrec, ntx);
                $write("  tx : "); for (m = 0; m < ntx; m = m + 1) $write("%0d", txlvl[m] ^ j_dp); $write("\n");
                $write("  rx : "); for (m = 0; m < nrec && m < ntx + 4; m = m + 1) $write("%0d", rec[m]); $write("\n");
            end else if (lead != 0) begin
                errors = errors + 1;
                $display("[%0t] ERR %s ppm=%0d: %0d extra strobe(s) before first SYNC bit",
                         $time, tag, ppm_i, lead);
            end
            if (nrec < ntx || nrec > ntx + 3 + MAX_LEAD) begin
                errors = errors + 1;
                $display("[%0t] ERR %s: strobe count %0d for %0d bits", $time, tag, nrec, ntx);
            end
            $display("[%0t] %s bits=%0d ppm=%0d match=%0b lead=%0d nstrobes=%0d lock_at_bit=%0d",
                     $time, tag, ntx, ppm_i, match, lead, nrec, lock_bit);
        end
    endtask

    // ----------------------------------------------------- one speed sweep
    task run_speed(input [1:0] spd);
        integer p, ppm_i;
        begin
            // reset with the new speed; idle level depends on speed
            rstn  = 1'b0;
            speed = spd;
            if (spd == 2'b00) begin ratio = RATIO_LS; ppm_max = PPM_LS; tag = "LS"; end
            else              begin ratio = RATIO_FS; ppm_max = PPM_FS; tag = "FS"; end
            bit_nom = 1.0 * CLK_PS * ratio;          // bit rate = f_clk / ratio
            bit_ps  = bit_nom;
            #1 drive_sym(SYM_J);
            repeat (8) @(posedge clk);
            rstn = 1'b1;
            repeat (8) @(posedge clk);

            $display("---- %s : %0d clk per bit, ppm range +/-%0d", tag, ratio, ppm_max);
            for (p = 0; p < NPKT; p = p + 1) begin
                case (p)
                    0:       ppm_i = 0;
                    1:       ppm_i =  ppm_max;
                    2:       ppm_i = -ppm_max;
                    default: ppm_i = $random(seed) % (ppm_max + 1);
                endcase
                send_packet(16 + (p * 8) % 64, p % 4, ppm_i);
            end
        end
    endtask

    // ---------------------------------------------------------------- HS
    task run_hs;
        integer n;
        begin
            rstn = 1'b0; speed = 2'b10; #1 drive_sym(SYM_J);
            repeat (8) @(posedge clk);
            rstn = 1'b1;
            strobes_hs = 0; locked_hs = 0; watch_hs = 1'b1;
            for (n = 0; n < 2000; n = n + 1) begin
                @(posedge clk); rx_serial = $random(seed);
            end
            watch_hs = 1'b0;
            if (strobes_hs != 0 || locked_hs != 0) begin
                errors = errors + 1;
                $display("ERR HS: strobes=%0d locked_cycles=%0d, expected 0", strobes_hs, locked_hs);
            end else $display("---- HS idle check OK");
        end
    endtask

    // ---------------------------------------------------------------- main
    initial begin
        run_speed(2'b00);       // LS
        run_speed(2'b01);       // FS
        run_hs;
        $display("--------------------------------------------------");
        if (errors == 0) $display("PASS tb_usb2_cdr");
        else             $display("FAIL tb_usb2_cdr errors=%0d", errors);
        $finish;
    end

    initial begin               // small VCD (no clk_sys) for off-line comparison
        $dumpfile("cdr_min.vcd");
        $dumpvars(0, rx_serial, dp, dm, speed, rec_clk, rec_data, locked, exp_bit, bit_err);
    end

    initial begin               // watchdog
        #(64'd10_000_000_000);       // 10 ms
        $display("FAIL tb_usb2_cdr : timeout");
        $finish;
    end
initial begin
    $shm_open("wave.shm");
    $shm_probe("ACTMF");
end
endmodule
