module usb2_cdr_top #(
    parameter integer CLK_HZ    = 480_000_000,
    parameter integer BIT_LS    = 1_500_000,
    parameter integer BIT_FS    = 12_000_000,
    parameter integer ACC_WIDTH = 32,
    parameter integer TED_W     = 6,
    parameter integer LOOP_W    = 24,
    parameter integer DG_LS     = 4,
    parameter integer DG_FS     = 3,
    parameter integer KP_HI     = 11,
    parameter integer KP_LO     = 12,
    parameter integer KI_HI     = 12,
    parameter integer KI_LO     = 13,
    parameter integer P_HOLD    = 1,
    parameter integer EDGE_MIN  = 4,
    parameter integer M_GOOD    = 3,
    parameter integer TED_THR   = 8,
    parameter integer TED_BAD   = 24,
    parameter integer BAD_MAX   = 3,
    parameter integer IDLE_QUAL = 8
)(
    input  wire       clk_sys,
    input  wire       rstn,
    input  wire       enable,
    input  wire [1:0] speed,
    input  wire       rx_serial,
    input  wire       dp,
    input  wire       dm,
 
    output wire       recovered_bit_clk,   // 1-cycle bit-centre strobe
    output wire       recovered_data,      // NRZI line level
    output wire       locked,
    output wire [TED_W-1:0] quality
);
    wire en_eff = enable & (speed != 2'b10);
 
    // ---------------- per-speed constants (selected by 'speed')
    //   clk_sys is fixed, so OS follows the bit rate: LS 320x, FS 40x.
    //   step_nom : NCO step = 2^W / OS
    //   edge_ref : NCO phase of an on-time edge, D = OS/2 + PIPE_LAT - 1/2 clk
    //   reset_ph : phase parked in IDLE/LOSS = edge_ref + EDGE_LAT steps
    //   lim      : loop clamp, LS 2 %, FS 0.5 % of step_nom
    //   se0_qual : SE0 clk_sys that qualifies EOP (1 bit);  timeout: 8 bits
    localparam integer PIPE_LAT = 2;
    localparam integer EDGE_LAT = 2;
    localparam integer OS_LS    = CLK_HZ / BIT_LS;      // 320
    localparam integer OS_FS    = CLK_HZ / BIT_FS;      // 40
    localparam [63:0]  TWO_W    = 64'd1 << ACC_WIDTH;
 
    localparam [63:0] STEP_LS = (TWO_W + OS_LS/2) / OS_LS;
    localparam [63:0] REF_LS  = TWO_W - (((OS_LS + 2*PIPE_LAT - 1) * STEP_LS) >> 1);
    localparam [63:0] RST_LS  = REF_LS + EDGE_LAT * STEP_LS;
    localparam [63:0] LIM_LS  = (STEP_LS + 25) / 50;
 
    localparam [63:0] STEP_FS = (TWO_W + OS_FS/2) / OS_FS;
    localparam [63:0] REF_FS  = TWO_W - (((OS_FS + 2*PIPE_LAT - 1) * STEP_FS) >> 1);
    localparam [63:0] RST_FS  = REF_FS + EDGE_LAT * STEP_FS;
    localparam [63:0] LIM_FS  = (STEP_FS + 100) / 200;
 
    wire is_ls = (speed == 2'b00);
    wire [ACC_WIDTH-1:0] step_nom    = is_ls ? STEP_LS[ACC_WIDTH-1:0] : STEP_FS[ACC_WIDTH-1:0];
    wire [ACC_WIDTH-1:0] edge_ref    = is_ls ? REF_LS[ACC_WIDTH-1:0]  : REF_FS[ACC_WIDTH-1:0];
    wire [ACC_WIDTH-1:0] reset_phase = is_ls ? RST_LS[ACC_WIDTH-1:0]  : RST_FS[ACC_WIDTH-1:0];
    wire [LOOP_W-1:0]    lim         = is_ls ? LIM_LS[LOOP_W-1:0]     : LIM_FS[LOOP_W-1:0];
    wire [9:0]           se0_qual    = is_ls ? OS_LS : OS_FS;
    wire [11:0]          timeout     = is_ls ? 8*OS_LS : 8*OS_FS;
    wire [4:0]           dg_w        = is_ls ? DG_LS : DG_FS;
 
    // ---------------- nets
    wire rx_sync, rx_s, d_p_sync, d_n_sync, se0, se1, idle, rx_s_d;
    wire [3:0] sample_shift, sample_window;
    wire [ACC_WIDTH-1:0] phase, phase_value, step_eff;
    wire signed [ACC_WIDTH-1:0] error;
    wire edge_w, ted_valid, bit_strobe, sample_tap;
    wire signed [TED_W-1:0] ted_value;
    wire signed [LOOP_W-1:0] loop_delta, p_term, i_delta, i_state;
    wire acc_reset, loop_reset, gain_sel;
    wire [1:0] fsm_state;
 
    usb2_cdr_sync_deglitch u_sync (
        .clk_sys(clk_sys), .rstn(rstn), .rx_serial(rx_serial), .d_p(dp), .d_n(dm),
        .speed(speed), .dg_w(dg_w),
        .rx_sync(rx_sync), .rx_s(rx_s), .d_p_sync(d_p_sync), .d_n_sync(d_n_sync),
        .se0(se0), .se1(se1), .idle(idle));
 
    usb2_cdr_oversampler #(.WIN(4)) u_os (
        .clk_sys(clk_sys), .rstn(rstn), .rx_s(rx_s),
        .sample_shift(sample_shift), .sample_window(sample_window), .rx_s_d(rx_s_d));
 
    usb2_cdr_error_detection #(.W(ACC_WIDTH), .TED_W(TED_W)) u_ted (
        .clk_sys(clk_sys), .rstn(rstn), .rx_s(rx_s), .rx_s_d(rx_s_d),
        .phase(phase), .edge_ref(edge_ref),
        .edge_o(edge_w), .phase_value(phase_value), .error(error),
        .ted_value(ted_value), .ted_valid(ted_valid));
 
    // loop frozen while the accumulator is parked (IDLE / LOSS)
    wire lf_valid = ted_valid & ~acc_reset;
 
    usb2_cdr_loop_filter #(.TED_W(TED_W), .LOOP_W(LOOP_W), .STEP_W(ACC_WIDTH),
                           .KP_HI(KP_HI), .KP_LO(KP_LO), .KI_HI(KI_HI), .KI_LO(KI_LO),
                           .P_HOLD(P_HOLD)) u_lf (
        .clk_sys(clk_sys), .rstn(rstn), .ted_value(ted_value), .ted_valid(lf_valid),
        .gain_sel(gain_sel), .loop_reset(loop_reset),
        .step_nom(step_nom), .lim(lim),
        .loop_delta(loop_delta), .p_term(p_term), .i_delta(i_delta), .i_state(i_state));
 
    usb2_cdr_phase_acc #(.W(ACC_WIDTH), .LOOP_W(LOOP_W)) u_nco (
        .clk_sys(clk_sys), .rstn(rstn), .acc_reset(acc_reset | ~en_eff),
        .loop_delta(loop_delta), .step_nom(step_nom), .lim(lim),
        .reset_phase(reset_phase),
        .phase(phase), .bit_strobe(bit_strobe), .step_eff(step_eff));
 
    usb2_cdr_sampler #(.W(ACC_WIDTH), .WIN(4)) u_smp (
        .clk_sys(clk_sys), .rstn(rstn), .sample_window(sample_window),
        .phase(phase), .step_eff(step_eff), .bit_strobe(bit_strobe),
        .sample_tap(sample_tap), .recovered_bit_clk(recovered_bit_clk),
        .recovered_data(recovered_data));
 
    usb2_cdr_lock_fsm #(.TED_W(TED_W), .EDGE_MIN(EDGE_MIN), .M_GOOD(M_GOOD),
                        .TED_THR(TED_THR), .TED_BAD(TED_BAD), .BAD_MAX(BAD_MAX),
                        .IDLE_QUAL(IDLE_QUAL)) u_fsm (
        .clk_sys(clk_sys), .rstn(rstn), .enable(en_eff),
        .se0(se0), .se1(se1), .idle(idle), .se0_qual(se0_qual), .timeout(timeout),
        .ted_value(ted_value), .ted_valid(ted_valid),
        .acc_reset(acc_reset), .loop_reset(loop_reset), .gain_sel(gain_sel),
        .locked(locked), .state_o(fsm_state));
 
    assign quality = ted_value;
endmodule
