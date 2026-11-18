//==============================================================================
// usb2_cdr_speed_cfg : every speed-dependent constant of the CDR in one place
//   clk_sys is fixed (CLK_HZ = 480 MHz), so the oversampling ratio follows the
//   bit rate:  OS_LS = CLK_HZ / 1.5 Mbps = 320,   OS_FS = CLK_HZ / 12 Mbps = 40.
//   speed : 2'b00 LS, 2'b01 FS, 2'b10 HS (HS is idle in this block; FS values)
//
//   step_nom    : NCO step = 2^W / OS           (one bit = 2^W phase units)
//   edge_ref    : NCO phase at which an on-time edge must be captured so that
//                 the strobe lands in the centre of the bit
//                   D = OS/2 + PIPE_LAT - 1/2 clk_sys between edge and strobe
//   reset_phase : NCO phase parked in IDLE/LOSS = edge_ref + EDGE_LAT steps
//   lim         : loop clamp, LS 2 %, FS 0.5 % of step_nom
//   se0_qual    : clk_sys of SE0 that qualifies as EOP (= 1 bit time)
//   timeout     : clk_sys allowed in ACQUIRE without lock (= 8 bit times)
//   dg_w        : deglitch width in clk_sys
//==============================================================================
module usb2_cdr_speed_cfg #(
    parameter integer CLK_HZ  = 480_000_000,
    parameter integer BIT_LS  = 1_500_000,
    parameter integer BIT_FS  = 12_000_000,
    parameter integer W       = 32,
    parameter integer LOOP_W  = 24,
    parameter integer DG_LS   = 4,
    parameter integer DG_FS   = 3
)(
    input  wire [1:0]          speed,
    output wire [W-1:0]        step_nom,
    output wire [W-1:0]        edge_ref,
    output wire [W-1:0]        reset_phase,
    output wire [LOOP_W-1:0]   lim,
    output wire [9:0]          se0_qual,
    output wire [11:0]         timeout,
    output wire [4:0]          dg_w
);
    localparam integer PIPE_LAT = 2;     // sample_window latency
    localparam integer EDGE_LAT = 2;     // edge -> edge_q -> FSM state

    localparam integer OS_LS = CLK_HZ / BIT_LS;     // 320
    localparam integer OS_FS = CLK_HZ / BIT_FS;     // 40

    localparam [63:0] TWO_W = 64'd1 << W;

    // ---------------- LS constants
    localparam [63:0] STEP_LS = (TWO_W + OS_LS/2) / OS_LS;
    localparam [63:0] D2_LS   = OS_LS + 2*PIPE_LAT - 1;                 // 2*D
    localparam [63:0] REF_LS  = TWO_W - ((D2_LS * STEP_LS) >> 1);
    localparam [63:0] RST_LS  = REF_LS + EDGE_LAT * STEP_LS;
    localparam [63:0] LIM_LS  = (STEP_LS + 25) / 50;                    // 2.0 %

    // ---------------- FS constants
    localparam [63:0] STEP_FS = (TWO_W + OS_FS/2) / OS_FS;
    localparam [63:0] D2_FS   = OS_FS + 2*PIPE_LAT - 1;
    localparam [63:0] REF_FS  = TWO_W - ((D2_FS * STEP_FS) >> 1);
    localparam [63:0] RST_FS  = REF_FS + EDGE_LAT * STEP_FS;
    localparam [63:0] LIM_FS  = (STEP_FS + 100) / 200;                  // 0.5 %

    wire is_ls = (speed == 2'b00);

    assign step_nom    = is_ls ? STEP_LS[W-1:0] : STEP_FS[W-1:0];
    assign edge_ref    = is_ls ? REF_LS[W-1:0]  : REF_FS[W-1:0];
    assign reset_phase = is_ls ? RST_LS[W-1:0]  : RST_FS[W-1:0];
    assign lim         = is_ls ? LIM_LS[LOOP_W-1:0] : LIM_FS[LOOP_W-1:0];
    assign se0_qual    = is_ls ? OS_LS[9:0]     : OS_FS[9:0];
    assign timeout     = is_ls ? (8*OS_LS)      : (8*OS_FS);
    assign dg_w        = is_ls ? DG_LS[4:0]     : DG_FS[4:0];
endmodule
