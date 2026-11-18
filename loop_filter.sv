module usb2_cdr_loop_filter #(
    parameter integer TED_W  = 6,
    parameter integer LOOP_W = 24,
    parameter integer STEP_W = 32,
    parameter integer KP_HI  = 11,
    parameter integer KP_LO  = 12,
    parameter integer KI_HI  = 12,
    parameter integer KI_LO  = 13,
    parameter integer P_HOLD = 1
)(
    input  wire                     clk_sys,
    input  wire                     rstn,
    input  wire signed [TED_W-1:0]  ted_value,
    input  wire                     ted_valid,
    input  wire                     gain_sel,       // 1 = high gain (ACQUIRE)
    input  wire                     loop_reset,
    input  wire [STEP_W-1:0]        step_nom,
    input  wire [LOOP_W-1:0]        lim,
    output wire signed [LOOP_W-1:0] loop_delta,
    output wire signed [LOOP_W-1:0] p_term,
    output wire signed [LOOP_W-1:0] i_delta,
    output reg  signed [LOOP_W-1:0] i_state
);
    localparam integer AW = STEP_W + TED_W + 2;     // product never overflows
 
    function signed [AW-1:0] sat;
        input signed [AW-1:0] x;
        input signed [AW-1:0] l;
        begin
            if (x > l)       sat = l;
            else if (x < -l) sat = -l;
            else             sat = x;
        end
    endfunction
 
    wire signed [AW-1:0] lim_x  = {{(AW-LOOP_W){1'b0}}, lim};
    wire signed [AW-1:0] ted_x  = ted_value;
    wire signed [AW-1:0] step_x = {{(AW-STEP_W){1'b0}}, step_nom};
    wire signed [AW-1:0] prod   = ted_x * step_x;            // ted * step_nom
 
    // ---- proportional
    wire signed [AW-1:0] p_raw = -(prod >>> (gain_sel ? KP_HI : KP_LO));
    wire signed [AW-1:0] p_sat = sat(p_raw, lim_x);
 
    reg signed [AW-1:0] p_hold;
    always @(posedge clk_sys) begin
        if (!rstn || loop_reset) p_hold <= {AW{1'b0}};
        else if (ted_valid)      p_hold <= p_sat;
    end
    wire signed [AW-1:0] p_use = (P_HOLD != 0) ? p_hold : p_sat;
    assign p_term = p_sat[LOOP_W-1:0];
 
    // ---- integral
    wire signed [AW-1:0] i_d = -(prod >>> (gain_sel ? KI_HI : KI_LO));
    assign i_delta = i_d[LOOP_W-1:0];
 
    wire signed [AW-1:0] i_ext  = {{(AW-LOOP_W){i_state[LOOP_W-1]}}, i_state};
    wire signed [AW-1:0] i_next = sat(i_ext + i_d, lim_x);
    always @(posedge clk_sys) begin
        if (!rstn || loop_reset) i_state <= {LOOP_W{1'b0}};
        else if (ted_valid)      i_state <= i_next[LOOP_W-1:0];
    end
 
    // ---- sum + clamp
    wire signed [AW-1:0] sum  = p_use + i_ext;
    wire signed [AW-1:0] sumc = sat(sum, lim_x);
    assign loop_delta = sumc[LOOP_W-1:0];
endmodule
