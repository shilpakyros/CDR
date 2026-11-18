module usb2_cdr_phase_acc #(
    parameter integer W      = 32,
    parameter integer LOOP_W = 24
)(
    input  wire                     clk_sys,
    input  wire                     rstn,
    input  wire                     acc_reset,
    input  wire signed [LOOP_W-1:0] loop_delta,
    input  wire [W-1:0]             step_nom,
    input  wire [LOOP_W-1:0]        lim,
    input  wire [W-1:0]             reset_phase,
    output reg  [W-1:0]             phase,
    output reg                      bit_strobe,
    output wire [W-1:0]             step_eff
);
    wire signed [W+1:0] nom = {2'b00, step_nom};
    wire signed [W+1:0] l   = {{(W+2-LOOP_W){1'b0}}, lim};
    wire signed [W+1:0] sum = nom + {{(W+2-LOOP_W){loop_delta[LOOP_W-1]}}, loop_delta};
    wire signed [W+1:0] lo  = nom - l;
    wire signed [W+1:0] hi  = nom + l;
    wire signed [W+1:0] stc = (sum < lo) ? lo : (sum > hi) ? hi : sum;
    assign step_eff = stc[W-1:0];
 
    wire [W:0] acc_next = {1'b0, phase} + {1'b0, step_eff};
 
    always @(posedge clk_sys) begin
        if (!rstn || acc_reset) begin
            phase      <= reset_phase;
            bit_strobe <= 1'b0;
        end else begin
            phase      <= acc_next[W-1:0];
            bit_strobe <= acc_next[W];

		
        end
    end
endmodule
