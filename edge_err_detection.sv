module usb2_cdr_error_detection #(
    parameter integer W     = 32,
    parameter integer TED_W = 6
)(
    input  wire                     clk_sys,
    input  wire                     rstn,
    input  wire                     rx_s,
    input  wire                     rx_s_d,
    input  wire [W-1:0]             phase,
    input  wire [W-1:0]             edge_ref,
    output wire                     edge_o,
    output reg  [W-1:0]             phase_value,
    output wire signed [W-1:0]      error,
    output wire signed [TED_W-1:0]  ted_value,
    output wire                     ted_valid
);
    wire edge_w = rx_s ^ rx_s_d;
    assign edge_o = edge_w;
 
    reg edge_q;
    always @(posedge clk_sys) begin
        if (!rstn) begin
            edge_q      <= 1'b0;
            phase_value <= {W{1'b0}};
        end else begin
            edge_q <= edge_w;
            if (edge_w) phase_value <= phase;
        end
    end
 
    assign error = $signed(phase_value - edge_ref);
    wire signed [TED_W-1:0] ted_raw = error[W-1 -: TED_W];
 
    assign ted_value = edge_q ? ted_raw : {TED_W{1'b0}};
    assign ted_valid = edge_q;
endmodule
