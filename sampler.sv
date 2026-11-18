module usb2_cdr_sampler #(
    parameter integer W   = 32,
    parameter integer WIN = 4
)(
    input  wire           clk_sys,
    input  wire           rstn,
    input  wire [WIN-1:0] sample_window,
    input  wire [W-1:0]   phase,
    input  wire [W-1:0]   step_eff,
    input  wire           bit_strobe,
    output wire           sample_tap,
    output wire           recovered_bit_clk,
    output reg            recovered_data
);
    wire late = (phase >= {1'b0, step_eff[W-1:1]});     // residual >= step/2
    assign sample_tap = late ? sample_window[1] : sample_window[0];
 
    always @(posedge clk_sys) begin
        if (!rstn)           recovered_data <= 1'b0;
        else if (bit_strobe) recovered_data <= sample_tap;
    end
    assign recovered_bit_clk = bit_strobe;
endmodule
