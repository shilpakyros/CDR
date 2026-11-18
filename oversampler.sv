module usb2_cdr_oversampler #(
    parameter integer WIN = 4
)(
    input  wire           clk_sys,
    input  wire           rstn,
    input  wire           rx_s,
    output wire [WIN-1:0] sample_shift,
    output reg  [WIN-1:0] sample_window,
    output reg            rx_s_d
);
    reg [WIN-1:0] shift_reg;
    assign sample_shift = shift_reg;
 
    always @(posedge clk_sys) begin
        if (!rstn) begin
            shift_reg     <= {WIN{rx_s}};
            sample_window <= {WIN{rx_s}};
            rx_s_d        <= rx_s;
        end else begin
            shift_reg     <= {shift_reg[WIN-2:0], rx_s};
            sample_window <= shift_reg;
            rx_s_d        <= rx_s;
        end
    end
endmodule
