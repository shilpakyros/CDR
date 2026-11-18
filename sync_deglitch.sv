module usb2_cdr_sync_deglitch (
    input  wire       clk_sys,
    input  wire       rstn,
    input  wire       rx_serial,
    input  wire       d_p,
    input  wire       d_n,
    input  wire [1:0] speed,
    input  wire [4:0] dg_w,
    output wire       rx_sync,
    output wire       rx_s,
    output wire       d_p_sync,
    output wire       d_n_sync,
    output wire       se0,
    output wire       se1,
    output wire       idle
);
    reg [1:0] rx_ff, dp_ff, dn_ff;
    always @(posedge clk_sys) begin
        rx_ff <= {rx_ff[0], rx_serial};
        dp_ff <= {dp_ff[0], d_p};
        dn_ff <= {dn_ff[0], d_n};
    end
    assign rx_sync  = rx_ff[1];
    assign d_p_sync = dp_ff[1];
    assign d_n_sync = dn_ff[1];
 
    reg [4:0] dg_cnt;
    reg       rx_s_r;
    wire      dg_diff = rx_sync ^ rx_s_r;
    always @(posedge clk_sys) begin
        if (!rstn) begin
            rx_s_r <= rx_sync;
            dg_cnt <= 5'd0;
        end else if (!dg_diff) begin
            dg_cnt <= 5'd0;
        end else if (dg_cnt == dg_w - 5'd1) begin
            rx_s_r <= rx_sync;
            dg_cnt <= 5'd0;
        end else begin
            dg_cnt <= dg_cnt + 5'd1;
        end
    end
    assign rx_s = (dg_w <= 5'd1) ? rx_sync : rx_s_r;
 
    wire is_ls = (speed == 2'b00);
    wire j_fs  =  d_p_sync & ~d_n_sync;
    wire j_ls  = ~d_p_sync &  d_n_sync;
    assign idle = is_ls ? j_ls : j_fs;
    assign se0  = ~d_p_sync & ~d_n_sync;
    assign se1  =  d_p_sync &  d_n_sync;
endmodule
