module usb2_cdr_lock_fsm #(
    parameter integer TED_W     = 6,
    parameter integer EDGE_MIN  = 4,
    parameter integer M_GOOD    = 3,        // max 3 (2-bit counter)
    parameter integer TED_THR   = 8,        // |ted| <= THR : small (1/8 bit)
    parameter integer TED_BAD   = 24,       // |ted| >  BAD : large (3/8 bit)
    parameter integer BAD_MAX   = 3,
    parameter integer IDLE_QUAL = 8         // clk_sys of J = line idle
)(
    input  wire                    clk_sys,
    input  wire                    rstn,
    input  wire                    enable,
    input  wire                    se0,
    input  wire                    se1,
    input  wire                    idle,
    input  wire [9:0]              se0_qual,
    input  wire [11:0]             timeout,
    input  wire signed [TED_W-1:0] ted_value,
    input  wire                    ted_valid,
    output wire                    acc_reset,
    output wire                    loop_reset,
    output wire                    gain_sel,
    output wire                    locked,
    output wire [1:0]              state_o
);
    localparam [1:0] S_IDLE = 2'd0, S_ACQ = 2'd1, S_TRK = 2'd2, S_LOSS = 2'd3;
 
    reg [1:0]  state;
    reg [2:0]  edge_cnt;
    reg [1:0]  good_cnt, bad_cnt;
    reg [11:0] to_cnt;
    reg [9:0]  se0_cnt, idle_cnt;
 
    wire [TED_W:0] ted_abs = ted_value[TED_W-1] ? -{ted_value[TED_W-1], ted_value}
                                                :  {ted_value[TED_W-1], ted_value};
    wire err_small = (ted_abs <= TED_THR);
    wire err_large = (ted_abs >  TED_BAD);
 
    wire line_bad = se0 | se1;
    always @(posedge clk_sys) begin
        if (!rstn) begin
            se0_cnt  <= 10'd0;
            idle_cnt <= 10'd0;
        end else begin
            if (!line_bad)                  se0_cnt  <= 10'd0;
            else if (se0_cnt < se0_qual)    se0_cnt  <= se0_cnt + 10'd1;
            if (!idle)                      idle_cnt <= 10'd0;
            else if (idle_cnt < IDLE_QUAL)  idle_cnt <= idle_cnt + 10'd1;
        end
    end
    wire se0_ok  = (se0_cnt  >= se0_qual);
    wire idle_ok = (idle_cnt >= IDLE_QUAL);
 
    wire timeout_ev = (state == S_ACQ) && (to_cnt == timeout - 12'd1);
 
    wire lock_ev = ted_valid && err_small &&
                   (edge_cnt >= EDGE_MIN-1) && (good_cnt >= M_GOOD-1);
    wire drop_ev = ted_valid && err_large && (bad_cnt >= BAD_MAX-1);
 
    always @(posedge clk_sys) begin
        if (!rstn || !enable) begin
            state    <= S_IDLE;
            edge_cnt <= 3'd0;
            good_cnt <= 2'd0;
            bad_cnt  <= 2'd0;
            to_cnt   <= 12'd0;
        end else begin
            case (state)
            S_IDLE: begin
                edge_cnt <= 3'd0; good_cnt <= 2'd0; bad_cnt <= 2'd0; to_cnt <= 12'd0;
                if (ted_valid) begin
                    edge_cnt <= 3'd1;
                    state    <= S_ACQ;
                end
            end
            S_ACQ: begin
                to_cnt <= to_cnt + 12'd1;
                if (ted_valid) begin
                    if (edge_cnt != 3'd7) edge_cnt <= edge_cnt + 3'd1;
                    if (err_small) begin
                        if (good_cnt != 2'd3) good_cnt <= good_cnt + 2'd1;
                    end else
                        good_cnt <= 2'd0;
                end
                if (se0_ok)          state <= S_LOSS;
                else if (lock_ev)  begin state <= S_TRK; bad_cnt <= 2'd0; end
                else if (timeout_ev) state <= S_IDLE;
            end
            S_TRK: begin
                if (ted_valid) begin
                    if (err_large) bad_cnt <= bad_cnt + 2'd1;
                    else           bad_cnt <= 2'd0;
                end
                if (se0_ok)        state <= S_LOSS;
                else if (drop_ev)  state <= S_IDLE;
            end
            S_LOSS: begin
                if (idle_ok) state <= S_IDLE;
            end
            endcase
        end
    end
 
    assign state_o    = state;
    assign locked     = (state == S_TRK);
    assign loop_reset = (state == S_IDLE);
    assign acc_reset  = (state == S_IDLE) || (state == S_LOSS);
    assign gain_sel   = (state == S_IDLE) || (state == S_ACQ);
endmodule
