/*
 * OLAF-8: Bounded-Memory Online Adaptive Fuzzy Inference Engine
 * Tiny Tapeout SKY130 / Verilog
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

module tt_um_olaf8 (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    wire [3:0] y;
    wire       done, admitted, busy;

    olaf8_core core (
        .x1(ui_in[7:4]),
        .x2(ui_in[3:0]),
        .start(uio_in[0] & ena),
        .clk(clk),
        .rst_n(rst_n),
        .y(y),
        .done(done),
        .admitted(admitted),
        .busy(busy)
    );

    // uo_out[3:0] = fuzzy output
    // uo_out[4]   = one-cycle done pulse
    // uo_out[5]   = one-cycle admission/replacement pulse
    // uo_out[6]   = busy
    // uo_out[7]   = reserved
    assign uo_out  = {1'b0, busy, admitted, done, y};

    // OLAF-8 does not drive the bidirectional pins.
    assign uio_out = 8'b0;
    assign uio_oe  = 8'b0;

    wire _unused = &{1'b0, uio_in[7:1], 1'b0};

endmodule


module olaf8_core (
    input  wire [3:0] x1,
    input  wire [3:0] x2,
    input  wire       start,
    input  wire       clk,
    input  wire       rst_n,
    output reg  [3:0] y,
    output reg        done,
    output reg        admitted,
    output reg        busy
);

    localparam [3:0] ADMIT_THRESHOLD = 4'd6;

    // Rule fields:
    // antecedent labels: 0=LOW, 1=MID, 2=HIGH
    // consequent: 4-bit singleton output
    // utility: 3-bit saturating score
    reg [1:0] rule_a1 [0:7];
    reg [1:0] rule_a2 [0:7];
    reg [3:0] rule_y  [0:7];
    reg [2:0] rule_u  [0:7];

    reg [3:0] x1_r, x2_r;
    reg [2:0] rule_idx;
    reg [3:0] max_fire;
    reg [2:0] max_idx;

    reg [10:0] sum_num;
    reg [6:0]  sum_den;

    // Restoring divider state.
    reg [10:0] div_num;
    reg [7:0]  div_rem;
    reg [6:0]  div_den;
    reg [10:0] div_quot;
    reg [3:0]  div_count;

    reg [1:0] state;
    localparam S_IDLE = 2'd0;
    localparam S_SCAN = 2'd1;
    localparam S_DIV  = 2'd2;

    function [3:0] memb;
        input [3:0] x;
        input [1:0] label;
        integer temp;
        begin
            case (label)
                2'd0: begin
                    temp = 15 - (2 * x);
                    if (temp < 0) memb = 4'd0;
                    else if (temp > 15) memb = 4'd15;
                    else memb = temp[3:0];
                end

                2'd1: begin
                    if (x <= 7)
                        temp = 2 * x;
                    else
                        temp = 30 - (2 * x);
                    if (temp < 0) memb = 4'd0;
                    else if (temp > 15) memb = 4'd15;
                    else memb = temp[3:0];
                end

                default: begin
                    temp = 2 * (x - 7);
                    if (temp < 0) memb = 4'd0;
                    else if (temp > 15) memb = 4'd15;
                    else memb = temp[3:0];
                end
            endcase
        end
    endfunction

    function [1:0] peak_label;
        input [3:0] x;
        begin
            if (x <= 4'd3)
                peak_label = 2'd0;
            else if (x <= 4'd11)
                peak_label = 2'd1;
            else
                peak_label = 2'd2;
        end
    endfunction

    wire [3:0] cur_m1   = memb(x1_r, rule_a1[rule_idx]);
    wire [3:0] cur_m2   = memb(x2_r, rule_a2[rule_idx]);
    wire [3:0] cur_fire = (cur_m1 < cur_m2) ? cur_m1 : cur_m2;
    wire [7:0] cur_prod = cur_fire * rule_y[rule_idx];

    wire [10:0] last_sum_num = sum_num + cur_prod;
    wire [6:0]  last_sum_den = sum_den + cur_fire;
    wire [3:0]  last_max_fire =
        (cur_fire > max_fire) ? cur_fire : max_fire;
    wire [2:0]  last_max_idx =
        (cur_fire > max_fire) ? rule_idx : max_idx;

    // Least-utility replacement selector.
    reg [2:0] replace_idx;
    reg [2:0] min_util;
    integer j;

    always @* begin
        replace_idx = 3'd0;
        min_util = rule_u[0];
        for (j = 1; j < 8; j = j + 1) begin
            if (rule_u[j] < min_util) begin
                min_util = rule_u[j];
                replace_idx = j[2:0];
            end
        end
    end

    // Restoring divider next state.
    wire [7:0] rem_shift = {div_rem[6:0], div_num[10]};
    wire       div_take  = (rem_shift >= {1'b0, div_den});
    wire [7:0] rem_next  =
        div_take ? (rem_shift - {1'b0, div_den}) : rem_shift;
    wire [10:0] quot_next = {div_quot[9:0], div_take};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            y          <= 4'd8;
            done       <= 1'b0;
            admitted   <= 1'b0;
            busy       <= 1'b0;

            x1_r       <= 4'd0;
            x2_r       <= 4'd0;
            rule_idx   <= 3'd0;
            max_fire   <= 4'd0;
            max_idx    <= 3'd0;
            sum_num    <= 11'd0;
            sum_den    <= 7'd0;

            div_num    <= 11'd0;
            div_rem    <= 8'd0;
            div_den    <= 7'd1;
            div_quot   <= 11'd0;
            div_count  <= 4'd0;

            // Deterministic initial rule base.
            rule_a1[0] <= 2'd0; rule_a2[0] <= 2'd0; rule_y[0] <= 4'd2;  rule_u[0] <= 3'd1;
            rule_a1[1] <= 2'd0; rule_a2[1] <= 2'd1; rule_y[1] <= 4'd5;  rule_u[1] <= 3'd1;
            rule_a1[2] <= 2'd0; rule_a2[2] <= 2'd2; rule_y[2] <= 4'd7;  rule_u[2] <= 3'd1;
            rule_a1[3] <= 2'd1; rule_a2[3] <= 2'd0; rule_y[3] <= 4'd5;  rule_u[3] <= 3'd1;
            rule_a1[4] <= 2'd1; rule_a2[4] <= 2'd1; rule_y[4] <= 4'd8;  rule_u[4] <= 3'd1;
            rule_a1[5] <= 2'd1; rule_a2[5] <= 2'd2; rule_y[5] <= 4'd10; rule_u[5] <= 3'd1;
            rule_a1[6] <= 2'd2; rule_a2[6] <= 2'd0; rule_y[6] <= 4'd7;  rule_u[6] <= 3'd1;
            rule_a1[7] <= 2'd2; rule_a2[7] <= 2'd2; rule_y[7] <= 4'd13; rule_u[7] <= 3'd1;
        end else begin
            done     <= 1'b0;
            admitted <= 1'b0;

            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        x1_r     <= x1;
                        x2_r     <= x2;
                        rule_idx <= 3'd0;
                        max_fire <= 4'd0;
                        max_idx  <= 3'd0;
                        sum_num  <= 11'd0;
                        sum_den  <= 7'd0;
                        busy     <= 1'b1;
                        state    <= S_SCAN;
                    end
                end

                S_SCAN: begin
                    busy    <= 1'b1;
                    sum_num <= sum_num + cur_prod;
                    sum_den <= sum_den + cur_fire;

                    if (cur_fire > max_fire) begin
                        max_fire <= cur_fire;
                        max_idx  <= rule_idx;
                    end

                    if (rule_idx == 3'd7) begin
                        div_num   <= last_sum_num;
                        div_den   <= (last_sum_den == 0) ? 7'd1 : last_sum_den;
                        div_rem   <= 8'd0;
                        div_quot  <= 11'd0;
                        div_count <= 4'd0;
                        max_fire  <= last_max_fire;
                        max_idx   <= last_max_idx;
                        state     <= S_DIV;
                    end else begin
                        rule_idx <= rule_idx + 3'd1;
                    end
                end

                S_DIV: begin
                    div_rem  <= rem_next;
                    div_num  <= {div_num[9:0], 1'b0};
                    div_quot <= quot_next;

                    if (div_count == 4'd10) begin
                        if (div_quot[10:4] != 0)
                            y <= 4'd15;
                        else
                            y <= div_quot[3:0];

                        if (max_fire < ADMIT_THRESHOLD) begin
                            rule_a1[replace_idx] <= peak_label(x1_r);
                            rule_a2[replace_idx] <= peak_label(x2_r);

                            if (div_quot[10:4] != 0)
                                rule_y[replace_idx] <= 4'd15;
                            else if (last_sum_den == 0)
                                rule_y[replace_idx] <= 4'd8;
                            else
                                rule_y[replace_idx] <= div_quot[3:0];

                            rule_u[replace_idx] <= 3'd4;
                            admitted <= 1'b1;
                        end else begin
                            if (rule_u[max_idx] != 3'd7)
                                rule_u[max_idx] <= rule_u[max_idx] + 3'd1;
                        end

                        done  <= 1'b1;
                        busy  <= 1'b0;
                        state <= S_IDLE;
                    end else begin
                        div_count <= div_count + 4'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
