`timescale 1 ns/1 ns

// 模块：top
// 功能：UART发送器，格式为：115200,8,n,1，可以把 i_val0, i_val1, i_val2, i_val3 变成10进制格式，放在一行里，通过 UART 发送出去，
// 参数：无
// 输入输出：详见下方注释
module uart_monitor #(
    parameter [15:0] CLK_DIV = 217 // UART分频倍率，例如若时钟频率为 36.864MHz, CLK_DIV=320，则 UART 波特率为 36.864MHz/320=115200
) (
    input  wire               rstn,
    input  wire               clk,
    input  wire               i_en,
    input  wire signed [15:0] i_val0,
    input  wire signed [15:0] i_val1,
    input  wire signed [15:0] i_val2,
    input  wire signed [15:0] i_val3,
    output wire               o_uart_tx  // UART TX 信号
);

enum logic [2:0] {IDLE, SELECT, WAIT, PARSING, SENDING} stat;

wire       tx_rdy;
reg        tx_en;
reg  [7:0] tx_data;

reg               itoa_en;
reg signed [15:0] itoa_val;
wire              itoa_oen;
wire       [ 7:0] itoa_str [6];

reg        [ 2:0] vcnt;

reg        [ 2:0] cnt;
reg        [ 7:0] eov;
wire       [ 7:0] s_str[8];

assign s_str[0] = itoa_str[0];
assign s_str[1] = itoa_str[1];
assign s_str[2] = itoa_str[2];
assign s_str[3] = itoa_str[3];
assign s_str[4] = itoa_str[4];
assign s_str[5] = itoa_str[5];
assign s_str[6] = 8'h20;
assign s_str[7] = eov;

always_comb begin
    tx_en = 1'b0;
    tx_data = '0;
    if(stat==SENDING) begin
        tx_en = 1'b1;
        tx_data = s_str[cnt];
    end
end

always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        stat <= IDLE;
        itoa_en <= 1'b0;
        itoa_val <= '0;
        vcnt <= '0;
        cnt <= '0;
        eov <= 8'h20;
    end else begin
        itoa_en <= 1'b0;
        case(stat)
            IDLE: if(i_en)
                stat <= SELECT;
            SELECT: begin
                if         (vcnt==3'd0) begin
                    vcnt <= vcnt + 3'd1;
                    stat <= WAIT;
                    itoa_en <= 1'b1;
                    itoa_val <= i_val0;
                    eov <= 8'h20;
                end else if(vcnt==3'd1) begin
                    vcnt <= vcnt + 3'd1;
                    stat <= WAIT;
                    itoa_en <= 1'b1;
                    itoa_val <= i_val1;
                    eov <= 8'h20;
                end else if(vcnt==3'd2) begin
                    vcnt <= vcnt + 3'd1;
                    stat <= WAIT;
                    itoa_en <= 1'b1;
                    itoa_val <= i_val2;
                    eov <= 8'h20;
                end else if(vcnt==3'd3) begin
                    vcnt <= vcnt + 3'd1;
                    stat <= WAIT;
                    itoa_en <= 1'b1;
                    itoa_val <= i_val3;
                    eov <= 8'h0A;
                end else begin
                    vcnt <= 3'd0;
                    stat <= IDLE;
                    eov <= 8'h20;
                end
            end
            WAIT:
                stat <= PARSING;
            PARSING: if(itoa_oen)
                stat <= SENDING;
            SENDING: if(tx_rdy) begin
                cnt <= cnt + 3'd1;
                if(cnt==3'd7)
                    stat <= SELECT;
            end
        endcase
    end

itoa itoa_i (
    .rstn      ( rstn         ),
    .clk       ( clk          ),
    .i_en      ( itoa_en      ),
    .i_val     ( itoa_val     ),
    .o_en      ( itoa_oen     ),
    .o_str     ( itoa_str     )
);

uart_tx #(
    .CLK_DIV   ( CLK_DIV      )
) uart_tx_i (
    .rstn      ( rstn         ),
    .clk       ( clk          ),
    .i_e       ( tx_en        ),
    .i_r       ( tx_rdy       ),
    .i_d       ( tx_data      ),
    .tx        ( o_uart_tx    )
);

endmodule
