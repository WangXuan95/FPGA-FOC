
//--------------------------------------------------------------------------------------------------------
// 模块：uart_monitor
// Type    : synthesizable
// Standard: Verilog 2001 (IEEE1364-2001)
// 功能：UART发送器，格式为：115200,8,n,1，可以把 i_val0, i_val1, i_val2, i_val3 变成10进制格式，放在一行里，通过 UART 发送出去
//--------------------------------------------------------------------------------------------------------

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
    output reg                o_uart_tx  // UART TX 信号
);

initial o_uart_tx = 1'b1;

localparam [2:0] IDLE    = 3'd0,
                 SELECT  = 3'd1,
                 WAIT    = 3'd2,
                 PARSING = 3'd3,
                 SENDING = 3'd4;
reg        [2:0] stat;

wire       tx_rdy;
reg        tx_en;
reg  [7:0] tx_data;

reg               itoa_en;
reg signed [15:0] itoa_val;
reg               itoa_oen;
reg        [ 7:0] itoa_str [0:5];

reg        [ 2:0] vcnt;

reg        [ 2:0] cnt;
reg        [ 7:0] eov;
wire       [ 7:0] s_str[0:7];

assign s_str[0] = itoa_str[0];
assign s_str[1] = itoa_str[1];
assign s_str[2] = itoa_str[2];
assign s_str[3] = itoa_str[3];
assign s_str[4] = itoa_str[4];
assign s_str[5] = itoa_str[5];
assign s_str[6] = 8'h20;
assign s_str[7] = eov;

always @ (*) begin
    tx_en = 1'b0;
    tx_data = 0;
    if(stat==SENDING) begin
        tx_en = 1'b1;
        tx_data = s_str[cnt];
    end
end

always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        stat <= IDLE;
        itoa_en <= 1'b0;
        itoa_val <= 0;
        vcnt <= 0;
        cnt <= 0;
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
            default: //SENDING:
                if(tx_rdy) begin
                    cnt <= cnt + 3'd1;
                    if(cnt==3'd7)
                        stat <= SELECT;
                end
        endcase
    end


reg [ 2:0] itoa_cnt;
reg        itoa_sign;
reg        itoa_zero;
reg [15:0] itoa_abs;
wire[15:0] itoa_rem_w = (itoa_abs % 16'd10);
reg [ 3:0] itoa_rem;


always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        itoa_cnt <= 3'd0;
        {itoa_sign, itoa_abs, itoa_zero, itoa_rem} <= 0;
        itoa_oen <= 1'b0;
        itoa_str[0] <= 0;
        itoa_str[1] <= 0;
        itoa_str[2] <= 0;
        itoa_str[3] <= 0;
        itoa_str[4] <= 0;
        itoa_str[5] <= 0;
    end else begin
        if(itoa_cnt==3'd0) begin
            if(itoa_en)
                itoa_cnt <= 3'd1;
            itoa_sign <= itoa_val[15];
            itoa_abs  <= itoa_val[15] ? $unsigned(-itoa_val) : $unsigned(itoa_val);
        end else begin
            itoa_cnt  <= (itoa_cnt + 3'd1);
            itoa_abs  <= (itoa_abs / 16'd10);
            itoa_rem  <= itoa_rem_w[3:0];
            itoa_zero <= (itoa_abs == 16'd0);
            if(itoa_cnt>3'd1) begin
                itoa_str[5] <= itoa_str[4];
                itoa_str[4] <= itoa_str[3];
                itoa_str[3] <= itoa_str[2];
                itoa_str[2] <= itoa_str[1];
                itoa_str[1] <= itoa_str[0];
                if(itoa_cnt>3'd2 && itoa_zero) begin
                    itoa_str[0] <= itoa_sign ? 8'h2D : 8'h20;
                    itoa_sign <= 1'b0;
                end else begin
                    itoa_str[0] <= {4'h3, itoa_rem};
                end
            end
        end
        itoa_oen <= itoa_cnt == 3'd7;
    end


reg [15:0] ccnt;
reg [ 3:0] tx_cnt;
reg [12:1] tx_shift;

assign tx_rdy = (tx_cnt==4'd0);

always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        o_uart_tx <= 1'b1;
        ccnt <= 0;
        tx_cnt <= 0;
        tx_shift <= 12'hFFF;
    end else begin
        if(tx_cnt==4'd0) begin
            o_uart_tx <= 1'b1;
            ccnt <= 0;
            if(tx_en) begin
                tx_cnt <= 4'd12;
                tx_shift <= {2'b10, tx_data[0], tx_data[1], tx_data[2], tx_data[3], tx_data[4], tx_data[5], tx_data[6], tx_data[7], 2'b11};
            end
        end else begin
            o_uart_tx <= tx_shift[tx_cnt];
            if( ccnt + 16'd1 < CLK_DIV ) begin
                ccnt <= ccnt + 16'd1;
            end else begin
                ccnt <= 0;
                tx_cnt <= tx_cnt - 4'd1;
            end
        end
    end


endmodule
