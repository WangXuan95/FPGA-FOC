
//--------------------------------------------------------------------------------------------------------
// 模块： hold_detect
// Type    : synthesizable
// Standard: Verilog 2001 (IEEE1364-2001)
// 功能： 检测 in 从高电平变为低电平并保持 SAMPLE_DELAY 个时钟周期，在 sn_adc 信号上产生一个时钟周期的高电平。
//--------------------------------------------------------------------------------------------------------

module hold_detect #(
    parameter [15:0]  SAMPLE_DELAY = 16'd100
) (
    input  wire  rstn,
    input  wire  clk,
    input  wire  in,
    output reg   out
);

reg        latch1, latch2;
reg [15:0] cnt;

always @ (posedge clk or negedge rstn)
    if(~rstn)
        {latch1, latch2} <= 2'b11;
    else
        {latch1, latch2} <= {in, latch1};

always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        out <= 1'b0;
        cnt <= 16'd0;
    end else begin
        out <= 1'b0;
        if(latch1) begin
            if(latch2) begin
                if( cnt != 16'd0 )
                    cnt <= cnt - 16'd1;
                out <= cnt == 16'd1;
            end else begin
                cnt <= SAMPLE_DELAY;
            end
        end else begin
            cnt <= 16'd0;
        end
    end

endmodule
