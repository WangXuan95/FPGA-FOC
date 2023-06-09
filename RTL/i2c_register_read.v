
//--------------------------------------------------------------------------------------------------------
// 模块: i2c_register_read
// Type    : synthesizable
// Standard: Verilog 2001 (IEEE1364-2001)
// 功能    : I2C 读控制器
//--------------------------------------------------------------------------------------------------------

module i2c_register_read #(
    parameter   [15:0] CLK_DIV       = 16'd16,
    parameter   [ 6:0] SLAVE_ADDR    = 7'h36,
    parameter   [ 7:0] REGISTER_ADDR = 8'h0E
) (
    input  wire        rstn,
    input  wire        clk,
    output reg         scl,
    inout              sda,
    input  wire        start,
    output wire        ready,
    output reg         done,
    output reg  [15:0] regout
);

localparam [15:0] CLK_DIV_PARSED = CLK_DIV>16'd0 ? CLK_DIV-16'd1 : 16'd0;

reg  sda_e, sda_o;

assign sda = sda_e ? sda_o : 1'bz;

reg        epoch;
reg [15:0] clkcnt;
reg [ 7:0] cnt;
reg [ 7:0] send_shift;
reg [15:0] recv_shift;

always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        epoch <= 1'b0;
        clkcnt <= 0;
    end else begin
        if(clkcnt==CLK_DIV_PARSED) begin
            epoch <= 1'b1;
            clkcnt <= 0;
        end else begin
            epoch <= 1'b0;
            clkcnt <= clkcnt + 16'd1;
        end
    end

assign ready = (cnt == 8'd0);

always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        {scl, sda_e, sda_o} <= 3'b111;
        cnt <= 0;
        send_shift <= 0;
        recv_shift <= 0;
        regout <= 0;
        done <= 1'b0;
    end else begin
        if(ready) begin
            {scl, sda_e, sda_o} <= 3'b111;
            if(start) begin
                cnt <= 8'd1;
            end
        end else if(done) begin
            done <= 1'b0;
            cnt <= 8'd0;
        end else if(epoch) begin
            cnt <= cnt + 8'd1;
            if(cnt<8'd2) begin
            end else if(cnt< 8'd4) begin
                sda_o <= 1'b0;
                send_shift <= {SLAVE_ADDR, 1'b0};
            end else if(cnt< 8'd37) begin
                scl <= cnt[1];
                if(cnt[1:0]==2'b01) begin
                    {sda_o, send_shift} <= {send_shift, 1'b1};
                end
            end else if(cnt< 8'd40) begin
                send_shift <= REGISTER_ADDR;
                scl <= cnt[1];
                sda_e <= 1'b0;
            end else if(cnt< 8'd73) begin
                scl <= cnt[1];
                if(cnt[1:0]==2'b01) begin
                    sda_e <= 1'b1;
                    {sda_o, send_shift} <= {send_shift, 1'b1};
                end
            end else if(cnt< 8'd77) begin
                scl <= cnt[1];
                sda_e <= 1'b0;
            end else if(cnt==8'd77) begin
                scl <= cnt[1];
                sda_e <= 1'b1;
                sda_o <= 1'b1;
            end else if(cnt< 8'd82) begin
                scl <= 1'b1;
            end else if(cnt< 8'd84) begin
                scl <= 1'b1;
                sda_o <= 1'b0;
                send_shift <= {SLAVE_ADDR, 1'b1};
            end else if(cnt< 8'd117) begin
                scl <= cnt[1];
                if(cnt[1:0]==2'b01) begin
                    {sda_o, send_shift} <= {send_shift, 1'b1};
                end
            end else if(cnt< 8'd121) begin
                scl <= cnt[1];
                sda_e <= 1'b0;
            end else if(cnt< 8'd153) begin
                scl <= cnt[1];
                sda_e <= 1'b0;
                if( cnt[1:0]==2'b11 )
                    recv_shift <= {recv_shift[14:0], sda};
            end else if(cnt< 8'd157) begin
                scl <= cnt[1];
                sda_e <= 1'b1;
                sda_o <= 1'b0;
            end else if(cnt< 8'd189) begin
                scl <= cnt[1];
                sda_e <= 1'b0;
                if( cnt[1:0]==2'b11 )
                    recv_shift <= {recv_shift[14:0], sda};
            end else if(cnt< 8'd193) begin
                scl <= cnt[1];
                sda_e <= 1'b1;
                sda_o <= 1'b1;
            end else if(cnt< 8'd195) begin
                scl <= 1'b0;
                sda_o <= 1'b0;
            end else if(cnt< 8'd198) begin
                scl <= 1'b1;
                sda_o <= 1'b0;
            end else if(cnt< 8'd204) begin
                sda_o <= 1'b1;
                regout <= recv_shift;
            end else begin
                done <= 1'b1;
            end
        end
    end

endmodule
