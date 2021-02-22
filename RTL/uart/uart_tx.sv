`timescale 1 ns/1 ns

module uart_tx #(
    parameter [15:0] CLK_DIV = 217 // 25MHz / 217 = 115207 ~= 115200
)(
    input  wire        rstn,       // active-low reset
    input  wire        clk,
    
    input  wire        i_e,
    output wire        i_r,
    input  wire [ 7:0] i_d,
    
    output reg         tx
);

reg [15:0] ccnt;
reg [ 3:0] cnt;
reg [12:1] tx_shift;

assign i_r = (cnt==4'd0);

always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        tx <= 1'b1;
        ccnt <= '0;
        cnt <= '0;
        tx_shift <= '1;
    end else begin
        if(cnt==4'd0) begin
            tx <= 1'b1;
            ccnt <= '0;
            if(i_e) begin
                cnt <= 4'd12;
                tx_shift <= {2'b10, i_d[0], i_d[1], i_d[2], i_d[3], i_d[4], i_d[5], i_d[6], i_d[7], 2'b11};
            end
        end else begin
            tx <= tx_shift[cnt];
            if( ccnt + 16'd1 < CLK_DIV ) begin
                ccnt <= ccnt + 16'd1;
            end else begin
                ccnt <= '0;
                cnt <= cnt - 4'd1;
            end
        end
    end

endmodule
