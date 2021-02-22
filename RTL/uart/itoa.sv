`timescale 1 ns/1 ns

module itoa(
    input  wire               rstn,
    input  wire               clk,
    input  wire               i_en,
    input  wire signed [15:0] i_val,
    output reg                o_en,
    output reg         [ 7:0] o_str [6]
);

reg [ 2:0] cnt;
reg        sign;
reg        zero;
reg [15:0] abs;
reg [ 3:0] rem;

always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        cnt <= 3'd0;
        {sign, abs, zero, rem} <= '0;
        o_en <= 1'b0;
        o_str <= '{6{'0}};
    end else begin
        if(cnt==3'd0) begin
            if(i_en)
                cnt <= 3'd1;
            sign <= i_val[15];
            abs <= i_val[15] ? $unsigned(-i_val) : $unsigned(i_val);
        end else begin
            cnt <= cnt + 3'd1;
            abs <= abs / 16'd10;
            rem <= abs % 16'd10;
            zero <= abs==16'd0;
            if(cnt>3'd1) begin
                o_str[5] <= o_str[4];
                o_str[4] <= o_str[3];
                o_str[3] <= o_str[2];
                o_str[2] <= o_str[1];
                o_str[1] <= o_str[0];
                if(cnt>3'd2 && zero) begin
                    o_str[0] <= sign ? 8'h2D : 8'h20;
                    sign <= 1'b0;
                end else begin
                    o_str[0] <= {4'h3, rem};
                end
            end
        end
        o_en <= cnt == 3'd7;
    end

endmodule
