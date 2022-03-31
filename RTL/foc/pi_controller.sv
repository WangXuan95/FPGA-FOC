
// 模块： pi_controller
// Type    : synthesizable
// Standard: SystemVerilog 2005 (IEEE1800-2005)
// 功能： PI 控制器

module pi_controller #(
    parameter logic [23:0] Kp = 24'd32768,
    parameter logic [23:0] Ki = 24'd2
) (
    input  wire               rstn,
    input  wire               clk,
    input  wire               i_en,
    input  wire signed [15:0] i_aim,
    input  wire signed [15:0] i_real,
    output reg                o_en,
    output wire signed [15:0] o_value
);

reg               en1, en2, en3, en4;
reg signed [31:0] pdelta, idelta, kpdelta1, kpdelta, kidelta, kpidelta, value;

assign o_value = value[31:16];

function automatic logic signed [31:0] protect_add(input logic signed [31:0] a, input logic signed [31:0] b);
    automatic logic signed [32:0] y;
    y = $signed({a[31],a}) + $signed({b[31],b});
    if(     y >  $signed(33'h7fffffff) )
        return   $signed(32'h7fffffff);
    else if(y < -$signed(33'h7fffffff) )
        return  -$signed(32'h7fffffff);
    else
        return   $signed(y[31:0]);
endfunction

function automatic logic signed [31:0] protect_mul(input logic signed [31:0] a, input logic signed [24:0] b);
    automatic logic signed [56:0] y;
    y = a * b;
    if(     y >  $signed(57'h7fffffff) )
        return   $signed(32'h7fffffff);
    else if(y < -$signed(57'h7fffffff) )
        return  -$signed(32'h7fffffff);
    else
        return   $signed(y[31:0]);
endfunction

always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        en1 <= 1'b0;
        pdelta <= 0;
    end else begin
        en1 <= i_en;
        if(i_en) begin
            pdelta <= $signed({{16{i_aim[15]}},i_aim}) - $signed({{16{i_real[15]}},i_real});
        end
    end

always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        en2 <= 1'b0;
        kpdelta1 <= 0;
        idelta <= 0;
    end else begin
        en2 <= en1;
        if(en1) begin
            kpdelta1 <= protect_mul(pdelta, $signed({1'b0,Kp}) );
            idelta  <= protect_add(idelta, pdelta);
        end
    end

always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        en3 <= 1'b0;
        kpdelta <= 0;
        kidelta <= 0;
    end else begin
        en3 <= en2;
        if(en2) begin
            kpdelta <= kpdelta1;
            kidelta <= protect_mul(idelta, $signed({1'b0,Ki}) );
        end
    end

always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        en4 <= 1'b0;
        kpidelta <= 0;
    end else begin
        en4 <= en3;
        if(en3) begin
            kpidelta <= protect_add(kpdelta, kidelta);
        end
    end

always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        o_en <= 1'b0;
        value <= 0;
    end else begin
        o_en <= en4;
        if(en4) begin
            value <= protect_add(value, kpidelta);
        end
    end

endmodule
