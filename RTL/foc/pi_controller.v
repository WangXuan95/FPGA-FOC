
//--------------------------------------------------------------------------------------------------------
// 模块： pi_controller
// Type    : synthesizable
// Standard: Verilog 2001 (IEEE1364-2001)
// 功能： PI 控制器
//--------------------------------------------------------------------------------------------------------

module pi_controller (
    input  wire               rstn,
    input  wire               clk,
    input  wire               i_en,
    input  wire        [30:0] i_Kp,
    input  wire        [30:0] i_Ki,
    input  wire signed [15:0] i_aim,
    input  wire signed [15:0] i_real,
    output reg                o_en,
    output wire signed [15:0] o_value
);

reg               en1, en2, en3, en4;
reg signed [31:0] pdelta, idelta, kpdelta1, kpdelta, kidelta, kpidelta, value;
reg        [30:0] Kp0=0, Ki0=0, Ki1=0;

assign o_value = value[31:16];


function  signed [31:0] protect_add;
    input signed [31:0] a, b;
    reg   signed [32:0] y;
//function automatic logic signed [31:0] protect_add(input logic signed [31:0] a, input logic signed [31:0] b);
//    automatic logic signed [32:0] y;
begin
    y = $signed({a[31],a}) + $signed({b[31],b});
    if(     y >  $signed(33'h7fffffff) )
        protect_add =  $signed(32'h7fffffff);
    else if(y < -$signed(33'h7fffffff) )
        protect_add = -$signed(32'h7fffffff);
    else
        protect_add =  $signed(y[31:0]);
end
endfunction


function  signed [31:0] protect_mul;
    input signed [31:0] a, b;
    reg   signed [56:0] y;
//function automatic logic signed [31:0] protect_mul(input logic signed [31:0] a, input logic signed [24:0] b);
//    automatic logic signed [56:0] y;
begin
    y = a * b;
    if(     y >  $signed(57'h7fffffff) )
        protect_mul =  $signed(32'h7fffffff);
    else if(y < -$signed(57'h7fffffff) )
        protect_mul = -$signed(32'h7fffffff);
    else
        protect_mul =  $signed(y[31:0]);
end
endfunction


always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        en1 <= 1'b0;
        en2 <= 1'b0;
        en3 <= 1'b0;
        en4 <= 1'b0;
        o_en <= 1'b0;
    end else begin
        en1 <= i_en;
        en2 <= en1;
        en3 <= en2;
        en4 <= en3;
        o_en <= en4;
    end


always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        Kp0 <= 0;
        Ki0 <= 0;
        pdelta <= 0;
    end else begin
        if(i_en) begin
            Kp0 <= i_Kp;
            Ki0 <= i_Ki;
            pdelta <= $signed({{16{i_aim[15]}},i_aim}) - $signed({{16{i_real[15]}},i_real});   // pdelta = i_aim - i_real
        end
    end

always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        Ki1 <= 0;
        kpdelta1 <= 0;
        idelta <= 0;
    end else begin
        if (en1) begin
            Ki1 <= Ki0;
            kpdelta1 <= protect_mul(pdelta, $signed({1'h0, Kp0}) );                            // kpdelta1 = Kp * pdelta
            idelta  <= protect_add(idelta, pdelta);                                            // idelta  += (i_aim - i_real)      that is: idelta = Σ pdelta
        end
    end

always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        kpdelta <= 0;
        kidelta <= 0;
    end else begin
        if (en2) begin
            kpdelta <= kpdelta1;                                                               // kpdelta = Kp * pdelta
            kidelta <= protect_mul(idelta, $signed({1'h0, Ki1}) );                             // kidelta = Ki * Σ pdelta
        end
    end

always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        kpidelta <= 0;
    end else begin
        if (en3)
            kpidelta <= protect_add(kpdelta, kidelta);                                         // kpidelta = Kp * pdelta + Ki * Σ pdelta
    end

always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        value <= 0;
    end else begin
        if (en4)
            value <= kpidelta;                                                                 // modified at 20230609, now it is a stardard PID
            // value <= protect_add(value, kpidelta);                                          //
    end

endmodule
