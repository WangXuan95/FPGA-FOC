
//--------------------------------------------------------------------------------------------------------
// 模块： park_tr
// Type    : synthesizable
// Standard: Verilog 2001 (IEEE1364-2001)
// 功能： park 变换器
//--------------------------------------------------------------------------------------------------------

module park_tr(
    input  wire               rstn,
    input  wire               clk,
    input  wire        [11:0] psi,
    input  wire               i_en,
    input  wire signed [15:0] i_ialpha, i_ibeta,
    output reg                o_en,
    output reg  signed [15:0] o_id, o_iq
);

wire signed [15:0] sin_psi, cos_psi;  // -1~+1 is mapped to -16384~+16384

reg               en_s1;
reg signed [31:0] alpha_cos, alpha_sin, beta_cos, beta_sin;

wire signed[31:0] ide = alpha_cos + beta_sin;
wire signed[31:0] iqe = beta_cos  - alpha_sin;

sincos u_sincos (
    .rstn        ( rstn       ),
    .clk         ( clk        ),
    .i_en        ( 1'b1       ),
    .i_theta     ( psi        ),
    .o_en        (            ),
    .o_sin       ( sin_psi    ),
    .o_cos       ( cos_psi    )
);

always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        {en_s1, alpha_cos, alpha_sin, beta_cos, beta_sin} <= 0;
    end else begin
        en_s1 <= i_en;
        alpha_cos <= i_ialpha * cos_psi;
        alpha_sin <= i_ialpha * sin_psi;
        beta_cos  <= i_ibeta  * cos_psi;
        beta_sin  <= i_ibeta  * sin_psi;
    end

always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        {o_en, o_id, o_iq} <= 0;
    end else begin
        o_en <= en_s1;
        if(en_s1) begin
            o_id <= ide[31:16];
            o_iq <= iqe[31:16];
        end
    end

endmodule
