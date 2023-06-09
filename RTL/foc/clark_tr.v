
//--------------------------------------------------------------------------------------------------------
// 模块： clark_tr 
// Type    : synthesizable
// Standard: Verilog 2001 (IEEE1364-2001)
// 功能： clark 变换
//--------------------------------------------------------------------------------------------------------

module clark_tr(
    input  wire               rstn,
    input  wire               clk,
    input  wire               i_en,
    input  wire signed [15:0] i_ia, i_ib, i_ic,   // range -8191 ~ 8191
    output reg                o_en,
    output reg  signed [15:0] o_ialpha, o_ibeta
);

// registers for pipeline stage 1
reg               en_s1;
reg signed [15:0] ax2_s1, bmc_s1, bpc_s1;

// registers for pipeline stage 2
reg               en_s2;
reg signed [15:0] ialpha_s2, i_beta1_s2, i_beta2_s2, i_beta3_s2;

// pipeline stage 1
always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        {en_s1, ax2_s1, bmc_s1, bpc_s1} <= 0;
    end else begin
        en_s1 <= i_en;
        ax2_s1 <= i_ia << 1;
        bmc_s1 <= i_ib - i_ic;
        bpc_s1 <= i_ib + i_ic;
    end

// pipeline stage 2
always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        {en_s2, ialpha_s2, i_beta1_s2, i_beta2_s2, i_beta3_s2} <= 0;
    end else begin
        en_s2 <= en_s1;
        ialpha_s2 <= ax2_s1 - bpc_s1;
        i_beta1_s2 <= bmc_s1                                     +
                      $signed({{ 1{bmc_s1[15]}}, bmc_s1[15: 1]}) +
                      $signed({{ 3{bmc_s1[15]}}, bmc_s1[15: 3]});
        i_beta2_s2 <= $signed({{ 4{bmc_s1[15]}}, bmc_s1[15: 4]}) +
                      $signed({{ 5{bmc_s1[15]}}, bmc_s1[15: 5]}) +
                      $signed({{ 7{bmc_s1[15]}}, bmc_s1[15: 7]});
        i_beta3_s2 <= $signed({{ 8{bmc_s1[15]}}, bmc_s1[15: 8]}) +
                      $signed({{10{bmc_s1[15]}}, bmc_s1[15:10]}) +
                      $signed({{11{bmc_s1[15]}}, bmc_s1[15:11]});
    end

// pipeline stage output
always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        {o_en, o_ialpha, o_ibeta} <= 1'b0;
    end else begin
        o_en <= en_s2;
        if(en_s2) begin
            o_ialpha <= ialpha_s2;
            o_ibeta <= i_beta1_s2 + i_beta2_s2 + i_beta3_s2;
        end
    end

endmodule
