`timescale 1 ns/1 ns

// 模块: as5600_read
// 功能：通过 I2C 接口从 AS5600 磁编码器中读出转子机械角度 φ
// 参数：详见下方注释
// 输入输出：详见下方注释
module as5600_read #(
    parameter   [15:0] CLK_DIV = 16'd10  // I2C SCL 时钟信号分频系数，SCL 时钟频率 = clk频率 / (4*CLK_DIV) ，例如若 clk 为 40MHz，CLK_DIV=10，则 SCL 频率为 40/(4*10) = 1MHz。注，AS5600 芯片要求 SCL 频率不超过 1MHz
)(
    input  wire        rstn,
    input  wire        clk,
    output wire        scl,              // I2C SCL 信号
    inout              sda,              // I2C SDA 信号
    output reg         o_en,             // 不断读取转子机械角度 φ，每读出一个新值，o_en 产生一个高电平脉冲
    output reg  [11:0] o_phi             // o_en 产生一个高电平脉冲的同时，o_phi 上产生最新的转自角度，取值范围0~4095。0对应0°；1024对应90°；2048对应180°；3072对应270°。
);

wire [15:0] regout;

assign o_phi = regout[11:0];

i2c_register_read #(
    .CLK_DIV ( CLK_DIV    )
) i2c_register_read_i (
    .rstn    ( rstn       ),
    .clk     ( clk        ),
    .scl     ( scl        ),
    .sda     ( sda        ),
    .start   ( 1'b1       ),
    .ready   (            ),
    .done    ( o_en       ),
    .regout  ( regout     )
);

endmodule
