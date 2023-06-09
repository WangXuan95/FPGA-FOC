
//--------------------------------------------------------------------------------------------------------
// 模块: adc_ad7928
// Type    : synthesizable
// Standard: Verilog 2001 (IEEE1364-2001)
// 功能：通过 SPI 接口从 ADC7928 (ADC芯片) 中读出 ADC 值。
// 参数：详见下方注释，该模块可以使用参数完全自由地配置单次转换要用多少个通道以及用哪些通道
// 输入输出：详见下方注释
//--------------------------------------------------------------------------------------------------------

module adc_ad7928 #(
    parameter [2:0] CH_CNT = 3'd7,  // 单次 ADC 转换使用的通道数为 CH_CNT+1，例如若 CH_CNT=0，则只使用 CH0 。若 CH_CNT=2，则使用 CH0,CH1,CH2。 若 CH_CNT=7，则使用 CH0,CH1,CH2,CH3,CH4,CH5,CH6,CH7。用的通道越多，ADC转换时延越长（即从 sn_adc 到 en_adc 之间的时间差越长）
    parameter [2:0] CH0 = 3'd0,     // 指示了 CH0 对应 AD7928 的哪个通道
    parameter [2:0] CH1 = 3'd1,     // 指示了 CH1 对应 AD7928 的哪个通道
    parameter [2:0] CH2 = 3'd2,     // 指示了 CH2 对应 AD7928 的哪个通道
    parameter [2:0] CH3 = 3'd3,     // 指示了 CH3 对应 AD7928 的哪个通道
    parameter [2:0] CH4 = 3'd4,     // 指示了 CH4 对应 AD7928 的哪个通道
    parameter [2:0] CH5 = 3'd5,     // 指示了 CH5 对应 AD7928 的哪个通道
    parameter [2:0] CH6 = 3'd6,     // 指示了 CH6 对应 AD7928 的哪个通道
    parameter [2:0] CH7 = 3'd7      // 指示了 CH7 对应 AD7928 的哪个通道
) (
    input  wire        rstn,
    input  wire        clk,
    // -------------------- SPI 接口，应该接到 AD7928 芯片上   ---------------------------------------------------------------
    output reg         spi_ss,      // SPI 接口：SS
    output reg         spi_sck,     // SPI 接口：SCK
    output reg         spi_mosi,    // SPI 接口：MOSI
    input  wire        spi_miso,    // SPI 接口：MISO
    // -------------------- 用户逻辑接口  ------------------------------------------------------------------------------------
    input  wire        i_sn_adc,    // ADC 转换开始信号，当 i_sn_adc 上出现高电平脉冲时，ADC转换开始
    output reg         o_en_adc,    // ADC 转换完成信号，当转换完成时，o_en_adc 产生一个时钟周期的高电平脉冲
    output wire [11:0] o_adc_value0,// 当 o_en_adc 产生一个时钟周期的高电平脉冲时，CH0 的 ADC 转换结果出现在该信号上
    output wire [11:0] o_adc_value1,// 当 o_en_adc 产生一个时钟周期的高电平脉冲时，CH1 的 ADC 转换结果出现在该信号上
    output wire [11:0] o_adc_value2,// 当 o_en_adc 产生一个时钟周期的高电平脉冲时，CH2 的 ADC 转换结果出现在该信号上
    output wire [11:0] o_adc_value3,// 当 o_en_adc 产生一个时钟周期的高电平脉冲时，CH3 的 ADC 转换结果出现在该信号上
    output wire [11:0] o_adc_value4,// 当 o_en_adc 产生一个时钟周期的高电平脉冲时，CH4 的 ADC 转换结果出现在该信号上
    output wire [11:0] o_adc_value5,// 当 o_en_adc 产生一个时钟周期的高电平脉冲时，CH5 的 ADC 转换结果出现在该信号上
    output wire [11:0] o_adc_value6,// 当 o_en_adc 产生一个时钟周期的高电平脉冲时，CH6 的 ADC 转换结果出现在该信号上
    output wire [11:0] o_adc_value7 // 当 o_en_adc 产生一个时钟周期的高电平脉冲时，CH7 的 ADC 转换结果出现在该信号上
);

localparam WAIT_CNT = 8'd6;

wire [2:0] channels [0:7];
assign channels[0] = CH0;
assign channels[1] = CH1;
assign channels[2] = CH2;
assign channels[3] = CH3;
assign channels[4] = CH4;
assign channels[5] = CH5;
assign channels[6] = CH6;
assign channels[7] = CH7;

reg  [ 7:0] cnt;
reg  [ 2:0] idx;
reg  [ 2:0] addr;
reg  [11:0] wshift;
reg         nfirst;
reg  [11:0] data_in_latch;
reg         sck_pre;
reg  [11:0] ch_value [0:7];

assign o_adc_value0 = ch_value[0];
assign o_adc_value1 = ch_value[1];
assign o_adc_value2 = ch_value[2];
assign o_adc_value3 = ch_value[3];
assign o_adc_value4 = ch_value[4];
assign o_adc_value5 = ch_value[5];
assign o_adc_value6 = ch_value[6];
assign o_adc_value7 = ch_value[7];

always @ (posedge clk or negedge rstn)
    if(~rstn)
        spi_sck <= 1'b1;
    else
        spi_sck <= sck_pre;

always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        cnt <= 0;
        idx <= 3'd7;
        addr <= 3'd0;
        wshift <= 12'hFFF;
        {spi_ss, sck_pre, spi_mosi} <= 3'b111;
    end else begin
        if(cnt==8'd0) begin
            {spi_ss, sck_pre, spi_mosi} <= 3'b111;
            if(idx != 3'd0) begin
                cnt <= 8'd1;
                idx <= idx - 3'd1;
            end else if(i_sn_adc) begin
                cnt <= 8'd1;
                idx <= CH_CNT;
            end
        end else if(cnt==8'd1) begin
            {spi_ss, sck_pre, spi_mosi} <= 3'b111;
            addr <= (idx == 3'd0) ? CH_CNT : idx - 3'd1;
            cnt <= cnt + 8'd1;
        end else if(cnt==8'd2) begin
            {spi_ss, sck_pre, spi_mosi} <= 3'b111;
            wshift <= {1'b1, 1'b0, 1'b0, channels[addr], 2'b11, 1'b0, 1'b0, 2'b11};
            cnt <= cnt + 8'd1;
        end else if(cnt<WAIT_CNT) begin
            {spi_ss, sck_pre, spi_mosi} <= 3'b111;
            cnt <= cnt + 8'd1;
        end else if(cnt<WAIT_CNT+8'd32) begin
            spi_ss <= 1'b0;
            sck_pre <= ~sck_pre;
            if(sck_pre)
                {spi_mosi,wshift} <= {wshift,1'b1};
            cnt <= cnt + 8'd1;
        end else begin
            spi_ss <= 1'b0;
            {sck_pre, spi_mosi} <= 2'b11;
            cnt <= 8'd0;
        end
    end


always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        o_en_adc <= 1'b0;
        nfirst <= 1'b0;
        data_in_latch <= 12'd0;
        ch_value[0] <= 12'd0;
        ch_value[1] <= 12'd0;
        ch_value[2] <= 12'd0;
        ch_value[3] <= 12'd0;
        ch_value[4] <= 12'd0;
        ch_value[5] <= 12'd0;
        ch_value[6] <= 12'd0;
        ch_value[7] <= 12'd0;
    end else begin
        o_en_adc <= 1'b0;
        if(cnt>=WAIT_CNT+8'd2 && cnt<WAIT_CNT+8'd32) begin
            if(spi_sck)
                data_in_latch <= {data_in_latch[10:0], spi_miso};
        end else if(cnt==WAIT_CNT+8'd32) begin
            if(idx == 3'd0) begin
                nfirst <= 1'b1;
                o_en_adc <= nfirst;
            end
            ch_value[idx] <= data_in_latch;
        end
    end

endmodule
