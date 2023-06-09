
//--------------------------------------------------------------------------------------------------------
// fpga_top
// Type    : synthesizable, FPGA's top
// Standard: Verilog 2001 (IEEE1364-2001)
// 功能：FOC 使用示例，是FPGA工程的顶层模块，控制电机的切向力矩一会顺时针一会逆时针，同时可以通过 UART 监测电流环控制的跟随曲线
// 参数：无
// 输入输出：详见下方注释
//--------------------------------------------------------------------------------------------------------

module fpga_top (
    input  wire clk_50m, // 连接 50MHz 晶振
    // ------- 3相 PWM 信号，（包含使能信号） -----------------------------------------------------------------------------------------------------
    output wire pwm_en,  // 3相共用的使能信号，当 pwm_en=0 时，6个MOS管全部关断。
    output wire pwm_a,   // A相PWM信号。当 =0 时。下桥臂导通；当 =1 时，上桥臂导通
    output wire pwm_b,   // B相PWM信号。当 =0 时。下桥臂导通；当 =1 时，上桥臂导通
    output wire pwm_c,   // C相PWM信号。当 =0 时。下桥臂导通；当 =1 时，上桥臂导通
    // ------- AD7928 (ADC 芯片) ，用于相电流检测 (SPI接口) ---------------------------------------------------------------------------------------
    output wire spi_ss,
    output wire spi_sck,
    output wire spi_mosi,
    input  wire spi_miso,
    // ------- AS5600 磁编码器，用于获取转子机械角度 (I2C接口) ------------------------------------------------------------------------------------
    output wire i2c_scl,
    inout       i2c_sda,
    // ------- UART: 打印转子直角坐标系下的 d 轴实际电流(id)、d 轴目标电流(id_aim)、q 轴实际电流(iq)、q 轴目标电流(iq_aim) ------------------------
    output wire uart_tx
);


wire               rstn;        // 复位信号，初始为0，当PLL锁相成功后置1。
wire               clk;         // 时钟信号，频率可取几十MHz。控制频率 = 时钟频率 / 2048。比如本例取时钟频率为 36.864MHz ，那么控制频率为 36.864MHz/2048=18kHz。（控制频率 = 3相电流采样的采样率 = PID算法的控制频率 = SVPWM占空比的更新频率）

wire        [11:0] phi;         // 从 AS5600 磁编码器读出的转子机械角度 φ ，取值范围0~4095。0对应0°；1024对应90°；2048对应180°；3072对应270°。

wire               sn_adc;      // 3相电流 ADC 采样时刻控制信号，当需要进行一次采样时，sn_adc 信号上产生一个时钟周期的高电平脉冲，指示ADC应该进行采样了。
wire               en_adc;      // 3相电流 ADC 采样结果有效信号，sn_adc 产生高电平脉冲后，adc_ad7928 模块开始采样3相电流，在转换结束后，在 en_adc 信号上产生一个周期的高电平脉冲，同时把 ADC 转换结果产生在 adc_value_a, adc_value_b, adc_value_c 信号上。
wire        [11:0] adc_value_a; // A 相电流检测 ADC 原始值
wire        [11:0] adc_value_b; // B 相电流检测 ADC 原始值
wire        [11:0] adc_value_c; // C 相电流检测 ADC 原始值

wire               en_idq;      // 出现高电平脉冲时说明 id 和 iq 出现了新值，每个控制周期 en_idq 会产生一个高电平脉冲
wire signed [15:0] id;          // 转子 d 轴（直轴）的实际电流值，
wire signed [15:0] iq;          // 转子 q 轴（交轴）的实际电流值，可正可负（若正代表逆时针，则负代表顺时针，反之亦然）
wire signed [15:0] id_aim;      // 转子 d 轴（直轴）的目标电流值，可正可负
reg  signed [15:0] iq_aim;      // 转子 q 轴（交轴）的目标电流值，可正可负（若正代表逆时针，则负代表顺时针，反之亦然）



// PLL，用 50MHz 时钟（clk_50m）产生 36.864 MHz 时钟（clk）
// 注：该模块仅适用于 Altera Cyclone IV FPGA ，对于其他厂家或系列的FPGA，请使用各自相同效果的IP核/原语（例如Xilinx的clock wizard）代替该模块。
wire [3:0] subwire0;
altpll u_altpll ( .inclk ( {1'b0, clk_50m} ), .clk ( {subwire0, clk} ), .locked ( rstn ),  .activeclock (),  .areset (1'b0), .clkbad (),  .clkena ({6{1'b1}}),  .clkloss (), .clkswitch (1'b0), .configupdate (1'b0), .enable0 (), .enable1 (),  .extclk (),  .extclkena ({4{1'b1}}), .fbin (1'b1), .fbmimicbidir (),  .fbout (), .fref (), .icdrclk (), .pfdena (1'b1), .phasecounterselect ({4{1'b1}}), .phasedone (), .phasestep (1'b1), .phaseupdown (1'b1),  .pllena (1'b1), .scanaclr (1'b0), .scanclk (1'b0), .scanclkena (1'b1), .scandata (1'b0), .scandataout (),  .scandone (), .scanread (1'b0), .scanwrite (1'b0), .sclkout0 (), .sclkout1 (), .vcooverrange (), .vcounderrange ());
defparam u_altpll.bandwidth_type = "AUTO", u_altpll.clk0_divide_by = 99,  u_altpll.clk0_duty_cycle = 50, u_altpll.clk0_multiply_by = 73, u_altpll.clk0_phase_shift = "0", u_altpll.compensate_clock = "CLK0", u_altpll.inclk0_input_frequency = 20000, u_altpll.intended_device_family = "Cyclone IV E",  u_altpll.lpm_hint = "CBX_MODULE_PREFIX=pll", u_altpll.lpm_type = "altpll",  u_altpll.operation_mode = "NORMAL",  u_altpll.pll_type = "AUTO", u_altpll.port_activeclock = "PORT_UNUSED",  u_altpll.port_areset = "PORT_UNUSED",  u_altpll.port_clkbad0 = "PORT_UNUSED",  u_altpll.port_clkbad1 = "PORT_UNUSED", u_altpll.port_clkloss = "PORT_UNUSED", u_altpll.port_clkswitch = "PORT_UNUSED", u_altpll.port_configupdate = "PORT_UNUSED", u_altpll.port_fbin = "PORT_UNUSED", u_altpll.port_inclk0 = "PORT_USED",  u_altpll.port_inclk1 = "PORT_UNUSED", u_altpll.port_locked = "PORT_USED", u_altpll.port_pfdena = "PORT_UNUSED",  u_altpll.port_phasecounterselect = "PORT_UNUSED", u_altpll.port_phasedone = "PORT_UNUSED", u_altpll.port_phasestep = "PORT_UNUSED", u_altpll.port_phaseupdown = "PORT_UNUSED",  u_altpll.port_pllena = "PORT_UNUSED", u_altpll.port_scanaclr = "PORT_UNUSED",  u_altpll.port_scanclk = "PORT_UNUSED", u_altpll.port_scanclkena = "PORT_UNUSED", u_altpll.port_scandata = "PORT_UNUSED", u_altpll.port_scandataout = "PORT_UNUSED", u_altpll.port_scandone = "PORT_UNUSED", u_altpll.port_scanread = "PORT_UNUSED", u_altpll.port_scanwrite = "PORT_UNUSED", u_altpll.port_clk0 = "PORT_USED", u_altpll.port_clk1 = "PORT_UNUSED", u_altpll.port_clk2 = "PORT_UNUSED",  u_altpll.port_clk3 = "PORT_UNUSED", u_altpll.port_clk4 = "PORT_UNUSED", u_altpll.port_clk5 = "PORT_UNUSED", u_altpll.port_clkena0 = "PORT_UNUSED",  u_altpll.port_clkena1 = "PORT_UNUSED", u_altpll.port_clkena2 = "PORT_UNUSED", u_altpll.port_clkena3 = "PORT_UNUSED",  u_altpll.port_clkena4 = "PORT_UNUSED", u_altpll.port_clkena5 = "PORT_UNUSED", u_altpll.port_extclk0 = "PORT_UNUSED", u_altpll.port_extclk1 = "PORT_UNUSED",  u_altpll.port_extclk2 = "PORT_UNUSED",  u_altpll.port_extclk3 = "PORT_UNUSED",  u_altpll.self_reset_on_loss_lock = "OFF",  u_altpll.width_clock = 5;
//assign rstn=1'b1; assign clk=clk_50m;


// 简易 I2C 读取控制器，实现 AS5600 磁编码器读取，读出当前转子机械角度 φ
wire [3:0] i2c_trash;    // 丢弃的高4位
i2c_register_read #(
    .CLK_DIV      ( 16'd10         ),  // i2c_scl 时钟信号分频系数，scl频率 = clk频率 / (4*CLK_DIV) ，例如在本例中 clk 为 36.864MHz，CLK_DIV=10，则 SCL 频率为 36864/(4*10) = 922kHz 。注，AS5600 芯片要求 SCL 频率不超过 1MHz
    .SLAVE_ADDR   ( 7'h36          ),  // AS5600's I2C slave address
    .REGISTER_ADDR( 8'h0E          )   // the register address to read
) u_as5600_read (
    .rstn         ( rstn           ),
    .clk          ( clk            ),
    .scl          ( i2c_scl        ), // I2C 接口： SCL
    .sda          ( i2c_sda        ), // I2C 接口： SDA
    .start        ( 1'b1           ), // 持续进行 I2C 读操作
    .ready        (                ),
    .done         (                ),
    .regout       ( {i2c_trash, phi} )
);



// AD7928 ADC 读取器，用于读取3相电流采样值（读出的是未经任何处理的ADC原始值）
adc_ad7928 #(
    .CH_CNT       ( 3'd2           ), // 该参数取2，指示我们只想要 CH0, CH1, CH2 这三个通道的 ADC 值
    .CH0          ( 3'd1           ), // 指示 CH0 对应 AD7928 的 通道1。（硬件上 A 相电流连接到 AD7928 的 通道1）
    .CH1          ( 3'd2           ), // 指示 CH1 对应 AD7928 的 通道2。（硬件上 B 相电流连接到 AD7928 的 通道2）
    .CH2          ( 3'd3           )  // 指示 CH2 对应 AD7928 的 通道3。（硬件上 C 相电流连接到 AD7928 的 通道3）
) u_adc_ad7928 (
    .rstn         ( rstn           ),
    .clk          ( clk            ),
    .spi_ss       ( spi_ss         ), // SPI 接口：SS
    .spi_sck      ( spi_sck        ), // SPI 接口：SCK
    .spi_mosi     ( spi_mosi       ), // SPI 接口：MOSI
    .spi_miso     ( spi_miso       ), // SPI 接口：MISO
    .i_sn_adc     ( sn_adc         ), // input : 当 sn_adc 出现高电平脉冲时，该模块开始进行一次（3路的）ADC 转换
    .o_en_adc     ( en_adc         ), // output: 当转换结束后，en_adc 产生一个周期的高电平脉冲
    .o_adc_value0 ( adc_value_a    ), // 当 en_adc 产生一个周期的高电平脉冲的同时，adc_value_a 上出现 A 相电流的 ADC 原始值
    .o_adc_value1 ( adc_value_b    ), // 当 en_adc 产生一个周期的高电平脉冲的同时，adc_value_b 上出现 B 相电流的 ADC 原始值
    .o_adc_value2 ( adc_value_c    ), // 当 en_adc 产生一个周期的高电平脉冲的同时，adc_value_c 上出现 C 相电流的 ADC 原始值
    .o_adc_value3 (                ), // 忽略其余 5 路 ADC 转换结果
    .o_adc_value4 (                ), // 忽略其余 5 路 ADC 转换结果
    .o_adc_value5 (                ), // 忽略其余 5 路 ADC 转换结果
    .o_adc_value6 (                ), // 忽略其余 5 路 ADC 转换结果
    .o_adc_value7 (                )  // 忽略其余 5 路 ADC 转换结果
);



// FOC + SVPWM 模块 （使用方法和原理详见 foc_top.sv）
foc_top #(
    .INIT_CYCLES  ( 16777216       ), // 本例中，时钟(clk)频率为 36.864MHz，INIT_CYCLES=16777216，则初始化时间为 16777216/36864000=0.45 秒
    .ANGLE_INV    ( 1'b0           ), // 本例中，角度传感器没装反（A->B->C->A 的旋转方向与 φ 增大的方向相同），则该参数应设为 0
    .POLE_PAIR    ( 8'd7           ), // 本例使用的电机的极对数为 7
    .MAX_AMP      ( 9'd384         ), // 384 / 512 = 0.75。说明 SVPWM 的最大振幅 占 最大振幅极限的 75%
    .SAMPLE_DELAY ( 9'd120         )  // 采样延时，取值范围0~511，考虑到3相的驱动 MOS 管从开始导通到电流稳定需要一定的时间，所以从3个下桥臂都导通，到 ADC 采样时刻之间需要一定的延时。该参数决定了该延时是多少个时钟周期，当延时结束时，该模块在 sn_adc 信号上产生一个高电平脉冲，指示外部 ADC “可以采样了”
) u_foc_top (
    .rstn         ( rstn           ),
    .clk          ( clk            ),
    .Kp           ( 31'd300000     ), // 电流环 PID 控制算法的 P 参数
    .Ki           ( 31'd30000      ), // 电流环 PID 控制算法的 I 参数
    .phi          ( phi            ), // input : 角度传感器输入（机械角度，简记为φ），取值范围0~4095。0对应0°；1024对应90°；2048对应180°；3072对应270°。
    .sn_adc       ( sn_adc         ), // output: 3相电流 ADC 采样时刻控制信号，当需要进行一次采样时，sn_adc 信号上产生一个时钟周期的高电平脉冲，指示ADC应该进行采样了。
    .en_adc       ( en_adc         ), // input : 3相电流 ADC 采样结果有效信号，sn_adc 产生高电平脉冲后，外部ADC开始采样3相电流，在转换结束后，应在 en_adc 信号上产生一个周期的高电平脉冲，同时把ADC转换结果产生在 adc_a, adc_b, adc_c 信号上
    .adc_a        ( adc_value_a    ), // input : A 相 ADC 采样结果
    .adc_b        ( adc_value_b    ), // input : B 相 ADC 采样结果
    .adc_c        ( adc_value_c    ), // input : C 相 ADC 采样结果
    .pwm_en       ( pwm_en         ),
    .pwm_a        ( pwm_a          ),
    .pwm_b        ( pwm_b          ),
    .pwm_c        ( pwm_c          ),
    .en_idq       ( en_idq         ), // output: 出现高电平脉冲时说明 id 和 iq 出现了新值，每个控制周期 en_idq 会产生一个高电平脉冲
    .id           ( id             ), // output: d 轴（直轴）的实际电流值，可正可负
    .iq           ( iq             ), // output: q 轴（交轴）的实际电流值，可正可负（若正代表逆时针，则负代表顺时针，反之亦然）
    .id_aim       ( id_aim         ), // input : d 轴（直轴）的目标电流值，可正可负，在不使用弱磁控制的情况下一般设为0
    .iq_aim       ( iq_aim         ), // input : q 轴（直轴）的目标电流值，可正可负（若正代表逆时针，则负代表顺时针，反之亦然）
    .init_done    (                )  // output: 初始化结束信号。在初始化结束前=0，在初始化结束后（进入FOC控制状态）=1
);



reg [23:0] cnt;
always @ (posedge clk or negedge rstn)   // 该 always 维护一个 24bit 的 自增计数器
    if(~rstn)
        cnt <= 24'd0;
    else
        cnt <= cnt + 24'd1;


assign id_aim = $signed(16'd0);          // 令 id_aim 恒等于 0

always @ (posedge clk or negedge rstn)   // 该 always 块令 iq_aim 交替地取 +200 和 -200 ，即电机的切向力矩一会顺时针一会逆时针
    if(~rstn) begin
        iq_aim <= $signed(16'd0);
    end else begin
        if(cnt[23])
            iq_aim <=  $signed(16'd200); // 令 id_aim = +200
        else
            iq_aim <= -$signed(16'd200); // 令 id_aim = -200
    end



// UART 发送器(监视器)，格式为：115200,8,n,1
uart_monitor #(
    .CLK_DIV      ( 16'd320        )   // UART分频倍率，在本例中取320。因为时钟频率为 36.864MHz, 36.864MHz/320=115200
) u_uart_monitor (
    .rstn         ( rstn           ),
    .clk          ( clk            ),
    .i_en         ( en_idq         ),  // input: 当 en_idq 上出现高电平脉冲时，启动 UART 发送
    .i_val0       ( id             ),  // input: 以十进制的形式发送变量 id
    .i_val1       ( id_aim         ),  // input: 以十进制的形式发送变量 id_aim
    .i_val2       ( iq             ),  // input: 以十进制的形式发送变量 iq
    .i_val3       ( iq_aim         ),  // input: 以十进制的形式发送变量 iq_aim
    .o_uart_tx    ( uart_tx        )   // output: UART 发送信号
);


endmodule
