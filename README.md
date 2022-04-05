![语言](https://img.shields.io/badge/语言-systemverilog_(IEEE1800_2005)-CAD09D.svg) ![仿真](https://img.shields.io/badge/仿真-iverilog-green.svg) ![部署](https://img.shields.io/badge/部署-quartus-blue.svg) ![部署](https://img.shields.io/badge/部署-vivado-FF1010.svg)

FpOC
===========================
基于 **FPGA** 的**磁场定向控制 (FOC)**，用于驱动**永磁同步电机 (PMSM)** 或**无刷直流电机 (BLDC)**

**FOC控制算法**对传感器采样速率和处理器算力提出了一定的要求，使用 **FPGA** 实现的 **FOC** 可以获得更好的**实时性**，并且更方便进行**多路扩展**和**多路反馈协同**。

本库实现了基于**角度传感器**（也就是磁编码器）的**有感 FOC**，即一个完整的**电流环**，可以进行**扭矩控制**。借助本库，你可以进一步使用 **纯FPGA** 或 **MCU+FPGA** 的方式实现更复杂的电机应用。

| ![diagram](./figures/diagram.png) |
| :---: |
| **图1**：系统框图 |

本库代码有详细的注释，如果你熟悉 Verilog 但不熟悉 FOC ，可以通过阅读代码来快速地学习 FOC （建议先阅读 FOC 原理 [6~9]）。一些用户在读代码时向本人反馈了一些疑问，我将它们整理在了 [FAQ](#FAQ) 里。

### 技术特点

* **平台无关** ：纯 RTL 编写，可以在 Altera 和 Xilinx 等各种 FPGA 上运行。
* 支持 **3路PWM** + **1路EN** ：PWM=1 时上桥臂 MOS 导通，PWM=0 时下桥臂 MOS 导通。 EN=0 时所有的 6 个 MOS 关断。
* 支持 **12bit 分辨率**的**角度传感器**和**相电流采样ADC**。对于>12bit的传感器，需要进行低位截断。对于<12bit的传感器，需要进行低位填充。
* 内部使用 **16bit 有符号整数**进行计算，考虑到传感器为 12bit，所以 16bit 计算是够用的。

### 目录

- [示例程序：让电机转起来](#示例程序：让电机转起来)
  - [搭建硬件](#搭建硬件)
  - [建立FPGA工程](#建立FPGA工程)
  - [调参](#调参)
  - [用串口监视电流环](#用串口监视电流环)
- [设计代码详解](#设计代码详解)
- [RTL仿真](#RTL仿真)
  - [clark_tr和park_tr子模块的仿真](clark_tr和park_tr的仿真)
  - [cartesian2polar和svpwm子模块的仿真](#cartesian2polar和svpwm的仿真)
- [FAQ](#FAQ)



# 示例程序：让电机转起来

**图1** 是示例程序的系统框图，它调用了我实现的 FOC 电流环模块 (foc_top.sv) 来实现一个简单的行为——控制电机的电流（扭矩）按顺时针、逆时针、顺时针、逆时针地交替运行。同时，使用 UART 打印电流的**控制目标值**和**实际值**，以便观察 FOC 电流环控制的质量。

该示例的所有代码都在 RTL 目录内。工程在 FPGA 目录内（这是一个 Quartus 工程，但由于本库是可移植性设计，也可以很容易地部署到 VIvado 上，只需要把不可移植的 PLL 核替换掉）。



## 搭建硬件

运行本库的电机驱动需要的硬件包括：

* **FPGA 开发板** ：需有至少 **10** 个 **GPIO** ，用来接：
    * **I2C (2*GPIO)** ， 用来连接 AS5600 磁编码器。
    * **SPI (4*GPIO)** ， 用来连接 AD7928 ADC。
    * **PWM (3*GPIO)** ， 用来输出 3 相 PWM 到电机驱动板。
    * **PWM_EN (1*GPIO)** ， 用来输出 1 路 EN (使能) 到电机驱动板（EN=低电平代表所有桥臂关断）。
    * **UART (1*GPIO)** ，单向(仅发送)的UART，连接计算机的串口，用于监测电流环的跟随曲线，**可不接**。
* **PMSM** 或 **BLDC 电机**。
* **电机驱动板**：要支持3相 PWM 输入信号（PWM=0时下桥臂导通，PWM=1时上桥臂导通），并且要内置**低侧电阻采样+放大器**对 3 相电流进行放大。
* **获取转子角度的磁编码器**：本库直接支持的型号是 AS5600 ，需要安装在电机上。
* **相电流采样的ADC**：本库直接支持的型号是 AD7928 ，用于采样电机驱动板放大后的 3 相电流
    * 网上似乎找不到现成卖的 AD7928 模块，所以如果你想DIY，就需要自己画 PCB。

如果你硬件动手能力很强，可以自己准备以上硬件。

不过！建议直接使用我画的一个自带 **AD7928** 的 **电机驱动板**，它的原理图如**图2**，我也提供了制造文件 [gerber_pcb_foc_shield.zip](./gerber_pcb_foc_shield.zip) ，你需要拿制造文件去打样PCB，然后按照**图2**来焊接。

> 注意，这个板子包括了**相电流采样的ADC**和**电机驱动板**的功能，但不包括 AS5600 磁编码器，磁编码器是需要安装在电机上的，需要你额外准备。

|                 ![wave](./figures/sch.png)                  |
| :---------------------------------------------------------: |
| **图2**：电机驱动板原理图，其中 AD7928 ADC 用来采样三相电流 |

这个板子在立创EDA开源，见 [oshwhub.com/wangxuan/arduino-foc-shield](https://oshwhub.com/wangxuan/arduino-foc-shield)

这个板子之所以被设计成 Arduino 扩展板的样子，是因为很多 FPGA 开发板也具有 Arduino 扩展板接口（比如 DE10-Nano 开发板），可以直接插上去。如果你的 FPGA 开发板没有 Arduino 扩展板接口也没关系，直接用杜邦线连接即可（建议用短的杜邦线）。

这里使用的电机驱动器是 **MP6540** 芯片, 我只试过驱动小功率云台电机，没试过大功率电机。



## 建立FPGA工程

你需要建立 FPGA 工程，把 RTL 目录（包括其子目录）里的所有 .sv 源文件加入工程，请以 fpga_top.sv 作为顶层文件。

### 时钟配置

fpga_top.sv 中有一处调用了 altpll 原语，用来把开发板晶振输入的 50MHz 时钟（clk\_50m 信号）变成 36.864MHz 的主时钟（clk 信号），altpll 原语只适用于 Altera Cyclone IV FPGA，如果你用的是其它系列的 FPGA，需要使用它们各自的 IP 核或原语（例如 Xilinx 的 clock wizard）替换它。

若你的开发板的晶振不是 50MHz ，你需要修改 PLL 的配置，保证主时钟 clk 信号的频率是 36.864MHz 即可。

实际上，主时钟 clk 的频率可以取小于 40MHz 的任意值。clk 是 FOC 系统的驱动时钟，clk 的频率会决定 SVPWM 的频率（SVPWM频率=clk 频率/2048），我选 36.864MHz 是因为可以让 SVPWM 频率 = 36864/2048 = 18kHz，只是为了凑个整数。

clk 的频率不能超过 40MHz 的原因是 adc_ad7928.sv 会通过二分频来产生 SPI 时钟（spi_sck），而 ADC7928 芯片要求 SPI 时钟不能超过 20MHz。

### 引脚约束

fpga_top.sv 的 IO 连接方法如下：

* **clk_50m** ：  连接在 FPGA 开发板的晶振上。
* **i2c_scl, i2c_sda** ： 连接 AS5600 (磁编码器) 的 I2C 接口。
* **spi_ss, spi_sck, spi_mosi, spi_miso** ： 连接 AD7928 (ADC芯片) 的 SPI 接口。 
* **pwm_a, pwm_b, pwm_c** ： 连接电机驱动板的 3 相 PWM 信号。
* **pwm_en** : 连接电机驱动板的 EN (使能) 信号。
    * 如果电机驱动板没有 EN 输入，则不接。
    * 如果电机驱动板有 3 路 EN 输入，每路对应 1 相，则应该进行一对三连接。
* **uart_tx** ： 连接 UART 转 USB 模块，插入计算机的 USB 口，用于监测电流环的跟随曲线，**可不接**。



## 调参

要让电机正常工作，你需要在 fpga_top.sv 中的第103行开始，根据实际情况调整 foc_top 模块的参数（Verilog parameter），包括：

- INIT_CYCLES： 决定了初始化步骤占多少个时钟(clk)周期，取值范围为1~4294967294。该值不能太短，因为要留足够的时间让转子回归电角度=0。在clk频率为 36.864MHz 的情况下，我们可以取 INIT_CYCLES=16777216，则初始化时间为 16777216/36864000=0.45 秒。
- ANGLE_INV：
  - 若角度传感器没装反（A相→B相→C相→A相 的旋转方向与角度传感器的读值的增大方向相同），则该参数应取 0
  - 若角度传感器装反了（A相→B相→C相→A相 的旋转方向与 角度传感器的读值的增大方向相反），则该参数应取 1
- POLE_PAIR：电机极对数，取值范围1~255，根据电机型号决定（注意：电角度ψ = 极对数N * 机械角度φ）
- MAX_AMP：SVPWM 的最大振幅，取值范围为1~511，该值越小，电机能达到的最大力矩越小；但考虑到使用3相下桥臂电阻采样法来采样电流，该值也不能太大，以保证3个下桥臂有足够的持续导通时间来供ADC进行采样。在本例中，使用默认值 9'd384 即可。
- SAMPLE_DELAY：采样延时，取值范围0~511，考虑到3相的驱动 MOS 管从开始导通到电流稳定需要一定的时间，所以从3个下桥臂都导通，到 ADC 采样时刻之间需要一定的延时。该参数决定了该延时是多少个时钟周期，当延时结束时，该模块在 sn_adc 信号上产生一个高电平脉冲，指示外部 ADC “可以采样了”。在本例中，使用默认值 9'd120 即可。
- Kp：PID 的 P 参数。
- Ki：PID 的 I 参数。

调好参后，综合并烧录到 FPGA 后，应该能看到电机正反交替运行。



## 用串口监视电流环

把 uart_tx 信号通过 **UART 转 USB 模块** （例如 CP2102、CH340 模块） 连接到电脑上，就可以用**串口助手**、**Putty**等软件来监测电流环的跟随效果。

> 注: UART 的格式是 115200,8,n,1

以下是串口打印的部分信息。其中第1~4列分别为：**d轴电流的实际值**，**d轴电流的目标值**，**q轴电流的实际值**，**q轴电流的目标值**。可以看到，即使目标值从+200突变到-200，实际值能跟着目标值走，说明电流环的 PID 控制是有效的。


     -5       0     206     200 
    -16       0     202     200 
     16       0     192     200 
     15       0     201     200 
      1       0     197     200 
     17       0    -211    -200 
     -6       0    -199    -200 
    -10       0    -210    -200 
     -3       0    -207    -200 
      0       0    -202    -200 
    -15       0    -211    -200 


另外，你可以借用 **Arduino IDE** 的**串口绘图器**来实时显示电流跟随曲线。前往[Arduino官网](https://www.arduino.cc/en/software)下载 **Arduino IDE**，安装后打开，在“**工具→端口**”中选择正确的COM口，然后点击“**工具→串口绘图器**”，**串口绘图器**会自动接收串口并使用上述4列数据画实时曲线图。

**图3** 是我这里绘制出的电流跟随曲线。蓝色曲线是第1列数据（d轴电流的实际值）；红色曲线是第2列数据（d轴电流的目标值）；绿色曲线是第3列数据（q轴电流的实际值）；土黄色曲线是第4列数据（q轴电流的目标值）。可以看到实际值能跟着目标值走。

| ![wave](./figures/wave.png) |
| :---: |
| **图3**：电流跟随曲线 |



# 设计代码详解

下表罗列了该工程使用的所有 **SystemVerilog** 代码文件，这些文件都在 RTL 目录下。结合**图1**就能看出每个模块的作用。

| 文件名 | 功能 | 备注 |
| :-- |   :-- |   :-- |
| fpga_top.sv | FPGA工程的顶层模块 |  |
| uart_monitor.sv | UART 发送器，用于数据监测 | 不需要的话可以移除       |
| i2c_register_read.sv | I2C 读取器，用于读取 AS5600 磁编码器 | |
| adc_ad7928.sv | AD7928 读取器 | |
| foc_top.sv | FOC+SVPWM （即**图1**中的蓝色部分的顶层） | 固定功能，一般不需要改动 |
| clark_tr.sv | Clark 变换 | 固定功能，一般不需要改动 |
| park_tr.sv | Park 变换  | 固定功能，一般不需要改动 |
| sincos.sv | 正弦/余弦计算器，被 park_tr.sv 调用 | 固定功能，一般不需要改动 |
| pi_controller.sv | PI 控制器（PID没有D） | 固定功能，一般不需要改动 |
| cartesian2polar.sv | 直角坐标系转极坐标系 | 固定功能，一般不需要改动 |
| svpwm.sv | SVPWM 调制器 | 固定功能，一般不需要改动 |
| hold_detect.sv | 监测3个下桥臂都导通时，延迟一段时间后触发 sn_adc 信号，指示ADC可以开始采样 | 固定功能，一般不需要改动 |


| ![diagram](./figures/diagram.png) |
| :---: |
| **图1**：系统框图 |

**图1**展示了这些模块的层次，我在设计模块层次时充分考虑了封装的合理性和代码重用 ：

* **粉色**部分是FPGA内的即传感器控制器，是**硬件相关逻辑**，如果角度传感器和 ADC 型号变了，这部分代码需要重写。
* **蓝色**部分是FPGA内的 FOC 的固定算法，是**硬件无关逻辑**，一般不需要修改，是本库的核心代码！
* **黄色**部分是FPGA内的**用户自定逻辑**，用户可以修改 user behavior 来实现各种电机应用。或者修改 uart_monitor 来监测其它变量。
* **淡橙色**部分是FPGA外部的硬件电路，也就是电机、电机驱动板、角度传感器这些东西。

另外，除了 fpga_top.sv 中调用的 altpll 原语外，该库的所有代码都使用纯 RTL 编写，可以轻易地移植到其它厂商（Xilinx、Lattice等）的 FPGA 上。



# RTL仿真

因为我并没有电机的 Verilog 模型，没法对整个 FOC 算法进行仿真，所以只对 FOC 中的部分模块进行了仿真。

仿真相关的文件都在 SIM 文件夹里，其中包括文件：

| 文件名                            | 功能                                                         |
| --------------------------------- | ------------------------------------------------------------ |
| tb_clark_park_tr.sv               | 对 clark_tr.sv （clark变换） 和 park_tr.sv （park变换）的仿真程序 |
| tb_clark_park_tr_run_iverilog.bat | 用 iverilog 运行 tb_clark_park_tr.sv 的命令脚本              |
| tb_svpwm.sv                       | 对 cartesian2polar.sv 和 svpwm.sv 和 park_tr.sv 的仿真程序   |
| tb_svpwm_run_iverilog.bat         | 用 iverilog 运行 tb_svpwm.sv 的命令脚本                      |

使用 iverilog 仿真前，需要安装 iverilog ，见：[iverilog_usage](https://github.com/WangXuan95/WangXuan95/blob/main/iverilog_usage/iverilog_usage.md)

## clark_tr和park_tr的仿真

我们先来运行 clark 变换和 park 变换的仿真。

双击 tb_clark_park_tr_run_iverilog.bat 可以直接运行仿真，运行完后会生成波形文件 dump.vcd 。请用 gtkwave 打开 dump.vcd ，导入**图4**中的这些信号，可以看到如**图4**的波形（需要你把这些信号改成模拟信号显示的形式，才能看得到**图4**这种效果：第一步，对于 Verilog 代码中声明为 signed 的信号（有符号数），需要右键该信号→Data Format→Signed Decimal。第二步，右键该信号→Data Format→Analog→Step ，即可把它变成如图的模拟信号的样子）。

对该**图4**波形的解读： 

- theta 是一个不断递增的角度（0→2π→0→2π→...）
- ia, ib, ic 是用 theta 生成的正弦波，相位各自相差 (2/3)*π （也就是说 ia, ib, ic）构成了三相正弦波。
- 使用 clark 变换把 ia, ib, ic 变换成 ialpha, ibeta ，得到一对正交的正弦波（相位相差 π/2 ）。
- 使用 park 变换把 ialpha, ibeta 变换到定子坐标系，得到定值 id 和 iq （因为实际的计算误差，所以得到的是近似的定值，而不是严格的定值）。

|     ![](./figures/tb_clark_park_tr.png)      |
| :------------------------------------------: |
| **图4**：对 clark_tr 与 park_tr 仿真的波形。 |

## cartesian2polar和svpwm的仿真

现在来运行 cartesian2polar （直角坐标转极坐标系）和 svpwm 的仿真。

双击 tb_svpwm_run_iverilog.bat 可以直接运行仿真，运行完后会生成波形文件 dump.vcd 。请用 gtkwave 打开 dump.vcd ，可以看到如**图5**的波形。

注意： pwma_duty、pwmb_duty、pwmc_duty 这三个信号不在顶层，你要在 svpwm 这个模块内才能找到这三个信号。

对**图5**波形的解读：

- theta 是一个不断递增的角度（0→2π→0→2π→...）
- x 和 y 是用 theta 生成的正交的正弦波（或者说，y是正弦波，x是余弦波）
- 把 (x,y) 视作直角坐标值，然后 cartesian2polar 把它转换成极坐标系 (ρ, φ) ，也即 (rho, phi) 
- 把 (rho, phi) 输给 svpwm ，产生了 pwma_duty、pwmb_duty、pwmc_duty 这三个马鞍波。如果你熟悉七段式 SVPWM 的原理，就应该知道为什么是马鞍波，这里不做赘述。
- pwma_duty、pwmb_duty、pwmc_duty 分别决定了 pwm_a, pwm_b, pwm_c 的占空比（duty这个单词就是占空比的意思）。

|            ![](./figures/tb_svpwm.png)            |
| :-----------------------------------------------: |
| **图5**：对 cartesian2polar 与 svpwm 仿真的波形。 |

放大波形，可以看到确实是 duty 值越大，对应的 pwm 信号的占空比就越大，如**图6**。

| ![](./figures/tb_svpwm_2.png) |
| :---------------------------: |
|     **图6**：图5的放大。      |



# FAQ

一些用户在读代码时向本人反馈了一些疑问，我将它们整理如下。

### 关于 PCB 设计

* **问**： **相电流低压侧3电阻采样需要运放放大电流信号才能作为AD输入，但你[开源的PCB]((https://oshwhub.com/wangxuan/arduino-foc-shield))似乎找不到运放放大？**
* **答**： 我[开源的PCB](https://oshwhub.com/wangxuan/arduino-foc-shield)使用了一体化的三相无刷电机驱动芯片MP6540（U1），3相电流采样电阻和放大器是集成在里面的。请注意MP6540的 SOA, SOB, SOC 三个引脚，它们就是三相电流放大后的电压输出。


### 关于 FOC 中的数学计算

* **问**： **三电阻采样后的电流重构的那部分代码如何理解？为什么 ia = ADCb + ADCc - 2*ADCa ？**
* **答**： 我们知道电机的相电流 ia, ib, ic 是双极性的（即有正有负的，分别代表流出电机和流入电机），但我们常用的ADC往往都是单极性的（即只能采样正电压）。那么，相电流采样-放大电路就必须考虑双极性到单极性的转换问题，工程上用的方法往往都是反向放大加偏置（包括MP6540内置采样-放大电路也是这种方案），它输出给ADC的电压遵循公式：
    ADCa = -R × ia + Voff
    ADCb = -R × ib + Voff
    ADCc = -R × ic + Voff
  其中 R 是放大系数（R>0，也叫跨阻放大系数，因为 R 的量纲和电阻一样），因为反向放大所以加了负号。Voff 是偏置电压，以此保证 ADCa, ADCb, ADCc 是单极性的。另外又有基尔霍夫电流定律（KCL）：
    ia + ib + ic = 0
  联立以上公式，推出：
  3 * Voff = ADCa + ADCb + ADCc
  令 R = 1/(3*k) ，于是：
  ia = (Voff - ADCa) / R
      = k × (3 * Voff - 3 × ADCa)
      = k × (ADCa + ADCb + ADCc - 3 × ADCa)
      = k × (ADCb + ADCc - 2×ADCa)
  于是就有了你问的  ia = ADCb + ADCc - 2×ADCa 。 你肯定会疑惑，系数 k 哪儿去了？这个并不在乎，因为整个FOC都是线性系统，通过调整PID参数，能跑就行，系数只在理论分析时有用。

* **问** : **你的程序里是不是都没有在乎系数K，包括clark变换中得到的Iα和Iβ和ia、ib、ic也并没有满足严格的公式关系，程序中得到的Iα和Iβ是理论值的2倍。这是不是也可以用调整PID参数的思想来解释？**
* **答**： 是的，我很多地方的代码也与理论公式的系数不同，比如 clark 变换公式本来是：
    Iα = Ia - Ib/2 - Ic/2
    Iβ = √3/2 * (Ib - Ic)
  我多乘了个2：
    Iα = 2 * Ia - Ib - Ic
    Iβ = √3 * (Ib - Ic)
  这是出于避免整数计算的截断导致的数据位丢失，比如 Ib/2 就会让 Ib 的最低 bit 丢失。不过实际上这种小误差基本不会影响控制质量。这种系数问题可以通过 PID 调参来消除。

* **问**: **你的代码中的 cartesian2polar.sv 是把电压矢量从转子直角坐标系 (Vd, Vq) 变换到转子极坐标系 (Vrρ, Vrθ)，目的是什么？**
* **答**： 书上一般会说 SVPWM 模块输入的是定子直角坐标系下的电压，但我实现的 SVPWM 输入的是定子极坐标系下的电压，两种方法在数学上是等价的。而且 SVPWM 在 FPGA 里用查找表(ROM)实现，因此两种方法对电路复杂度影响不大。另外，输入极坐标系的 SVPWM 还带来 2 个好处和 1 个代价，好处 1 是更方便在开发过程中让电机开环地转起来（只需要让角度递增即可）。好处 2 是极坐标系下的 park 变换更简单，只需要用电压在转子坐标系中的角度减去电角度就能得到电压在定子坐标系中的角度。 代价是需要在 PID 的后面、park 变换的前面实现一个直角坐标系转极坐标系的运算，即 cartesian2polar.sv

### 关于 ADC 采样时机

* **问**： **AD7928只有一个T/H(采样保持器)，也就意味着这个AD一次只能保持一个通道的数据，也就是说当通道1 打开的时候采集A相的电流，然后采集完再打开通道2采集B相的电流，那么这并不是一个同步采集的过程，如何实现AD采样三相电流的同步输出？**
* **答**： 在电流采样问题上，我们与大多数 FOC 方案相同，因为 MP6540 里的采样电阻在下桥臂，所以规定 SVPWM 在每个控制周期（约 55us，对应频率18kHz）内至少有一段时间里 3 相都是下通上闭，称为采样窗口。FOC的基础振幅（也就是 foc_top.sv 里的 MAX_AMP 参数）越大，采样窗口越短。当 MAX_AMP=9'd511 (最大值) 时，采样窗口长度就是 0 了。而默认的 MAX_AMP=9'd384 大概会让采样窗口长度为十几us。
AD7928 只有一个T/H，所以采样窗口内要做 3 次采样，这个过程是 hold_detect.sv 和 adc_ad7928.sv 共同控制的，hold_detect.sv负责在采样窗口开始时延迟一段时间（来让电流趋于稳定）后发出 sn_adc 信号脉冲，来告诉 adc_ad7928.sv 可以开始工作了（可以通过 foc_top.sv 里的 SAMPLE_DELAY 来调整该延迟）。然后 adc_ad7928.sv 内部就会自动串行地完成 3 个通道的采样，最后再同步提交3个采样结果（提交的同时产生 o_en_adc 信号脉冲）。注意adc_ad7928.sv是一个通用的 AD7928 控制器，每收到一个 i_sn_adc 信号脉冲，就进行一系列串行采样。其中采样多少次，每次采样哪一个通道，都可以通过 adc_ad7928.sv 里的 parameter 来配置。所有通道采样结束后，产生o_en_adc信号脉冲，同时同步提交所有通道的结果。
我把 foc_top.sv 用来连接 ADC控制器的接口（也就是sn_adc, en_adc, adc_a, adc_b, adc_c 这几个信号）设计成同步读入3通道数据，是出于通用性、简约性和可移植性的原则考虑，因为这样的同步读入接口是 ADC 的一种高度抽象，最容易让人理解其时序（即，sn_adc脉冲命令ADC控制器开始工作。en_adc指示ADC读取器结束工作，同时 adc_a, adc_b, adc_c 上产生结果）。如果用户用了其它 ADC 型号，只需按照这个时序的抽象来具象地编写 ADC 控制器即可，而 foc_top.sv 并不关心拟用的是1个串行的ADC还是3个并行的ADC，反正你都要给我同步提交。当然，用户必须自己算好 hold_detect.sv 的延时 + ADC 采样三个通道（也即 sn_adc 脉冲和 en_adc 脉冲的时间差） 是小于采样窗口的长度的。


* **问**： **进行串行采样的时候不会出现这样的问题：第一个时钟周期采到的是A相的电流，第2个时钟周期采的是B相，第3个时钟周期采到的是C相，而我们知道相电流是正弦变化的，这三个时钟周期采的A B C 三相电流并不是同一时刻的相电流，因此这将产生误差，而时钟周期是很短的，是不是产生的误差几乎可以忽略不计呢？**
* **答**： 首先指正一个不准确的地方， AD7928 的接口是 SPI （一种串行接口），其采样是多个时钟周期（而不是一个时钟周期）内完成的。因此三相的采样间隔是几十个时钟周期（具体数字我忘了，你可以仿真确定一下）。
虽然不同步，但3相的采样毕竟都是在同一个控制周期的采样窗口（几微秒）内。而相电流的变化通常以控制周期（即几十微秒）为尺度变化，例如 3000r/min（50r/s）的转子，若极对数=7，其相电流是350Hz（周期28ms）的正弦波，其变化在几微秒内是可以忽略的。



# 参考资料

* [1] [Sensorless FOC for PMSM](https://www.microchip.com/stellent/groups/SiteComm_sg/documents/Training_Tutorials/en532365.pdf), MicroChip.
* [2] [Current sensing in BLDC motor application](https://www.st.com/resource/en/application_note/dm00666970-current-sensing-in-bldc-motor-application-stmicroelectronics.pdf), ST.
* [3] [Center-Aligned SVPWM Realization](https://www.ti.com/lit/an/sprabs6/sprabs6.pdf), TI
* [4] [MP6540, 3-phase, brushless DC motor drivers](https://www.monolithicpower.com/en/mp6540-mp6540a.html), MPS.
* [5] [AD7928, 8-Channel, 1 MSPS, 12-Bit ADC](https://www.analog.com/en/products/ad7928.html), Analog Devices.
* [6] [深入浅出讲解FOC算法与SVPWM技术](https://zhuanlan.zhihu.com/p/147659820), 稚晖 - 知乎
* [7] [如何从零开始写一套自己的FOC矢量控制程序](https://zhuanlan.zhihu.com/p/103758450?utm_source=qzone), 上官致远 - 知乎
* [8] [STM32电动机控制应用系列讲座](https://www.bilibili.com/video/BV1vT4y1j7kc)
* [9] [BLDC电机基础](https://www.bilibili.com/video/BV1TW411d7k6)
