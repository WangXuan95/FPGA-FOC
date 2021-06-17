![test](https://img.shields.io/badge/test-passing-green.svg)
![docs](https://img.shields.io/badge/docs-passing-green.svg)

FpOC
===========================
基于 **FPGA** 的**磁场定向控制 (FOC)**，用于驱动**永磁同步电机 (PMSM)** 

# 简介

**FOC控制算法**对**传感器采样速率**和**处理器算力**提出了一定的要求，使用 **FPGA** 实现的 **FOC** 可以获得更好的**实时性**和零**延迟抖动**，并且更方便进行**多路扩展**。

本库实现了基于**角度传感器**（例如磁编码器）的**有感 FOC**，即一个完整的**电流反馈环**，可以进行**扭矩控制**。借助本库，你可以进一步使用 **FPGA** 、**软核 MCU** 或**外置 MCU** 实现更复杂的电机应用。

| ![diagram](https://github.com/WangXuan95/FpOC/blob/main/diagram.png) |
| :---: |
| 图1：系统框图 |

该项目代码有**详细的注释**，结合参考资料 [6~9]，可以带你快速地熟悉 **FOC** 。另外，一些用户在读代码时向本人反馈了一些疑问，我将它们整理在了 [FAQ](#FAQ) 里。

## 技术特点

* **平台无关** ：纯 RTL 编写，可以在 Altera 和 Xilinx 等各种 FPGA 上运行。
* 支持 **12bit 分辨率**的**角度传感器**和**相电流采样ADC**，对于>12bit的传感器，需要进行低位截断。对于<12bit的传感器，需要进行低位补0。
* 支持 **3路PWM** + **1路EN** ：PWM=1 时上桥臂导通，PWM=0 时下桥臂导通。 EN=0 时所有 MOS 关断。
* 使用 **16bit 有符号整数**进行计算，降低了资源消耗，考虑到传感器为 12bit，16bit 计算是够用的。

# 运行示例

**图1** 是本库的系统框图，实现了一个简单的行为——控制电机的电流（扭矩）按顺时针，逆时针交替运行。同时，使用 UART 打印电流的**控制目标值**和**实际值**，以便观察控制的质量。

该示例的所有代码都在 [./RTL](https://github.com/WangXuan95/FpOC/blob/main/RTL) 目录下。工程在 [./FPGA](https://github.com/WangXuan95/FpOC/blob/main/FPGA) 目录下，需要用 Quartus 软件打开。

## 准备硬件

需要准备以下硬件：

* **FPGA 开发板** ：需有至少 **10** 个 **GPIO** ，用来接：
    * **I2C (2*GPIO)** ， 用来连接 AS5600 磁编码器。
    * **SPI (4*GPIO)** ， 用来连接 AD7928 ADC。
    * **PWM (3*GPIO)** ， 用来输出 3 相 PWM 到电机驱动板。
    * **PWM_EN (1*GPIO)** ， 用来输出 1 路 EN (使能) 到电机驱动板（EN=低电平代表所有桥臂关断）。
    * **UART (1*GPIO)** ，单向(仅发送)的UART，连接计算机的串口，用于监测电流环的跟随曲线，**可不接**。
* **PMSM** 或 **BLDC 电机**。
* **获取转子角度的磁编码器**：本库支持的型号是 AS5600 ，需要安装在电机上。
* **电机驱动板**：要支持3相 PWM 输入信号，并且要内置**低侧电阻采样法+放大器**对 3 相电流进行放大。
* **相电流采样的ADC**：本库支持的型号是 AD7928 ，用于采样电机驱动板放大后的 3 相电流
    * 网上似乎找不到现成卖的 AD7928 模块，需要自己画 PCB。

建议使用我画的一个自带 **AD7928 ADC** 的 **电机驱动板**，原理图和PCB开源[在此](https://oshwhub.com/wangxuan/arduino-foc-shield)，它一个板子就能实现以上**相电流采样的ADC**和**电机驱动板**的功能。（使用的电机驱动器是 **MP6540** 芯片, 我只试过驱动小功率云台电机，没试过大功率电机）

## 硬件连接与引脚分配

该工程的顶层文件是 [top.sv](https://github.com/WangXuan95/FpOC/blob/main/RTL/top.sv) ，它的 IO 连接方法如下：

* **clk_50m** ：  连接在 FPGA 开发板自带的 **50MHz** 晶振上。
    * 若不是 **50MHz** 也没关系，读 [top.sv](https://github.com/WangXuan95/FpOC/blob/main/RTL/top.sv) 会发现，它用 PLL 把 **50MHz** 转换成 **36.864MHz** 去用，所以你只需要修改 PLL 的配置，把输入频率改成板上晶振的实际频率，输出频率仍保持 **36.864MHz** 即可。
* **i2c_scl, i2c_sda** ： 连接 AS5600 (磁编码器) 的 I2C 接口。
* **spi_ss, spi_sck, spi_mosi, spi_miso** ： 连接 AD7928 (ADC芯片) 的 SPI 接口。 
* **pwm_a, pwm_b, pwm_c** ： 连接电机驱动板的 3 相 PWM 信号。
* **pwm_en** : 连接电机驱动板的 EN (使能) 信号。
    * 如果电机驱动板没有 EN 输入，则不接。
    * 如果电机驱动板有 3 路 EN 输入，每个对应 1 相，则应该进行 1 对 3 连接。
* **uart_tx** ： 连接 UART 转 USB 模块，插入计算机的 USB 口，用于监测电流环的跟随曲线，**可不接**。

连接好后别忘了根据实际情况使用 Quartus （或者手动修改[./FPGA/foc.qsf](https://github.com/WangXuan95/FpOC/blob/main/FPGA/foc.qsf)）**修改FPGA芯片型号**，**修改引脚约束**。

## 调参

[foc_top.sv](https://github.com/WangXuan95/FpOC/blob/main/RTL/foc/foc_top.sv) 中有一些参数可以调整，例如电机的**极对数**、**PID参数**等，每个参数的含义详见 [foc_top.sv](https://github.com/WangXuan95/FpOC/blob/main/RTL/foc/foc_top.sv) 。可以通过修改 [top.sv](https://github.com/WangXuan95/FpOC/blob/main/RTL/top.sv) 的 97~103 行来修改这些参数。

## 运行示例

综合并烧录到 FPGA 后，可以看到电机正反交替运行。

## 监视串口

把 uart_tx 信号通过 **UART 转 USB 模块** （例如CP2102模块） 连接到电脑上，就可以用**串口助手**、**Putty**等软件来监测电流环的跟随效果。

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


另外，你可以借用 **Arduino IDE** 的**串口绘图器**来实时显示电流跟随曲线。前往[该网站](https://www.arduino.cc/en/software)下载 **Arduino IDE**，安装后打开，在“**工具→端口**”中选择正确的COM口，然后点击“**工具→串口绘图器**”，**串口绘图器**会自动接收串口并使用上述4列数据画实时曲线图。

**图2** 是我这里绘制出的电流跟随曲线。蓝色曲线是第1列数据（d轴电流的实际值）；红色曲线是第2列数据（d轴电流的目标值）；绿色曲线是第3列数据（q轴电流的实际值）；土黄色曲线是第4列数据（q轴电流的目标值）。可以看到实际值能跟着目标值走。

| ![wave](https://github.com/WangXuan95/FpOC/blob/main/wave.png) |
| :---: |
| 图2：电流跟随曲线 |

# 代码详解

下表罗列了该工程使用的所有 **(System-)Verilog** 代码文件，这些文件都在 [./RTL](https://github.com/WangXuan95/FpOC/blob/main/RTL) 目录下。，结合**图1**就能看出每个模块的作用。

| 文件名 | 功能 | 备注 |
| :-- |   :-- |   :-- |  
| top.sv | FPGA工程的顶层模块 |  |
| pll.v  | 使用 50MHz 时钟生成 36.864MHz 时钟 | 只支持 Altera Cyclone IV，其它型号的 FPGA 需要用相应的IP核或原语代替 |
| uart_monitor.sv | UART 发送器，用于数据监测 | 不需要的话可以删除 |
| uart_tx.sv | UART 发送控制器，被 uart_monitor.sv 调用 | 不需要的话可以删除 |
| itoa.sv | 数字转十进制字符串，被 uart_monitor.sv 调用 | 不需要的话可以删除 |
| as5600_read.sv | AS5600 磁编码器读取器 | |
| i2c_register_read.sv | I2C 读取器，被 as5600_read.sv 调用 | |
| adc_ad7928.sv | AD7928 读取器 | |
| foc_top.sv | FOC+SVPWM （即**图1**中的青色部分） | 固定算法，一般不需要改动 |
| clark_tr.sv | Clark 变换 | 固定算法，一般不需要改动 |
| park_tr.sv | Park 变换  | 固定算法，一般不需要改动 |
| sincos.sv | 正弦/余弦计算器，被 park_tr.sv 调用 | 固定算法，一般不需要改动 |
| pi_controller.sv | PID 控制器（只有P和I） | 固定算法，一般不需要改动 |
| cartesian2polar.sv | 直角坐标系转极坐标系 | 固定算法，一般不需要改动 |
| svpwm.sv | SVPWM 调制器 | 固定算法，一般不需要改动 |
| hold_detect.sv | 监测3个下桥臂都导通时，延迟一段时间后触发 sn_adc 信号，指示ADC可以开始采样 | 固定算法，一般不需要改动 |


| ![diagram](https://github.com/WangXuan95/FpOC/blob/main/diagram.png) |
| :---: |
| 图1：系统框图 |

在**图1**中：

* **淡橙色**部分是FPGA外部的硬件，包括：
    * Gate Driver、3相半桥(6个MOSFET)、3相采样电阻+放大器 。（这些通常集成在**电机驱动板**里）。
    * 3相采样 ADC。
    * 角度传感器。
* **粉色**部分是FPGA中的**硬件相关逻辑**，即传感器控制器，如果角度传感器和 ADC 型号变了，这部分代码需要重写。
* **青色**部分是FPGA中的 FOC 的固定算法，属于**硬件无关逻辑**，一般不需要改变。
* **黄色**部分是**用户自定逻辑**，用户可以修改 user behavior 来实现各种电机应用。或者修改 uart_monitor 来监测其它变量。

另外，除了 [pll.v](https://github.com/WangXuan95/FpOC/blob/main/RTL/pll.v) 外，该库的所有代码都使用纯 RTL 编写，可以轻易地移植到其它厂商（Xilinx、Lattice等）的 FPGA 上。 [pll.v](https://github.com/WangXuan95/FpOC/blob/main/RTL/pll.v) 只是用来把 50MHz 时钟变成 36.864MHz 时钟的，只适用于 Altera Cyclone IV FPGA，当使用其它厂商或系列的FPGA时，需要使用它们各自的 IP 核或原语（例如Xilinx的clock wizard）来代替。

[top.sv](https://github.com/WangXuan95/FpOC/blob/main/RTL/top.sv) 和 [foc_top.sv](https://github.com/WangXuan95/FpOC/blob/main/RTL/foc/foc_top.sv) 注有详细的注释。如果你了解 FOC 算法，可以直接读懂。如果刚入门 FOC，可以结合参考资料[6~9]去阅读。


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


### 关于时钟周期和控制周期

* **问**： **时钟频率为什么选择36.864MHz？有什么特殊含义嘛？我如果直接用FPGA的50MHz（不经过PLL变成36.864MHz）会出现什么问题呢？**
* **答**： 其实选 36.864MHz 只是因为可以让控制周期为 36864/2048 = 18kHz，凑个整。
foc_top.sv 和 as5600_read.sv 对时钟频率是没有要求的，多高都可以（唯一的限制是频率太高时序不收敛）
但 adc_ad7928.sv 限制了频率不能大于 40.96MHz，因为 ADC7928 的 SPI 时钟不能超过 20.48MHz，而 adc_ad7928.sv 内部用驱动时钟2分频作为SPI时钟。
综上，时钟频率是可调的，但要小于 40.96MHz。

* **问**： **代码里 控制频率= 时钟频率 / 2048，为什么？ 我觉得控制频率和时钟频率应该是没有必然的联系，时钟频率是FPGA的时钟频率，而控制频率是pwm的频率，一般是几十khz，你是根据控制频率来确定的时钟频率嘛？**
* **答**： 理想地来讲，FPGA时钟频率和控制周期确实没有关系，但要考虑实际，PWM不也是FPGA的时钟驱动的？既然如此，必然有个分频系数（控制频率 = 时钟频率 / 分频系数），别的驱动器可能具有可调的分频系数，但FpOC里为了简单，分频系数=2048。这意味着 FpOC 如果要调节控制频率，就只能调时钟频率，目前受限于上述ADC7928的频率限制，最高的控制频率是 40.96MHz/2048=20kHz。


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
