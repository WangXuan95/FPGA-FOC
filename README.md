![test](https://img.shields.io/badge/test-passing-green.svg)
![docs](https://img.shields.io/badge/docs-passing-green.svg)

FpOC
===========================
基于 **FPGA** 的**磁场定向控制 (FOC)**，用于驱动**永磁同步电机 (PMSM)** 

# 简介

**FOC控制算法**对**传感器采样速率**和**处理器算力**提出了一定的要求，使用 **FPGA** 实现的 **FOC** 可以获得更好的**实时性**和零**延迟抖动**，并且更方便进行**多路扩展**。

本库实现了基于**角度传感器**（例如磁编码器）的**有感 FOC**（一个完整的**电流反馈环**），可以进行**扭矩控制**。借助本库，可以进一步使用 **FPGA** 、**软核 MCU** 或**外置 MCU** 实现更复杂的电机应用。

| ![diagram](https://github.com/WangXuan95/FpOC/blob/main/diagram.png) |
| :---: |
| 图1：系统框图 |

该项目代码有**详细的注释**，结合其它科普资料（见[6][8][9]），可以用来快速地熟悉 **FOC** 。

## 特点

* **平台无关** ：纯 RTL 编写，可以在 Altera 和 Xilinx 等各种 FPGA 上运行。
* 支持 **12bit 分辨率**的**角度传感器**和**相电流采样ADC**，对于>12bit的传感器，需要进行低位截断。对于<12bit的传感器，需要进行低位补0。
* 支持 **3路PWM** + **1路EN** ：PWM=1 时上半桥导通，PWM=0 时下半桥导通。 EN=0 时所有 MOS 关断。
* 使用 **16bit 有符号整数**进行计算，降低了资源消耗，考虑到传感器为 12bit，16bit 计算是够用的。

# 运行示例

**图1** 是本库的系统框图，实现了一个简单的行为——控制电机的电流（扭矩）按顺时针，逆时针交替运行。同时，使用 UART 打印电流的**控制目标值**和**实际值**，以便观察控制的质量。

该示例的所有代码都在 [./RTL](https://github.com/WangXuan95/FpOC/blob/main/RTL) 目录下。工程在 [./FPGA](https://github.com/WangXuan95/FpOC/blob/main/FPGA) 目录下，需要用 Quartus 软件打开。

## 准备硬件

需要准备以下硬件：

* PMSM 电机
* FPGA 开发板
* AD7928 ADC 模块，用于进行相电流采样（好像没有现成卖的，需要自己画模块）
* 电机驱动板，要支持3相 EN+PWM 输入信号，并且使用低侧电阻采样法+放大器对3相电流进行放大。

可以直接使用我画的 电机驱动板（自带AD7928），立创EDA工程[在此](https://oshwhub.com/wangxuan/arduino-foc-shield)，只需要把它接一个 FPGA 开发板即可。

## 硬件连接与引脚分配

见该工程的顶层文件 [top.sv](https://github.com/WangXuan95/FpOC/blob/main/RTL/top.sv) 的注释，以下外设需要连到 FPGA 的引脚上（普通IO引脚即可）：

* **晶振**， 50MHz 时钟，连接在 clk_50m 信号上。
* **角度传感器 AS5600** , I2C 接口, 2根线: i2c_scl, i2c_sda
* **ADC AD7928** , SPI接口，4根线: spi_ss, spi_sck, spi_mosi, spi_miso
* **3相PWM输出信号**（通常接在Gate Driver上，例如MP6540/DRV8301），4根线: pwm_en, pwm_a, pwm_b, pwm_c

另外还有一个 UART 发送信号 (uart_tx) 是可选的，可以把它连接在 UART 转 USB 模块上，通过 UART 来监测电流环的跟随曲线。

连接好后别忘了使用 Quartus （或者手动修改[./FPGA/foc.qsf](https://github.com/WangXuan95/FpOC/blob/main/FPGA/foc.qsf)）根据实际情况**修改FPGA芯片型号**，**修改引脚约束**。

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
| 图1：电流跟随曲线 |

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

* **淡橙色**部分是FPGA外部的硬件，包括Gate Driver、3相半桥(6个MOSFET)、采样电阻、放大器、ADC芯片、角度传感器等部件，取决于电路使用什么方案。例如，有些集成度很高的芯片（例如MP6540）可以把Gate Driver、3相半桥、采样电阻、放大器集成在同一个芯片里。
* **粉色**部分是FPGA中的**硬件相关逻辑**，即传感器控制器，如果角度传感器和 ADC 型号变了，这部分代码需要重写。
* **青色**部分是FPGA中的 FOC 的固定算法，属于**硬件无关逻辑**，一般不需要改变。
* **黄色**部分是**用户自定逻辑**，用户可以修改 user behavior 来实现各种电机应用。或者修改 uart_monitor 来监测其它变量。

另外，除了 [pll.v](https://github.com/WangXuan95/FpOC/blob/main/RTL/pll.v) 外，该库的所有代码都使用纯 RTL 编写，可以轻易地移植到其它厂商（Xilinx、Lattice等）的 FPGA 上。 [pll.v](https://github.com/WangXuan95/FpOC/blob/main/RTL/pll.v) 只是用来把 50MHz 时钟变成 36.864MHz 时钟的，只适用于 Altera Cyclone IV FPGA，当使用其它厂商或系列的FPGA时，需要使用它们各自的 IP 核或原语（例如Xilinx的clock wizard）来代替。

[top.sv](https://github.com/WangXuan95/FpOC/blob/main/RTL/top.sv) 和 [foc_top.sv](https://github.com/WangXuan95/FpOC/blob/main/RTL/foc/foc_top.sv) 注有详细的注释。如果你了解 FOC 算法，可以直接读懂。如果刚入门 FOC，可以结合参考资料[6][8][9]去阅读。

## 可扩展功能

* 使用自己编写的模块来代替 [as5600_read.sv](https://github.com/WangXuan95/FpOC/blob/main/RTL/sensors/as5600_read.sv)，以适配其它型号的角度传感器。
* 使用自己编写的模块来代替 [adc_ad7928.sv](https://github.com/WangXuan95/FpOC/blob/main/RTL/sensors/adc_ad7928.sv)，以适配其它型号的 ADC。
* 在 [top.sv](https://github.com/WangXuan95/FpOC/blob/main/RTL/top.sv) 中添加外环（速度环、位置环等）进一步实现各种电机应用。

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
