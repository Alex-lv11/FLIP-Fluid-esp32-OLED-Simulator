## FLIP Fluid Simulator
Based on ESP32-S3 SuperMini four-layer PCB design, using a 20×25×5 soft-pack lithium battery

## Project Overview
Currently, only **hardware PCB-related design files** are available. Firmware code is stored merely as a supporting resource and has not yet been modified for adaptation to this board's hardware.
The overall hardware main control is upgraded to **ESP32-S3 SuperMini**, paired with an MPU6050 six-axis sensor to achieve shake-to-wake functionality. Two HT16K33 chips drive 216 0402 LED matrixes, powered by a 3.7V soft-pack lithium battery, with built-in charging and voltage regulation circuits. Two four-layer PCBs are designed, with half-holes at the board edges supporting dual-board stacking and soldering. An option for a silkscreen-free minimalist appearance is available, focusing on long battery life and wake-on-motion scenarios.

Currently, the PCB is only theoretically designed and has not undergone physical soldering, power-on, or functionality testing. Existing firmware code only supports the OLED screen. Matrix drive adaptation and debugging will be carried out after hardware prototyping and soldering are completed.

## Core Hardware Features
1. Main Controller: ESP32-S3 SuperMini, supporting deep sleep and RTC external interrupt wake-up.
2. Motion Detection: MPU6050 six-axis accelerometer and gyroscope, hardware INT interrupt enables low-power standby wake-up.
3. Display Driver: Dual HT16K33A driving 216 0402 LED matrix, customizable animations, posture data, and fluid simulation visuals (conceptual).
4. Power Solution: Compatible with 20×25×5mm 3.7V soft-pack lithium battery, 250mAh capacity, supports Type-C charging (may switch to magnetic charging in future), LDO regulated 3.3V output.
5. Recommended PCB Process: 4-layer FR4 material, 1oz copper, 1mm thickness, with tin/solder mask (OSP) surface finish, half-hole structure at board edges, all silkscreen off.
6. Low Power Design: Idle sleep only maintains low-power sensor sampling, overall static current about 160μA (theoretical), full LED matrix operates upon wake-up.

## Warehouse File List
| File | Description | File Format |
| ---- | ---- | ---- |
| SCH_Schematic1_2026-06-20.pdf | Complete schematic, view all circuits, pins, and power filter design | PDF |
| BOM_Board1_PCB1_2026-06-20.xlsx | Complete bill of materials, including reference designators, quantities, packages, and component parameters | Excel |
| PickAndPlace_PCB1_2026-06-20.csv | SMT placement coordinates file, for factory assembly production | CSV |
| Lichuang EDA Project (.epro) | Native project source file, complete editable design of schematic + four-layer PCB | Lichuang EDA Project |
| Gerber Project |
| Gerber PCB Manufacturing Files (gbr/drl) | PCB production films, can be sent directly to the board manufacturer | Gerber RS-274X |


## Work Mode Introduction
## Sleep Mode
When the device is idle, the ESP32 enters deep sleep. Only the MPU6050 maintains low-acceleration sampling to monitor movement. The static current of the whole device is about 160μA. With a 250mAh pouch battery, it can sleep for 40 days.

## Wake-up Mode
When a hand raise or shake motion reaches the set acceleration threshold, the MPU6050 outputs an interrupt signal to wake up the ESP32. The dot matrix screen refreshes fluid simulation and attitude data. After a period of inactivity, it automatically returns to sleep.

## PCB Prototyping Recommendations
1. Board Material: Four-layer FR4, 1oz copper thickness, uniform board thickness of 1mm.
2. Surface Process: Prefer lead-free solder coating; half-holes offer more stable soldering; choose OSP for smooth pads.
3. Design Process Parameters: Minimum trace/space 4mil, via inner diameter 0.2mm, outer diameter 0.45mm, board edge half-holes for panel stacking alignment.
4. Optional: Turn off top layer; all silkscreen on bottom layer to achieve a pure white minimalist appearance.

## Supplementary Notes
1. The PCB only completes theoretical design and has not undergone physical soldering, power-on, or functional testing. There is room for circuit and layout optimization.
2. The main control solution uses the ESP32-S3 SuperMini development board, supporting floating-point operations and low power consumption.
3. Battery selection: 20×25×5 pouch lithium battery with 250mAh, balancing overall size and endurance. For longer endurance, a larger pouch battery can be used.
4. Engineering files have passed DRC rule checks: no short circuits, trace width, or via process violations. Can be prototyped directly for free.
5. The warehouse primarily contains hardware PCB projects. Firmware is only stored for reference; firmware-related optimizations are not covered in this hardware document.


## FLIP Fluid Simulator
基于 ESP32-S3 SuperMini 四层PCB设计 使用20×25×5软包锂电池

## 项目简介
目前仅有**硬件PCB相关设计文件** 固件代码仅作为配套资源存放 暂未针对本板硬件适配修改
硬件整体主控更换为 **ESP32-S3 SuperMini** 搭配MPU6050六轴传感器实现摇晃硬件唤醒 两片HT16K33驱动216颗0402 LED点阵 采用3.7V软包锂电池供电 自带充电稳压电路 两张四层PCB布局 板边半孔支持双板叠焊 可选择无丝印极简外观 主打长续航休眠姿态唤醒场景

当前PCB仅完成理论设计 未经过实物焊接 上电与功能测试 现有固件代码仅适配OLED屏幕 待硬件打样焊接完成后再进行点阵驱动适配调试 

## 硬件核心特性
1. 主控：ESP32-S3 SuperMini 支持深度睡眠RTC外部中断唤醒
2. 姿态检测：MPU6050六轴加速度陀螺仪 硬件INT中断实现低功耗值守唤醒
3. 显示驱动：双片HT16K33A 驱动216颗0402 LED点阵 可自定义动画 姿态数据 流体仿真画面(设想)
4. 供电方案：适配20×25×5mm 3.7V软包锂电池 容量250mAh 支持Type-C充电(后续可能会改为磁吸充电) LDO稳定输出3.3V
5. 建议PCB工艺：4层FR4板材 1oz铜 板厚1mm 喷锡/OSP表面处理 板边半孔结构 关闭全部丝印
6. 低功耗设计：静置休眠仅维持传感器低功耗采样 整机静态电流约160μA(理论) 唤醒后全点阵点亮工作

## 仓库文件清单
| 文件 | 用途说明 | 文件格式 |
| ---- | ---- | ---- |
| SCH_Schematic1_2026-06-20.pdf | 完整原理图，查看全部电路、引脚、电源滤波设计 | PDF |
| BOM_Board1_PCB1_2026-06-20.xlsx | 完整物料清单，含位号、数量、封装、器件参数 | Excel |
| PickAndPlace_PCB1_2026-06-20.csv | SMT贴片坐标文件，工厂贴片生产专用 | CSV |
| 立创EDA工程(.epro) | 原生工程源文件，原理图+四层PCB完整可编辑设计 | 立创EDA工程 |
| Gerber工程 |
| Gerber制板文件(gbr/drl) | PCB生产底片，可直接发给打板厂生产 | Gerber RS-274X |



## 工作模式介绍
## 休眠模式
设备静置时ESP32进入深度睡眠 仅MPU6050保持低加速度采样监测晃动 整机静态电流约160μA 搭配250mAh软包电池可休眠40天

## 工作唤醒模式
检测到抬手 摇晃动作达到设定加速度阈值后 MPU6050输出中断信号唤醒ESP32 点阵屏刷新流体仿真 姿态数据 持续无操作一段时间后自动切回休眠

## PCB打样建议
1. 板材：四层FR4 铜厚1oz 板厚统一选择1mm
2. 表面工艺：优先选择无铅喷锡 半孔焊接稳定性更强 追求平整焊盘可选OSP
3. 设计工艺参数：线宽/线距最小4mil 过孔内径0.2mm 外径0.45mm 板边半孔用于叠板对接
4. 可选关闭顶层 底层全部丝印 实现纯白无文字极简外观

## 补充说明
1. PCB仅完成理论设计 未经过实物焊接 上电 功能测试 存在电路 布局优化空间
2. 主控方案选用ESP32-S3 SuperMini开发板 浮点运算 低功耗
3. 电池选用20×25×5软包锂电 容量250mAh 兼顾整机体积与续航 如需更长续航可更换更大尺寸软包电池
4. 工程文件已完成DRC规则校验 无短路 线宽 过孔工艺违规 可直接免费打样
5. 仓库主体为硬件PCB工程 固件仅配套存放 固件相关优化内容不在本硬件文档介绍范围内
