## FLIP Fluid Simulator V6.0 - Octagon Gravity Control Edition

Octagon display version (aimed at fixing the previous incorrect circular rendering) Serial gravity control Adjustable in 360-degree directions
This is the sixth version of the project
Brand new octagonal display area
Supports real-time control of gravity direction and strength via serial commands
Physics simulation maintains circular boundaries, display uses an octagon
Perfect balance between aesthetics and stability

## Core Upgrades
Brand new octagonal rendering area, saying goodbye to square and circular displays
360-degree gravity direction free control
Adjust angle and strength via serial commands, effective in real-time
Screen shows gravity arrow, direction clearly visible
Supports shortcut commands: up, down, left, right, and zero gravity
Physics collision maintains circular shape, display uses octagon
Automatically adapts to 16, 32, 64 resolutions
Screen shows gravity percentage and angle in real-time

## Stability and Optimization
Particles automatically separate to prevent stacking
Circular boundary collisions do not penetrate
Smooth motion with FLIP-PIC hybrid interpolation
Pressure iteration count automatically adapts to resolution
Over-relaxation iterations accelerate convergence

## Serial Command Set
angle value - set gravity angle: 0 Right, 90 Up, 180 Left, 270 Down
scale value - set gravity strength from 0 to 10
down, up, left, right - shortcut directions
zero - turn off gravity
help - view help

## Hardware Support
Automatically adapts to ESP32 hardware I2C
Automatically adapts to ESP8266 software I2C
SSD1306 128x64 I2C OLED
Ultra-low resource usage, stable long-term operation

## Features
Most powerful interaction, complete gravity control via serial
Simplest configuration, one-click resolution modification
Most stable physics, no crashes or divergence
Arduino direct compilation, zero extra configuration

## FLIP Fluid Simulator V6.0 - Octagon Gravity Control Edition

正八边形显示版(旨在修复先前的圆形渲染不正确) 串口重力控制 360 度方向可调
这是项目第六版
全新正八边形显示区域
支持串口指令实时控制重力方向与强度
物理模拟保持圆形边界 显示使用八边形
完美平衡美观与稳定

## 核心升级内容
全新正八边形渲染区域 告别方形与圆形
360 度重力方向自由控制
串口指令调节角度与强度 实时生效
屏幕显示重力箭头 方向直观可见
支持快捷指令上下左右与零重力
物理碰撞保持圆形 显示使用八边形
自动适配 16 32 64 分辨率
屏幕实时显示重力百分比与角度

## 稳定与优化
粒子自动分离防止堆叠
圆形边界碰撞不穿透
FLIP PIC 混合插值流畅运动
压力迭代次数自动适配分辨率
超松弛迭代加速收敛

## 串口指令集
angle 数值 设置重力角度 0 右 90 上 180 左 270 下
scale 数值 设置重力强度 0 到 10
down up left right 快捷方向
zero 关闭重力
help 查看帮助

## 硬件支持
自动适配 ESP32 硬件 I2C
自动适配 ESP8266 软件 I2C
SSD1306 128x64 I2C OLED
超低占用 长时间稳定运行

## 特点
交互最强大 串口完全控制重力
配置最简单 一键修改分辨率
物理最稳定 不崩溃不发散
Arduino 直接编译 零额外配置
