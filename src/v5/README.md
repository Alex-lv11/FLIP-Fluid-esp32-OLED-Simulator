## FLIP Fluid Simulator V5.0 - Grid Display Edition
Pure grid display version 16x16 32x32 64x64 adaptive resolution
This is the fifth generation final optimized version of the project
First implementation of pure grid fluid rendering
Does not display particles, directly renders the liquid through the density field
Visual effect is the cleanest, most suitable for OLED display

## Core Upgrades
Brand new grid density rendering mode, say goodbye to particle points
Supports one-click switching between 16, 32, 64 resolutions
Automatically adjusts particle count and computational intensity based on resolution
Automatically adapts to display size, centered full-screen display
Simplified physical logic, removes obstacles, focuses on fluid effects
Cells automatically adjust fill size according to density
Solid boundaries displayed as solid, hierarchy clearer
Real-time display of resolution and FPS on screen

## Stability and Optimization
Pressure iteration count automatically adapts to resolution
Particle separation algorithm prevents stacking
Boundary collisions are softer, no penetration
FLIP-PIC hybrid interpolation ensures smooth motion
Incompressibility strictly maintains volume

## Hardware Support
Automatically adapts to ESP32 hardware I2C
Automatically adapts to ESP8266 software I2C
SSD1306 128x64 I2C OLED
Ultra-low resource usage, stable long-term operation

## Features
Simple configuration, just modify the resolution
Complete code structure, easy for secondary modification
Direct Arduino compilation, zero extra configuration

## FLIP Fluid Simulator V5.0 - Grid Display Edition
纯网格显示版 16x16 32x32 64x64 自适应分辨率
这是项目第五代最终优化版本
首次实现纯网格流体渲染
不显示粒子 直接通过密度场渲染液体
视觉效果最整洁 最适合 OLED 展示

## 核心升级内容
全新网格密度渲染模式 告别粒子点
支持 16 32 64 三种分辨率一键切换
根据分辨率自动调整粒子数与计算强度
自动适配显示尺寸 全屏居中显示
简化物理逻辑 移除障碍物 专注流体效果
单元格根据密度自动调整填充大小
固体边界实心显示 层次更清晰
屏幕实时显示分辨率与 FPS

## 稳定与优化
压力迭代次数随分辨率自动适配
粒子分离算法防止堆叠
边界碰撞更柔和 不穿透
FLIP PIC 混合插值保证流畅运动
不可压缩条件严格保持体积

## 硬件支持
自动适配 ESP32 硬件 I2C
自动适配 ESP8266 软件 I2C
SSD1306 128x64 I2C OLED
超低资源占用 长时间稳定运行

## 特点
配置简单 直接修改分辨率即可
代码结构完整 易于二次修改
Arduino 直接编译 零额外配置
