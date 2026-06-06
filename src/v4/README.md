## FLIP Fluid Simulator V4.0 - Full Professional Edition

This is the fourth-generation full version of the project.
It uses a pure C++ object-oriented FLIP algorithm architecture.
Deeply optimized for embedded microcontrollers (actually done by AI, I only know how to write messy code (T^T)).
Supports automatic multi-resolution switching.

## Core Feature Upgrades
Complete object-oriented FLIP fluid engine class encapsulation.
Supports 16, 32, and 64 multi-resolution simulation configurations.
Automatically adapts to both ESP32 and ESP8266 hardware platforms.
Automatic selection between hardware I2C and software I2C.
Added particle density calculation and velocity field visualization.
Includes real-time collision interaction with circular obstacles.
Particles automatically separate to prevent excessive stacking.
Velocity color mapping displays motion intensity.
Real-time FPS and particle count displayed on screen.
Dual physical collision with boundaries and obstacles.

## Stability Mechanisms
Relaxation pressure iteration for fast convergence.
Strict incompressibility ensures volume conservation.
PIC and FLIP hybrid interpolation reduces jitter.
Particle position correction prevents drift.
Multi-grid velocity transfer for more accurate results.

## Hardware Support
Automatically adapts to ESP32 hardware I2C.
Automatically adapts to ESP8266 software I2C.
SSD1306 128×64 I2C OLED screen.
Supports up to 700 particles (theoretical, but in practice it can be set higher (I don’t know why)).

## Features
Can run stably for long periods without crashing.
Physics effects highly approximate real liquids.
Smooth display with real-time interactive response.
Clear code structure easy to extend.
Compiles and flashes directly in a pure Arduino environment.
Only depends on the U8g2 library, no other components needed.

## FLIP Fluid Simulator V4.0 - Full Professional Edition

这是项目的第四代完整版本
采用纯 C++ 面向对象 FLIP 算法架构
专为嵌入式单片机深度优化(其实是ai干的 本人只会写屎山代码(T^T))
支持多分辨率自动切换

## 核心功能升级
完整面向对象 FLIP 流体引擎类封装
支持 16 32 64 多分辨率模拟配置
自动适配 ESP32 与 ESP8266 双硬件平台
硬件 I2C 与软件 I2C 自动选择
新增粒子密度计算与速度场可视化
加入圆形障碍物实时碰撞交互
粒子自动分离防止过度堆叠
速度颜色映射显示运动强度
实时 FPS 与粒子数量屏幕显示
边界与障碍物双重物理碰撞

## 稳定机制
松弛压力迭代快速收敛
不可压缩条件严格保证体积守恒
PIC 与 FLIP 混合插值减少抖动
粒子位置修正避免漂移
多重网格速度传输更精准

## 硬件支持
自动适配 ESP32 硬件 I2C
自动适配 ESP8266 软件 I2C
SSD1306 128×64 I2C OLED 屏幕
粒子数量最高支持 700 颗(理论 但实际能设更高(我不知道为什么))

## 特点
可长时间稳定运行不崩溃
物理效果高度接近真实液体
显示流畅交互实时响应
代码结构清晰易于扩展
纯 Arduino 环境直接编译烧录
仅依赖 U8g2 库无需其他组件
