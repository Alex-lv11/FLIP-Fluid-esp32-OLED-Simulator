## FLIP Fluid Simulator V2.0 - Stability & Boundary Fix

Stable optimized version | Fixed boundary issues | Solved stress diffusion
This is an important fixed version based on the original FLIP fluid simulation algorithm

It specifically solves unstable problems in the initial version such as liquid explosion pressure divergence boundary penetration and chaotic particle movement
Greatly improved the stability of physical simulation on ESP32

## Major Improvements
Fixed boundary collision logic
Changed from hard rebound to soft boundary constraints particles no longer fly out of the screen

Solved pressure field divergence issue
Adjusted iteration count time step and relaxation factor to avoid numerical explosion

Optimized particle interpolation weights
Added safety kernel function to prevent abnormal grid velocity

Reduced computational load
Reduced pressure iterations adapted to the limited resource environment of ESP32

More stable FLIP PIC blending
Improved liquid smoothness and physical realism

## Core Optimizations
Reduced time step DT to improve numerical stability
Added SOR Successive Over Relaxation iteration to accelerate pressure solution convergence
Strictly limited particle positions to prevent out of bounds
Optimized gravity application method only applied to fluid areas
Simplified initialization for more orderly particle distribution

## Hardware
ESP32 + SSD1306 128×64 I2C OLED
SDA=21 / SCL=22

## Features
Stable operation for a long time no crash no explosion
Liquid flow is more natural closer to real physics


稳定优化版 | 修复边界问题 | 解决压力发散
这是在初代 FLIP 流体算法基础上的重要修复版本 专门解决了初代版本中出现的液体爆炸 压力发散 边界穿透 粒子乱飞等不稳定问题 大幅提升 ESP32 上的物理模拟稳定性

## 主要改进
修复边界碰撞逻辑：
从硬反弹改为软边界约束，粒子不再飞出屏幕

解决压力场发散问题：
调整迭代次数、时间步长、松弛因子，避免数值爆炸

优化粒子插值权重：
增加安全核函数（kernel），防止网格速度异常

降低计算负载：
减少压力迭代，适配 ESP32 小资源环境

更稳定的 FLIP/PIC 混合：
提升液体平滑度与物理真实性

## 核心优化点
减小时间步长 DT 提升数值稳定性
加入 SOR 超松弛迭代 加速压力求解收敛
严格限制粒子位置 杜绝越界
优化重力施加方式 只对流体区域生效
简化初始化 粒子分布更整齐

## 硬件
ESP32 + SSD1306 128×64 I2C OLED
SDA=21 / SCL=22

## 特点
可长时间稳定运行 不崩溃 不爆炸
液体流动更自然 更接近真实物理
