## FLIP Fluid Simulator V3.0 - Numerical Stability Edition

Numerical Stability Edition with Adaptive Time Step, completely preventing explosions and divergence.  
This is the third-generation optimized version of the project.  
Based on the previous stable version, numerical safety has been implemented.

Can run continuously for a long time without crashing, diverging, or particles flying off.

## Core Upgrades
Added adaptive time step to automatically adjust calculation precision based on particle velocity.

Implemented CFL constraint condition to fundamentally prevent fluid numerical explosions.

Added global velocity limit to prevent uncontrolled particle motion.

Used sub-step integration to maintain smoothness and stability even during high-speed movement.

New pressure iteration damping makes the pressure field more stable and non-oscillating.

Unified grid size configuration supporting automatic switching between 8, 16, and 32 grid sizes.

Optimized particle initialization layout for a neater and more aesthetic appearance.

## Key Stability Mechanisms
Multiple boundary protections: dual safeguards with soft boundary + hard limit.

Damping iterations to prevent pressure field divergence.

Strict velocity clamping to prevent exceeding limits.

## Hardware Configuration
ESP32 main control chip
SSD1306 128×64 I2C OLED display
Wiring definitions: SDA 21, SCL 22

## Features
Stability: Can run continuously for a long time.
Fluid movement is natural, physical effects are realistic.
Grid size automatically adapts to display effect.
Pure Arduino environment, directly compile and upload.
Only depends on the U8g2 library, no other external dependencies.

## FLIP Fluid Simulator V3.0 - Numerical Stability Edition

数值稳定版 自适应时间步长 彻底杜绝爆炸发散
这是项目的第三代优化版本
在之前稳定版的基础上实现了数值安全化

可长时间连续运行不崩溃 不发散 不乱飞
## 核心升级内容
加入自适应时间步长 根据粒子速度自动调整计算精度

实现CFL 约束条件 从根源防止流体数值爆炸

增加全局速度上限 避免粒子运动失控

使用子步积分 高速运动下依然保持平滑稳定

新增压力迭代阻尼 压力场更稳定不震荡

统一网格尺寸配置 支持 8 16 32 三种网格大小自动切换

优化粒子初始化布局 更整齐更美观

## 关键稳定机制
多重边界保护 软边界 + 硬限制双重保障

阻尼迭代 防止压力场发散

严格速度钳制 杜绝越界

## 硬件配置
ESP32 主控芯片
SSD1306 128×64 I2C OLED 显示屏
接线定义 SDA21 SCL22

## 特点
稳定性 可长时间连续运行
流体运动自然 物理效果真实
网格大小自动适配显示效果
纯 Arduino 环境 直接编译烧录
仅依赖 U8g2 库 无其他外部依赖
