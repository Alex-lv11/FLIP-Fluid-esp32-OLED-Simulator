## FLIP Fluid Simulator V1.0 - Initial Prototype
Initial Basic Version | 16x16 Grid | ESP32 + SSD1306 OLED
This is the first fully functional version of the project, implemented based on the FLIP (Fluid-Implicit-Particle) fluid algorithm, specifically designed for the ESP32 microcontroller + 0.96-inch I2C OLED (128×64)

It uses a MAC staggered grid + particle hybrid method to achieve realistic fluid physics
Supports an auto-rotating gravity field — fluid flows naturally according to the gravity direction
Built-in incompressibility condition (pressure projection) to ensure liquid volume conservation
Bi-directional interpolation from particles to grid and grid to particles for smooth liquid motion
Boundary collision and rebound for realistic liquid physical behavior

## Hardware
ESP32
SSD1306 128×64 I2C OLED
Wiring: SDA=21 / SCL=22

## Display Effects
The left 64×64 area renders liquid particles
The right side displays the particle count and gravity direction in real time
Particles are drawn using 2×2 pixels

初代基础版 | 16x16 网格 | ESP32 + SSD1306 OLED
这是项目的第一个完整可运行版本 基于 FLIP（Fluid-Implicit-Particle）流体算法实现 专为 ESP32 单片机 + 0.96 寸 I2C OLED（128×64）设计

采用 MAC 交错网格 + 粒子混合法 实现真实流体物理
支持 自动旋转重力场 流体可随重力方向自然流动
内置 不可压缩条件（压力投影） 保证液体体积守恒
粒子到网格 网格到粒子双向插值 实现流畅液体运动
边界碰撞与反弹 模拟真实液体物理行为

## 硬件
ESP32
SSD1306 128×64 I2C OLED
接线：SDA=21 / SCL=22

## 显示效果
左侧 64×64 区域渲染液体粒子
右侧实时显示粒子数量 重力方向
粒子使用 2×2 像素绘制
