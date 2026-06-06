## FLIP Fluid Simulator V7 / V8 / V9 - Stability & Optimization Updates

V7, V8, and V9 have no major functional changes. The core focus is on code optimization and stability fixes. These three versions do not add any new visual effects or interactive features. They are entirely focused on fixing bugs, optimizing indexing, improving runtime stability, and correcting rendering offsets. These versions are key optimizations that take the project from 'runnable' to 'perfectly stable'.

## Core Optimization Details (V7→V8→V9)
1. Fixed grid indexing errors (the most critical fix)
   Thoroughly corrected x/y and i/j index confusion
   Unified usage of cellIdx = j * fNumX + i as the standard 2D array index
   Resolved fluid edge misalignment, disappearance, and abnormal diffusion issues

2. Optimized particle density calculation
   Corrected density field mapping errors
   Automatically completed fluid cell markers
   Edges display more completely without missing pixels

3. Precise circular rendering area
   Expanded the circular mask radius to avoid cutoff of edge pixels
   Optimized coordinate mapping for perfect centering at 20 resolution
   Fluid no longer overflows boundaries

4. Accurate display coordinate alignment
   Adjusted drawing offsets X_SHIFT / Y_SHIFT
   Fluid fully stays within the circular area without drifting
   Pixel-level alignment on OLED 128x64 screen

5. Performance and hardware stability optimization
   Lowered I2C clock to improve ESP32 compatibility
   Reduced serial baud rate to avoid garbled data
   Optimized particle count and iteration steps for smoother operation

6. Code structure standardization
   Unified variable naming
   Simplified redundant calculations
   Improved readability and maintainability

## Version Positioning
V7: Fix indexing errors + basic stability
V8: Optimize rendering alignment + improve display precision
V9: Final stable version + full platform compatibility + perfect adaptation for 16 resolution

## Summary
V7~V9 have no new features. They are purely optimization, purely fixes, and purely stabilization iterations. They ensure fluid simulation does not crash, drift, penetrate models, and displays accurately.

## FLIP Fluid Simulator V7 / V8 / V9 - Stability & Optimization Updates

V7、V8、V9 无功能大改版 核心为代码优化与稳定性修复

这三个版本没有新增任何视觉效果或交互功能

全部专注于修复 BUG、优化索引、提升运行稳定性、修正渲染偏移

是项目从 “可运行” 走向 “完美稳定” 的关键优化版本

## 核心优化内容（V7→V8→V9）
1. 修复网格索引错误（最关键修复）
彻底修正 x/y 与 i/j 索引混乱
统一使用 cellIdx = j * fNumX + i 标准二维数组索引
解决流体边缘错位、不显示、异常扩散问题

3. 优化粒子密度计算
修正密度场映射错误
自动补全流体单元格标记
边缘显示更完整、不丢像素

5. 圆形渲染区域精准化
扩大圆形掩码半径 避免边缘像素被截断
优化坐标映射 使 20 分辨率完美居中
流体不再穿出边界

7. 显示坐标精准对齐
调整绘制偏移量 X_SHIFT / Y_SHIFT
流体完全落在圆形内部 不漂移
像素级对齐 OLED 128x64 屏幕

9. 性能与硬件稳定性优化
降低 I2C 时钟 提升 ESP32 兼容性
降低串口波特率 避免乱码
优化粒子数量与迭代次数 运行更流畅

11. 代码结构规范化
统一变量命名
简化冗余计算
提升可读性与可维护性

## 版本定位
V7：修复索引错误 + 基础稳定性
V8：优化渲染对齐 + 显示精度提升
V9：最终稳定版 + 全平台兼容 + 16 分辨率完美适配

## 总结
V7~V9 没有新功能
是纯优化、纯修复、纯稳定化的迭代版本
让流体模拟不崩溃、不漂移、不穿模、显示精准
