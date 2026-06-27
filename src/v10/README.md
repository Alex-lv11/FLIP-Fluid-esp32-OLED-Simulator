## FLIP Fluid Simulator V10
ESP32 embedded real-time FLIP fluid simulator, tilt and shake dual-gravity interaction, OLED real-time rendering of circular liquid simulation, with MPU6050 motion sleep/wake and RGB status light

## Hardware List
- ESP32 main controller
- SSD1306 128*64 I2C OLED (temporary)
- MPU6050 6-axis accelerometer
- HT16K33A x2 driving 16*16 LED display

### Pin Definition
- OLED/MPU I2C SDA: 8
- OLED/MPU I2C SCL: 9
- MPU interrupt wake-up pin: 4

## Project Features
1. FLIP particle-grid coupled fluid solver, circular tank boundary collision
2. Pitch tilt generates basic gravity, vigorous shaking adds impact gravity, total gravity multiplier capped
3. Angle change detection, auto sleep after 3 seconds of stillness, wake up with sensor movement
4. OLED renders fluid particles in real time, top of the screen shows pitch angle, gravity parameters, sleep countdown
5. RGB indicator light: green flashes during power-on self-test, red blinks on initialization errors

## Instructions
1. Install dependent libraries
2. Connect pins as defined and upload the code to ESP32
3. Power on, green light flashes for self-test, OLED loads fluid scene
4. Tilt device to change liquid flow, shake quickly to amplify fluid impact
5. Remains still with no angle change for 3 seconds to auto sleep, shake board to wake the system

## Adjustable Core Parameters
- IDLE_SLEEP_MS: stillness sleep duration
- ANGLE_CHANGE_THRESHOLD: threshold for detecting angle changes
- SHAKE_G_THRESHOLD: minimum G value to trigger impact gravity
- TOTAL_GRAV_MAX: maximum global gravity multiplier
- SIM_RESOLUTION fluid grid resolution, MAX_PARTICLES maximum particle count and other simulation parameters

## Dependencies
- U8g2lib
- Adafruit_NeoPixel
- Adafruit MPU6050
- Adafruit Unified Sensor

## FLIP Fluid Simulator V10
ESP32 嵌入式实时FLIP流体模拟器，倾斜+晃动双重力交互，OLED实时渲染圆形液体仿真，带MPU6050运动休眠唤醒与RGB状态灯

## 硬件清单
- ESP32主控
- SSD1306 128*64 I2C OLED(暂时)
- MPU6050 六轴加速度传感器
- HT16K33A x2 驱动16*16 LED显示屏

### 引脚定义
- OLED/MPU I2C SDA：8
- OLED/MPU I2C SCL：9
- MPU中断唤醒引脚：4

## 项目特性
1. FLIP粒子网格耦合流体求解，圆形水缸边界碰撞
2. 俯仰倾斜生成基础重力 剧烈晃动叠加冲击重力 总重力倍率封顶限制
3. 角度变化检测 静置3秒自动进入休眠 移动传感器硬件唤醒
4. OLED实时渲染流体粒子 屏幕顶部打印俯仰角 重力参数 休眠倒计时
5. RGB指示灯：开机自检绿灯闪烁 初始化异常红灯循环报错

## 使用说明
1. 安装依赖库
2. 按引脚接线 上传代码至ESP32
3. 开机绿灯闪烁自检 屏幕加载流体场景
4. 倾斜设备改变液体流向 快速晃动放大流体冲击力
5. 静置无角度变动3秒自动休眠 晃动板子唤醒整机

## 可调核心参数
- IDLE_SLEEP_MS：静置休眠时长
- ANGLE_CHANGE_THRESHOLD：角度变动判定阈值
- SHAKE_G_THRESHOLD：触发冲击重力的G值下限
- TOTAL_GRAV_MAX：全局重力倍率上限
- SIM_RESOLUTION 流体网格分辨率、MAX_PARTICLES最大粒子数量等仿真参数

## 依赖库
- U8g2lib
- Adafruit_NeoPixel
- Adafruit MPU6050
- Adafruit Unified Sensor
