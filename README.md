# FLIP-Fluid-esp32-OLED-Simulator
FLIP particle fluid simulation implemented on ESP32 with real-time rendering on a 128×64 I2C OLED display; typical frame rate exceeds 30 FPS, peaking at up to 60 FPS with rendering disabled
Also compatible with ESP8266, albeit at lower frame rates

## Bill of Materials
ESP32-WROOM-32 / ESP8266 microcontroller
0.96-inch 128×64 SSD1306 I2C OLED screen

## Wiring Instructions
OLED VCC → 3.3V power pin
OLED GND → Common GND pin
For ESP32:
OLED SDA → GPIO21
OLED SCL → GPIO22
For ESP8266:
OLED SDA → GPIO4
OLED SCL → GPIO5

## Required Libraries (Install via Arduino Library Manager)
1.U8G2 (developed by oliver)
2.Wire (built into official ESP32/ESP8266 core, no extra installation required)

## Operation Guide
Install the ESP32 or ESP8266 board support package within the Arduino IDE
Complete installation of the listed dependent libraries
Open the project’s .ino source file; select board option ESP32-WROOM-DA Module (choose Generic ESP8266 Module for ESP8266 hardware), then compile and upload firmware

## License
MIT License

ESP32实现FLIP粒子流体模拟算法 128×64 I2C OLED屏幕实时渲染 常规帧率30FPS+ 无渲染最高可达60FPS
ESP8266也可实现 只是帧率较低
## 硬件清单
- ESP32-WROOM-32/ESP8266
- 0.96寸 128*64 SSD1306 I2C OLED

## 硬件接线
OLED VCC → 3.3V
OLED GND → GND
OLED SDA → GPIO21(ESP32)
OLED SCL → GPIO22(ESP32)
OLED SDA → GPIO4(ESP8266)
OLED SCL → GPIO5(ESP8266)

## 依赖库（Arduino库管理器搜索安装）
1. U8G2 by oliver
2. Wire（ESP32/ESP8266内核自带，无需手动安装）

## 使用方法
1. Arduino IDE安装ESP32/ESP8266开发板支持包
2. 安装上述依赖库
3. 打开工程ino文件，选择ESP32-WROOM-DA Module(ESP8266 选 Generic ESP8266 Module)编译烧录即可
4. 
## License
MIT License
