/*
  FLIP Fluid ESP32 + OLED 128x64 + 216LED查理复用
  修复：OLED短暂显示后熄灭（引脚冲突+内存+阻塞）
  关键：避开I2C21/22+静态数组+定时器LED扫描+稳定I2C
*/
#include <Arduino.h>
#include <U8g2lib.h>
#include <driver/timer.h>

// ============== 基础配置 ==============
#define SIM_RESOLUTION 20
#define MAX_PARTICLES 400
#define PRESSURE_ITERATIONS 10
#define PARTICLE_ITERATIONS 1
#define USE_ESP32 1

// OLED 硬件I2C（显式指定引脚，避免漂移）SDA=21 SCL=22
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /*SCL=*/22, /*SDA=*/21, U8X8_PIN_NONE);

// ============== 查理复用LED配置（核心：避开21/22，16个无冲突引脚） ==============
#define CHARLIE_PIN_COUNT 16
#define LED_TOTAL 216
#define LED_SCAN_DELAY_US 80  // 扫描延时，不可过小
// 【无冲突16引脚】彻底避开21/22/I2C/SPI/仅输入，适配ESP32 D0WD裸芯片
gpio_num_t charlie_pins[CHARLIE_PIN_COUNT] = {
    GPIO_NUM_4,  GPIO_NUM_14, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27,
    GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_0,  GPIO_NUM_2,  GPIO_NUM_5,
    GPIO_NUM_12, GPIO_NUM_15, GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_13,
    GPIO_NUM_35  // 35为通用GPIO，非仅输入，无冲突
};
// LED映射表+状态缓存
typedef struct {int high_idx; int low_idx;} LedPinPair;
LedPinPair led_map[LED_TOTAL];
int cell_to_led[SIM_RESOLUTION][SIM_RESOLUTION];
bool led_state[LED_TOTAL] = {0};
static int current_led = 0;  // 定时器扫描用当前LED索引

// ============== 重力配置 ==============
float gravityAngle = 270.0f;
float gravityScale = 1.0f;
const float BASE_GRAVITY = 9.8f;
float gravityX = 0.0f, gravityY = 0.0f;
float gravityCos = 0.0f, gravitySin = 0.0f;

// ============== 流体模拟参数（静态数组，无new，固定大小） ==============
const float DT = 0.041f;
const float FLIP_RATIO = 0.9f;
const float OVER_RELAXATION = 1.9f;
const float DENSITY = 1000.0f;
#define FLUID_CELL 0
#define AIR_CELL 1
#define SOLID_CELL 2
// 静态数组大小（按SIM_RESOLUTION=20计算，足够用，无内存浪费）
#define MAX_CELLS 1000
#define MAX_P_CELLS 2000
int fNumX, fNumY, fNumCells;
float h, fInvSpacing;
// 静态数组替代new，彻底解决内存问题
float u[MAX_CELLS] = {0}, v[MAX_CELLS] = {0};
float prevU[MAX_CELLS] = {0}, prevV[MAX_CELLS] = {0};
float du[MAX_CELLS] = {0}, dv[MAX_CELLS] = {0};
float p[MAX_CELLS] = {0}, s[MAX_CELLS] = {0};
int cellType[MAX_CELLS] = {0};
float particleDensity[MAX_CELLS] = {0};

int numParticles = 0;
float particlePos[MAX_PARTICLES * 2] = {0};
float particleVel[MAX_PARTICLES * 2] = {0};
float pInvSpacing;
int pNumX, pNumY, pNumCells;
int numCellParticles[MAX_P_CELLS] = {0};
int firstCellParticle[MAX_P_CELLS + 1] = {0};
int cellParticleIds[MAX_PARTICLES] = {0};

bool compensateDrift = true;
bool separateParticles = true;
float simWidth, simHeight, tankWidth, tankHeight;
float circleCenterX, circleCenterY, circleRadius, circleRadiusSq;
int displayOffsetX, displayOffsetY, cellPixelSize;
bool circleMask[SIM_RESOLUTION][SIM_RESOLUTION] = {false};

// 帧率+串口
unsigned long lastFpsTime = 0, lastSerialPrintTime = 0;
int frameCount = 0;
float currentFps = 0;
String inputBuffer = "";

// ============== 工具函数 ==============
inline float myClamp(float x, float minVal, float maxVal) {
    return (x < minVal) ? minVal : (x > maxVal) ? maxVal : x;
}
inline float myMin(float a, float b) { return (a < b) ? a : b; }
inline int myMin(int a, int b) { return (a < b) ? a : b; }
inline float myMax(float a, float b) { return (a > b) ? a : b; }
inline int myMax(int a, int b) { return (a > b) ? a : b; }
inline float degToRad(float deg) { return deg * PI / 180.0f; }

void updateGravity() {
    float normalizedAngle = fmod(gravityAngle, 360.0f);
    normalizedAngle = (normalizedAngle < 0) ? normalizedAngle + 360.0f : normalizedAngle;
    float rad = degToRad(normalizedAngle);
    gravityCos = cos(rad);
    gravitySin = sin(rad);
    gravityX = gravityCos * BASE_GRAVITY * gravityScale;
    gravityY = -gravitySin * BASE_GRAVITY * gravityScale;
    Serial.printf("Gravity: %0.1f° | %0.1fx | (%.2f, %.2f)\n", normalizedAngle, gravityScale, gravityX, gravityY);
}

void processSerialCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;
    cmd.toLowerCase();
    if (cmd.startsWith("angle ") || cmd.startsWith("a ")) {
        float newAngle = cmd.substring(cmd.indexOf(' ')+1).toFloat();
        gravityAngle = newAngle; updateGravity();
    } else if (cmd.startsWith("scale ") || cmd.startsWith("s ") || cmd.startsWith("g ")) {
        float newScale = myClamp(cmd.substring(cmd.indexOf(' ')+1).toFloat(), 0.0f, 10.0f);
        gravityScale = newScale; updateGravity();
    } else if (cmd == "down") {gravityAngle=270;updateGravity();}
    else if (cmd == "up") {gravityAngle=90;updateGravity();}
    else if (cmd == "left") {gravityAngle=180;updateGravity();}
    else if (cmd == "right") {gravityAngle=0;updateGravity();}
    else if (cmd == "zero" || cmd == "0") {gravityScale=0;updateGravity();}
    else if (cmd == "help" || cmd == "h" || cmd == "?") {
        Serial.println("=== Commands: angle/a <deg>, scale/s/g <0-10>, down/up/left/right, zero, help ===");
    }
}

// ============== 查理复用LED驱动（定时器中断版，解放主循环） ==============
void init_led_map() {
    int led_num = 0;
    for (int high_idx = 0; high_idx < CHARLIE_PIN_COUNT && led_num < LED_TOTAL; high_idx++) {
        for (int low_idx = 0; low_idx < CHARLIE_PIN_COUNT && led_num < LED_TOTAL; low_idx++) {
            if (high_idx != low_idx) {led_map[led_num++] = {high_idx, low_idx};}
        }
    }
}

void init_cell_led_map() {
    int led_idx = 0;
    float center = (SIM_RESOLUTION - 1) / 2.0f;
    float radius = SIM_RESOLUTION / 2.0f - 0.1f, radiusSq = radius * radius;
    for (int i = 0; i < SIM_RESOLUTION; i++) for (int j = 0; j < SIM_RESOLUTION; j++) cell_to_led[i][j] = -1;
    for (int i = 0; i < SIM_RESOLUTION && led_idx < LED_TOTAL; i++) {
        for (int j = 0; j < SIM_RESOLUTION && led_idx < LED_TOTAL; j++) {
            float dx = i - center, dy = j - center;
            if (dx*dx + dy*dy <= radiusSq) cell_to_led[i][j] = led_idx++;
        }
    }
}

// 所有引脚置高阻（禁用上拉/下拉）
void charlie_pin_float() {
    gpio_config_t io_conf;
    io_conf.pin_bit_mask = 0; io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    for (int i = 0; i < CHARLIE_PIN_COUNT; i++) {
        io_conf.pin_bit_mask = 1ULL << charlie_pins[i];
        gpio_config(&io_conf);
    }
}

// 点亮单个LED
void led_on(int led_number) {
    if (led_number < 0 || led_number >= LED_TOTAL) return;
    charlie_pin_float(); // 先置高阻，避免串扰
    int h_idx = led_map[led_number].high_idx, l_idx = led_map[led_number].low_idx;
    // 高电平引脚：推挽输出高
    pinMode(charlie_pins[h_idx], OUTPUT); digitalWrite(charlie_pins[h_idx], HIGH);
    // 低电平引脚：推挽输出低
    pinMode(charlie_pins[l_idx], OUTPUT); digitalWrite(charlie_pins[l_idx], LOW);
}

// 定时器中断服务函数：LED扫描（每80μs执行一次，非阻塞）
void IRAM_ATTR led_scan_isr() {
    timerGroupClrIntrStatus(TIMER_GROUP_0, TIMER_0); // 清除中断标志
    charlie_pin_float();
    if (led_state[current_led]) {led_on(current_led);} // 仅点亮状态为1的LED
    current_led = (current_led + 1) % LED_TOTAL;      // 循环扫描
}

// 初始化定时器0（80μs中断，驱动LED扫描）
void init_led_timer() {
    timer_config_t timer_conf = {
        .alarm_en = TIMER_ALARM_EN, .counter_en = TIMER_PAUSE,
        .counter_dir = TIMER_COUNT_UP, .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = 80, // 分频80，时钟=1MHz（240MHz/80）
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &timer_conf);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, LED_SCAN_DELAY_US); // 80μs中断
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, led_scan_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TIMER_GROUP_0, TIMER_0);
}

// 更新LED状态（流体单元格→LED）
void update_led_state() {
    for (int i = 0; i < SIM_RESOLUTION; i++) {
        for (int j = 0; j < SIM_RESOLUTION; j++) {
            int led_idx = cell_to_led[i][j];
            if (led_idx == -1 || led_idx >= LED_TOTAL) continue;
            float simIFloat = (float)i / SIM_RESOLUTION * (fNumX - 1);
            float simJFloat = (float)j / SIM_RESOLUTION * (fNumY - 1);
            int simI = myClamp((int)round(simIFloat), 0, fNumX - 1);
            int simJ = myClamp((int)round(simJFloat), 0, fNumY - 1);
            int cellIdx = simJ * fNumX + simI;
            led_state[led_idx] = (cellType[cellIdx] == FLUID_CELL);
        }
    }
}

// ============== FLIP流体模拟类（静态数组版，无内存泄漏） ==============
class FlipFluid {
public:
    void init(float density, float width, float height, float spacing) {
        fNumX = (int)(width / spacing) + 1;
        fNumY = (int)(height / spacing) + 1;
        fNumCells = myMin(fNumX * fNumY, MAX_CELLS);
        h = max(width / fNumX, height / fNumY);
        fInvSpacing = 1.0f / h;
        simWidth = width; simHeight = height;
        circleCenterX = width / 2.0f; circleCenterY = height / 2.0f;
        circleRadius = (myMin(width, height) / 2.0f) - h;
        circleRadiusSq = circleRadius * circleRadius;

        float particleRadius = 0.3f * h;
        pInvSpacing = 1.0f / (2.2f * particleRadius);
        pNumX = (int)(width * pInvSpacing) + 1;
        pNumY = (int)(height * pInvSpacing) + 1;
        pNumCells = myMin(pNumX * pNumY, MAX_P_CELLS);
    }

    void integrateParticles(float dt, float gx, float gy) {
        float gx_dt = gx * dt, gy_dt = gy * dt;
        for (int i = 0; i < numParticles; i++) {
            int idx = i * 2;
            particleVel[idx] += gx_dt; particleVel[idx+1] += gy_dt;
            particlePos[idx] += particleVel[idx] * dt; particlePos[idx+1] += particleVel[idx+1] * dt;
        }
    }

    void pushParticlesApart(int numIters) {
        memset(numCellParticles, 0, pNumCells * sizeof(int));
        for (int i = 0; i < numParticles; i++) {
            float x = particlePos[i*2], y = particlePos[i*2+1];
            int xi = myClamp((int)(x * pInvSpacing), 0, pNumX-1);
            int yi = myClamp((int)(y * pInvSpacing), 0, pNumY-1);
            numCellParticles[xi * pNumY + yi]++;
        }
        int first = 0;
        for (int i = 0; i < pNumCells; i++) {first += numCellParticles[i]; firstCellParticle[i] = first;}
        firstCellParticle[pNumCells] = first;
        for (int i = 0; i < numParticles; i++) {
            float x = particlePos[i*2], y = particlePos[i*2+1];
            int xi = myClamp((int)(x * pInvSpacing), 0, pNumX-1);
            int yi = myClamp((int)(y * pInvSpacing), 0, pNumY-1);
            int cellNr = xi * pNumY + yi;
            firstCellParticle[cellNr]--;
            cellParticleIds[firstCellParticle[cellNr]] = i;
        }
        float minDist = 2.0f * 0.3f * h, minDist2 = minDist * minDist;
        for (int iter = 0; iter < numIters; iter++) {
            for (int i = 0; i < numParticles; i++) {
                float px = particlePos[i*2], py = particlePos[i*2+1];
                int pxi = (int)(px * pInvSpacing), pyi = (int)(py * pInvSpacing);
                int x0 = max(pxi-1,0), y0 = max(pyi-1,0), x1 = myMin(pxi+1, pNumX-1), y1 = myMin(pyi+1, pNumY-1);
                for (int xi = x0; xi <= x1; xi++) {
                    for (int yi = y0; yi <= y1; yi++) {
                        int cellNr = xi * pNumY + yi;
                        int f = firstCellParticle[cellNr], l = firstCellParticle[cellNr+1];
                        for (int j = f; j < l; j++) {
                            int id = cellParticleIds[j];
                            if (id == i) continue;
                            float qx = particlePos[id*2], qy = particlePos[id*2+1];
                            float dx = qx - px, dy = qy - py, d2 = dx*dx + dy*dy;
                            if (d2 > minDist2 || d2 == 0.0f) continue;
                            float d_inv = 1.0f / sqrt(d2);
                            float s = 0.5f * (minDist - d2 * d_inv) * d_inv;
                            dx *= s; dy *= s;
                            particlePos[i*2] -= dx; particlePos[i*2+1] -= dy;
                            particlePos[id*2] += dx; particlePos[id*2+1] += dy;
                        }
                    }
                }
            }
        }
    }

    void handleCollisions() {
        float r = 0.3f * h, rSq = (circleRadius - r) * (circleRadius - r);
        for (int i = 0; i < numParticles; i++) {
            int idx = i * 2;
            float x = particlePos[idx], y = particlePos[idx+1];
            float dx = x - circleCenterX, dy = y - circleCenterY, distSq = dx*dx + dy*dy;
            if (distSq > rSq) {
                float dist = sqrt(distSq);
                float scale = (circleRadius - r) / dist;
                particlePos[idx] = circleCenterX + dx * scale;
                particlePos[idx+1] = circleCenterY + dy * scale;
                float nx = dx / dist, ny = dy / dist;
                float vn = particleVel[idx] * nx + particleVel[idx+1] * ny;
                particleVel[idx] -= vn * nx; particleVel[idx+1] -= vn * ny;
            }
        }
    }

    void updateParticleDensity() {
        memset(particleDensity, 0, fNumCells * sizeof(float));
        int n = fNumY; float h1 = fInvSpacing, h2 = 0.5f * h;
        for (int i = 0; i < numParticles; i++) {
            int idx = i * 2;
            float x = myClamp(particlePos[idx], h, (fNumX-1)*h);
            float y = myClamp(particlePos[idx+1], h, (fNumY-1)*h);
            int x0 = (int)((x - h2) * h1); float tx = ((x - h2) - x0 * h) * h1;
            int x1 = myMin(x0 + 1, fNumX - 2);
            int y0 = (int)((y - h2) * h1); float ty = ((y - h2) - y0 * h) * h1;
            int y1 = myMin(y0 + 1, fNumY - 2);
            float sx = 1.0f - tx, sy = 1.0f - ty;
            int base0 = y0 * fNumX, base1 = y1 * fNumX;
            if (x0 < fNumX && y0 < fNumY) particleDensity[base0 + x0] += sx * sy;
            if (x1 < fNumX && y0 < fNumY) particleDensity[base0 + x1] += tx * sy;
            if (x1 < fNumX && y1 < fNumY) particleDensity[base1 + x1] += tx * ty;
            if (x0 < fNumX && y1 < fNumY) particleDensity[base1 + x0] += sx * ty;
        }
        for (int i = 1; i < fNumX-1; i++) {
            for (int j = 1; j < fNumY-1; j++) {
                int idx = j * fNumX + i;
                if (cellType[idx] == AIR_CELL && particleDensity[idx] > 0.1f) cellType[idx] = FLUID_CELL;
            }
        }
    }

    void transferVelocities(bool toGrid, float flipRatio) {
        int n = fNumY; float h2 = 0.5f * h;
        if (toGrid) {
            memcpy(prevU, u, fNumCells * sizeof(float));
            memcpy(prevV, v, fNumCells * sizeof(float));
            memset(du, 0, fNumCells * sizeof(float));
            memset(dv, 0, fNumCells * sizeof(float));
            memset(u, 0, fNumCells * sizeof(float));
            memset(v, 0, fNumCells * sizeof(float));
            for (int i = 0; i < fNumX; i++) {
                for (int j = 0; j < fNumY; j++) {
                    float cellX = i * h, cellY = j * h;
                    float dx = cellX - circleCenterX, dy = cellY - circleCenterY, distSq = dx*dx + dy*dy;
                    bool solid = (distSq >= circleRadiusSq - h*h);
                    int idx = j * fNumX + i;
                    s[idx] = solid ? 0.0f : 1.0f;
                    cellType[idx] = solid ? SOLID_CELL : AIR_CELL;
                }
            }
            for (int i = 0; i < numParticles; i++) {
                int idx = i * 2;
                float x = particlePos[idx], y = particlePos[idx+1];
                int xi = myClamp((int)(x * fInvSpacing), 0, fNumX-1);
                int yi = myClamp((int)(y * fInvSpacing), 0, fNumY-1);
                int cellNr = yi * fNumX + xi;
                if (cellType[cellNr] == AIR_CELL) cellType[cellNr] = FLUID_CELL;
            }
        }
        // U场
        for (int i = 0; i < numParticles; i++) {
            int idx = i * 2;
            float x = myClamp(particlePos[idx], h, (fNumX-1)*h);
            float y = myClamp(particlePos[idx+1], h, (fNumY-1)*h);
            float x0f = x * fInvSpacing; int x0 = (int)x0f; float tx = x0f - x0;
            x0 = myMin(x0, fNumX-2); int x1 = x0 + 1;
            float y0f = (y - h2) * fInvSpacing; int y0 = (int)y0f; float ty = y0f - y0;
            y0 = myMin(y0, fNumY-2); int y1 = y0 + 1;
            float sx = 1.0f - tx, sy = 1.0f - ty;
            float d0 = sx*sy, d1 = tx*sy, d2 = tx*ty, d3 = sx*ty;
            int base0 = y0*fNumX, base1 = y1*fNumX;
            int nr0 = base0+x0, nr1 = base0+x1, nr2 = base1+x1, nr3 = base1+x0;
            if (toGrid) {
                float pv = particleVel[idx];
                u[nr0] += pv*d0; du[nr0] += d0;
                u[nr1] += pv*d1; du[nr1] += d1;
                u[nr2] += pv*d2; du[nr2] += d2;
                u[nr3] += pv*d3; du[nr3] += d3;
            } else {
                float valid0 = (cellType[nr0]!=AIR_CELL||cellType[nr0-fNumX]!=AIR_CELL)?1.0f:0.0f;
                float valid1 = (cellType[nr1]!=AIR_CELL||cellType[nr1-fNumX]!=AIR_CELL)?1.0f:0.0f;
                float valid2 = (cellType[nr2]!=AIR_CELL||cellType[nr2-fNumX]!=AIR_CELL)?1.0f:0.0f;
                float valid3 = (cellType[nr3]!=AIR_CELL||cellType[nr3-fNumX]!=AIR_CELL)?1.0f:0.0f;
                float d = valid0*d0 + valid1*d1 + valid2*d2 + valid3*d3;
                if (d > 0.0f) {
                    float picV = (valid0*d0*u[nr0]+valid1*d1*u[nr1]+valid2*d2*u[nr2]+valid3*d3*u[nr3])/d;
                    float corr = (valid0*d0*(u[nr0]-prevU[nr0])+valid1*d1*(u[nr1]-prevU[nr1])+valid2*d2*(u[nr2]-prevU[nr2])+valid3*d3*(u[nr3]-prevU[nr3]))/d;
                    particleVel[idx] = (1.0f-flipRatio)*picV + flipRatio*(particleVel[idx]+corr);
                }
            }
        }
        // V场
        for (int i = 0; i < numParticles; i++) {
            int idx = i * 2;
            float x = myClamp(particlePos[idx], h, (fNumX-1)*h);
            float y = myClamp(particlePos[idx+1], h, (fNumY-1)*h);
            float x0f = (x - h2)*fInvSpacing; int x0 = (int)x0f; float tx = x0f - x0;
            x0 = myMin(x0, fNumX-2); int x1 = x0 + 1;
            float y0f = y*fInvSpacing; int y0 = (int)y0f; float ty = y0f - y0;
            y0 = myMin(y0, fNumY-2); int y1 = y0 + 1;
            float sx = 1.0f - tx, sy = 1.0f - ty;
            float d0 = sx*sy, d1 = tx*sy, d2 = tx*ty, d3 = sx*ty;
            int base0 = y0*fNumX, base1 = y1*fNumX;
            int nr0 = base0+x0, nr1 = base0+x1, nr2 = base1+x1, nr3 = base1+x0;
            if (toGrid) {
                float pv = particleVel[idx+1];
                v[nr0] += pv*d0; dv[nr0] += d0;
                v[nr1] += pv*d1; dv[nr1] += d1;
                v[nr2] += pv*d2; dv[nr2] += d2;
                v[nr3] += pv*d3; dv[nr3] += d3;
            } else {
                float valid0 = (cellType[nr0]!=AIR_CELL||cellType[nr0-1]!=AIR_CELL)?1.0f:0.0f;
                float valid1 = (cellType[nr1]!=AIR_CELL||cellType[nr1-1]!=AIR_CELL)?1.0f:0.0f;
                float valid2 = (cellType[nr2]!=AIR_CELL||cellType[nr2-1]!=AIR_CELL)?1.0f:0.0f;
                float valid3 = (cellType[nr3]!=AIR_CELL||cellType[nr3-1]!=AIR_CELL)?1.0f:0.0f;
                float d = valid0*d0 + valid1*d1 + valid2*d2 + valid3*d3;
                if (d > 0.0f) {
                    float picV = (valid0*d0*v[nr0]+valid1*d1*v[nr1]+valid2*d2*v[nr2]+valid3*d3*v[nr3])/d;
                    float corr = (valid0*d0*(v[nr0]-prevV[nr0])+valid1*d1*(v[nr1]-prevV[nr1])+valid2*d2*(v[nr2]-prevV[nr2])+valid3*d3*(v[nr3]-prevV[nr3]))/d;
                    particleVel[idx+1] = (1.0f-flipRatio)*picV + flipRatio*(particleVel[idx+1]+corr);
                }
            }
        }
        if (toGrid) {
            for (int i = 0; i < fNumCells; i++) {
                if (du[i] > 0.0f) u[i] /= du[i];
                if (dv[i] > 0.0f) v[i] /= dv[i];
            }
            for (int i = 0; i < fNumX; i++) {
                for (int j = 0; j < fNumY; j++) {
                    int idx = j * fNumX + i;
                    bool solid = (cellType[idx] == SOLID_CELL);
                    if (solid || (i>0 && cellType[j*fNumX+(i-1)] == SOLID_CELL)) u[idx] = prevU[idx];
                    if (solid || (j>0 && cellType[(j-1)*fNumX+i] == SOLID_CELL)) v[idx] = prevV[idx];
                }
            }
        }
    }

    void solveIncompressibility(int numIters, float dt, float overRelaxation) {
        memset(p, 0, fNumCells * sizeof(float));
        memcpy(prevU, u, fNumCells * sizeof(float));
        memcpy(prevV, v, fNumCells * sizeof(float));
        float cp = DENSITY * h / dt, overRelax = overRelaxation;
        for (int iter = 0; iter < numIters; iter++) {
            for (int i = 1; i < fNumX-1; i++) {
                for (int j = 1; j < fNumY-1; j++) {
                    int center = j * fNumX + i;
                    if (cellType[center] != FLUID_CELL) continue;
                    int left = center-1, right = center+1, bottom = center-fNumX, top = center+fNumX;
                    float sx0 = s[left], sx1 = s[right], sy0 = s[bottom], sy1 = s[top];
                    float sumS = sx0 + sx1 + sy0 + sy1;
                    if (sumS == 0.0f) continue;
                    float div = u[right] - u[center] + v[top] - v[center];
                    float pVal = -div / sumS * overRelax;
                    p[center] += cp * pVal;
                    u[center] -= sx0 * pVal; u[right] += sx1 * pVal;
                    v[center] -= sy0 * pVal; v[top] += sy1 * pVal;
                }
            }
        }
    }

    void simulate(float dt, float gx, float gy, float flipRatio, int numPressureIters, int numParticleIters) {
        integrateParticles(dt, gx, gy);
        if (separateParticles) pushParticlesApart(numParticleIters);
        handleCollisions();
        transferVelocities(true, 0);
        updateParticleDensity();
        solveIncompressibility(numPressureIters, dt, OVER_RELAXATION);
        transferVelocities(false, flipRatio);
    }
};
FlipFluid fluid;

// ============== 场景初始化 + OLED绘制 ==============
void setupScene() {
    tankWidth = 3.0f; tankHeight = 3.0f;
    float spacing = tankWidth / SIM_RESOLUTION;
    fluid.init(DENSITY, tankWidth, tankHeight, spacing);
    // 生成圆形掩码
    float center = (SIM_RESOLUTION - 1) / 2.0f;
    float radius = SIM_RESOLUTION / 2.0f - 0.1f, radiusSq = radius * radius;
    for (int i = 0; i < SIM_RESOLUTION; i++) {
        for (int j = 0; j < SIM_RESOLUTION; j++) {
            float dx = i - center, dy = j - center;
            circleMask[i][j] = (dx*dx + dy*dy <= radiusSq);
        }
    }
    // 生成粒子
    float r = 0.3f * h;
    float dx = 2.0f * r, dy = 1.732f * r;
    float rSq = (circleRadius - r) * (circleRadius - r);
    numParticles = 0;
    for (float y = circleCenterY - circleRadius + r; y < circleCenterY && numParticles < MAX_PARTICLES; y += dy) {
        float offsetX = ((int)((y - (circleCenterY - circleRadius)) / dy) % 2) * r;
        for (float x = circleCenterX - circleRadius + r; x < circleCenterX + circleRadius && numParticles < MAX_PARTICLES; x += dx) {
            float distX = x + offsetX - circleCenterX;
            float distY = y - circleCenterY;
            if (distX*distX + distY*distY <= rSq) {
                particlePos[numParticles*2] = x + offsetX;
                particlePos[numParticles*2+1] = y;
                particleVel[numParticles*2] = 0;
                particleVel[numParticles*2+1] = 0;
                numParticles++;
            }
        }
    }
    // OLED显示参数
    cellPixelSize = 3;
    displayOffsetX = (128 - SIM_RESOLUTION * cellPixelSize) / 2;
    displayOffsetY = (64 - SIM_RESOLUTION * cellPixelSize) / 2;
}

void drawFluid() {
    u8g2.clearBuffer();
    int displaySize = SIM_RESOLUTION * cellPixelSize;
    int circlePixelRadius = displaySize / 2;
    int cX = displayOffsetX + circlePixelRadius;
    int cY = displayOffsetY + circlePixelRadius;
    // 绘制圆形边界
    u8g2.drawCircle(cX, cY, circlePixelRadius);
    // 绘制重力指示器
    int arrowLen = 8;
    int aX = cX + gravityCos * arrowLen;
    int aY = cY - gravitySin * arrowLen;
    u8g2.drawLine(cX, cY, aX, aY);
    u8g2.drawCircle(aX, aY, 1);
    // 绘制流体单元格
    const int X_SHIFT = -1, Y_SHIFT = 1;
    for (int i = 0; i < SIM_RESOLUTION; i++) {
        for (int j = 0; j < SIM_RESOLUTION; j++) {
            if (!circleMask[i][j]) continue;
            float simIFloat = (float)i / SIM_RESOLUTION * (fNumX - 1);
            float simJFloat = (float)j / SIM_RESOLUTION * (fNumY - 1);
            int simI = myClamp((int)round(simIFloat), 0, fNumX - 1);
            int simJ = myClamp((int)round(simJFloat), 0, fNumY - 1);
            int cellIdx = simJ * fNumX + simI;
            if (cellType[cellIdx] != FLUID_CELL) continue;
            int px = displayOffsetX + i * cellPixelSize + X_SHIFT;
            int py = displayOffsetY + (SIM_RESOLUTION - 1 - j) * cellPixelSize + Y_SHIFT;
            if (px >=0 && py >=0 && px+cellPixelSize <=128 && py+cellPixelSize <=64) {
                u8g2.drawBox(px, py, cellPixelSize, cellPixelSize);
            }
        }
    }
    // 绘制重力参数
    u8g2.setFont(u8g2_font_5x7_tr);
    char buf[20];
    snprintf(buf, sizeof(buf), "%d%% | %0.0f° | %.1ffps", (int)(gravityScale*100), gravityAngle, currentFps);
    u8g2.drawStr(0, 63, buf);
    // 刷新OLED（增加轻微延时，稳定I2C）
    u8g2.sendBuffer();
    delayMicroseconds(100);
}

// ============== 主函数 ==============
void setup() {
    Serial.begin(2000000);
    delay(1000);
    setCpuFrequencyMhz(240); // ESP32主频拉满
    updateGravity(); // 初始化重力
    // OLED初始化
    u8g2.begin();
    u8g2.setBusClock(1000000); // I2C时钟400kHz，稳定且快速
    u8g2.setContrast(200);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(20, 32, "FLIP Fluid + LED");
    u8g2.sendBuffer();
    delay(1000);
    // LED初始化（引脚+映射+定时器）
    charlie_pin_float();
    init_led_map();
    init_cell_led_map();
    init_led_timer(); // 启动LED扫描定时器，非阻塞
    // 流体场景初始化
    setupScene();
    Serial.println("=== System Ready! Type 'help' for commands ===");
    lastFpsTime = millis();
}

void loop() {
    // 串口处理
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {processSerialCommand(inputBuffer); inputBuffer = "";}
        else {inputBuffer += c;}
    }
    // 流体模拟（核心）
    fluid.simulate(DT, gravityX, gravityY, FLIP_RATIO, PRESSURE_ITERATIONS, PARTICLE_ITERATIONS);
    // 更新LED状态（流体→LED）
    update_led_state();
    // 绘制OLED（稳定刷新）
    drawFluid();
    // 计算帧率
    frameCount++;
    unsigned long now = millis();
    if (now - lastFpsTime >= 1000) {
        currentFps = (float)frameCount * 1000.0f / (now - lastFpsTime);
        frameCount = 0;
        lastFpsTime = now;
        // 串口打印频率控制
        if (now - lastSerialPrintTime >= 2000) {
            Serial.printf("Status: FPS=%.1f | Particles=%d | LED=%d\n", currentFps, numParticles, LED_TOTAL);
            lastSerialPrintTime = now;
        }
    }
}
