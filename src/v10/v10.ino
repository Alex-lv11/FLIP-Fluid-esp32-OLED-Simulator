#include <Arduino.h>
#include <U8g2lib.h>
#include <Adafruit_NeoPixel.h>
#include <cstring>
#include <Wire.h>
#include <esp_sleep.h>
#include <cmath>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ===================== 【全部可调参数 集中顶部】 =====================
// 硬件引脚
#define RGB_LED_PIN 48
#define RGB_LED_COUNT 1
#define MPU_INT_PIN 4
#define MPU_SDA 8
#define MPU_SCL 9
#define MPU_ADDR 0x68
#define IDLE_SLEEP_MS 3000
#define ANGLE_CHANGE_THRESHOLD float(5.0)

// 俯仰角度对应基础重力规则
#define GRAV_SCALE_MIN float(0.0)
#define GRAV_SCALE_MAX_BASE float(1)
#define PITCH_MAX_ANGLE float(90.0)

// 晃动冲击叠加规则
#define SHAKE_G_THRESHOLD float(1.3)
#define SHAKE_GAIN_PER_G float(2)
#define TOTAL_GRAV_MAX float(5)

// FLIP流体性能配置
#define SIM_RESOLUTION 20
#define MAX_PARTICLES 400
#define PRESSURE_ITERATIONS 10
#define PARTICLE_ITERATIONS 1
#define USE_ESP32

// ===================== 外设实例 =====================
Adafruit_NeoPixel rgbLed(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
#define COLOR_GREEN rgbLed.Color(0, 100, 0)
#define COLOR_RED rgbLed.Color(100, 0, 0)
#define COLOR_OFF rgbLed.Color(0, 0, 0)

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, MPU_SCL, MPU_SDA);
Adafruit_MPU6050 mpu;

#define ICACHE_FLASH_ATTR __attribute__((section(".irom0.text")))

// ===================== 全局重力/姿态变量 =====================
float gravityDirAngle = float(270.0);
float baseGravScale = float(0.0);
float shakeAddScale = float(0.0);
float totalGravScale = float(0.0);
const float BASE_GRAVITY = float(9.8);
float gravityX = float(0.0);
float gravityY = float(0.0);

float ax, ay, az;
float pitchAngle = float(0.0);
float totalGValue = float(1.0);
float lastAngle = float(270.0);
unsigned long lastMoveTime = 0;

// FLIP流体常量
const float DT = float(0.041);
const float FLIP_RATIO = float(0.9);
const float OVER_RELAXATION = float(1.9);
const float DENSITY = float(1000.0);
#define FLUID_CELL 0
#define AIR_CELL 1
#define SOLID_CELL 2

// FLIP流体内存
int fNumX, fNumY, fNumCells;
float h, fInvSpacing;
float *u, *v;
float *prevU, *prevV;
float *du, *dv;
float *p;
float *s;
int *cellType;
float *particleDensity;
int numParticles = 0;
float particlePos[MAX_PARTICLES * 2];
float particleVel[MAX_PARTICLES * 2];
float pInvSpacing;
int pNumX, pNumY, pNumCells;
int *numCellParticles;
int *firstCellParticle;
int *cellParticleIds;
bool compensateDrift = true;
bool separateParticles = true;
float simWidth, simHeight;
float tankWidth, tankHeight;
float circleCenterX, circleCenterY;
float circleRadius, circleRadiusSq;
int displayOffsetX, displayOffsetY;
int cellPixelSize;
unsigned long lastFpsTime = 0;
int frameCount = 0;
float currentFps = float(0.0);
bool circleMask[SIM_RESOLUTION][SIM_RESOLUTION];

// ===================== 工具函数 =====================
inline float myClamp(float x, float minVal, float maxVal) {
    return (x < minVal) ? minVal : (x > maxVal) ? maxVal : x;
}
inline float myMin(float a, float b) { return (a < b) ? a : b; }
inline int myMin(int a, int b) { return (a < b) ? a : b; }
inline float myMax(float a, float b) { return (a > b) ? a : b; }
inline int myMax(int a, int b) { return (a > b) ? a : b; }
inline float degToRad(float deg) { return deg * PI / float(180.0); }
inline float radToDeg(float rad) { return rad * float(180.0) / PI; }

void updateGravityVector() {
    float normalizedAngle = fmod(gravityDirAngle, float(360.0));
    if (normalizedAngle < float(0.0)) normalizedAngle += float(360.0);
    float rad = degToRad(normalizedAngle);
    float gravityCos = cos(rad);
    float gravitySin = sin(rad);
    float gravCoef = totalGravScale * BASE_GRAVITY;
    gravityX = gravityCos * gravCoef;
    gravityY = -gravitySin * gravCoef;
}

// 读取Adafruit库加速度数据
void readMPUAccel() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    ax = a.acceleration.x;
    ay = a.acceleration.y;
    az = a.acceleration.z;

    // 合加速度G值
    float totalAcc = sqrt(ax*ax + ay*ay + az*az);
    totalGValue = totalAcc / float(9.81);

    // 俯仰角Pitch
    pitchAngle = radToDeg(atan2(ay, sqrt(ax*ax + az*az)));
    float absPitch = fabs(pitchAngle);
    baseGravScale = myClamp(absPitch / PITCH_MAX_ANGLE, GRAV_SCALE_MIN, float(1.0)) * GRAV_SCALE_MAX_BASE;

    // 设备倾斜总角度，控制流体重力方向
    float newAngle = radToDeg(atan2(-ay, -ax));
    gravityDirAngle = newAngle;
    updateGravityVector();

    // 角度差值判断设备是否移动
    float angleDiff = fabs(newAngle - lastAngle);
    if (angleDiff > float(180.0)) angleDiff = float(360.0) - angleDiff;

    // 晃动叠加重力倍率
    shakeAddScale = float(0.0);
    if(totalGValue > SHAKE_G_THRESHOLD){
        float overG = totalGValue - SHAKE_G_THRESHOLD;
        shakeAddScale = overG * SHAKE_GAIN_PER_G;
    }
    totalGravScale = myClamp(baseGravScale + shakeAddScale, GRAV_SCALE_MIN, TOTAL_GRAV_MAX);

    if (angleDiff > ANGLE_CHANGE_THRESHOLD) {
        lastMoveTime = millis();
    }
    lastAngle = newAngle;
}

// 深度休眠函数，彻底修复一休眠立刻重启BUG
void enterDeepSleep() {
    // 屏幕提示休眠
    u8g2.clearBuffer();
    u8g2.drawStr(15,32,"Enter DeepSleep");
    u8g2.sendBuffer();
    delay(300);
    u8g2.setPowerSave(1);

    // 关闭RGB灯带
    rgbLed.setPixelColor(0, COLOR_OFF);
    rgbLed.show();

    // 释放I2C总线，消除引脚漏电电平干扰
    Wire.end();
    // 唤醒引脚改为纯输入浮空，禁止内部上拉冲突
    pinMode(MPU_INT_PIN, INPUT);

    // GPIO4下降沿唤醒
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 0);
    esp_deep_sleep_start();
}

// ===================== FLIP流体核心 =====================
class FlipFluid {
public:
    void init(float density, float width, float height, float spacing, int maxParts) {
        fNumX = (int)(width / spacing) + 1;
        fNumY = (int)(height / spacing) + 1;
        fNumCells = fNumX * fNumY;
        h = myMax(width / fNumX, height / fNumY);
        fInvSpacing = float(1.0) / h;

        u = new float[fNumCells]();
        v = new float[fNumCells]();
        prevU = new float[fNumCells]();
        prevV = new float[fNumCells]();
        du = new float[fNumCells]();
        dv = new float[fNumCells]();
        p = new float[fNumCells]();
        s = new float[fNumCells]();
        cellType = new int[fNumCells]();
        particleDensity = new float[fNumCells]();

        float particleRadius = float(0.3) * h;
        pInvSpacing = float(1.0) / (float(2.2) * particleRadius);
        pNumX = (int)(width * pInvSpacing) + 1;
        pNumY = (int)(height * pInvSpacing) + 1;
        pNumCells = pNumX * pNumY;

        numCellParticles = new int[pNumCells]();
        firstCellParticle = new int[pNumCells + 1]();
        cellParticleIds = new int[maxParts]();

        simWidth = width;
        simHeight = height;
        circleCenterX = width / float(2.0);
        circleCenterY = height / float(2.0);
        circleRadius = (myMin(width, height) / float(2.0)) - h;
        circleRadiusSq = circleRadius * circleRadius;
    }

    void integrateParticles(float dt, float gx, float gy) {
        float gx_dt = gx * dt;
        float gy_dt = gy * dt;
        float dt_val = dt;
        for (int i = 0; i < numParticles; i++) {
            int idx = i * 2;
            particleVel[idx] += gx_dt;
            particleVel[idx+1] += gy_dt;
            particlePos[idx] += particleVel[idx] * dt_val;
            particlePos[idx+1] += particleVel[idx+1] * dt_val;
        }
    }

    void pushParticlesApart(int numIters) {
        for (int i = 0; i < pNumCells; i++) numCellParticles[i] = 0;
        for (int i = 0; i < numParticles; i++) {
            float x = particlePos[i * 2];
            float y = particlePos[i * 2 + 1];
            int xi = myClamp((int)(x * pInvSpacing), 0, pNumX - 1);
            int yi = myClamp((int)(y * pInvSpacing), 0, pNumY - 1);
            numCellParticles[xi * pNumY + yi]++;
        }
        int first = 0;
        for (int i = 0; i < pNumCells; i++) {
            first += numCellParticles[i];
            firstCellParticle[i] = first;
        }
        firstCellParticle[pNumCells] = first;
        for (int i = 0; i < numParticles; i++) {
            float x = particlePos[i * 2];
            float y = particlePos[i * 2 + 1];
            int xi = myClamp((int)(x * pInvSpacing), 0, pNumX - 1);
            int yi = myClamp((int)(y * pInvSpacing), 0, pNumY - 1);
            int cellNr = xi * pNumY + yi;
            firstCellParticle[cellNr]--;
            cellParticleIds[firstCellParticle[cellNr]] = i;
        }
        float minDist = float(2.0) * float(0.3) * h;
        float minDist2 = minDist * minDist;
        for (int iter = 0; iter < numIters; iter++) {
            for (int i = 0; i < numParticles; i++) {
                float px = particlePos[i * 2];
                float py = particlePos[i * 2 + 1];
                int pxi = (int)(px * pInvSpacing);
                int pyi = (int)(py * pInvSpacing);
                int x0 = myMax(pxi - 1, 0);
                int y0 = myMax(pyi - 1, 0);
                int x1 = myMin(pxi + 1, pNumX - 1);
                int y1 = myMin(pyi + 1, pNumY - 1);
                for (int xi = x0; xi <= x1; xi++) {
                    for (int yi = y0; yi <= y1; yi++) {
                        int cellNr = xi * pNumY + yi;
                        int f = firstCellParticle[cellNr];
                        int l = firstCellParticle[cellNr + 1];
                        for (int j = f; j < l; j++) {
                            int id = cellParticleIds[j];
                            if (id == i) continue;
                            float qx = particlePos[id * 2];
                            float qy = particlePos[id * 2 + 1];
                            float dx = qx - px;
                            float dy = qy - py;
                            float d2 = dx * dx + dy * dy;
                            if (d2 > minDist2 || d2 == float(0.0)) continue;
                            float d_inv = float(1.0) / sqrt(d2);
                            float s = float(0.5) * (minDist - d2 * d_inv) * d_inv;
                            dx *= s; dy *= s;
                            particlePos[i * 2] -= dx;
                            particlePos[i * 2 + 1] -= dy;
                            particlePos[id * 2] += dx;
                            particlePos[id * 2 + 1] += dy;
                        }
                    }
                }
            }
        }
    }

    void handleCollisions() {
        float r = float(0.3) * h;
        float rSq = (circleRadius - r) * (circleRadius - r);
        for (int i = 0; i < numParticles; i++) {
            int idx = i * 2;
            float x = particlePos[idx];
            float y = particlePos[idx+1];
            float dx = x - circleCenterX;
            float dy = y - circleCenterY;
            float distSq = dx * dx + dy * dy;
            if (distSq > rSq) {
                float dist = sqrt(distSq);
                float scale = (circleRadius - r) / dist;
                particlePos[idx] = circleCenterX + dx * scale;
                particlePos[idx+1] = circleCenterY + dy * scale;
                float nx = dx / dist;
                float ny = dy / dist;
                float vn = particleVel[idx] * nx + particleVel[idx+1] * ny;
                particleVel[idx] -= vn * nx;
                particleVel[idx+1] -= vn * ny;
            }
        }
    }

    void updateParticleDensity() {
        int n = fNumY;
        float h1 = fInvSpacing;
        float h2 = float(0.5) * h;
        float *d = particleDensity;
        memset(d, 0, fNumCells * sizeof(float));
        for (int i = 0; i < numParticles; i++) {
            int idx = i * 2;
            float x = particlePos[idx];
            float y = particlePos[idx+1];
            x = myClamp(x, h, (fNumX - 1) * h);
            y = myClamp(y, h, (fNumY - 1) * h);
            int x0 = (int)((x - h2) * h1);
            float tx = ((x - h2) - x0 * h) * h1;
            int x1 = myMin(x0 + 1, fNumX - 2);
            int y0 = (int)((y - h2) * h1);
            float ty = ((y - h2) - y0 * h) * h1;
            int y1 = myMin(y0 + 1, fNumY - 2);
            float sx = float(1.0) - tx;
            float sy = float(1.0) - ty;
            int base0 = y0 * fNumX;
            int base1 = y1 * fNumX;
            if (x0 < fNumX && y0 < fNumY) d[base0 + x0] += sx * sy;
            if (x1 < fNumX && y0 < fNumY) d[base0 + x1] += tx * sy;
            if (x1 < fNumX && y1 < fNumY) d[base1 + x1] += tx * ty;
            if (x0 < fNumX && y1 < fNumY) d[base1 + x0] += sx * ty;
        }
        for (int i = 1; i < fNumX-1; i++) {
            for (int j = 1; j < fNumY-1; j++) {
                int idx = j * fNumX + i;
                if (cellType[idx] == AIR_CELL && particleDensity[idx] > float(0.1)) {
                    cellType[idx] = FLUID_CELL;
                }
            }
        }
    }

    void transferVelocities(bool toGrid, float flipRatio) {
        int n = fNumY;
        float h2 = float(0.5) * h;
        if (toGrid) {
            memcpy(prevU, u, fNumCells * sizeof(float));
            memcpy(prevV, v, fNumCells * sizeof(float));
            memset(du, 0, fNumCells * sizeof(float));
            memset(dv, 0, fNumCells * sizeof(float));
            memset(u, 0, fNumCells * sizeof(float));
            memset(v, 0, fNumCells * sizeof(float));
            for (int i = 0; i < fNumX; i++) {
                for (int j = 0; j < fNumY; j++) {
                    float cellX = i * h;
                    float cellY = j * h;
                    float dx = cellX - circleCenterX;
                    float dy = cellY - circleCenterY;
                    float distSq = dx * dx + dy * dy;
                    bool solid = (distSq >= circleRadiusSq - h*h);
                    int idx = j * fNumX + i;
                    s[idx] = solid ? float(0.0) : float(1.0);
                    cellType[idx] = solid ? SOLID_CELL : AIR_CELL;
                }
            }
            for (int i = 0; i < numParticles; i++) {
                int idx = i * 2;
                float x = particlePos[idx];
                float y = particlePos[idx+1];
                int xi = myClamp((int)(x * fInvSpacing), 0, fNumX - 1);
                int yi = myClamp((int)(y * fInvSpacing), 0, fNumY - 1);
                int cellNr = yi * fNumX + xi;
                if (cellType[cellNr] == AIR_CELL) cellType[cellNr] = FLUID_CELL;
            }
        }
        for (int i = 0; i < numParticles; i++) {
            int idx = i * 2;
            float x = particlePos[idx];
            float y = particlePos[idx+1];
            x = myClamp(x, h, (fNumX - 1) * h);
            y = myClamp(y, h, (fNumY - 1) * h);
            float x0f = x * fInvSpacing;
            int x0 = (int)x0f;
            float tx = x0f - x0;
            x0 = myMin(x0, fNumX - 2);
            int x1 = x0 + 1;
            float y0f = (y - h2) * fInvSpacing;
            int y0 = (int)y0f;
            float ty = y0f - y0;
            y0 = myMin(y0, fNumY - 2);
            int y1 = y0 + 1;
            float sx = float(1.0) - tx;
            float sy = float(1.0) - ty;
            float d0 = sx * sy, d1 = tx * sy, d2 = tx * ty, d3 = sx * ty;
            int base0 = y0 * fNumX;
            int base1 = y1 * fNumX;
            int nr0 = base0 + x0;
            int nr1 = base0 + x1;
            int nr2 = base1 + x1;
            int nr3 = base1 + x0;
            if (toGrid) {
                float pv = particleVel[idx];
                u[nr0] += pv * d0; du[nr0] += d0;
                u[nr1] += pv * d1; du[nr1] += d1;
                u[nr2] += pv * d2; du[nr2] += d2;
                u[nr3] += pv * d3; du[nr3] += d3;
            } else {
                float valid0 = (cellType[nr0] != AIR_CELL || cellType[nr0 - fNumX] != AIR_CELL) ? float(1.0) : float(0.0);
                float valid1 = (cellType[nr1] != AIR_CELL || cellType[nr1 - fNumX] != AIR_CELL) ? float(1.0) : float(0.0);
                float valid2 = (cellType[nr2] != AIR_CELL || cellType[nr2 - fNumX] != AIR_CELL) ? float(1.0) : float(0.0);
                float valid3 = (cellType[nr3] != AIR_CELL || cellType[nr3 - fNumX] != AIR_CELL) ? float(1.0) : float(0.0);
                float d = valid0 * d0 + valid1 * d1 + valid2 * d2 + valid3 * d3;
                if (d > float(0.0)) {
                    float picV = (valid0 * d0 * u[nr0] + valid1 * d1 * u[nr1] + valid2 * d2 * u[nr2] + valid3 * d3 * u[nr3]) / d;
                    float corr = (valid0 * d0 * (u[nr0] - prevU[nr0]) + valid1 * d1 * (u[nr1] - prevU[nr1]) + valid2 * d2 * (u[nr2] - prevU[nr2]) + valid3 * d3 * (u[nr3] - prevU[nr3])) / d;
                    float flipV = particleVel[idx] + corr;
                    particleVel[idx] = (float(1.0) - flipRatio) * picV + flipRatio * flipV;
                }
            }
        }
        for (int i = 0; i < numParticles; i++) {
            int idx = i * 2;
            float x = particlePos[idx];
            float y = particlePos[idx+1];
            x = myClamp(x, h, (fNumX - 1) * h);
            y = myClamp(y, h, (fNumY - 1) * h);
            float x0f = (x - h2) * fInvSpacing;
            int x0 = (int)x0f;
            float tx = x0f - x0;
            x0 = myMin(x0, fNumX - 2);
            int x1 = x0 + 1;
            float y0f = y * fInvSpacing;
            int y0 = (int)y0f;
            float ty = y0f - y0;
            y0 = myMin(y0, fNumY - 2);
            int y1 = y0 + 1;
            float sx = float(1.0) - tx;
            float sy = float(1.0) - ty;
            float d0 = sx * sy, d1 = tx * sy, d2 = tx * ty, d3 = sx * ty;
            int base0 = y0 * fNumX;
            int base1 = y1 * fNumX;
            int nr0 = base0 + x0;
            int nr1 = base0 + x1;
            int nr2 = base1 + x1;
            int nr3 = base1 + x0;
            if (toGrid) {
                float pv = particleVel[idx+1];
                v[nr0] += pv * d0; dv[nr0] += d0;
                v[nr1] += pv * d1; dv[nr1] += d1;
                v[nr2] += pv * d2; dv[nr2] += d2;
                v[nr3] += pv * d3; dv[nr3] += d3;
            } else {
                float valid0 = (cellType[nr0] != AIR_CELL || cellType[nr0 - 1] != AIR_CELL) ? float(1.0) : float(0.0);
                float valid1 = (cellType[nr1] != AIR_CELL || cellType[nr1 - 1] != AIR_CELL) ? float(1.0) : float(0.0);
                float valid2 = (cellType[nr2] != AIR_CELL || cellType[nr2 - 1] != AIR_CELL) ? float(1.0) : float(0.0);
                float valid3 = (cellType[nr3] != AIR_CELL || cellType[nr3 - 1] != AIR_CELL) ? float(1.0) : float(0.0);
                float d = valid0 * d0 + valid1 * d1 + valid2 * d2 + valid3 * d3;
                if (d > float(0.0)) {
                    float picV = (valid0 * d0 * v[nr0] + valid1 * d1 * v[nr1] + valid2 * d2 * v[nr2] + valid3 * d3 * v[nr3]) / d;
                    float corr = (valid0 * d0 * (v[nr0] - prevV[nr0]) + valid1 * d1 * (v[nr1] - prevV[nr1]) + valid2 * d2 * (v[nr2] - prevV[nr2]) + valid3 * d3 * (v[nr3] - prevV[nr3])) / d;
                    float flipV = particleVel[idx+1] + corr;
                    particleVel[idx+1] = (float(1.0) - flipRatio) * picV + flipRatio * flipV;
                }
            }
        }
        if (toGrid) {
            for (int i = 0; i < fNumCells; i++) {
                if (du[i] > float(0.0)) u[i] /= du[i];
                if (dv[i] > float(0.0)) v[i] /= dv[i];
            }
            for (int i = 0; i < fNumX; i++) {
                for (int j = 0; j < fNumY; j++) {
                    int idx = j * fNumX + i;
                    bool solid = (cellType[idx] == SOLID_CELL);
                    if (solid || (i > 0 && cellType[j * fNumX + (i - 1)] == SOLID_CELL))
                        u[idx] = prevU[idx];
                    if (solid || (j > 0 && cellType[(j - 1) * fNumX + i] == SOLID_CELL))
                        v[idx] = prevV[idx];
                }
            }
        }
    }

    void solveIncompressibility(int numIters, float dt, float overRelaxation) {
        memset(p, 0, fNumCells * sizeof(float));
        memcpy(prevU, u, fNumCells * sizeof(float));
        memcpy(prevV, v, fNumCells * sizeof(float));
        float cp = DENSITY * h / dt;
        float overRelax = overRelaxation;
        for (int iter = 0; iter < numIters; iter++) {
            for (int i = 1; i < fNumX - 1; i++) {
                for (int j = 1; j < fNumY - 1; j++) {
                    int center = j * fNumX + i;
                    if (cellType[center] != FLUID_CELL) continue;
                    int left = center - 1;
                    int right = center + 1;
                    int bottom = center - fNumX;
                    int top = center + fNumX;
                    float sx0 = s[left], sx1 = s[right];
                    float sy0 = s[bottom], sy1 = s[top];
                    float sumS = sx0 + sx1 + sy0 + sy1;
                    if (sumS == float(0.0)) continue;
                    float div = u[right] - u[center] + v[top] - v[center];
                    float pVal = -div / sumS * overRelax;
                    p[center] += cp * pVal;
                    u[center] -= sx0 * pVal;
                    u[right] += sx1 * pVal;
                    v[center] -= sy0 * pVal;
                    v[top] += sy1 * pVal;
                }
            }
        }
    }

    void simulate(float dt, float gx, float gy, float flipRatio, int numPressureIters, int numParticleIters, float overRelaxation) {
        integrateParticles(dt, gx, gy);
        if (separateParticles) pushParticlesApart(numParticleIters);
        handleCollisions();
        transferVelocities(true, float(0.0));
        updateParticleDensity();
        solveIncompressibility(numPressureIters, dt, overRelaxation);
        transferVelocities(false, flipRatio);
    }
};
FlipFluid fluid;

void setupScene() {
    tankWidth = float(3.0);
    tankHeight = float(3.0);
    float spacing = tankWidth / SIM_RESOLUTION;
    fluid.init(DENSITY, tankWidth, tankHeight, spacing, MAX_PARTICLES);
    float center = (SIM_RESOLUTION - float(1.0)) / float(2.0);
    float radius = SIM_RESOLUTION / float(2.0) - float(0.1);
    float radiusSq = radius * radius;
    for (int i = 0; i < SIM_RESOLUTION; i++) {
        for (int j = 0; j < SIM_RESOLUTION; j++) {
            float dx = i - center;
            float dy = j - center;
            circleMask[i][j] = (dx * dx + dy * dy) <= radiusSq;
        }
    }
    float r = float(0.3) * h;
    float dx = float(2.0) * r;
    float dy = float(1.732) * r;
    float rSq = (circleRadius - r) * (circleRadius - r);
    numParticles = 0;
    for (float y = circleCenterY - circleRadius + r; y < circleCenterY && numParticles < MAX_PARTICLES; y += dy) {
        float offsetX = ((int)((y - (circleCenterY - circleRadius)) / dy) % 2) * r;
        for (float x = circleCenterX - circleRadius + r; x < circleCenterX + circleRadius && numParticles < MAX_PARTICLES; x += dx) {
            float distX = x + offsetX - circleCenterX;
            float distY = y - circleCenterY;
            if (distX * distX + distY * distY <= rSq) {
                particlePos[numParticles * 2] = x + offsetX;
                particlePos[numParticles * 2 + 1] = y;
                particleVel[numParticles * 2] = float(0.0);
                particleVel[numParticles * 2 + 1] = float(0.0);
                numParticles++;
            }
        }
    }
    cellPixelSize = 3;
    displayOffsetX = (128 - SIM_RESOLUTION * cellPixelSize) / 2;
    displayOffsetY = (64 - SIM_RESOLUTION * cellPixelSize) / 2;
}

void drawFluid() {
    u8g2.clearBuffer();
    int displaySize = SIM_RESOLUTION * cellPixelSize;
    int circlePixelRadius = displaySize / 2;
    int circlePixelCenterX = displayOffsetX + circlePixelRadius;
    int circlePixelCenterY = displayOffsetY + circlePixelRadius;
    u8g2.drawCircle(circlePixelCenterX, circlePixelCenterY, circlePixelRadius);
    int arrowLen = 8;
    float gravityCos = cos(degToRad(gravityDirAngle));
    float gravitySin = sin(degToRad(gravityDirAngle));
    int arrowX = circlePixelCenterX + gravityCos * arrowLen;
    int arrowY = circlePixelCenterY - gravitySin * arrowLen;
    u8g2.drawLine(circlePixelCenterX, circlePixelCenterY, arrowX, arrowY);
    u8g2.drawCircle(arrowX, arrowY, 1);
    const int X_SHIFT = -1;
    const int Y_SHIFT = 1;
    for (int i = 0; i < SIM_RESOLUTION; i++) {
        for (int j = 0; j < SIM_RESOLUTION; j++) {
            if (!circleMask[i][j]) continue;
            float simIFloat = (float)i / SIM_RESOLUTION * (fNumX - float(1.0));
            float simJFloat = (float)j / SIM_RESOLUTION * (fNumY - float(1.0));
            int simI = myClamp((int)round(simIFloat), 0, fNumX - 1);
            int simJ = myClamp((int)round(simJFloat), 0, fNumY - 1);
            int cellIdx = simJ * fNumX + simI;
            if (cellType[cellIdx] != FLUID_CELL) continue;
            int px = displayOffsetX + i * cellPixelSize + X_SHIFT;
            int py = displayOffsetY + (SIM_RESOLUTION - 1 - j) * cellPixelSize + Y_SHIFT;
            if (px >= 0 && py >=0 && px + cellPixelSize <= 128 && py + cellPixelSize <=64) {
                u8g2.drawBox(px, py, cellPixelSize, cellPixelSize);
            }
        }
    }
    u8g2.setFont(u8g2_font_5x7_tr);
    char buf[60];
    unsigned long idleRemain = (IDLE_SLEEP_MS - (millis() - lastMoveTime)) / 1000;
    snprintf(buf, sizeof(buf), "Pit:%.0f Base:%.2f ShakeG:%.2f TotalG:%.2f Wait:%lus",
             pitchAngle, baseGravScale, totalGValue, totalGravScale, idleRemain);
    u8g2.drawStr(0, 63, buf);
    u8g2.sendBuffer();
}

void ledQuickGreenFlash(int times, int delayMs) {
    for(int i=0; i<times; i++){
        rgbLed.setPixelColor(0, COLOR_GREEN);
        rgbLed.show();
        delay(delayMs);
        rgbLed.setPixelColor(0, COLOR_OFF);
        rgbLed.show();
        delay(delayMs);
    }
}
void ledErrorRedLoop() {
    while(1){
        rgbLed.setPixelColor(0, COLOR_RED);
        rgbLed.show();
        delay(200);
        rgbLed.setPixelColor(0, COLOR_OFF);
        rgbLed.show();
        delay(200);
    }
}

// ===================== Setup =====================
void setup() {
    rgbLed.begin();
    rgbLed.setPixelColor(0, COLOR_OFF);
    rgbLed.show();
    ledQuickGreenFlash(2, 300);

    setCpuFrequencyMhz(240);

    Wire.begin(MPU_SDA, MPU_SCL, 100000UL);
    delay(100);

    // Adafruit MPU6050 初始化
    if (!mpu.begin(MPU_ADDR)) {
        ledErrorRedLoop();
    }
    // ±16G 加速度量程
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    pinMode(MPU_INT_PIN, INPUT);

    lastMoveTime = millis();
    lastAngle = gravityDirAngle;

    u8g2.begin();
    u8g2.setBusClock(1000000);
    u8g2.setContrast(200);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(8,30,"FLIP Tilt+Shake Gravity");
    u8g2.sendBuffer();

    bool sceneInitOk = true;
    try{
        setupScene();
    }catch(...){
        sceneInitOk = false;
    }
    if(!sceneInitOk){
        ledErrorRedLoop();
    }
    delay(500);
}

void loop() {
    readMPUAccel();

    unsigned long idleTime = millis() - lastMoveTime;
    if(idleTime >= IDLE_SLEEP_MS){
        enterDeepSleep();
    }

    fluid.simulate(DT, gravityX, gravityY, FLIP_RATIO, PRESSURE_ITERATIONS, PARTICLE_ITERATIONS, OVER_RELAXATION);

    drawFluid();

    frameCount++;
    unsigned long now = millis();
    if (now - lastFpsTime >= 1000) {
        currentFps = frameCount * float(1000.0) / (now - lastFpsTime);
        frameCount = 0;
        lastFpsTime = now;
        rgbLed.setPixelColor(0, COLOR_GREEN);
        rgbLed.show();
        delay(80);
        rgbLed.setPixelColor(0, COLOR_OFF);
        rgbLed.show();
    }
}
