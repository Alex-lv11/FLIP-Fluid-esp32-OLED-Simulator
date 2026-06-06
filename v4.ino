/*
  FLIP Fluid Simulation for ESP32/ESP8266 with OLED 128x64
  Based on Ten Minute Physics FLIP implementation
  
  Required libraries:
  - U8g2 by oliver: https://github.com/olikraus/u8g2
*/

#include <Arduino.h>
#include <U8g2lib.h>

// ============== 配置区域 ==============
// 选择屏幕分辨率: 16, 32, 或 64 (必须是2的幂且能被128和64整除或适配)
#define SIM_RESOLUTION 16

// 选择平台
#define USE_ESP32
//#define USE_ESP8266

// 选择OLED类型 (根据你的硬件修改)
#ifdef USE_ESP8266
  // ESP8266 软件I2C (D1=GPIO5=SCL, D2=GPIO4=SDA)
  U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 5, /* data=*/ 4, /* reset=*/ U8X8_PIN_NONE);
#else
  // ESP32 硬件I2C
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#endif

// ============== 模拟参数 ==============
#define SIM_SIZE SIM_RESOLUTION
#define MAX_PARTICLES 700  // ESP8266内存有限，减少粒子数

// 物理常量
const float DT = 0.016f; //0.016f
const float GRAVITY = -9.8f;
const float FLIP_RATIO = 0.9f;
const float OVER_RELAXATION = 1.9f;
const int PRESSURE_ITERATIONS = 20;
const int PARTICLE_ITERATIONS = 2;
const float DENSITY = 1000.0f;

// 单元格类型
#define FLUID_CELL 0
#define AIR_CELL 1
#define SOLID_CELL 2

// ============== 全局变量 ==============
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
uint8_t particleColor[MAX_PARTICLES];

float pInvSpacing;
int pNumX, pNumY, pNumCells;
int *numCellParticles;
int *firstCellParticle;
int *cellParticleIds;

float obstacleX = 0.0f, obstacleY = 0.0f;
float obstacleRadius = 0.15f;
bool compensateDrift = true;
bool separateParticles = true;
float obstacleVelX = 0.0f, obstacleVelY = 0.0f;

float simWidth, simHeight;
float tankWidth, tankHeight;

int displayOffsetX, displayOffsetY;
float displayScale;

unsigned long lastFpsTime = 0;
int frameCount = 0;
float currentFps = 0;

// ============== 工具函数 ==============
float myClamp(float x, float minVal, float maxVal) {
    if (x < minVal) return minVal;
    if (x > maxVal) return maxVal;
    return x;
}

// 自定义min函数，处理类型不匹配
float myMin(float a, float b) {
    return (a < b) ? a : b;
}

int myMin(int a, int b) {
    return (a < b) ? a : b;
}

// ============== FLIP 类实现 ==============
class FlipFluid {
public:
    void init(float density, float width, float height, float spacing, int maxParts) {
        fNumX = (int)(width / spacing) + 1;
        fNumY = (int)(height / spacing) + 1;
        fNumCells = fNumX * fNumY;
        h = max(width / fNumX, height / fNumY);
        fInvSpacing = 1.0f / h;
        
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
        
        float particleRadius = 0.3f * h;
        pInvSpacing = 1.0f / (2.2f * particleRadius);
        pNumX = (int)(width * pInvSpacing) + 1;
        pNumY = (int)(height * pInvSpacing) + 1;
        pNumCells = pNumX * pNumY;
        
        numCellParticles = new int[pNumCells]();
        firstCellParticle = new int[pNumCells + 1]();
        cellParticleIds = new int[maxParts]();
        
        simWidth = width;
        simHeight = height;
        
        Serial.printf("Grid: %dx%d = %d cells\n", fNumX, fNumY, fNumCells);
        Serial.printf("Particles: max %d\n", maxParts);
    }
    
    void integrateParticles(float dt, float gravity) {
        for (int i = 0; i < numParticles; i++) {
            particleVel[i * 2 + 1] += dt * gravity;
            particlePos[i * 2] += particleVel[i * 2] * dt;
            particlePos[i * 2 + 1] += particleVel[i * 2 + 1] * dt;
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
        
        float minDist = 2.0f * 0.3f * h;
        float minDist2 = minDist * minDist;
        
        for (int iter = 0; iter < numIters; iter++) {
            for (int i = 0; i < numParticles; i++) {
                float px = particlePos[i * 2];
                float py = particlePos[i * 2 + 1];
                
                int pxi = (int)(px * pInvSpacing);
                int pyi = (int)(py * pInvSpacing);
                int x0 = max(pxi - 1, 0);
                int y0 = max(pyi - 1, 0);
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
                            
                            if (d2 > minDist2 || d2 == 0.0f) continue;
                            
                            float d = sqrt(d2);
                            float s = 0.5f * (minDist - d) / d;
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
        float r = 0.3f * h;
        float obsR = obstacleRadius;  //  renamed 'or' to 'obsR'
        float minDist = obsR + r;      // 使用新变量名
        float minDist2 = minDist * minDist;
        float minX = h + r;
        float maxX = (fNumX - 1) * h - r;
        float minY = h + r;
        float maxY = (fNumY - 1) * h - r;
        
        for (int i = 0; i < numParticles; i++) {
            float x = particlePos[i * 2];
            float y = particlePos[i * 2 + 1];
            
            // 障碍物碰撞
            float dx = x - obstacleX;
            float dy = y - obstacleY;
            float d2 = dx * dx + dy * dy;
            if (d2 < minDist2) {
                particleVel[i * 2] = obstacleVelX;
                particleVel[i * 2 + 1] = obstacleVelY;
            }
            
            // 墙壁碰撞
            if (x < minX) { x = minX; particleVel[i * 2] = 0; }
            if (x > maxX) { x = maxX; particleVel[i * 2] = 0; }
            if (y < minY) { y = minY; particleVel[i * 2 + 1] = 0; }
            if (y > maxY) { y = maxY; particleVel[i * 2 + 1] = 0; }
            
            particlePos[i * 2] = x;
            particlePos[i * 2 + 1] = y;
        }
    }
    
    void transferVelocities(bool toGrid, float flipRatio) {
        int n = fNumY;
        float h2 = 0.5f * h;
        
        if (toGrid) {
            memcpy(prevU, u, fNumCells * sizeof(float));
            memcpy(prevV, v, fNumCells * sizeof(float));
            
            memset(du, 0, fNumCells * sizeof(float));
            memset(dv, 0, fNumCells * sizeof(float));
            memset(u, 0, fNumCells * sizeof(float));
            memset(v, 0, fNumCells * sizeof(float));
            
            for (int i = 0; i < fNumCells; i++) {
                cellType[i] = (s[i] == 0.0f) ? SOLID_CELL : AIR_CELL;
            }
            
            for (int i = 0; i < numParticles; i++) {
                float x = particlePos[i * 2];
                float y = particlePos[i * 2 + 1];
                int xi = myClamp((int)(x * fInvSpacing), 0, fNumX - 1);
                int yi = myClamp((int)(y * fInvSpacing), 0, fNumY - 1);
                int cellNr = xi * n + yi;
                if (cellType[cellNr] == AIR_CELL) cellType[cellNr] = FLUID_CELL;
            }
        }
        
        // U 场 (水平速度)
        for (int i = 0; i < numParticles; i++) {
            float x = particlePos[i * 2];
            float y = particlePos[i * 2 + 1];
            x = myClamp(x, h, (fNumX - 1) * h);
            y = myClamp(y, h, (fNumY - 1) * h);
            
            float x0f = (x - 0.0f) * fInvSpacing;
            int x0 = (int)x0f;
            float tx = x0f - x0;
            x0 = myMin(x0, fNumX - 2);
            float x1f = x0 + 1;
            int x1 = myMin((int)x1f, fNumX - 2);
            
            float y0f = (y - h2) * fInvSpacing;
            int y0 = (int)y0f;
            float ty = y0f - y0;
            y0 = myMin(y0, fNumY - 2);
            float y1f = y0 + 1;
            int y1 = myMin((int)y1f, fNumY - 2);
            
            float sx = 1.0f - tx;
            float sy = 1.0f - ty;
            
            float d0 = sx * sy, d1 = tx * sy, d2 = tx * ty, d3 = sx * ty;
            int nr0 = x0 * n + y0;
            int nr1 = x1 * n + y0;
            int nr2 = x1 * n + y1;
            int nr3 = x0 * n + y1;
            
            if (toGrid) {
                float pv = particleVel[i * 2];
                u[nr0] += pv * d0; du[nr0] += d0;
                u[nr1] += pv * d1; du[nr1] += d1;
                u[nr2] += pv * d2; du[nr2] += d2;
                u[nr3] += pv * d3; du[nr3] += d3;
            } else {
                float valid0 = (cellType[nr0] != AIR_CELL || cellType[nr0 - n] != AIR_CELL) ? 1.0f : 0.0f;
                float valid1 = (cellType[nr1] != AIR_CELL || cellType[nr1 - n] != AIR_CELL) ? 1.0f : 0.0f;
                float valid2 = (cellType[nr2] != AIR_CELL || cellType[nr2 - n] != AIR_CELL) ? 1.0f : 0.0f;
                float valid3 = (cellType[nr3] != AIR_CELL || cellType[nr3 - n] != AIR_CELL) ? 1.0f : 0.0f;
                
                float d = valid0 * d0 + valid1 * d1 + valid2 * d2 + valid3 * d3;
                if (d > 0.0f) {
                    float picV = (valid0 * d0 * u[nr0] + valid1 * d1 * u[nr1] + 
                                 valid2 * d2 * u[nr2] + valid3 * d3 * u[nr3]) / d;
                    float corr = (valid0 * d0 * (u[nr0] - prevU[nr0]) + 
                                 valid1 * d1 * (u[nr1] - prevU[nr1]) +
                                 valid2 * d2 * (u[nr2] - prevU[nr2]) + 
                                 valid3 * d3 * (u[nr3] - prevU[nr3])) / d;
                    float flipV = particleVel[i * 2] + corr;
                    particleVel[i * 2] = (1.0f - flipRatio) * picV + flipRatio * flipV;
                }
            }
        }
        
        // V 场 (垂直速度)
        for (int i = 0; i < numParticles; i++) {
            float x = particlePos[i * 2];
            float y = particlePos[i * 2 + 1];
            x = myClamp(x, h, (fNumX - 1) * h);
            y = myClamp(y, h, (fNumY - 1) * h);
            
            float x0f = (x - h2) * fInvSpacing;
            int x0 = (int)x0f;
            float tx = x0f - x0;
            x0 = myMin(x0, fNumX - 2);
            float x1f = x0 + 1;
            int x1 = myMin((int)x1f, fNumX - 2);
            
            float y0f = (y - 0.0f) * fInvSpacing;
            int y0 = (int)y0f;
            float ty = y0f - y0;
            y0 = myMin(y0, fNumY - 2);
            float y1f = y0 + 1;
            int y1 = myMin((int)y1f, fNumY - 2);
            
            float sx = 1.0f - tx;
            float sy = 1.0f - ty;
            
            float d0 = sx * sy, d1 = tx * sy, d2 = tx * ty, d3 = sx * ty;
            int nr0 = x0 * n + y0;
            int nr1 = x1 * n + y0;
            int nr2 = x1 * n + y1;
            int nr3 = x0 * n + y1;
            
            if (toGrid) {
                float pv = particleVel[i * 2 + 1];
                v[nr0] += pv * d0; dv[nr0] += d0;
                v[nr1] += pv * d1; dv[nr1] += d1;
                v[nr2] += pv * d2; dv[nr2] += d2;
                v[nr3] += pv * d3; dv[nr3] += d3;
            } else {
                float valid0 = (cellType[nr0] != AIR_CELL || cellType[nr0 - 1] != AIR_CELL) ? 1.0f : 0.0f;
                float valid1 = (cellType[nr1] != AIR_CELL || cellType[nr1 - 1] != AIR_CELL) ? 1.0f : 0.0f;
                float valid2 = (cellType[nr2] != AIR_CELL || cellType[nr2 - 1] != AIR_CELL) ? 1.0f : 0.0f;
                float valid3 = (cellType[nr3] != AIR_CELL || cellType[nr3 - 1] != AIR_CELL) ? 1.0f : 0.0f;
                
                float d = valid0 * d0 + valid1 * d1 + valid2 * d2 + valid3 * d3;
                if (d > 0.0f) {
                    float picV = (valid0 * d0 * v[nr0] + valid1 * d1 * v[nr1] + 
                                 valid2 * d2 * v[nr2] + valid3 * d3 * v[nr3]) / d;
                    float corr = (valid0 * d0 * (v[nr0] - prevV[nr0]) + 
                                 valid1 * d1 * (v[nr1] - prevV[nr1]) +
                                 valid2 * d2 * (v[nr2] - prevV[nr2]) + 
                                 valid3 * d3 * (v[nr3] - prevV[nr3])) / d;
                    float flipV = particleVel[i * 2 + 1] + corr;
                    particleVel[i * 2 + 1] = (1.0f - flipRatio) * picV + flipRatio * flipV;
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
                    bool solid = (cellType[i * n + j] == SOLID_CELL);
                    if (solid || (i > 0 && cellType[(i - 1) * n + j] == SOLID_CELL))
                        u[i * n + j] = prevU[i * n + j];
                    if (solid || (j > 0 && cellType[i * n + j - 1] == SOLID_CELL))
                        v[i * n + j] = prevV[i * n + j];
                }
            }
        }
    }
    
    void solveIncompressibility(int numIters, float dt, float overRelaxation) {
        memset(p, 0, fNumCells * sizeof(float));
        memcpy(prevU, u, fNumCells * sizeof(float));
        memcpy(prevV, v, fNumCells * sizeof(float));
        
        int n = fNumY;
        float cp = DENSITY * h / dt;
        
        for (int iter = 0; iter < numIters; iter++) {
            for (int i = 1; i < fNumX - 1; i++) {
                for (int j = 1; j < fNumY - 1; j++) {
                    int center = i * n + j;
                    if (cellType[center] != FLUID_CELL) continue;
                    
                    int left = (i - 1) * n + j;
                    int right = (i + 1) * n + j;
                    int bottom = i * n + j - 1;
                    int top = i * n + j + 1;
                    
                    float sx0 = s[left], sx1 = s[right];
                    float sy0 = s[bottom], sy1 = s[top];
                    float sumS = sx0 + sx1 + sy0 + sy1;
                    if (sumS == 0.0f) continue;
                    
                    float div = u[right] - u[center] + v[top] - v[center];
                    float pVal = -div / sumS * overRelaxation;
                    
                    p[center] += cp * pVal;
                    u[center] -= sx0 * pVal;
                    u[right] += sx1 * pVal;
                    v[center] -= sy0 * pVal;
                    v[top] += sy1 * pVal;
                }
            }
        }
    }
    
    void updateParticleColors() {
        for (int i = 0; i < numParticles; i++) {
            float vx = particleVel[i * 2];
            float vy = particleVel[i * 2 + 1];
            float speed = sqrt(vx * vx + vy * vy);
            uint8_t gray = (uint8_t)myClamp(speed * 50.0f, 0.0f, 255.0f);
            particleColor[i] = gray;
        }
    }
    
    void simulate(float dt, float gravity, float flipRatio, int numPressureIters, 
                  int numParticleIters, float overRelaxation) {
        integrateParticles(dt, gravity);
        if (separateParticles) pushParticlesApart(numParticleIters);
        handleCollisions();
        transferVelocities(true, 0);
        solveIncompressibility(numPressureIters, dt, overRelaxation);
        transferVelocities(false, flipRatio);
        updateParticleColors();
    }
};

FlipFluid fluid;

// ============== 场景设置 ==============
void setupScene() {
    tankWidth = 3.0f;
    tankHeight = 3.0f * 64.0f / 128.0f;
    
    float res = SIM_RESOLUTION;
    float h_spacing = tankHeight / res;
    
    float relWaterHeight = 0.8f;
    float relWaterWidth = 0.6f;
    
    float r = 0.3f * h_spacing;
    float dx = 2.0f * r;
    float dy = 1.732f * r;
    
    int numX = (int)((relWaterWidth * tankWidth - 2.0f * h_spacing - 2.0f * r) / dx);
    int numY = (int)((relWaterHeight * tankHeight - 2.0f * h_spacing - 2.0f * r) / dy);
    
    numParticles = myMin(numX * numY, MAX_PARTICLES);
    
    Serial.printf("Creating %d particles (%dx%d)\n", numParticles, numX, numY);
    
    fluid.init(DENSITY, tankWidth, tankHeight, h_spacing, MAX_PARTICLES);
    
    int p = 0;
    for (int i = 0; i < numX && p < MAX_PARTICLES; i++) {
        for (int j = 0; j < numY && p < MAX_PARTICLES; j++) {
            particlePos[p * 2] = h_spacing + r + dx * i + ((j % 2) * r);
            particlePos[p * 2 + 1] = h_spacing + r + dy * j;
            particleVel[p * 2] = 0;
            particleVel[p * 2 + 1] = 0;
            particleColor[p] = 128;
            p++;
        }
    }
    numParticles = p;
    
    int n = fNumY;
    for (int i = 0; i < fNumX; i++) {
        for (int j = 0; j < fNumY; j++) {
            bool solid = (i == 0 || i == fNumX - 1 || j == 0);
            s[i * n + j] = solid ? 0.0f : 1.0f;
        }
    }
    
    int screenW = 128;
    int screenH = 64;
    int drawSize = myMin(screenW, screenH);
    
    displayOffsetX = (screenW - drawSize) / 2;
    displayOffsetY = 0;
    displayScale = drawSize / max(tankWidth, tankHeight);
    
    obstacleX = tankWidth * 0.5f;
    obstacleY = tankHeight * 0.5f;
}

// ============== 显示函数 ==============
void drawFluid() {
    u8g2.clearBuffer();
    
    int x0 = displayOffsetX;
    int y0 = displayOffsetY;
    int size = 64;
    
    u8g2.drawFrame(x0, y0, size, size);
    
    for (int i = 0; i < numParticles; i++) {
        float px = particlePos[i * 2];
        float py = particlePos[i * 2 + 1];
        
        int sx = x0 + (int)(px * displayScale);
        int sy = y0 + size - (int)(py * displayScale);
        
        uint8_t gray = particleColor[i];
        
        if (gray > 200) {
            u8g2.drawDisc(sx, sy, 1);
        } else if (gray > 100) {
            u8g2.drawCircle(sx, sy, 1);
        } else {
            u8g2.drawPixel(sx, sy);
        }
    }
    
    int ox = x0 + (int)(obstacleX * displayScale);
    int oy = y0 + size - (int)(obstacleY * displayScale);
    int orad = (int)(obstacleRadius * displayScale);
    u8g2.drawDisc(ox, oy, orad);
    
    u8g2.setFont(u8g2_font_5x7_tr);
    char buf[32];
    snprintf(buf, sizeof(buf), "FPS:%.1f P:%d", currentFps, numParticles);
    u8g2.drawStr(0, 63, buf);
    
    u8g2.sendBuffer();
}

// ============== Arduino 标准函数 ==============
void setup() {
    Serial.begin(500000);
    delay(1000);
    
    Serial.println("FLIP Fluid on ESP32/ESP8266");
    Serial.printf("Resolution: %dx%d\n", SIM_RESOLUTION, SIM_RESOLUTION);
    
    u8g2.begin();
    u8g2.setContrast(255);
    
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(10, 20, "FLIP Fluid");
    u8g2.drawStr(10, 35, "Loading...");
    u8g2.sendBuffer();
    
    setupScene();
    
    Serial.println("Setup complete");
    
    delay(500);
}

void loop() {
    unsigned long startTime = micros();
    
    fluid.simulate(DT, GRAVITY, FLIP_RATIO, PRESSURE_ITERATIONS, 
                   PARTICLE_ITERATIONS, OVER_RELAXATION);
    
    drawFluid();
    
    frameCount++;
    unsigned long now = millis();
    if (now - lastFpsTime >= 1000) {
        currentFps = frameCount * 1000.0f / (now - lastFpsTime);
        frameCount = 0;
        lastFpsTime = now;
        
        Serial.printf("FPS: %.1f, Particles: %d\n", currentFps, numParticles);
    }
}
