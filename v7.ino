/*
  FLIP Fluid Simulation for ESP32/ESP8266 with OLED 128x64
  基于网格的流体显示 - 16x16, 32x32, 或 64x64 像素圆形显示
  可配置重力方向和大小
*/

#include <Arduino.h>
#include <U8g2lib.h>

// ============== 配置区域 ==============
// 选择模拟分辨率: 16, 32, 或 64
#define SIM_RESOLUTION 20


// 选择平台
#define USE_ESP32
//#define USE_ESP8266

// OLED 设置
#ifdef USE_ESP8266
  U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 5, /* data=*/ 4, /* reset=*/ U8X8_PIN_NONE);
#else
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#endif

// ============== 重力配置（可修改） ==============
// 重力方向：角度制，0=向右，90=向上，180=向左，270/ -90=向下
// 也可以输入负数角度，会自动转换
float gravityAngle = 270.0f;    // 默认向下（270度）

// 重力大小倍数：1.0 = 正常地球重力(9.8)，0 = 无重力，2.0 = 2倍重力
float gravityScale = 1.0f;       // 默认1倍重力

// 基础重力加速度（地球重力）
const float BASE_GRAVITY = 9.8f;

// 计算后的重力分量（自动计算，无需手动修改）
float gravityX = 0.0f;
float gravityY = 0.0f;

// ============== 模拟参数 ==============
#if SIM_RESOLUTION == 16
  #define MAX_PARTICLES 200
  #define PRESSURE_ITERATIONS 10
#elif SIM_RESOLUTION == 32
  #define MAX_PARTICLES 800
  #define PRESSURE_ITERATIONS 15
#else // 64
  #define MAX_PARTICLES 800
  #define PRESSURE_ITERATIONS 20
#endif

#define PARTICLE_ITERATIONS 2

const float DT = 0.041f;
const float FLIP_RATIO = 0.9f;
const float OVER_RELAXATION = 1.9f;
const float DENSITY = 1000.0f;

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

float pInvSpacing;
int pNumX, pNumY, pNumCells;
int *numCellParticles;
int *firstCellParticle;
int *cellParticleIds;

bool compensateDrift = true;
bool separateParticles = true;

float simWidth, simHeight;
float tankWidth, tankHeight;

// 圆形区域参数
float circleCenterX, circleCenterY;
float circleRadius;

// 显示相关
int displayOffsetX, displayOffsetY;
int cellPixelSize;

unsigned long lastFpsTime = 0;
int frameCount = 0;
float currentFps = 0;

// 串口输入缓冲区
String inputBuffer = "";

// ============== 工具函数 ==============
float myClamp(float x, float minVal, float maxVal) {
    if (x < minVal) return minVal;
    if (x > maxVal) return maxVal;
    return x;
}

float myMin(float a, float b) { return (a < b) ? a : b; }
int myMin(int a, int b) { return (a < b) ? a : b; }

float myMax(float a, float b) { return (a > b) ? a : b; }
int myMax(int a, int b) { return (a > b) ? a : b; }

// 将角度转换为弧度
float degToRad(float deg) {
    return deg * PI / 180.0f;
}

// 更新重力向量
void updateGravity() {
    // 标准化角度到 0-360
    float normalizedAngle = fmod(gravityAngle, 360.0f);
    if (normalizedAngle < 0) normalizedAngle += 360.0f;
    
    // 转换角度为弧度（标准数学坐标：0度=右，90=上，180=左，270=下）
    // 但在屏幕坐标中Y轴是反的，所以调整一下
    float rad = degToRad(normalizedAngle);
    
    // 计算重力分量
    // 注意：在物理模拟中，Y轴向上为正，所以向下的重力是负值
    // 角度270（向下）-> cos(270)=0, sin(270)=-1 -> (0, -1) * -9.8 = (0, -9.8)
    // 实际上我们需要的是加速度方向，所以：
    // 0度（右）: (9.8, 0)
    // 90度（上）: (0, 9.8)  
    // 180度（左）: (-9.8, 0)
    // 270度（下）: (0, -9.8)
    
    gravityX = cos(rad) * BASE_GRAVITY * gravityScale;
    gravityY = sin(rad) * BASE_GRAVITY * gravityScale;
    
    // 因为屏幕Y轴向下为正，所以反转Y分量使物理正确
    gravityY = -gravityY;
    
    // 调试输出
    Serial.print("Gravity updated: Angle=");
    Serial.print(normalizedAngle);
    Serial.print("° Scale=");
    Serial.print(gravityScale);
    Serial.print(" -> (");
    Serial.print(gravityX);
    Serial.print(", ");
    Serial.print(gravityY);
    Serial.println(")");
}

// 处理串口命令
void processSerialCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;
    
    cmd.toLowerCase();
    
    // 命令格式: "angle 270" 或 "a 270"
    if (cmd.startsWith("angle ") || cmd.startsWith("a ")) {
        int spaceIndex = cmd.indexOf(' ');
        if (spaceIndex != -1) {
            String valStr = cmd.substring(spaceIndex + 1);
            float newAngle = valStr.toFloat();
            gravityAngle = newAngle;
            updateGravity();
            Serial.print("Angle set to: ");
            Serial.print(gravityAngle);
            Serial.println(" degrees");
        }
    }
    // 命令格式: "scale 1.5" 或 "s 1.5" 或 "g 1.5"
    else if (cmd.startsWith("scale ") || cmd.startsWith("s ") || cmd.startsWith("g ")) {
        int spaceIndex = cmd.indexOf(' ');
        if (spaceIndex != -1) {
            String valStr = cmd.substring(spaceIndex + 1);
            float newScale = valStr.toFloat();
            gravityScale = myClamp(newScale, 0.0f, 10.0f);  // 限制最大10倍
            updateGravity();
            Serial.print("Gravity scale set to: ");
            Serial.print(gravityScale);
            Serial.println("x");
        }
    }
    // 快捷命令: "down", "up", "left", "right"
    else if (cmd == "down") {
        gravityAngle = 270;
        updateGravity();
        Serial.println("Gravity: DOWN");
    }
    else if (cmd == "up") {
        gravityAngle = 90;
        updateGravity();
        Serial.println("Gravity: UP");
    }
    else if (cmd == "left") {
        gravityAngle = 180;
        updateGravity();
        Serial.println("Gravity: LEFT");
    }
    else if (cmd == "right") {
        gravityAngle = 0;
        updateGravity();
        Serial.println("Gravity: RIGHT");
    }
    else if (cmd == "zero" || cmd == "0") {
        gravityScale = 0;
        updateGravity();
        Serial.println("Gravity: ZERO");
    }
    // 显示帮助
    else if (cmd == "help" || cmd == "h" || cmd == "?") {
        Serial.println("=== Commands ===");
        Serial.println("angle <deg>  or a <deg>  - Set angle (0=right, 90=up, 180=left, 270=down)");
        Serial.println("scale <val>  or s <val>  - Set gravity scale (0-10, 1.0=normal)");
        Serial.println("down/up/left/right       - Quick direction set");
        Serial.println("zero                     - Zero gravity");
        Serial.println("help                     - Show this help");
        Serial.println("================");
    }
    else {
        Serial.println("Unknown command. Type 'help' for commands.");
    }
}

// ============== FLIP 类 ==============
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
        
        circleCenterX = width / 2.0f;
        circleCenterY = height / 2.0f;
        circleRadius = (myMin(width, height) / 2.0f) - h;
    }
    
    void integrateParticles(float dt, float gx, float gy) {
        for (int i = 0; i < numParticles; i++) {
            particleVel[i * 2] += dt * gx;      // X方向重力
            particleVel[i * 2 + 1] += dt * gy;  // Y方向重力
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
        
        for (int i = 0; i < numParticles; i++) {
            float x = particlePos[i * 2];
            float y = particlePos[i * 2 + 1];
            
            float dx = x - circleCenterX;
            float dy = y - circleCenterY;
            float dist = sqrt(dx * dx + dy * dy);
            
            if (dist > circleRadius - r) {
                float scale = (circleRadius - r) / dist;
                float newX = circleCenterX + dx * scale;
                float newY = circleCenterY + dy * scale;
                
                particlePos[i * 2] = newX;
                particlePos[i * 2 + 1] = newY;
                
                float nx = dx / dist;
                float ny = dy / dist;
                float vn = particleVel[i * 2] * nx + particleVel[i * 2 + 1] * ny;
                particleVel[i * 2] -= vn * nx;
                particleVel[i * 2 + 1] -= vn * ny;
            }
        }
    }
    
    void updateParticleDensity() {
        int n = fNumY;
        float h1 = fInvSpacing;
        float h2 = 0.5f * h;
        
        float *d = particleDensity;
        for (int i = 0; i < fNumCells; i++) d[i] = 0.0f;
        
        for (int i = 0; i < numParticles; i++) {
            float x = particlePos[i * 2];
            float y = particlePos[i * 2 + 1];
            
            x = myClamp(x, h, (fNumX - 1) * h);
            y = myClamp(y, h, (fNumY - 1) * h);
            
            int x0 = (int)((x - h2) * h1);
            float tx = ((x - h2) - x0 * h) * h1;
            int x1 = myMin(x0 + 1, fNumX - 2);
            
            int y0 = (int)((y - h2) * h1);
            float ty = ((y - h2) - y0 * h) * h1;
            int y1 = myMin(y0 + 1, fNumY - 2);
            
            float sx = 1.0f - tx;
            float sy = 1.0f - ty;
            
            if (x0 < fNumX && y0 < fNumY) d[x0 * n + y0] += sx * sy;
            if (x1 < fNumX && y0 < fNumY) d[x1 * n + y0] += tx * sy;
            if (x1 < fNumX && y1 < fNumY) d[x1 * n + y1] += tx * ty;
            if (x0 < fNumX && y1 < fNumY) d[x0 * n + y1] += sx * ty;
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
            
            for (int i = 0; i < fNumX; i++) {
                for (int j = 0; j < fNumY; j++) {
                    float cellX = i * h;
                    float cellY = j * h;
                    float dx = cellX - circleCenterX;
                    float dy = cellY - circleCenterY;
                    float dist = sqrt(dx * dx + dy * dy);
                    
                    bool solid = (dist >= circleRadius - h);
                    s[i * n + j] = solid ? 0.0f : 1.0f;
                    cellType[i * n + j] = solid ? SOLID_CELL : AIR_CELL;
                }
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
        
        // U 场
        for (int i = 0; i < numParticles; i++) {
            float x = particlePos[i * 2];
            float y = particlePos[i * 2 + 1];
            x = myClamp(x, h, (fNumX - 1) * h);
            y = myClamp(y, h, (fNumY - 1) * h);
            
            float x0f = (x - 0.0f) * fInvSpacing;
            int x0 = (int)x0f;
            float tx = x0f - x0;
            x0 = myMin(x0, fNumX - 2);
            int x1 = myMin(x0 + 1, fNumX - 2);
            
            float y0f = (y - h2) * fInvSpacing;
            int y0 = (int)y0f;
            float ty = y0f - y0;
            y0 = myMin(y0, fNumY - 2);
            int y1 = myMin(y0 + 1, fNumY - 2);
            
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
        
        // V 场
        for (int i = 0; i < numParticles; i++) {
            float x = particlePos[i * 2];
            float y = particlePos[i * 2 + 1];
            x = myClamp(x, h, (fNumX - 1) * h);
            y = myClamp(y, h, (fNumY - 1) * h);
            
            float x0f = (x - h2) * fInvSpacing;
            int x0 = (int)x0f;
            float tx = x0f - x0;
            x0 = myMin(x0, fNumX - 2);
            int x1 = myMin(x0 + 1, fNumX - 2);
            
            float y0f = (y - 0.0f) * fInvSpacing;
            int y0 = (int)y0f;
            float ty = y0f - y0;
            y0 = myMin(y0, fNumY - 2);
            int y1 = myMin(y0 + 1, fNumY - 2);
            
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
    
    void simulate(float dt, float gx, float gy, float flipRatio, int numPressureIters, 
                  int numParticleIters, float overRelaxation) {
        integrateParticles(dt, gx, gy);  // 传入重力分量
        if (separateParticles) pushParticlesApart(numParticleIters);
        handleCollisions();
        transferVelocities(true, 0);
        updateParticleDensity();
        solveIncompressibility(numPressureIters, dt, overRelaxation);
        transferVelocities(false, flipRatio);
    }
};

FlipFluid fluid;

// 预计算的圆形掩码
bool circleMask[SIM_RESOLUTION][SIM_RESOLUTION];

// ============== 场景设置 ==============
void setupScene() {
    tankWidth = 3.0f;
    tankHeight = 3.0f;
    
    float spacing = tankWidth / SIM_RESOLUTION;
    
    fluid.init(DENSITY, tankWidth, tankHeight, spacing, MAX_PARTICLES);
    
    // 预计算圆形掩码
    float center = (SIM_RESOLUTION - 1) / 2.0f;
    float radius = SIM_RESOLUTION / 2.0f - 0.5f;
    float radiusSq = radius * radius;
    
    for (int i = 0; i < SIM_RESOLUTION; i++) {
        for (int j = 0; j < SIM_RESOLUTION; j++) {
            float dx = i - center;
            float dy = j - center;
            circleMask[i][j] = (dx * dx + dy * dy) <= radiusSq;
        }
    }
    
    // 在圆形区域内创建水块
    float r = 0.3f * h;
    float dx = 2.0f * r;
    float dy = 1.732f * r;
    
    numParticles = 0;
    
    for (float y = circleCenterY - circleRadius + r; 
         y < circleCenterY && numParticles < MAX_PARTICLES; y += dy) {
        for (float x = circleCenterX - circleRadius + r; 
             x < circleCenterX + circleRadius && numParticles < MAX_PARTICLES; x += dx) {
            
            float distX = x - circleCenterX;
            float distY = y - circleCenterY;
            float distSq = distX * distX + distY * distY;
            
            if (distSq <= (circleRadius - r) * (circleRadius - r)) {
                float offsetX = ((int)((y - (circleCenterY - circleRadius)) / dy) % 2) * r;
                
                float finalX = x + offsetX;
                float finalY = y;
                
                float finalDistX = finalX - circleCenterX;
                float finalDistY = finalY - circleCenterY;
                if (finalDistX * finalDistX + finalDistY * finalDistY <= 
                    (circleRadius - r) * (circleRadius - r)) {
                    
                    particlePos[numParticles * 2] = finalX;
                    particlePos[numParticles * 2 + 1] = finalY;
                    particleVel[numParticles * 2] = 0;
                    particleVel[numParticles * 2 + 1] = 0;
                    numParticles++;
                }
            }
        }
    }
    
    cellPixelSize = 64 / SIM_RESOLUTION;
    displayOffsetX = (128 - 64) / 2;
    displayOffsetY = 0;
}

// ============== 圆形显示 ==============
void drawFluid() {
    u8g2.clearBuffer();
    
    int displaySize = SIM_RESOLUTION * cellPixelSize;
    int circlePixelRadius = displaySize / 2;
    int circlePixelCenterX = displayOffsetX + circlePixelRadius;
    int circlePixelCenterY = displayOffsetY + circlePixelRadius;
    
    // 绘制圆形边框
    u8g2.drawCircle(circlePixelCenterX, circlePixelCenterY, circlePixelRadius);
    
    // 绘制重力方向指示器（小箭头）
    float arrowRad = degToRad(gravityAngle);
    int arrowLen = 8;
    int arrowX = circlePixelCenterX + cos(arrowRad) * arrowLen;
    int arrowY = circlePixelCenterY - sin(arrowRad) * arrowLen;  // 注意Y轴翻转
    
    u8g2.drawLine(circlePixelCenterX, circlePixelCenterY, arrowX, arrowY);
    // 箭头头部
    u8g2.drawCircle(arrowX, arrowY, 1);
    
    // 绘制流体单元格
    for (int i = 0; i < SIM_RESOLUTION; i++) {
        for (int j = 0; j < SIM_RESOLUTION; j++) {
            if (!circleMask[i][j]) continue;
            
            int simI = (int)((float)i / SIM_RESOLUTION * (fNumX - 2)) + 1;
            int simJ = (int)((float)j / SIM_RESOLUTION * (fNumY - 2)) + 1;
            
            simI = myClamp(simI, 0, fNumX - 1);
            simJ = myClamp(simJ, 0, fNumY - 1);
            
            int cellIdx = simI * fNumY + simJ;
            bool isFluid = (cellType[cellIdx] == FLUID_CELL);
            
            if (isFluid) {
                int px = displayOffsetX + i * cellPixelSize;
                int py = displayOffsetY + (SIM_RESOLUTION - 1 - j) * cellPixelSize;
                
                if (cellPixelSize == 1) {
                    u8g2.drawPixel(px, py);
                } else {
                    u8g2.drawBox(px, py, cellPixelSize, cellPixelSize);
                }
            }
        }
    }
    
    // 显示信息
    u8g2.setFont(u8g2_font_5x7_tr);
    char buf[32];
    // 显示角度和倍数
    snprintf(buf, sizeof(buf), "%d%% %.0f", (int)(gravityScale * 100), gravityAngle);
    u8g2.drawStr(0, 63, buf);
    
    u8g2.sendBuffer();
}

// ============== Arduino 标准函数 ==============
void setup() {
    Serial.begin(2000000);
    delay(3000);
    
    // 初始化重力
    updateGravity();
    
    u8g2.begin();
    u8g2.setBusClock(1000000);
    u8g2.setContrast(255);
    
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(10, 20, "FLIP Fluid");
    u8g2.drawStr(10, 35, "Circle+Gravity");
    u8g2.drawStr(10, 50, "Type 'help'");
    u8g2.sendBuffer();
    
    setupScene();
    
    Serial.println("Setup complete. Type 'help' for commands.");
    delay(500);
}

void loop() {
    // 处理串口输入
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            processSerialCommand(inputBuffer);
            inputBuffer = "";
        } else {
            inputBuffer += c;
        }
    }
    
    // 使用当前重力分量进行模拟
    fluid.simulate(DT, gravityX, gravityY, FLIP_RATIO, PRESSURE_ITERATIONS, 
                   PARTICLE_ITERATIONS, OVER_RELAXATION);
    
    drawFluid();
    
    frameCount++;
    unsigned long now = millis();
    if (now - lastFpsTime >= 1000) {
        currentFps = frameCount * 1000.0f / (now - lastFpsTime);
        frameCount = 0;
        lastFpsTime = now;
        
        Serial.print("FPS:");
        Serial.print(currentFps);
        Serial.print(" G:");
        Serial.print(gravityScale);
        Serial.print(" A:");
        Serial.print(gravityAngle);
        Serial.print(" gx:");
        Serial.print(gravityX);
        Serial.print(" gy:");
        Serial.print(gravityY);
        Serial.print("\n");
    }
}
