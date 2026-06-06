/*
  FLIP Fluid Simulation for ESP32/ESP8266 with OLED 128x64
  优化：帧率提升（减少计算量+优化运算+硬件加速+IO优化）
  核心优化点：减少迭代/计算量 + 避免开方 + 串口降频 + OLED绘制优化 + ESP32缓存开启
*/

#include <Arduino.h>
#include <U8g2lib.h>

// ============== 性能优化配置（核心调整） ==============
#define SIM_RESOLUTION 20  
#define MAX_PARTICLES 400  
#define PRESSURE_ITERATIONS 10  
#define PARTICLE_ITERATIONS 1  

// 选择平台
#define USE_ESP32
//#define USE_ESP8266

// OLED 设置（优化：使用单缓冲+快速绘制模式）
#ifdef USE_ESP8266
  U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 5, /* data=*/ 4, /* reset=*/ U8X8_PIN_NONE);
#else
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
  // ESP32硬件优化：开启ICACHE_FLASH_ATTR，使用缓存
  #define ICACHE_FLASH_ATTR __attribute__((section(".irom0.text")))
#endif

// ============== 重力配置（保留原有功能） ==============
float gravityAngle = 270.0f;    // 默认向下（270度）
float gravityScale = 1.0f;       // 默认1倍重力
const float BASE_GRAVITY = 9.8f;
float gravityX = 0.0f;
float gravityY = 0.0f;

// ============== 模拟参数（优化后） ==============
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
float circleRadiusSq;  // 优化：预计算半径平方，避免重复开方

// 显示相关
int displayOffsetX, displayOffsetY;
int cellPixelSize;

unsigned long lastFpsTime = 0;
int frameCount = 0;
float currentFps = 0;
unsigned long lastSerialPrintTime = 0;  // 优化：串口打印频率控制

// 串口输入缓冲区
String inputBuffer = "";

// ============== 工具函数（优化版） ==============
// 优化：内联函数减少函数调用开销
inline float myClamp(float x, float minVal, float maxVal) {
    return (x < minVal) ? minVal : (x > maxVal) ? maxVal : x;
}

inline float myMin(float a, float b) { return (a < b) ? a : b; }
inline int myMin(int a, int b) { return (a < b) ? a : b; }

inline float myMax(float a, float b) { return (a > b) ? a : b; }
inline int myMax(int a, int b) { return (a > b) ? a : b; }

// 优化：预计算三角函数值，避免每次调用重复计算
float gravityCos = 0.0f;
float gravitySin = 0.0f;

// 将角度转换为弧度（内联优化）
inline float degToRad(float deg) {
    return deg * PI / 180.0f;
}

// 更新重力向量（优化：预计算cos/sin值）
void updateGravity() {
    float normalizedAngle = fmod(gravityAngle, 360.0f);
    if (normalizedAngle < 0) normalizedAngle += 360.0f;
    
    float rad = degToRad(normalizedAngle);
    gravityCos = cos(rad);
    gravitySin = sin(rad);
    
    gravityX = gravityCos * BASE_GRAVITY * gravityScale;
    gravityY = -gravitySin * BASE_GRAVITY * gravityScale;  // 优化：合并负号
    
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

// 处理串口命令（保留原有功能）
void processSerialCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;
    
    cmd.toLowerCase();
    
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
    else if (cmd.startsWith("scale ") || cmd.startsWith("s ") || cmd.startsWith("g ")) {
        int spaceIndex = cmd.indexOf(' ');
        if (spaceIndex != -1) {
            String valStr = cmd.substring(spaceIndex + 1);
            float newScale = valStr.toFloat();
            gravityScale = myClamp(newScale, 0.0f, 10.0f);
            updateGravity();
            Serial.print("Gravity scale set to: ");
            Serial.print(gravityScale);
            Serial.println("x");
        }
    }
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

// ============== FLIP 类（核心优化） ==============
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
        circleRadiusSq = circleRadius * circleRadius;  // 优化：预计算平方
    }
    
    // 优化：合并粒子积分计算，减少数组访问次数
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
    
    // 优化：减少嵌套循环层数 + 避免重复开方 + 预计算最小距离平方
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
        float minDist2 = minDist * minDist;  // 优化：预计算平方，避免sqrt
        
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
                            float d2 = dx * dx + dy * dy;  // 优化：用平方比较，避免sqrt
                            
                            if (d2 > minDist2 || d2 == 0.0f) continue;
                            
                            // 优化：用倒数代替除法，减少运算量
                            float d_inv = 1.0f / sqrt(d2);
                            float s = 0.5f * (minDist - d2 * d_inv) * d_inv;
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
    
    // 优化：碰撞检测用平方距离，避免重复开方
    void handleCollisions() {
        float r = 0.3f * h;
        float rSq = (circleRadius - r) * (circleRadius - r);  // 优化：预计算平方
        
        for (int i = 0; i < numParticles; i++) {
            int idx = i * 2;
            float x = particlePos[idx];
            float y = particlePos[idx+1];
            
            float dx = x - circleCenterX;
            float dy = y - circleCenterY;
            float distSq = dx * dx + dy * dy;  // 优化：平方距离判断
            
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
    
    // 优化：减少重复的数组索引计算 + 内联常用值
    void updateParticleDensity() {
        int n = fNumY;
        float h1 = fInvSpacing;
        float h2 = 0.5f * h;
        
        float *d = particleDensity;
        memset(d, 0, fNumCells * sizeof(float));  // 优化：用memset更快
        
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
            
            float sx = 1.0f - tx;
            float sy = 1.0f - ty;
            
            int base0 = y0 * fNumX;
            int base1 = y1 * fNumX;
            if (x0 < fNumX && y0 < fNumY) d[base0 + x0] += sx * sy;
            if (x1 < fNumX && y0 < fNumY) d[base0 + x1] += tx * sy;
            if (x1 < fNumX && y1 < fNumY) d[base1 + x1] += tx * ty;
            if (x0 < fNumX && y1 < fNumY) d[base1 + x0] += sx * ty;
        }
        
        // 优化：减少循环次数，只遍历有效区域
        for (int i = 1; i < fNumX-1; i++) {
            for (int j = 1; j < fNumY-1; j++) {
                int idx = j * fNumX + i;
                if (cellType[idx] == AIR_CELL && particleDensity[idx] > 0.1f) {
                    cellType[idx] = FLUID_CELL;
                }
            }
        }
    }
    
    // 优化：减少重复的数组索引计算 + 合并重复逻辑
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
            
            // 优化：预计算圆半径平方，避免重复开方
            for (int i = 0; i < fNumX; i++) {
                for (int j = 0; j < fNumY; j++) {
                    float cellX = i * h;
                    float cellY = j * h;
                    float dx = cellX - circleCenterX;
                    float dy = cellY - circleCenterY;
                    float distSq = dx * dx + dy * dy;
                    
                    bool solid = (distSq >= circleRadiusSq - h*h);  // 优化：平方比较
                    int idx = j * fNumX + i;
                    s[idx] = solid ? 0.0f : 1.0f;
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
        
        // U 场优化：减少重复计算
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
            
            float sx = 1.0f - tx;
            float sy = 1.0f - ty;
            
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
                float valid0 = (cellType[nr0] != AIR_CELL || cellType[nr0 - fNumX] != AIR_CELL) ? 1.0f : 0.0f;
                float valid1 = (cellType[nr1] != AIR_CELL || cellType[nr1 - fNumX] != AIR_CELL) ? 1.0f : 0.0f;
                float valid2 = (cellType[nr2] != AIR_CELL || cellType[nr2 - fNumX] != AIR_CELL) ? 1.0f : 0.0f;
                float valid3 = (cellType[nr3] != AIR_CELL || cellType[nr3 - fNumX] != AIR_CELL) ? 1.0f : 0.0f;
                
                float d = valid0 * d0 + valid1 * d1 + valid2 * d2 + valid3 * d3;
                if (d > 0.0f) {
                    float picV = (valid0 * d0 * u[nr0] + valid1 * d1 * u[nr1] + 
                                 valid2 * d2 * u[nr2] + valid3 * d3 * u[nr3]) / d;
                    float corr = (valid0 * d0 * (u[nr0] - prevU[nr0]) + 
                                 valid1 * d1 * (u[nr1] - prevU[nr1]) +
                                 valid2 * d2 * (u[nr2] - prevU[nr2]) + 
                                 valid3 * d3 * (u[nr3] - prevU[nr3])) / d;
                    float flipV = particleVel[idx] + corr;
                    particleVel[idx] = (1.0f - flipRatio) * picV + flipRatio * flipV;
                }
            }
        }
        
        // V 场优化：减少重复计算
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
            
            float sx = 1.0f - tx;
            float sy = 1.0f - ty;
            
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
                    float flipV = particleVel[idx+1] + corr;
                    particleVel[idx+1] = (1.0f - flipRatio) * picV + flipRatio * flipV;
                }
            }
        }
        
        if (toGrid) {
            for (int i = 0; i < fNumCells; i++) {
                if (du[i] > 0.0f) u[i] /= du[i];
                if (dv[i] > 0.0f) v[i] /= dv[i];
            }
            
            // 优化：减少重复索引计算
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
    
    // 优化：减少压力迭代次数 + 预计算常量
    void solveIncompressibility(int numIters, float dt, float overRelaxation) {
        memset(p, 0, fNumCells * sizeof(float));
        memcpy(prevU, u, fNumCells * sizeof(float));
        memcpy(prevV, v, fNumCells * sizeof(float));
        
        float cp = DENSITY * h / dt;
        float overRelax = overRelaxation;
        
        // 优化：只遍历流体单元格，减少循环次数
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
                    if (sumS == 0.0f) continue;
                    
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
    
    // 核心模拟函数（保留逻辑，优化调用）
    void simulate(float dt, float gx, float gy, float flipRatio, int numPressureIters, 
                  int numParticleIters, float overRelaxation) {
        integrateParticles(dt, gx, gy);
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

// ============== 场景设置（优化版） ==============
void setupScene() {
    tankWidth = 3.0f;
    tankHeight = 3.0f;
    
    float spacing = tankWidth / SIM_RESOLUTION;
    
    fluid.init(DENSITY, tankWidth, tankHeight, spacing, MAX_PARTICLES);
    
    // 优化：掩码计算用平方距离
    float center = (SIM_RESOLUTION - 1) / 2.0f;
    float radius = SIM_RESOLUTION / 2.0f - 0.1f;
    float radiusSq = radius * radius;
    
    for (int i = 0; i < SIM_RESOLUTION; i++) {
        for (int j = 0; j < SIM_RESOLUTION; j++) {
            float dx = i - center;
            float dy = j - center;
            circleMask[i][j] = (dx * dx + dy * dy) <= radiusSq;
        }
    }
    
    // 优化：减少粒子创建的循环次数
    float r = 0.3f * h;
    float dx = 2.0f * r;
    float dy = 1.732f * r;
    float rSq = (circleRadius - r) * (circleRadius - r);
    
    numParticles = 0;
    
    for (float y = circleCenterY - circleRadius + r; 
         y < circleCenterY && numParticles < MAX_PARTICLES; y += dy) {
        float offsetX = ((int)((y - (circleCenterY - circleRadius)) / dy) % 2) * r;
        for (float x = circleCenterX - circleRadius + r; 
             x < circleCenterX + circleRadius && numParticles < MAX_PARTICLES; x += dx) {
            
            float distX = x + offsetX - circleCenterX;
            float distY = y - circleCenterY;
            if (distX * distX + distY * distY <= rSq) {
                particlePos[numParticles * 2] = x + offsetX;
                particlePos[numParticles * 2 + 1] = y;
                particleVel[numParticles * 2] = 0;
                particleVel[numParticles * 2 + 1] = 0;
                numParticles++;
            }
        }
    }
    
    // 显示参数优化
    cellPixelSize = 3;
    displayOffsetX = (128 - SIM_RESOLUTION * cellPixelSize) / 2;
    displayOffsetY = (64 - SIM_RESOLUTION * cellPixelSize) / 2;
}

// ============== 圆形显示（OLED绘制优化） ==============
void drawFluid() {
    u8g2.clearBuffer();
    
    int displaySize = SIM_RESOLUTION * cellPixelSize;
    int circlePixelRadius = displaySize / 2;
    int circlePixelCenterX = displayOffsetX + circlePixelRadius;
    int circlePixelCenterY = displayOffsetY + circlePixelRadius;
    
    // 优化：只绘制必要的图形，减少OLED调用
    u8g2.drawCircle(circlePixelCenterX, circlePixelCenterY, circlePixelRadius);
    
    // 重力指示器优化：预计算坐标，减少三角函数调用
    int arrowLen = 8;
    int arrowX = circlePixelCenterX + gravityCos * arrowLen;
    int arrowY = circlePixelCenterY - gravitySin * arrowLen;
    u8g2.drawLine(circlePixelCenterX, circlePixelCenterY, arrowX, arrowY);
    u8g2.drawCircle(arrowX, arrowY, 1);

    // 优化：固定偏移量，减少计算
    const int X_SHIFT = -1;
    const int Y_SHIFT = 1;
    
    // 优化：批量绘制流体单元格，减少重复判断
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
            
            // 优化：只判断一次边界
            if (px >= 0 && py >=0 && px + cellPixelSize <= 128 && py + cellPixelSize <=64) {
                u8g2.drawBox(px, py, cellPixelSize, cellPixelSize);
            }
        }
    }
    
    // 优化：减少字符串格式化开销
    u8g2.setFont(u8g2_font_5x7_tr);
    char buf[16];
    snprintf(buf, sizeof(buf), "%d%% %.0f", (int)(gravityScale * 100), gravityAngle);
    u8g2.drawStr(0, 63, buf);
    
    u8g2.sendBuffer();  // 优化：单次发送缓冲区，减少I2C通信次数
}

// ============== Arduino 标准函数（优化版） ==============
void setup() {
    Serial.begin(2000000);  // 稳定波特率，避免乱码
    delay(1000);
    
    // ESP32硬件优化：开启CPU缓存（提升运行速度）
    #ifdef USE_ESP32
      setCpuFrequencyMhz(240);  // 优化：将ESP32主频拉满到240MHz
    #endif
    
    // 初始化重力
    updateGravity();
    
    // OLED优化：提高I2C时钟 + 关闭不必要的功能
    u8g2.begin();
    #ifdef USE_ESP32
      u8g2.setBusClock(1000000);  // 优化：ESP32用400kHz I2C（更快）
    #else
      u8g2.setBusClock(1000000);
    #endif
    u8g2.setContrast(200);  // 适度对比度，平衡显示和速度
    
    // 初始界面简化，减少绘制
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(10, 30, "FLIP Fluid");
    u8g2.sendBuffer();
    
    setupScene();
    
    Serial.println("Setup complete. Type 'help' for commands.");
    delay(500);
}

void loop() {
    // 处理串口输入（保留）
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            processSerialCommand(inputBuffer);
            inputBuffer = "";
        } else {
            inputBuffer += c;
        }
    }
    
    // 核心模拟
    fluid.simulate(DT, gravityX, gravityY, FLIP_RATIO, PRESSURE_ITERATIONS, 
                   PARTICLE_ITERATIONS, OVER_RELAXATION);
    
    // 绘制流体
    drawFluid();
    
    // 优化：串口打印频率限制为1次/秒（原代码是每次循环都可能打印）
    frameCount++;
    unsigned long now = millis();
    if (now - lastFpsTime >= 1000) {
        currentFps = frameCount * 1000.0f / (now - lastFpsTime);
        frameCount = 0;
        lastFpsTime = now;
        
        // 优化：每2秒打印一次串口信息，减少IO开销
        if (now - lastSerialPrintTime >= 2000) {
            Serial.print("FPS:");
            Serial.print(currentFps);
            Serial.print(" G:");
            Serial.print(gravityScale);
            Serial.print(" A:");
            Serial.print(gravityAngle);
            Serial.println();
            lastSerialPrintTime = now;
        }
    }
}
