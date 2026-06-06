#include <Arduino.h>
#include <U8g2lib.h>

// ============== 配置区域 ==============
// 选择模拟分辨率: 16, 32, 或 64
#define SIM_RESOLUTION 16

// 选择平台
#define USE_ESP32
//#define USE_ESP8266

// OLED 设置
#ifdef USE_ESP8266
  U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 5, /* data=*/ 4, /* reset=*/ U8X8_PIN_NONE);
#else
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#endif

// ============== 重力配置 ==============
float gravityAngle = 270.0f;
float gravityScale = 1.0f;
const float BASE_GRAVITY = 9.8f;
float gravityX = 0.0f;
float gravityY = 0.0f;

// ============== 模拟参数 ==============
#if SIM_RESOLUTION == 16
  #define MAX_PARTICLES 800
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

// 圆形区域参数（物理模拟用）
float circleCenterX, circleCenterY;
float circleRadius;

// 显示相关
int displayOffsetX, displayOffsetY;
int cellPixelSize;

unsigned long lastFpsTime = 0;
int frameCount = 0;
float currentFps = 0;

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

float degToRad(float deg) {
    return deg * PI / 180.0f;
}

void updateGravity() {
    float normalizedAngle = fmod(gravityAngle, 360.0f);
    if (normalizedAngle < 0) normalizedAngle += 360.0f;
    
    float rad = degToRad(normalizedAngle);
    
    gravityX = cos(rad) * BASE_GRAVITY * gravityScale;
    gravityY = -sin(rad) * BASE_GRAVITY * gravityScale;
    
    Serial.print("Gravity: A=");
    Serial.print(normalizedAngle);
    Serial.print(" S=");
    Serial.print(gravityScale);
    Serial.print(" (");
    Serial.print(gravityX);
    Serial.print(", ");
    Serial.print(gravityY);
    Serial.println(")");
}

void processSerialCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;
    
    String cmdLower = cmd;
    cmdLower.toLowerCase();
    
    if (cmdLower.startsWith("angle ") || cmdLower.startsWith("a ")) {
        int spaceIndex = cmd.indexOf(' ');
        if (spaceIndex != -1) {
            String valStr = cmd.substring(spaceIndex + 1);
            gravityAngle = valStr.toFloat();
            updateGravity();
        }
    }
    else if (cmdLower.startsWith("scale ") || cmdLower.startsWith("s ") || cmdLower.startsWith("g ")) {
        int spaceIndex = cmd.indexOf(' ');
        if (spaceIndex != -1) {
            String valStr = cmd.substring(spaceIndex + 1);
            gravityScale = myClamp(valStr.toFloat(), 0.0f, 10.0f);
            updateGravity();
        }
    }
    else if (cmdLower == "down") { gravityAngle = 270; updateGravity(); }
    else if (cmdLower == "up") { gravityAngle = 90; updateGravity(); }
    else if (cmdLower == "left") { gravityAngle = 180; updateGravity(); }
    else if (cmdLower == "right") { gravityAngle = 0; updateGravity(); }
    else if (cmdLower == "zero" || cmdLower == "0") { gravityScale = 0; updateGravity(); }
    else if (cmdLower == "help" || cmdLower == "h" || cmdLower == "?") {
        Serial.println("=== Commands ===");
        Serial.println("angle <deg>/a <deg> - Set angle (0=right, 90=up, 180=left, 270=down)");
        Serial.println("scale <val>/s <val> - Set gravity scale (0-10)");
        Serial.println("down/up/left/right   - Quick direction");
        Serial.println("zero                 - Zero gravity");
        Serial.println("===================");
    }
}

// ============== 正八边形渲染工具 ==============
// 计算正八边形的包含关系
// 对于SIM_RESOLUTION网格，正八边形的几何定义：
// 水平/垂直边中点距离中心: a = SIM_RESOLUTION/2
// 对角边距离: b = a - 5 (斜边偏移)
// 或者反过来: 如果水平边半长是4，那么对角顶点到中心距离需要调整

// 正八边形参数（基于显示网格坐标 0-SIM_RESOLUTION-1）
struct OctagonParams {
    float center;      // 中心坐标
    float halfSize;    // 半边长（水平/垂直边到中心的距离）
    float cornerCut;   // 角被切掉的距离
    float slope;       // 斜边斜率 (应该是1.0，45度)
};

OctagonParams octParams;

void initOctagonParams() {
    float size = SIM_RESOLUTION;
    octParams.center = (size - 1) / 2.0f;
    octParams.halfSize = SIM_RESOLUTION / 2.0f - 0.5f;  // 7.5 for 16
    octParams.cornerCut = 4.0f;  // 切角量，使得水平边为8像素
}

// 检查网格点(i,j)是否在正八边形内（用于渲染）
bool isInsideOctagon(int i, int j) {
    float cx = octParams.center;
    float cy = octParams.center;
    
    float dx = abs(i - cx);
    float dy = abs(j - cy);
    float half = octParams.halfSize;
    float cut = octParams.cornerCut;
    
    // 切角条件: 如果dx和dy都很大，需要满足 dx + dy <= half + (half - cut)
    float maxSum = half + (half - cut);  // 2*half - cut
    
    // 八边形条件:
    // 1. 在边界框内 (dx <= half && dy <= half) - 这一步可以省略显式检查
    // 2. 不在切角区域内: dx + dy <= maxSum
    
    return (dx <= half) && (dy <= half) && (dx + dy <= maxSum);
}

// 绘制正八边形边框
void drawOctagonFrame(int offsetX, int offsetY, int pixelSize) {
    // 计算八边形顶点（在网格坐标中）
    float c = octParams.center;
    float h = octParams.halfSize;
    float cut = octParams.cornerCut;
    
    // 8个顶点 (按顺序)
    int vertices[8][2] = {
        {(int)(c - h + cut), (int)(c - h)},      // 左上偏上
        {(int)(c + h - cut), (int)(c - h)},      // 右上偏上
        {(int)(c + h),       (int)(c - h + cut)}, // 右上偏右
        {(int)(c + h),       (int)(c + h - cut)}, // 右下偏右
        {(int)(c + h - cut), (int)(c + h)},       // 右下偏下
        {(int)(c - h + cut), (int)(c + h)},       // 左下偏下
        {(int)(c - h),       (int)(c + h - cut)}, // 左下偏左
        {(int)(c - h),       (int)(c - h + cut)}  // 左上偏左
    };
    
    // 转换到屏幕坐标并绘制线条
    for (int v = 0; v < 8; v++) {
        int next = (v + 1) % 8;
        
        int x1 = offsetX + vertices[v][0] * pixelSize;
        int y1 = offsetY + (SIM_RESOLUTION - 1 - vertices[v][1]) * pixelSize;  // Y翻转
        int x2 = offsetX + vertices[next][0] * pixelSize;
        int y2 = offsetY + (SIM_RESOLUTION - 1 - vertices[next][1]) * pixelSize;
        
        u8g2.drawLine(x1, y1, x2, y2);
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
            particleVel[i * 2] += dt * gx;
            particleVel[i * 2 + 1] += dt * gy;
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
            
            // 物理模拟保持圆形边界
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

// ============== 场景设置 ==============
void setupScene() {
    tankWidth = 3.0f;
    tankHeight = 3.0f;
    
    float spacing = tankWidth / SIM_RESOLUTION;
    
    fluid.init(DENSITY, tankWidth, tankHeight, spacing, MAX_PARTICLES);
    
    // 初始化正八边形参数
    initOctagonParams();
    
    // 在圆形区域内创建水块（物理模拟用圆形）
    float r = 0.3f * h;
    float dx = 2.0f * r;
    float dy = 1.732f * r;
    
    numParticles = 0;
    
    // 填充下半部分（重力向下时）
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

// ============== 正八边形显示 ==============
void drawFluid() {
    u8g2.clearBuffer();
    
    // 绘制正八边形边框
    drawOctagonFrame(displayOffsetX, displayOffsetY, cellPixelSize);
    
    // 绘制重力方向指示器（在八边形中心）
    float cx = octParams.center * cellPixelSize + displayOffsetX;
    float cy = octParams.center * cellPixelSize + displayOffsetY;
    float rad = degToRad(gravityAngle);
    int arrowLen = 6;
    int arrowX = cx + cos(rad) * arrowLen;
    int arrowY = cy - sin(rad) * arrowLen;
    u8g2.drawLine((int)cx, (int)cy, arrowX, arrowY);
    u8g2.drawDisc(arrowX, arrowY, 1);
    
    // 绘制流体单元格 - 只在正八边形区域内
    for (int i = 0; i < SIM_RESOLUTION; i++) {
        for (int j = 0; j < SIM_RESOLUTION; j++) {
            // 检查是否在正八边形内（渲染用）
            if (!isInsideOctagon(i, j)) continue;
            
            // 映射到模拟网格
            int simI = (int)((float)i / SIM_RESOLUTION * (fNumX - 2)) + 1;
            int simJ = (int)((float)j / SIM_RESOLUTION * (fNumY - 2)) + 1;
            
            simI = myClamp(simI, 0, fNumX - 1);
            simJ = myClamp(simJ, 0, fNumY - 1);
            
            int cellIdx = simI * fNumY + simJ;
            bool isFluid = (cellType[cellIdx] == FLUID_CELL);
            
            if (isFluid) {
                // 屏幕坐标（Y轴翻转）
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
    snprintf(buf, sizeof(buf), "%d%% %.0f", (int)(gravityScale * 100), gravityAngle);
    u8g2.drawStr(0, 63, buf);
    
    u8g2.sendBuffer();
}

// ============== Arduino 标准函数 ==============
void setup() {
    Serial.begin(2000000);
    delay(3000);
    
    updateGravity();
    
    u8g2.begin();
    u8g2.setBusClock(1000000);
    u8g2.setContrast(255);
    
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(10, 20, "FLIP Fluid");
    u8g2.drawStr(10, 35, "Octagon");
    u8g2.drawStr(10, 50, "Type 'help'");
    u8g2.sendBuffer();
    
    setupScene();
    
    Serial.println("Setup complete. Type 'help' for commands.");
    delay(500);
}

void loop() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            processSerialCommand(inputBuffer);
            inputBuffer = "";
        } else {
            inputBuffer += c;
        }
    }
    
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
        Serial.print(" P:");
        Serial.print(numParticles);
        Serial.print("\n");
    }
}
