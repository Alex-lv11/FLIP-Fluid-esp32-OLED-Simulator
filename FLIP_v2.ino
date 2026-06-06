// NVIDIA FLIP 流体算法 - 修复版
// 修复边界处理和压力发散问题

#include <Wire.h>
#include <U8g2lib.h>

#ifdef ESP32
  #define SDA_PIN 21
  #define SCL_PIN 22
#else
  #define SDA_PIN 4
  #define SCL_PIN 5
#endif

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, SCL_PIN, SDA_PIN, U8X8_PIN_NONE);

#define ROWS 16
#define COLS 16
#define NUM_PARTICLES 80  // 固定数量，不计算

// ========== 可调参数 ==========
float GRAVITY_STRENGTH = 0.6;          // 重力减小，更稳定0.1
float GRAVITY_ANGLE_SPEED = 0.02;      // 旋转更慢0.01
float DT = 0.01;                       // 时间步长减小0.05
int PRESSURE_ITERATIONS = 1;          // 减少迭代，避免发散20
float FLIP_BLENDING = 0.9;             // 稍微PIC一点，更稳定0.9
// ==============================

struct Particle {
  float x, y;
  float u, v;
};

Particle particles[NUM_PARTICLES];

struct MACGrid {
  float u[ROWS][COLS+1];
  float v[ROWS+1][COLS];
  float uPrev[ROWS][COLS+1];
  float vPrev[ROWS+1][COLS];
  float pressure[ROWS][COLS];
  bool isFluid[ROWS][COLS];
};

MACGrid grid;

float angle = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  u8g2.begin();
  u8g2.setFont(u8g2_font_5x7_tf);
  
  initFluid();
  Serial.println("FLIP Fixed");
}

void initFluid() {
  memset(&grid, 0, sizeof(grid));
  
  // 底部矩形区域初始化粒子，更整齐
  int idx = 0;
  int startX = 8;
  int endX = 24;
  int startY = 20;
  int endY = 30;
  
  for (float y = startY; y < endY && idx < NUM_PARTICLES; y += 0.8) {
    for (float x = startX; x < endX && idx < NUM_PARTICLES; x += 0.8) {
      particles[idx].x = x + random(20)/100.0;
      particles[idx].y = y + random(20)/100.0;
      particles[idx].u = 0;
      particles[idx].v = 0;
      idx++;
    }
  }
  
  // 补充剩余
  while (idx < NUM_PARTICLES) {
    particles[idx].x = random(startX, endX);
    particles[idx].y = random(startY, endY);
    particles[idx].u = 0;
    particles[idx].v = 0;
    idx++;
  }
  
  Serial.print("Init: ");
  Serial.println(idx);
}

// 安全的双线性权重计算
float kernel(float r) {
  r = fabs(r);
  if (r < 1.0) return 1.0 - r;
  return 0;
}

void particlesToGrid() {
  // 清空
  memset(grid.u, 0, sizeof(grid.u));
  memset(grid.v, 0, sizeof(grid.v));
  memset(grid.isFluid, 0, sizeof(grid.isFluid));
  
  float weightU[ROWS][COLS+1] = {0};
  float weightV[ROWS+1][COLS] = {0};
  
  // 保存旧速度
  memcpy(grid.uPrev, grid.u, sizeof(grid.u));
  memcpy(grid.vPrev, grid.v, sizeof(grid.v));
  
  // 散布到网格
  for (int p = 0; p < NUM_PARTICLES; p++) {
    float x = particles[p].x;
    float y = particles[p].y;
    
    // 标记流体
    int gi = constrain((int)x, 0, COLS-1);
    int gj = constrain((int)y, 0, ROWS-1);
    grid.isFluid[gj][gi] = true;
    
    // 散布到 u 网格 (i+0.5, j)
    float xu = x;
    float yu = y;
    int iu = (int)(xu - 0.5);
    int ju = (int)yu;
    
    for (int j = ju-1; j <= ju+1; j++) {
      for (int i = iu-1; i <= iu+1; i++) {
        if (i < 0 || i > COLS || j < 0 || j >= ROWS) continue;
        float w = kernel(xu - (i + 0.5)) * kernel(yu - j);
        if (w > 0) {
          grid.u[j][i] += particles[p].u * w;
          weightU[j][i] += w;
        }
      }
    }
    
    // 散布到 v 网格 (i, j+0.5)
    float xv = x;
    float yv = y;
    int iv = (int)xv;
    int jv = (int)(yv - 0.5);
    
    for (int j = jv-1; j <= jv+1; j++) {
      for (int i = iv-1; i <= iv+1; i++) {
        if (i < 0 || i >= COLS || j < 0 || j > ROWS) continue;
        float w = kernel(xv - i) * kernel(yv - (j + 0.5));
        if (w > 0) {
          grid.v[j][i] += particles[p].v * w;
          weightV[j][i] += w;
        }
      }
    }
  }
  
  // 归一化
  for (int j = 0; j < ROWS; j++) {
    for (int i = 0; i <= COLS; i++) {
      if (weightU[j][i] > 0.001) grid.u[j][i] /= weightU[j][i];
    }
  }
  for (int j = 0; j <= ROWS; j++) {
    for (int i = 0; i < COLS; i++) {
      if (weightV[j][i] > 0.001) grid.v[j][i] /= weightV[j][i];
    }
  }
}

void addExternalForces() {
  angle += GRAVITY_ANGLE_SPEED;
  float gx = sin(angle) * GRAVITY_STRENGTH;
  float gy = cos(angle) * GRAVITY_STRENGTH;
  
  // 只对流体格子施加重力
  for (int j = 0; j < ROWS; j++) {
    for (int i = 0; i <= COLS; i++) {
      int gi = constrain(i, 0, COLS-1);
      if (grid.isFluid[j][gi]) grid.u[j][i] += gx * DT;
    }
  }
  for (int j = 0; j <= ROWS; j++) {
    for (int i = 0; i < COLS; i++) {
      int gj = constrain(j, 0, ROWS-1);
      if (grid.isFluid[gj][i]) grid.v[j][i] += gy * DT;
    }
  }
}

void pressureProjection() {
  // 初始化压力
  for (int j = 0; j < ROWS; j++) {
    for (int i = 0; i < COLS; i++) {
      grid.pressure[j][i] = 0;
    }
  }
  
  // 计算散度
  float div[ROWS][COLS] = {0};
  for (int j = 0; j < ROWS; j++) {
    for (int i = 0; i < COLS; i++) {
      if (!grid.isFluid[j][i]) continue;
      div[j][i] = (grid.u[j][i+1] - grid.u[j][i]) + (grid.v[j+1][i] - grid.v[j][i]);
      div[j][i] /= DT;  // 除以DT，这是正确的
    }
  }
  
  // 带松弛的雅可比迭代
  float omega = 1.5;  // 超松弛因子，加速收敛
  
  for (int iter = 0; iter < PRESSURE_ITERATIONS; iter++) {
    for (int j = 0; j < ROWS; j++) {
      for (int i = 0; i < COLS; i++) {
        if (!grid.isFluid[j][i]) continue;
        
        // 邻居压力
        float pL = (i > 0 && grid.isFluid[j][i-1]) ? grid.pressure[j][i-1] : grid.pressure[j][i];
        float pR = (i < COLS-1 && grid.isFluid[j][i+1]) ? grid.pressure[j][i+1] : grid.pressure[j][i];
        float pB = (j > 0 && grid.isFluid[j-1][i]) ? grid.pressure[j-1][i] : grid.pressure[j][i];
        float pT = (j < ROWS-1 && grid.isFluid[j+1][i]) ? grid.pressure[j+1][i] : grid.pressure[j][i];
        
        // 固体边界：镜像压力（Neumann边界条件）
        if (i == 0) pL = grid.pressure[j][i];
        if (i == COLS-1) pR = grid.pressure[j][i];
        if (j == 0) pB = grid.pressure[j][i];
        if (j == ROWS-1) pT = grid.pressure[j][i];
        
        // 求解
        float pNew = (pL + pR + pB + pT - div[j][i]) * 0.25;
        grid.pressure[j][i] = (1-omega) * grid.pressure[j][i] + omega * pNew;
      }
    }
  }
  
  // 修正速度，注意压力梯度方向
  for (int j = 0; j < ROWS; j++) {
    for (int i = 0; i <= COLS; i++) {
      int gi = constrain(i, 0, COLS-1);
      if (!grid.isFluid[j][gi]) continue;
      
      float pL = (i > 0) ? grid.pressure[j][i-1] : grid.pressure[j][0];
      float pR = (i < COLS) ? grid.pressure[j][i] : grid.pressure[j][COLS-1];
      
      grid.u[j][i] -= DT * (pR - pL);
    }
  }
  for (int j = 0; j <= ROWS; j++) {
    for (int i = 0; i < COLS; i++) {
      int gj = constrain(j, 0, ROWS-1);
      if (!grid.isFluid[gj][i]) continue;
      
      float pB = (j > 0) ? grid.pressure[j-1][i] : grid.pressure[0][i];
      float pT = (j < ROWS) ? grid.pressure[j][i] : grid.pressure[ROWS-1][i];
      
      grid.v[j][i] -= DT * (pT - pB);
    }
  }
}

void applyBoundary() {
  // 固体墙：切向自由滑移，法向无穿透
  for (int j = 0; j < ROWS; j++) {
    grid.u[j][0] = 0;        // 左墙
    grid.u[j][COLS] = 0;     // 右墙
  }
  for (int i = 0; i < COLS; i++) {
    grid.v[0][i] = 0;        // 底墙
    grid.v[ROWS][i] = 0;     // 顶墙
  }
}

void gridToParticles() {
  for (int p = 0; p < NUM_PARTICLES; p++) {
    float x = particles[p].x;
    float y = particles[p].y;
    
    // u 速度
    float xu = x;
    float yu = y;
    int iu = (int)(xu - 0.5);
    int ju = (int)yu;
    
    float picU = 0, flipU = 0, wSum = 0;
    for (int j = ju-1; j <= ju+1; j++) {
      for (int i = iu-1; i <= iu+1; i++) {
        if (i < 0 || i > COLS || j < 0 || j >= ROWS) continue;
        float w = kernel(xu - (i + 0.5)) * kernel(yu - j);
        if (w > 0) {
          picU += grid.u[j][i] * w;
          flipU += (grid.u[j][i] - grid.uPrev[j][i]) * w;
          wSum += w;
        }
      }
    }
    if (wSum > 0.001) {
      picU /= wSum;
      flipU /= wSum;
      float newU = FLIP_BLENDING * (particles[p].u + flipU) + (1 - FLIP_BLENDING) * picU;
      particles[p].u = newU;
    }
    
    // v 速度
    float xv = x;
    float yv = y;
    int iv = (int)xv;
    int jv = (int)(yv - 0.5);
    
    float picV = 0, flipV = 0;
    wSum = 0;
    for (int j = jv-1; j <= jv+1; j++) {
      for (int i = iv-1; i <= iv+1; i++) {
        if (i < 0 || i >= COLS || j < 0 || j > ROWS) continue;
        float w = kernel(xv - i) * kernel(yv - (j + 0.5));
        if (w > 0) {
          picV += grid.v[j][i] * w;
          flipV += (grid.v[j][i] - grid.vPrev[j][i]) * w;
          wSum += w;
        }
      }
    }
    if (wSum > 0.001) {
      picV /= wSum;
      flipV /= wSum;
      float newV = FLIP_BLENDING * (particles[p].v + flipV) + (1 - FLIP_BLENDING) * picV;
      particles[p].v = newV;
    }
  }
}

void advectParticles() {
  for (int p = 0; p < NUM_PARTICLES; p++) {
    // 简单欧拉积分，更稳定
    particles[p].x += particles[p].u * DT;
    particles[p].y += particles[p].v * DT;
    
    // 软边界：推开而不是反弹
    float margin = 1.0;
    if (particles[p].x < margin) {
      particles[p].x = margin + (margin - particles[p].x) * 0.5;
      particles[p].u *= -0.3;
    }
    if (particles[p].x > COLS - margin) {
      particles[p].x = COLS - margin - (particles[p].x - (COLS - margin)) * 0.5;
      particles[p].u *= -0.3;
    }
    if (particles[p].y < margin) {
      particles[p].y = margin + (margin - particles[p].y) * 0.5;
      particles[p].v *= -0.3;
    }
    if (particles[p].y > ROWS - margin) {
      particles[p].y = ROWS - margin - (particles[p].y - (ROWS - margin)) * 0.5;
      particles[p].v *= -0.3;
    }
    
    // 最终硬限制
    particles[p].x = constrain(particles[p].x, 0.5, COLS - 0.5);
    particles[p].y = constrain(particles[p].y, 0.5, ROWS - 0.5);
  }
}

void updatePhysics() {
  particlesToGrid();
  addExternalForces();
  pressureProjection();
  applyBoundary();
  gridToParticles();
  advectParticles();
}

void drawToOLED() {
  u8g2.clearBuffer();
  
  // 绘制粒子
  for (int p = 0; p < NUM_PARTICLES; p++) {
    int x = (int)(particles[p].x * 2);
    int y = (int)(particles[p].y * 2);
    if (x >= 0 && x < 64 && y >= 0 && y < 64) {
      u8g2.drawBox(x, y, 2, 2);
    }
  }
  
  // 边框
  u8g2.drawFrame(0, 0, 64, 64);
  
  // 信息
  u8g2.setCursor(70, 10);
  u8g2.print("N:");
  u8g2.print(NUM_PARTICLES);
  
  u8g2.sendBuffer();
}

void loop() {
  updatePhysics();
  drawToOLED();
  delay(0);
}
