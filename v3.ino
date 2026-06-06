// FLIP 流体 - 数值稳定版
// 自适应时间步长，防止发散

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

// ========== 网格配置 ==========
#define GRID_SIZE 8           // 16或32，统一配置
#define ROWS GRID_SIZE
#define COLS GRID_SIZE
#define NUM_PARTICLES (int)(ROWS * COLS * 0.3)  // 30%填充

// ========== 物理参数 ==========
float GRAVITY_STRENGTH = 0.3;          // 重力减小
float GRAVITY_ANGLE_SPEED = 0.02;      // 旋转更慢
float MAX_VELOCITY = 10.0;               // 最大速度限制
float FLIP_BLENDING = 0.95;
int PRESSURE_ITERATIONS = 30;

// 自适应时间步长
float DT = 0.016;                       // 基础60fps
float TARGET_CFL = 0.5;                 // CFL数目标

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
  Serial.begin(500000);
  Wire.begin(SDA_PIN, SCL_PIN);
  u8g2.begin();
  u8g2.setFont(u8g2_font_5x7_tf);
  
  initFluid();
  
  Serial.print("Grid: ");
  Serial.print(COLS);
  Serial.print("x");
  Serial.print(ROWS);
  Serial.print(" Particles: ");
  Serial.println(NUM_PARTICLES);
}

void initFluid() {
  memset(&grid, 0, sizeof(grid));
  
  // 底部填充，留出顶部空间
  int idx = 0;
  int margin = COLS / 4;
  int fillHeight = ROWS * 0.4;  // 填充底部40%
  
  for (int y = ROWS - fillHeight; y < ROWS && idx < NUM_PARTICLES; y++) {
    for (int x = margin; x < COLS - margin && idx < NUM_PARTICLES; x++) {
      // 每个格子2x2粒子
      for (int dy = 0; dy < 2 && idx < NUM_PARTICLES; dy++) {
        for (int dx = 0; dx < 2 && idx < NUM_PARTICLES; dx++) {
          particles[idx].x = x + 0.25 + dx * 0.5;
          particles[idx].y = y + 0.25 + dy * 0.5;
          particles[idx].u = 0;
          particles[idx].v = 0;
          idx++;
        }
      }
    }
  }
  
  // 补充剩余
  while (idx < NUM_PARTICLES) {
    particles[idx].x = random(margin, COLS - margin);
    particles[idx].y = random(ROWS/2, ROWS);
    particles[idx].u = 0;
    particles[idx].v = 0;
    idx++;
  }
}

// 线性核函数
inline float kernel(float r) {
  r = fabs(r);
  return (r < 1.0f) ? (1.0f - r) : 0.0f;
}

void particlesToGrid() {
  memset(grid.u, 0, sizeof(grid.u));
  memset(grid.v, 0, sizeof(grid.v));
  memset(grid.isFluid, 0, sizeof(grid.isFluid));
  
  memcpy(grid.uPrev, grid.u, sizeof(grid.u));
  memcpy(grid.vPrev, grid.v, sizeof(grid.v));
  
  float weightU[ROWS][COLS+1] = {0};
  float weightV[ROWS+1][COLS] = {0};
  
  for (int p = 0; p < NUM_PARTICLES; p++) {
    float x = particles[p].x;
    float y = particles[p].y;
    
    // 标记流体
    int gi = constrain((int)x, 0, COLS-1);
    int gj = constrain((int)y, 0, ROWS-1);
    grid.isFluid[gj][gi] = true;
    
    // 散布 u (位置: i+0.5, j)
    int iu = (int)(x - 0.5f);
    int ju = (int)y;
    for (int j = max(0, ju-1); j <= min(ROWS-1, ju+1); j++) {
      for (int i = max(0, iu-1); i <= min(COLS, iu+1); i++) {
        float w = kernel(x - (i + 0.5f)) * kernel(y - j);
        if (w > 0) {
          grid.u[j][i] += particles[p].u * w;
          weightU[j][i] += w;
        }
      }
    }
    
    // 散布 v (位置: i, j+0.5)
    int iv = (int)x;
    int jv = (int)(y - 0.5f);
    for (int j = max(0, jv-1); j <= min(ROWS, jv+1); j++) {
      for (int i = max(0, iv-1); i <= min(COLS-1, iv+1); i++) {
        float w = kernel(x - i) * kernel(y - (j + 0.5f));
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
      if (weightU[j][i] > 0.001f) grid.u[j][i] /= weightU[j][i];
    }
  }
  for (int j = 0; j <= ROWS; j++) {
    for (int i = 0; i < COLS; i++) {
      if (weightV[j][i] > 0.001f) grid.v[j][i] /= weightV[j][i];
    }
  }
}

void addForces() {
  angle += GRAVITY_ANGLE_SPEED;
  float gx = sin(angle) * GRAVITY_STRENGTH;
  float gy = cos(angle) * GRAVITY_STRENGTH;
  
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

void projectPressure() {
  // 计算散度
  float div[ROWS][COLS];
  for (int j = 0; j < ROWS; j++) {
    for (int i = 0; i < COLS; i++) {
      div[j][i] = 0;
      if (!grid.isFluid[j][i]) continue;
      div[j][i] = (grid.u[j][i+1] - grid.u[j][i] + grid.v[j+1][i] - grid.v[j][i]) / DT;
    }
  }
  
  // 压力清零
  for (int j = 0; j < ROWS; j++) {
    for (int i = 0; i < COLS; i++) {
      grid.pressure[j][i] = 0;
    }
  }
  
  // 带阻尼的雅可比迭代，防止发散
  float damping = 0.8f;
  
  for (int iter = 0; iter < PRESSURE_ITERATIONS; iter++) {
    for (int j = 0; j < ROWS; j++) {
      for (int i = 0; i < COLS; i++) {
        if (!grid.isFluid[j][i]) continue;
        
        // 邻居
        float pL = (i > 0 && grid.isFluid[j][i-1]) ? grid.pressure[j][i-1] : grid.pressure[j][i];
        float pR = (i < COLS-1 && grid.isFluid[j][i+1]) ? grid.pressure[j][i+1] : grid.pressure[j][i];
        float pB = (j > 0 && grid.isFluid[j-1][i]) ? grid.pressure[j-1][i] : grid.pressure[j][i];
        float pT = (j < ROWS-1 && grid.isFluid[j+1][i]) ? grid.pressure[j+1][i] : grid.pressure[j][i];
        
        // 求解并阻尼
        float pNew = (pL + pR + pB + pT - div[j][i]) * 0.25f;
        grid.pressure[j][i] = damping * grid.pressure[j][i] + (1-damping) * pNew;
      }
    }
  }
  
  // 速度修正
  for (int j = 0; j < ROWS; j++) {
    for (int i = 0; i <= COLS; i++) {
      int gi = constrain(i, 0, COLS-1);
      if (!grid.isFluid[j][gi]) continue;
      float pL = (i > 0) ? grid.pressure[j][i-1] : 0;
      float pR = (i < COLS) ? grid.pressure[j][i] : 0;
      grid.u[j][i] -= DT * (pR - pL);
    }
  }
  for (int j = 0; j <= ROWS; j++) {
    for (int i = 0; i < COLS; i++) {
      int gj = constrain(j, 0, ROWS-1);
      if (!grid.isFluid[gj][i]) continue;
      float pB = (j > 0) ? grid.pressure[j-1][i] : 0;
      float pT = (j < ROWS) ? grid.pressure[j][i] : 0;
      grid.v[j][i] -= DT * (pT - pB);
    }
  }
}

void applyBoundary() {
  // 固体墙
  for (int j = 0; j < ROWS; j++) {
    grid.u[j][0] = 0;
    grid.u[j][COLS] = 0;
  }
  for (int i = 0; i < COLS; i++) {
    grid.v[0][i] = 0;
    grid.v[ROWS][i] = 0;
  }
}

void gridToParticles() {
  for (int p = 0; p < NUM_PARTICLES; p++) {
    float x = particles[p].x;
    float y = particles[p].y;
    
    // 插值 u
    int iu = (int)(x - 0.5f);
    int ju = (int)y;
    float picU = 0, deltaU = 0, wSum = 0;
    
    for (int j = max(0, ju-1); j <= min(ROWS-1, ju+1); j++) {
      for (int i = max(0, iu-1); i <= min(COLS, iu+1); i++) {
        float w = kernel(x - (i + 0.5f)) * kernel(y - j);
        if (w > 0) {
          picU += grid.u[j][i] * w;
          deltaU += (grid.u[j][i] - grid.uPrev[j][i]) * w;
          wSum += w;
        }
      }
    }
    
    if (wSum > 0.001f) {
      picU /= wSum;
      deltaU /= wSum;
      particles[p].u = FLIP_BLENDING * (particles[p].u + deltaU) + (1-FLIP_BLENDING) * picU;
    }
    
    // 插值 v
    int iv = (int)x;
    int jv = (int)(y - 0.5f);
    float picV = 0, deltaV = 0;
    wSum = 0;
    
    for (int j = max(0, jv-1); j <= min(ROWS, jv+1); j++) {
      for (int i = max(0, iv-1); i <= min(COLS-1, iv+1); i++) {
        float w = kernel(x - i) * kernel(y - (j + 0.5f));
        if (w > 0) {
          picV += grid.v[j][i] * w;
          deltaV += (grid.v[j][i] - grid.vPrev[j][i]) * w;
          wSum += w;
        }
      }
    }
    
    if (wSum > 0.001f) {
      picV /= wSum;
      deltaV /= wSum;
      particles[p].v = FLIP_BLENDING * (particles[p].v + deltaV) + (1-FLIP_BLENDING) * picV;
    }
    
    // 速度限制！防止CFL violation
    float speed = sqrt(particles[p].u * particles[p].u + particles[p].v * particles[p].v);
    if (speed > MAX_VELOCITY) {
      particles[p].u = (particles[p].u / speed) * MAX_VELOCITY;
      particles[p].v = (particles[p].v / speed) * MAX_VELOCITY;
    }
  }
}

void advectParticles() {
  // 自适应时间步长：如果速度大，减小步长
  float maxSpeed = 0;
  for (int p = 0; p < NUM_PARTICLES; p++) {
    float s = abs(particles[p].u) + abs(particles[p].v);
    if (s > maxSpeed) maxSpeed = s;
  }
  
  // CFL条件
  float dt = DT;
  if (maxSpeed > 0.1f) {
    dt = min(DT, TARGET_CFL / maxSpeed);
  }
  
  // 子步积分
  int substeps = (int)(DT / dt) + 1;
  dt = DT / substeps;
  
  for (int step = 0; step < substeps; step++) {
    for (int p = 0; p < NUM_PARTICLES; p++) {
      particles[p].x += particles[p].u * dt;
      particles[p].y += particles[p].v * dt;
      
      // 软边界
      float margin = 0.5f;
      if (particles[p].x < margin) {
        particles[p].x = margin + (margin - particles[p].x) * 0.5f;
        particles[p].u *= -0.2f;
      }
      if (particles[p].x > COLS - margin) {
        particles[p].x = COLS - margin - (particles[p].x - (COLS - margin)) * 0.5f;
        particles[p].u *= -0.2f;
      }
      if (particles[p].y < margin) {
        particles[p].y = margin + (margin - particles[p].y) * 0.5f;
        particles[p].v *= -0.2f;
      }
      if (particles[p].y > ROWS - margin) {
        particles[p].y = ROWS - margin - (particles[p].y - (ROWS - margin)) * 0.5f;
        particles[p].v *= -0.2f;
      }
      
      // 硬限制
      particles[p].x = constrain(particles[p].x, 0.1f, COLS - 0.1f);
      particles[p].y = constrain(particles[p].y, 0.1f, ROWS - 0.1f);
    }
  }
}

void updatePhysics() {
  particlesToGrid();
  addForces();
  projectPressure();
  applyBoundary();
  gridToParticles();
  advectParticles();
}

void drawToOLED() {
  u8g2.clearBuffer();
  
  // 绘制粒子
  int pixelSize = (GRID_SIZE == 32) ? 2 : 4;
  for (int p = 0; p < NUM_PARTICLES; p++) {
    int x = (int)(particles[p].x * pixelSize);
    int y = (int)(particles[p].y * pixelSize);
    if (x >= 0 && x < 64 && y >= 0 && y < 64) {
      u8g2.drawBox(x, y, pixelSize, pixelSize);
    }
  }
  
  // 边框
  u8g2.drawFrame(0, 0, 64, 64);
  
  // 信息
  u8g2.setCursor(70, 10);
  u8g2.print(GRID_SIZE);
  u8g2.print("x");
  u8g2.print(GRID_SIZE);
  
  u8g2.sendBuffer();
}

void loop() {
  updatePhysics();
  drawToOLED();
  delay(0);
}
