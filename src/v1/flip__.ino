// ESP32/ESP8266 版本，32x32 网格，30% 液体粒子

#include <Wire.h>
#include <U8g2lib.h>

// ESP32: SDA=GPIO21, SCL=GPIO22
// ESP8266: SDA=GPIO4, SCL=GPIO5
#ifdef ESP32
  #define SDA_PIN 21
  #define SCL_PIN 22
#else
  #define SDA_PIN 4
  #define SCL_PIN 5
#endif

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL_PIN, /* data=*/ SDA_PIN, /* reset=*/ U8X8_PIN_NONE);

#define ROWS 16
#define COLS 16
#define TOTAL_CELLS (ROWS * COLS)
#define LIQUID_RATIO 0.3
#define NUM_PARTICLES (int)(TOTAL_CELLS * LIQUID_RATIO)

// ========== 可调物理参数 ==========
float GRAVITY_STRENGTH = 0.3;          // 重力强度
float GRAVITY_ANGLE_SPEED = 0.015;     // 重力旋转速度
float DT = 0.1;                        // 时间步长
int PRESSURE_ITERATIONS = 20;          // 压力求解迭代次数
float FLIP_BLENDING = 0.95;            // FLIP/PIC 混合系数 (0=PIC, 1=FLIP)
// ==================================

// 粒子（拉格朗日视角）
struct Particle {
  float x, y;        // 位置
  float u, v;        // 速度
};

Particle particles[NUM_PARTICLES];

// 交错网格 MAC
// 速度存储在格子边缘
struct MACGrid {
  float u[ROWS][COLS+1];      // x方向速度，存储在左右边缘 (i+0.5, j)
  float v[ROWS+1][COLS];      // y方向速度，存储在上下边缘 (i, j+0.5)
  float uPrev[ROWS][COLS+1];  // 上一帧速度（FLIP用）
  float vPrev[ROWS+1][COLS];
  float pressure[ROWS][COLS]; // 压力，存储在格子中心
  bool isFluid[ROWS][COLS];   // 流体标记
  float phi[ROWS][COLS];      //  signed distance function（表面追踪）
};

MACGrid grid;

float angle = 0;
float gravityX = 0, gravityY = 0;

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Wire.begin(SDA_PIN, SCL_PIN);
  u8g2.begin();
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setDrawColor(1);
  
  initFluid();
  
  Serial.print("FLIP Fluid: ");
  Serial.print(COLS);
  Serial.print("x");
  Serial.print(ROWS);
  Serial.print(" Grid, ");
  Serial.print(NUM_PARTICLES);
  Serial.println(" Particles");
}

// ========== 1. 初始化 ==========
void initFluid() {
  // 清空网格
  memset(&grid, 0, sizeof(grid));
  
  // 初始化粒子（底部聚集）
  int idx = 0;
  for (int y = ROWS - 8; y < ROWS && idx < NUM_PARTICLES; y++) {
    for (int x = 4; x < COLS - 4 && idx < NUM_PARTICLES; x += 1) {
      // 每个格子内随机分布4个粒子
      for (int py = 0; py < 2 && idx < NUM_PARTICLES; py++) {
        for (int px = 0; px < 2 && idx < NUM_PARTICLES; px++) {
          particles[idx].x = x + 0.25 + px * 0.5;
          particles[idx].y = y + 0.25 + py * 0.5;
          particles[idx].u = 0;
          particles[idx].v = 0;
          idx++;
        }
      }
    }
  }
  
  // 随机填充剩余
  while (idx < NUM_PARTICLES) {
    particles[idx].x = random(4, COLS - 4) + random(100) / 100.0;
    particles[idx].y = random(ROWS / 2, ROWS - 4) + random(100) / 100.0;
    particles[idx].u = 0;
    particles[idx].v = 0;
    idx++;
  }
}

// ========== 2. 粒子 -> 网格 (PIC) ==========
void particlesToGrid() {
  // 清零并标记
  for (int j = 0; j < ROWS; j++) {
    for (int i = 0; i < COLS; i++) {
      grid.isFluid[j][i] = false;
      grid.phi[j][i] = 999;  // 空气
    }
  }
  
  for (int j = 0; j < ROWS; j++) {
    for (int i = 0; i <= COLS; i++) {
      grid.u[j][i] = 0;
      grid.uPrev[j][i] = grid.u[j][i];
    }
  }
  for (int j = 0; j <= ROWS; j++) {
    for (int i = 0; i < COLS; i++) {
      grid.v[j][i] = 0;
      grid.vPrev[j][i] = grid.v[j][i];
    }
  }
  
  // 计算权重和
  float weightU[ROWS][COLS+1] = {0};
  float weightV[ROWS+1][COLS] = {0};
  
  // 散布粒子到网格
  for (int p = 0; p < NUM_PARTICLES; p++) {
    float x = particles[p].x;
    float y = particles[p].y;
    
    // 标记流体格子
    int gi = (int)x;
    int gj = (int)y;
    if (gi >= 0 && gi < COLS && gj >= 0 && gj < ROWS) {
      grid.isFluid[gj][gi] = true;
      // 简单的SDF估计
      float dist = sqrt((x - gi - 0.5)*(x - gi - 0.5) + (y - gj - 0.5)*(y - gj - 0.5));
      if (dist < grid.phi[gj][gi]) grid.phi[gj][gi] = dist;
    }
    
    // 双线性权重散布到 u 网格 (交错网格)
    // u 存储在 (i+0.5, j)
    float xu = x;
    float yu = y;
    int iu = (int)(xu - 0.5);
    int ju = (int)yu;
    
    for (int jOff = 0; jOff <= 1; jOff++) {
      for (int iOff = 0; iOff <= 1; iOff++) {
        int i = iu + iOff;
        int j = ju + jOff;
        if (i < 0 || i > COLS || j < 0 || j >= ROWS) continue;
        
        float wx = 1.0 - abs(xu - (i + 0.5));
        float wy = 1.0 - abs(yu - j);
        float w = wx * wy;
        
        if (w > 0) {
          grid.u[j][i] += particles[p].u * w;
          weightU[j][i] += w;
        }
      }
    }
    
    // 双线性权重散布到 v 网格
    // v 存储在 (i, j+0.5)
    float xv = x;
    float yv = y;
    int iv = (int)xv;
    int jv = (int)(yv - 0.5);
    
    for (int jOff = 0; jOff <= 1; jOff++) {
      for (int iOff = 0; iOff <= 1; iOff++) {
        int i = iv + iOff;
        int j = jv + jOff;
        if (i < 0 || i >= COLS || j < 0 || j > ROWS) continue;
        
        float wx = 1.0 - abs(xv - i);
        float wy = 1.0 - abs(yv - (j + 0.5));
        float w = wx * wy;
        
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
      if (weightU[j][i] > 0.001) {
        grid.u[j][i] /= weightU[j][i];
      }
    }
  }
  for (int j = 0; j <= ROWS; j++) {
    for (int i = 0; i < COLS; i++) {
      if (weightV[j][i] > 0.001) {
        grid.v[j][i] /= weightV[j][i];
      }
    }
  }
}

// ========== 3. 添加外力（重力） ==========
void addExternalForces() {
  // 更新重力方向
  angle += GRAVITY_ANGLE_SPEED;
  if (angle > 2 * PI) angle -= 2 * PI;
  gravityX = sin(angle) * GRAVITY_STRENGTH;
  gravityY = cos(angle) * GRAVITY_STRENGTH;
  
  // 对流体格子应用重力
  for (int j = 0; j < ROWS; j++) {
    for (int i = 0; i <= COLS; i++) {
      // u 速度受 x 方向重力
      int gi = (i < COLS) ? i : COLS - 1;
      if (grid.isFluid[j][gi]) {
        grid.u[j][i] += gravityX * DT;
      }
    }
  }
  for (int j = 0; j <= ROWS; j++) {
    for (int i = 0; i < COLS; i++) {
      // v 速度受 y 方向重力
      int gj = (j < ROWS) ? j : ROWS - 1;
      if (grid.isFluid[gj][i]) {
        grid.v[j][i] += gravityY * DT;
      }
    }
  }
}

// ========== 4. 压力投影（不可压缩） ==========
void pressureProjection() {
  // 初始化压力
  for (int j = 0; j < ROWS; j++) {
    for (int i = 0; i < COLS; i++) {
      grid.pressure[j][i] = 0;
    }
  }
  
  // 计算散度（速度场的散度）
  float divergence[ROWS][COLS] = {0};
  
  for (int j = 0; j < ROWS; j++) {
    for (int i = 0; i < COLS; i++) {
      if (!grid.isFluid[j][i]) continue;
      
      // div(u) = du/dx + dv/dy
      // 在格子中心计算
      float du_dx = grid.u[j][i+1] - grid.u[j][i];
      float dv_dy = grid.v[j+1][i] - grid.v[j][i];
      divergence[j][i] = du_dx + dv_dy;
    }
  }
  
  // 雅可比迭代求解压力泊松方程：laplacian(p) = div(u) / DT
  for (int iter = 0; iter < PRESSURE_ITERATIONS; iter++) {
    float maxResidual = 0;
    
    for (int j = 0; j < ROWS; j++) {
      for (int i = 0; i < COLS; i++) {
        if (!grid.isFluid[j][i]) continue;
        
        // 邻居压力（空气格子压力为0）
        float pLeft = (i > 0 && grid.isFluid[j][i-1]) ? grid.pressure[j][i-1] : 0;
        float pRight = (i < COLS-1 && grid.isFluid[j][i+1]) ? grid.pressure[j][i+1] : 0;
        float pBottom = (j > 0 && grid.isFluid[j-1][i]) ? grid.pressure[j-1][i] : 0;
        float pTop = (j < ROWS-1 && grid.isFluid[j+1][i]) ? grid.pressure[j+1][i] : 0;
        
        // 固体边界：压力梯度为0（Neumann边界）
        if (i == 0 || !grid.isFluid[j][i-1]) pLeft = grid.pressure[j][i];
        if (i == COLS-1 || !grid.isFluid[j][i+1]) pRight = grid.pressure[j][i];
        if (j == 0 || !grid.isFluid[j-1][i]) pBottom = grid.pressure[j][i];
        if (j == ROWS-1 || !grid.isFluid[j+1][i]) pTop = grid.pressure[j][i];
        
        // 邻居数量（用于Jacobi迭代）
        int numNeighbors = 4;
        if (i == 0) numNeighbors--;
        if (i == COLS-1) numNeighbors--;
        if (j == 0) numNeighbors--;
        if (j == ROWS-1) numNeighbors--;
        
        if (numNeighbors == 0) continue;
        
        // 泊松方程：laplacian(p) = divergence / DT
        // 离散形式：(pLeft + pRight + pBottom + pTop - 4*p) = divergence / DT
        float newP = (pLeft + pRight + pBottom + pTop - divergence[j][i] / DT) / numNeighbors;
        
        float residual = abs(newP - grid.pressure[j][i]);
        if (residual > maxResidual) maxResidual = residual;
        
        grid.pressure[j][i] = newP;
      }
    }
  }
  
  // 应用压力梯度修正速度：u_new = u_old - DT * grad(p)
  for (int j = 0; j < ROWS; j++) {
    for (int i = 0; i <= COLS; i++) {
      int gi = (i < COLS) ? i : COLS - 1;
      if (!grid.isFluid[j][gi]) continue;
      
      // dp/dx 在 u 位置
      float pLeft = (i > 0) ? grid.pressure[j][i-1] : 0;
      float pRight = (i < COLS) ? grid.pressure[j][i] : 0;
      if (i == 0) pLeft = pRight;
      if (i == COLS) pRight = pLeft;
      
      grid.u[j][i] -= DT * (pRight - pLeft);
    }
  }
  for (int j = 0; j <= ROWS; j++) {
    for (int i = 0; i < COLS; i++) {
      int gj = (j < ROWS) ? j : ROWS - 1;
      if (!grid.isFluid[gj][i]) continue;
      
      // dp/dy 在 v 位置
      float pBottom = (j > 0) ? grid.pressure[j-1][i] : 0;
      float pTop = (j < ROWS) ? grid.pressure[j][i] : 0;
      if (j == 0) pBottom = pTop;
      if (j == ROWS) pTop = pBottom;
      
      grid.v[j][i] -= DT * (pTop - pBottom);
    }
  }
}

// ========== 5. 边界条件 ==========
void applyBoundaryConditions() {
  // 固体墙：无穿透
  for (int j = 0; j < ROWS; j++) {
    grid.u[j][0] = 0;           // 左墙
    grid.u[j][COLS] = 0;        // 右墙
  }
  for (int i = 0; i < COLS; i++) {
    grid.v[0][i] = 0;           // 底墙
    grid.v[ROWS][i] = 0;        // 顶墙
  }
  
  // 自由表面边界（空气-流体界面）：压力为0（已处理）
}

// ========== 6. 网格 -> 粒子 (FLIP/PIC) ==========
void gridToParticles() {
  for (int p = 0; p < NUM_PARTICLES; p++) {
    float x = particles[p].x;
    float y = particles[p].y;
    
    // 双线性插值获取网格 u 速度
    float xu = x;
    float yu = y;
    int iu = (int)(xu - 0.5);
    int ju = (int)yu;
    
    float picU = 0;
    float deltaU = 0;
    float weightSum = 0;
    
    for (int jOff = 0; jOff <= 1; jOff++) {
      for (int iOff = 0; iOff <= 1; iOff++) {
        int i = iu + iOff;
        int j = ju + jOff;
        if (i < 0 || i > COLS || j < 0 || j >= ROWS) continue;
        
        float wx = 1.0 - abs(xu - (i + 0.5));
        float wy = 1.0 - abs(yu - j);
        float w = wx * wy;
        
        if (w > 0) {
          picU += grid.u[j][i] * w;
          deltaU += (grid.u[j][i] - grid.uPrev[j][i]) * w;
          weightSum += w;
        }
      }
    }
    if (weightSum > 0.001) {
      picU /= weightSum;
      deltaU /= weightSum;
    }
    
    // FLIP 速度 = 旧速度 + 速度变化
    float flipU = particles[p].u + deltaU;
    
    // 混合
    particles[p].u = FLIP_BLENDING * flipU + (1.0 - FLIP_BLENDING) * picU;
    
    // 双线性插值获取网格 v 速度
    float xv = x;
    float yv = y;
    int iv = (int)xv;
    int jv = (int)(yv - 0.5);
    
    float picV = 0;
    float deltaV = 0;
    weightSum = 0;
    
    for (int jOff = 0; jOff <= 1; jOff++) {
      for (int iOff = 0; iOff <= 1; iOff++) {
        int i = iv + iOff;
        int j = jv + jOff;
        if (i < 0 || i >= COLS || j < 0 || j > ROWS) continue;
        
        float wx = 1.0 - abs(xv - i);
        float wy = 1.0 - abs(yv - (j + 0.5));
        float w = wx * wy;
        
        if (w > 0) {
          picV += grid.v[j][i] * w;
          deltaV += (grid.v[j][i] - grid.vPrev[j][i]) * w;
          weightSum += w;
        }
      }
    }
    if (weightSum > 0.001) {
      picV /= weightSum;
      deltaV /= weightSum;
    }
    
    float flipV = particles[p].v + deltaV;
    particles[p].v = FLIP_BLENDING * flipV + (1.0 - FLIP_BLENDING) * picV;
  }
}

// ========== 7. 粒子平流 ==========
void advectParticles() {
  for (int p = 0; p < NUM_PARTICLES; p++) {
    // RK2 积分
    float xMid = particles[p].x + particles[p].u * DT * 0.5;
    float yMid = particles[p].y + particles[p].v * DT * 0.5;
    
    // 边界限制（简单处理）
    if (xMid < 0.5) xMid = 0.5;
    if (xMid > COLS - 0.5) xMid = COLS - 0.5;
    if (yMid < 0.5) yMid = 0.5;
    if (yMid > ROWS - 0.5) yMid = ROWS - 0.5;
    
    particles[p].x += particles[p].u * DT;
    particles[p].y += particles[p].v * DT;
    
    // 硬边界反弹
    if (particles[p].x < 0.5) {
      particles[p].x = 0.5;
      particles[p].u *= -0.5;
    }
    if (particles[p].x > COLS - 0.5) {
      particles[p].x = COLS - 0.5;
      particles[p].u *= -0.5;
    }
    if (particles[p].y < 0.5) {
      particles[p].y = 0.5;
      particles[p].v *= -0.5;
    }
    if (particles[p].y > ROWS - 0.5) {
      particles[p].y = ROWS - 0.5;
      particles[p].v *= -0.5;
    }
  }
}

// ========== 主循环 ==========
void updatePhysics() {
  // 完整的 FLIP 算法步骤
  particlesToGrid();           // 1. 粒子->网格 (散布)
  addExternalForces();         // 2. 添加外力
  pressureProjection();        // 3. 压力投影（泊松求解）
  applyBoundaryConditions();   // 4. 边界条件
  gridToParticles();           // 5. 网格->粒子 (FLIP)
  advectParticles();           // 6. 粒子平流
}

// ========== 显示 ==========
void drawToOLED() {
  u8g2.clearBuffer();
  
  // 绘制粒子（每个粒子 2x2 像素）
  for (int p = 0; p < NUM_PARTICLES; p++) {
    int x = (int)(particles[p].x * 2);
    int y = (int)(particles[p].y * 2);
    if (x >= 0 && x < 64 && y >= 0 && y < 64) {
      u8g2.drawBox(x, y, 2, 2);
    }
  }
  
  // 信息栏
  u8g2.setCursor(66, 10);
  u8g2.print("P:");
  u8g2.print(NUM_PARTICLES);
  
  u8g2.setCursor(66, 20);
  u8g2.print("Gx:");
  u8g2.print((int)(gravityX * 10));
  
  u8g2.setCursor(66, 30);
  u8g2.print("Gy:");
  u8g2.print((int)(gravityY * 10));
  
  u8g2.setCursor(66, 50);
  u8g2.print("FLIP");
  
  u8g2.sendBuffer();
}

void loop() {
  updatePhysics();
  drawToOLED();
  delay(0);
}
