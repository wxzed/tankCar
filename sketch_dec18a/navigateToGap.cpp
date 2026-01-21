/*
 * TOF探头导航系统 - 寻找最大空洞并移动
 * 数据格式：32x20矩阵，0=无障碍物，1=有障碍物
 * 每层间隔：5cm
 * 角度间隔：2.4度
 * 
 * 注意：Arduino已自动包含基本数学函数（sin, radians等）
 */
#include "navigateToGap.h"
#include "sensordata.h"
#include "motor.h"
#include <Arduino.h>
#include <Adafruit_SSD1306.h>

// 外部声明OLED显示对象
extern Adafruit_SSD1306 display;
// Serial2已经在ESP32核心库中预定义，无需外部声明
// 数据矩阵定义 (32列 x 20行)
const int COLS = TOTAL_POINTS;
const int ROWS = GRID_ROWS;
const float ANGLE_STEP = 2.4;  // 每点角度间隔（度）
const float LAYER_HEIGHT = 5.0; // 每层高度间隔（cm）
const float CAR_WIDTH = 20.0;   // 车身宽度（cm）- 根据实际小车尺寸修改
const float CENTER_COL = (COLS - 1) / 2.0; // 中心列索引（15.5）
const int MIN_LAYERS_FOR_GAP = 5; // 空洞必须至少在这几层都存在才认为是有效空洞
const int START_LAYER = 0;        // 开始分析的层数（0表示从第一层开始）
const float MIN_GAP_EXTRA_DEPTH = 30.0; // 洞口后的最小有效深度（cm），不足则放弃该空洞
const float MIN_GAP_DEPTH = 40.0; // 最小空洞深度（cm），如果小于此值，尝试角度微调寻找更深的空洞
const float MIN_BACK_DEPTH = 10.0; // 洞口后方最小可接受深度（cm），不足则放弃该空洞
const float ANGLE_ADJUST_STEP = 10.0; // 角度微调步长（度），左右各尝试这个角度
const int MAX_ANGLE_ADJUSTMENTS = 2; // 最大角度微调次数（左右各几次）
#ifndef PI
#define PI 3.14159265358979323846
#endif

// 空洞结构体
struct Gap {
  int startCol;      // 起始列
  int endCol;        // 结束列
  int width;         // 宽度（列数）
  int totalArea;     // 总面积（所有层的空洞点数）
  float centerAngle; // 中心角度（度）
  float actualWidth; // 实际物理宽度（cm）- 在最远层的宽度
  float distance;    // 距离（cm）- 最远层的距离
  int maxRow;        // 空洞延伸到的最大层数
  int entranceRow;   // 洞口入口行（两边开始有障碍物的行）
  float entranceDist; // 洞口入口距离（cm）
  int leftObstacleRow;  // 左边障碍物开始的行
  int rightObstacleRow; // 右边障碍物开始的行
  float tiltAngle;     // 洞口倾斜角度（度），正数表示右边高，负数表示左边高
};

// 全局变量
Gap maxGap;
bool gapFound = false;

// 路径地图（用于标记路径，2表示路径）
int pathMap[ROWS][COLS];

// 导航状态机（枚举定义在头文件中）
NavigationState navState = NAV_IDLE;
float navForwardDist = 0.0;
float navTurnAngle = 0.0;
float navTurnBackAngle = 0.0;
static NavigationState lastLoggedNavState = NAV_IDLE; // 调试：记录上一次打印的状态
static bool moveToEntranceStarted = false; // 标记是否已经启动了前进到入口的动作
static bool pathCompleted = false; // 标记路径是否刚完成，需要在重新检测前延时

// 搜索模式相关变量
static int searchStep = 0;  // 搜索步骤：0=旋转, 1=检测, 2=旋转, 3=检测...
static float searchAngleStep = 30.0; // 每次搜索旋转的角度（度）
static float searchTotalAngle = 0.0;  // 累计旋转的总角度（度）
static float maxSearchAngle = 360.0;  // 最大搜索角度（度），360度表示旋转一圈
static bool searchDirectionLeft = true; // 搜索方向：true=左转，false=右转

// 搜索原因（枚举定义在头文件中）
static SearchReason currentSearchReason = SEARCH_NO_REASON; // 当前搜索原因

/*
 * 获取当前导航状态
 */
NavigationState getNavigationState() {
  return navState;
}

/*
 * 获取当前搜索原因
 */
SearchReason getSearchReason() {
  return currentSearchReason;
}

/*
 * 获取下一步执行步骤信息（用于屏幕显示）
 * 参数：step1/value1, step2/value2, step3/value3 - 最多3个步骤
 * step: "left", "right", "line" 或空字符串
 * value: 对应的角度（度）或距离（cm）
 */
void getNextStepInfo(char* step1, float* value1, char* step2, float* value2, char* step3, float* value3) {
  // 初始化
  strcpy(step1, "");
  strcpy(step2, "");
  strcpy(step3, "");
  *value1 = 0.0;
  *value2 = 0.0;
  *value3 = 0.0;
  
  NavigationState state = navState;
  
  if (!gapFound) {
    // 没有找到空洞，不显示步骤
    return;
  }
  
  // 如果角度偏差很小（小于5度），直接直行
  if (abs(maxGap.centerAngle) < 5.0) {
    strcpy(step1, "line");
    // 直行目标：到达洞口入口（与串口打印的执行步骤保持一致）
    *value1 = maxGap.entranceDist;
    return;
  }
  
  // 根据当前状态和下一步状态显示步骤
  if (state == NAV_TURN_TO_GAP) {
    // 当前正在转向空洞方向，下一步是前进
    if (navTurnAngle < 0) {
      strcpy(step1, "left");
      *value1 = abs(navTurnAngle);
    } else {
      strcpy(step1, "right");
      *value1 = navTurnAngle;
    }
    strcpy(step2, "line");
    *value2 = navForwardDist;
    if (abs(navTurnBackAngle) > 0.5) {
      if (navTurnBackAngle < 0) {
        strcpy(step3, "left");
        *value3 = abs(navTurnBackAngle);
      } else {
        strcpy(step3, "right");
        *value3 = navTurnBackAngle;
      }
    }
  } else if (state == NAV_MOVE_TO_ENTRANCE) {
    // 当前正在前进到入口，下一步是转回
    strcpy(step1, "line");
    *value1 = navForwardDist;
    if (abs(navTurnBackAngle) > 0.5) {
      if (navTurnBackAngle < 0) {
        strcpy(step2, "left");
        *value2 = abs(navTurnBackAngle);
      } else {
        strcpy(step2, "right");
        *value2 = navTurnBackAngle;
      }
    }
    // 不包含通过空洞的步骤，到达洞口入口后重新检测规划
  } else if (state == NAV_TURN_BACK) {
    // 当前正在转回，转回完成后重新检测规划
    if (abs(navTurnBackAngle) > 0.5) {
      if (navTurnBackAngle < 0) {
        strcpy(step1, "left");
        *value1 = abs(navTurnBackAngle);
      } else {
        strcpy(step1, "right");
        *value1 = navTurnBackAngle;
      }
    }
    // 不包含通过空洞的步骤
  } else if (state == NAV_IDLE || state == NAV_MOVE_THROUGH) {
    // 空闲或持续前进，显示完整路径
    if (abs(maxGap.centerAngle) >= 5.0) {
      // 第一步：转向
      if (maxGap.centerAngle < 0) {
        strcpy(step1, "left");
        *value1 = abs(maxGap.centerAngle);
      } else {
        strcpy(step1, "right");
        *value1 = maxGap.centerAngle;
      }
      // 第二步：前进到入口
      strcpy(step2, "line");
      *value2 = navForwardDist;
      // 第三步：转回（如果需要）
      if (abs(navTurnBackAngle) > 0.5) {
        if (navTurnBackAngle < 0) {
          strcpy(step3, "left");
          *value3 = abs(navTurnBackAngle);
        } else {
          strcpy(step3, "right");
          *value3 = navTurnBackAngle;
        }
      }
      // 不包含通过空洞的步骤，到达洞口入口后重新检测规划
    } else {
      // 直接直行到洞口入口
      strcpy(step1, "line");
      *value1 = maxGap.entranceDist;
    }
  }
}

void findLargestGap();
void navigateToGap();
void planAndDisplayPath(); 
void printPathMap();
void printExecutionSteps(float startDist, float startAngle, float entranceDist, float entranceAngle, float turnBackAngle);
void printZValuesSummary(); // 调试：打印z值统计
// void printCurrentScanMap();  // 调试：直接基于当前zValues生成并打印占据图（暂不打印）
void fillPointCloudGrid(); // 填充pointCloudGrid数据
void stopMotors(); 
void moveForward();
void moveForwardDistance(float distance);
void turnLeft(float angle);
void turnRight(float angle);
void initializePathMap();

/*
 * 调试：打印当前帧 zValues 的统计信息和部分列值
 */
void printZValuesSummary() {
  int minVal = 1000000;
  int maxVal = -1000000;
  int zeroCount = 0;
  for (int i = 0; i < TOTAL_POINTS; i++) {
    int v = zValues[i];
    if (v == 0) zeroCount++;
    if (v < minVal) minVal = v;
    if (v > maxVal) maxVal = v;
  }
  Serial.print("【zValues统计】min=");
  Serial.print(minVal);
  Serial.print(" mm, max=");
  Serial.print(maxVal);
  Serial.print(" mm, zeroCount=");
  Serial.print(zeroCount);
  Serial.print("/");
  Serial.println(TOTAL_POINTS);

  // 打印中间若干列的值，帮助判断视野中心的距离
  int center = TOTAL_POINTS / 2;
  int span = 4; // 打印中心左右各4个点
  Serial.print("中心列距离(mm): ");
  for (int i = center - span; i <= center + span; i++) {
    if (i < 0 || i >= TOTAL_POINTS) continue;
    Serial.print(zValues[i]);
    Serial.print(" ");
  }
  Serial.println();
}

/* 
 * 填充全局 pointCloudGrid 数据
 */
void fillPointCloudGrid() {
  for (int row = 0; row < ROWS; row++) {
      int threshold = row * ROW_INTERVAL_CM * 10; 
      for (int col = 0; col < COLS; col++) {

       pointCloudGrid[row][col] = (zValues[col] <= threshold) ? 1 : 0;

      }
  }
} 

/*
 * 检查是否有近距离障碍物（碰撞检测）
 * 参数：thresholdMm - 阈值（毫米），小于此距离认为有碰撞风险
 *       startCol - 起始列索引
 *       endCol - 结束列索引
 * 返回：true表示检测到碰撞风险，false表示安全
 * 注意：只检测中间几列，避免侧边障碍物误判
 */
bool checkCollisionRisk(int thresholdMm, int startCol, int endCol) {
  // 限制在有效范围内
  if (startCol < 0) startCol = 0;
  if (endCol >= TOTAL_POINTS) endCol = TOTAL_POINTS - 1;
  if (startCol > endCol) return false;
  
  int count = 0;
  // 只检测指定范围内的列（通常是中间几列）
  for (int i = startCol; i <= endCol; i++) {
    if (zValues[i] > 0 && zValues[i] < thresholdMm) {
      count++;
    }
  }
  
  // 根据检测范围调整阈值：范围越小，需要的点数越少
  int requiredCount = 3; // 中间列只需要3个点即可确认
  if (endCol - startCol + 1 >= TOTAL_POINTS / 2) {
    requiredCount = 5; // 如果检测范围较大，需要5个点
  }
  
  return count >= requiredCount;
}

void runGapTest()  {
  // 允许在以下状态重新规划：
  // 1. NAV_IDLE - 空闲状态
  // 2. NAV_MOVE_THROUGH - 通过空洞状态（持续前进中）
  // 3. NAV_SEARCHING - 搜索模式（在检测步骤时）
  // 注意：在 NAV_TURN_TO_GAP 和 NAV_MOVE_TO_ENTRANCE 期间不重新规划，避免中断动作
  
  if (navState != NAV_IDLE && navState != NAV_MOVE_THROUGH && navState != NAV_SEARCHING) {
    return;
  }
  
  // 在搜索模式下，只在检测步骤（奇数步骤）时才检测
  if (navState == NAV_SEARCHING && searchStep % 2 == 0) {
    return; // 跳过，还在转向中
  }
  
  fillPointCloudGrid();
  
  // 分析TOF数据，找到最大空洞
  findLargestGap();
  bool gapExecutable = true; // 默认允许执行导航（但始终会标注路径）

  // 检查洞口后方的有效深度，如果不足，仍标注路径，但不执行导航
  // （已去除：洞口后深度不足的额外规则，只保留后方可用深度判断）

  // 检查洞口后方的可用深度，如果不足，仍标注路径，但不执行导航
  if (gapFound) {
    float backDepth = maxGap.distance - maxGap.entranceDist;
    if (backDepth < MIN_BACK_DEPTH) {
      Serial.print("【放弃空洞】洞口后方深度仅 ");
      Serial.print(backDepth, 1);
      Serial.println(" cm，低于要求，继续寻找其他空洞");
      Serial.println("【放弃前的空洞信息】");
      Serial.print("  宽度=");
      Serial.print(maxGap.actualWidth, 1);
      Serial.print("cm, 总深度=");
      Serial.print(maxGap.distance, 1);
      Serial.print("cm, 入口距离=");
      Serial.print(maxGap.entranceDist, 1);
      Serial.print("cm, 角度=");
      Serial.print(maxGap.centerAngle, 1);
      Serial.println("度");
      gapExecutable = false;          // 不执行导航，但继续标注路径
      currentSearchReason = SEARCH_NO_GAP;
    Serial.print("【原因】洞口后方可用深度不足，阈值=");
    Serial.print(MIN_BACK_DEPTH, 1);
    Serial.println(" cm");
    }
  }
  
  // 如果找到空洞但深度不够，尝试角度微调寻找更深的空洞
  if (gapFound && maxGap.distance < MIN_GAP_DEPTH) {
    Serial.print("【深度不足】当前空洞深度仅");
    Serial.print(maxGap.distance, 1);
    Serial.print("cm，尝试角度微调寻找更深的空洞...\n");
    
    Gap bestGap = maxGap; // 保存当前最佳空洞
    bool foundBetter = false;
    
    // 尝试左右微调角度
    for (int adjust = 1; adjust <= MAX_ANGLE_ADJUSTMENTS; adjust++) {
      // 计算角度偏移对应的列偏移
      float angleOffset = adjust * ANGLE_ADJUST_STEP;
      int colOffset = (int)(angleOffset / ANGLE_STEP);
      
      // 尝试左偏和右偏
      for (int direction = -1; direction <= 1; direction += 2) {
        if (direction == 0) continue; // 跳过0（已经检测过了）
        
        // 创建偏移后的pointCloudGrid（通过列偏移模拟角度调整）
        // 注意：这里我们通过调整列索引范围来模拟旋转，而不是真的旋转数据
        // 实际上，我们需要在findLargestGap中传入列偏移参数
        // 但为了简单，我们可以创建一个临时函数来查找偏移后的空洞
        
        // 临时保存原始数据
        int tempPointCloudGrid[ROWS][COLS];
        for (int r = 0; r < ROWS; r++) {
          for (int c = 0; c < COLS; c++) {
            tempPointCloudGrid[r][c] = pointCloudGrid[r][c];
          }
        }
        
        // 应用列偏移（循环移位）
        for (int r = 0; r < ROWS; r++) {
          int tempRow[COLS];
          for (int c = 0; c < COLS; c++) {
            int srcCol = c - direction * colOffset;
            // 处理边界：循环移位
            while (srcCol < 0) srcCol += COLS;
            while (srcCol >= COLS) srcCol -= COLS;
            tempRow[c] = pointCloudGrid[r][srcCol];
          }
          for (int c = 0; c < COLS; c++) {
            pointCloudGrid[r][c] = tempRow[c];
          }
        }
        
        // 在偏移后的数据中查找空洞
        findLargestGap();
        
        // 如果找到更深的空洞，保存它（在恢复数据之前）
        if (gapFound && maxGap.distance > bestGap.distance) {
          Gap tempGap = maxGap; // 保存当前找到的空洞
          // 调整角度：需要加上偏移角度
          tempGap.centerAngle += direction * angleOffset;
          bestGap = tempGap;
          foundBetter = true;
          Serial.print("  【找到更深空洞】");
          Serial.print("深度=");
          Serial.print(tempGap.distance, 1);
          Serial.print("cm, 角度=");
          Serial.print(tempGap.centerAngle, 1);
          Serial.print("度 (偏移");
          Serial.print(direction * angleOffset, 1);
          Serial.println("度)");
        }
        
        // 恢复原始数据
        for (int r = 0; r < ROWS; r++) {
          for (int c = 0; c < COLS; c++) {
            pointCloudGrid[r][c] = tempPointCloudGrid[r][c];
          }
        }
      }
    }
    
    // 如果找到更好的空洞，使用它；否则使用原始空洞
    maxGap = bestGap;
    gapFound = true;
    if (foundBetter) {
      Serial.println("【使用角度微调后的更深空洞】");
    } else {
      Serial.println("【未找到更深的空洞，使用原始空洞】");
    }
  }
  
  if (gapFound) {
    // 如果之前是在 NAV_MOVE_THROUGH 状态（直行通过空洞），标记路径完成
    // 这样在重新规划前会停止5秒
    if (navState == NAV_MOVE_THROUGH) {
      pathCompleted = true;
    }
    
    Serial.println("\n========== 找到最佳空洞 ==========");
    Serial.print("宽度=");
    Serial.print(maxGap.actualWidth, 1);
    Serial.print("cm, 深度=");
    Serial.print(maxGap.distance, 1);
    Serial.print("cm, 角度=");
    Serial.print(maxGap.centerAngle, 1);
    Serial.println("度");
    Serial.print("洞口后深度=");
    Serial.print(maxGap.distance - maxGap.entranceDist, 1);
    Serial.println("cm");
    
    // 规划并显示路径（无论是否执行，都标注路径，内部已打印一次路径地图）
    planAndDisplayPath();
    
    // 如果本次不执行导航，仅保留路径标注，继续搜索
    if (!gapExecutable) {
      Serial.println("【仅标注路径，不执行导航，继续搜索】");
      gapFound = false;
      navState = NAV_SEARCHING;
      searchStep = 0;
      searchTotalAngle = 0.0;
      currentSearchReason = SEARCH_NO_GAP;
      return;
    }
    
    // 初始化导航状态机并执行
    navigateToGap();
    
    // 如果之前在搜索模式，退出搜索模式
    if (navState == NAV_SEARCHING) {
      searchStep = 0; // 重置搜索步骤
      searchTotalAngle = 0.0; // 重置累计角度
      currentSearchReason = SEARCH_NO_REASON; // 清除搜索原因
    }
  } else {
    // 未找到空洞
    if (navState == NAV_IDLE) {
      // 如果是在空闲状态未找到，进入搜索模式
      Serial.println("【未找到合适的空洞，进入搜索模式】");
      // 打印当前地图，帮助调试
      Serial.println("【当前地图状态】");
      printPathMap();
      navState = NAV_SEARCHING;
      searchStep = 0; // 从第一步开始：旋转
      searchTotalAngle = 0.0; // 重置累计角度
      currentSearchReason = SEARCH_NO_GAP; // 设置搜索原因：未找到空洞
      // 屏幕显示会在主循环的updateDisplay()中自动更新
    } else if (navState == NAV_SEARCHING) {
      // 如果已经在搜索模式，继续下一步搜索
      // searchStep 会在 updateNavigation() 中更新
    } else {
      // 其他状态（如 NAV_MOVE_THROUGH），保持当前状态
    }
  }
}

/*
 * 计算空洞的实际物理宽度（在最远层）
 * 参数：起始列、结束列、最大层数
 * 返回：实际宽度（cm）
 */
float calculateActualWidth(int startCol, int endCol, int maxRow) {
  // 计算空洞的角度范围（度）
  float angleRange = (endCol - startCol + 1) * ANGLE_STEP;
  
  // 计算最远层的距离（cm）
  float dist = (maxRow + 1) * LAYER_HEIGHT;
  
  // 计算实际物理宽度
  // 使用正弦函数计算：宽度 = 2 * 距离 * sin(角度范围/2)
  float angleRad = radians(angleRange / 2.0);
  float actualWidth = 2.0 * dist * sin(angleRad);
  
  return actualWidth;
}

/*
 * 查找空洞的最大延伸层数（连续的空洞层数）
 * 参数：起始列、结束列
 * 返回：最大层数（0-ROWS-1），返回-1表示该列范围内没有完整的空洞
 * 注意：检查从START_LAYER开始，连续多少层该列范围内都是0
 */
int findMaxRowForGap(int startCol, int endCol) {
  int maxRow = -1;
  bool foundObstacle = false;
  
  for (int row = START_LAYER; row < ROWS; row++) {
    // 检查这一层中，指定列范围内的所有列是否都是0
    bool allZero = true;
    for (int col = startCol; col <= endCol; col++) {
      if (pointCloudGrid[row][col] != 0) {
        allZero = false;
        foundObstacle = true;
        break;
      }
    }
    
    // 只有当所有列都是0时，才认为这一层有空洞
    if (allZero) {
      maxRow = row;
    } else {
      // 如果遇到障碍物，停止查找（空洞必须是连续的）
      // 但如果之前已经找到过空洞，就返回之前的结果
      break;
    }
  }
  
  return maxRow;
}

/*
 * 查找左边障碍物开始的行
 * 参数：起始列
 * 返回：左边障碍物开始的行（0-ROWS-1），返回-1表示没有找到
 */
int findLeftObstacleRow(int startCol) {
  if (startCol <= 0) return -1;
  
  for (int row = START_LAYER; row < ROWS; row++) {
    if (pointCloudGrid[row][startCol - 1] != 0) {
      return row;
    }
  }
  return -1;
}

/*
 * 查找右边障碍物开始的行
 * 参数：结束列
 * 返回：右边障碍物开始的行（0-ROWS-1），返回-1表示没有找到
 */
int findRightObstacleRow(int endCol) {
  if (endCol >= COLS - 1) return -1;
  
  for (int row = START_LAYER; row < ROWS; row++) {
    if (pointCloudGrid[row][endCol + 1] != 0) {
      return row;
    }
  }
  return -1;
}

/*
 * 查找洞口入口行（两边开始有障碍物的行）
 * 参数：起始列、结束列
 * 返回：洞口入口行（0-ROWS-1），返回-1表示没有找到洞口入口
 * 注意：从START_LAYER开始查找，找到第一个两边有障碍物的行
 */
int findEntranceRowForGap(int startCol, int endCol) {
  for (int row = START_LAYER; row < ROWS; row++) {
    // 检查这一层中，空洞范围内是否都是0
    bool allZero = true;
    for (int col = startCol; col <= endCol; col++) {
      if (pointCloudGrid[row][col] != 0) {
        allZero = false;
        break;
      }
    }
    
    // 如果这一层空洞范围内都是0，检查两边是否有障碍物
    if (allZero) {
      // 检查左边是否有障碍物（startCol-1列）
      bool leftHasObstacle = false;
      if (startCol > 0 && pointCloudGrid[row][startCol - 1] != 0) {
        leftHasObstacle = true;
      }
      
      // 检查右边是否有障碍物（endCol+1列）
      bool rightHasObstacle = false;
      if (endCol < COLS - 1 && pointCloudGrid[row][endCol + 1] != 0) {
        rightHasObstacle = true;
      }
      
      // 如果两边都有障碍物，或者至少一边有障碍物，说明这是洞口入口
      if (leftHasObstacle || rightHasObstacle) {
        return row;
      }
    }
  }
  
  // 如果没找到两边有障碍物的行，返回空洞开始的行（START_LAYER）
  return START_LAYER;
}

/*
 * 计算洞口倾斜角度
 * 根据左右两边障碍物开始的行差计算倾斜角度
 * 参数：左边障碍物行、右边障碍物行、空洞中心列
 * 返回：倾斜角度（度），正数表示右边高（需要左转），负数表示左边高（需要右转）
 */
float calculateGapTiltAngle(int leftRow, int rightRow, float centerCol) {
  // 如果两边都没有障碍物，返回0（无倾斜）
  if (leftRow < 0 && rightRow < 0) {
    return 0.0;
  }
  
  // 如果只有一边有障碍物，无法判断倾斜，返回0
  if (leftRow < 0 || rightRow < 0) {
    return 0.0;
  }
  
  // 计算行差（行数越大，距离越远）
  int rowDiff = rightRow - leftRow;
  
  // 如果行差为0，说明洞口是水平的，无倾斜
  if (rowDiff == 0) {
    return 0.0;
  }
  
  // 计算左右两边的距离（cm）
  float leftDist = (leftRow + 1) * LAYER_HEIGHT;
  float rightDist = (rightRow + 1) * LAYER_HEIGHT;
  float distDiff = rightDist - leftDist;
  
  // 计算空洞的宽度（角度范围对应的物理宽度）
  // 使用洞口入口的平均距离
  float avgDist = (leftDist + rightDist) / 2.0;
  float gapAngleRange = (maxGap.endCol - maxGap.startCol + 1) * ANGLE_STEP;
  float gapWidthCm = 2.0 * avgDist * sin(radians(gapAngleRange / 2.0));
  
  // 计算倾斜角度：atan(高度差/宽度)
  float tiltAngle = atan(distDiff / gapWidthCm) * 180.0 / PI;
  
  // 取反：从小车视角
  float finalTiltAngle = -tiltAngle;
  
  return finalTiltAngle;
}

/*
 * 检查空洞在指定列范围内，在多少层中存在
 * 参数：起始列、结束列
 * 返回：存在的层数
 */
int countLayersWithGap(int startCol, int endCol) {
  int layerCount = 0;
  for (int row = START_LAYER; row < ROWS; row++) {
    bool hasGap = true;
    for (int col = startCol; col <= endCol; col++) {
      if (pointCloudGrid[row][col] != 0) {
        hasGap = false;
        break;
      }
    }
    if (hasGap) {
      layerCount++;
    }
  }
  return layerCount;
}

/*
 * 查找最大空洞（宽度大于车身宽度）
 * 算法：枚举所有可能的列范围，检查在多少层中存在，选择最大的
 */
void findLargestGap() {
  gapFound = false;
  maxGap.totalArea = 0;
  maxGap.actualWidth = 0;
  
  // 枚举所有可能的列范围（起始列和结束列）
  for (int startCol = 0; startCol < COLS; startCol++) {
    for (int endCol = startCol; endCol < COLS; endCol++) {
      int width = endCol - startCol + 1;
      
      // 检查这个列范围在多少层中存在（完全为空）
      int layerCount = countLayersWithGap(startCol, endCol);
      
      // 只考虑在足够多层都存在的空洞
      if (layerCount >= MIN_LAYERS_FOR_GAP) {
        // 查找这个空洞的最大延伸层数
        int maxRow = findMaxRowForGap(startCol, endCol);
        
        if (maxRow >= 0) {
          // 计算实际物理宽度
          float actualWidth = calculateActualWidth(startCol, endCol, maxRow);
          
          // 只考虑宽度大于车身宽度的空洞
          if (actualWidth > CAR_WIDTH) {
            // 计算总面积（所有层中的空洞点数）
            int totalArea = 0;
            for (int r = START_LAYER; r <= maxRow; r++) {
              for (int c = startCol; c <= endCol; c++) {
                if (pointCloudGrid[r][c] == 0) {
                  totalArea++;
                }
              }
            }
            
            // 优先选择深度最深（maxRow最大）的空洞
            // 如果深度相同，则选择宽度最大的
            // 如果深度和宽度都相同，则选择面积最大的
            bool isBetter = false;
            if (!gapFound) {
              isBetter = true;
            } else if (maxRow > maxGap.maxRow) {
              // 深度更深，优先选择
              isBetter = true;
            } else if (maxRow == maxGap.maxRow) {
              // 深度相同，选择宽度更大的
              if (actualWidth > maxGap.actualWidth) {
                isBetter = true;
              } else if (actualWidth == maxGap.actualWidth && totalArea > maxGap.totalArea) {
                // 宽度也相同，选择面积更大的
                isBetter = true;
              }
            }
            
            if (isBetter) {
              maxGap.startCol = startCol;
              maxGap.endCol = endCol;
              maxGap.width = width;
              maxGap.totalArea = totalArea;
              maxGap.actualWidth = actualWidth;
              maxGap.maxRow = maxRow;
              maxGap.distance = (maxRow + 1) * LAYER_HEIGHT;
              // 查找洞口入口位置
              maxGap.entranceRow = findEntranceRowForGap(startCol, endCol);
              
              // 确保入口行不会大于等于洞底行
              // 如果入口行大于等于洞底行，使用洞底行减1（或者使用START_LAYER）
              if (maxGap.entranceRow >= maxRow) {
                // 如果入口行等于或大于洞底行，说明没有找到合适的入口
                // 使用洞底行减1，或者使用START_LAYER
                if (maxRow > 0) {
                  maxGap.entranceRow = maxRow - 1;  // 使用洞底前一行作为入口
                } else {
                  maxGap.entranceRow = START_LAYER;  // 如果洞底就是第一行，使用START_LAYER
                }
              }
              
              maxGap.entranceDist = (maxGap.entranceRow + 1) * LAYER_HEIGHT;
              
              // 洞口后可用深度：洞底距离 - 入口距离
              float backDepth = maxGap.distance - maxGap.entranceDist;
              if (backDepth < MIN_BACK_DEPTH) {
                // 洞口后没有足够深度，放弃这个空洞
                continue;
              }
              // 查找左右障碍物行，用于计算倾斜角度
              maxGap.leftObstacleRow = findLeftObstacleRow(startCol);
              maxGap.rightObstacleRow = findRightObstacleRow(endCol);
              // 计算洞口倾斜角度
              float gapCenterCol = (startCol + endCol) / 2.0;
              maxGap.tiltAngle = calculateGapTiltAngle(maxGap.leftObstacleRow, maxGap.rightObstacleRow, gapCenterCol);
              
              
              gapFound = true;
            }
          }
        }
      }
    }
  }
  
  // 计算中心角度
  if (gapFound) {
    // 中心列（相对于中心点）
    float centerCol = (maxGap.startCol + maxGap.endCol) / 2.0;
    // 计算角度：中心列相对于中心点的偏移
    // 列>16是左边（负角度，左转），列<16是右边（正角度，右转）
    float offsetFromCenter = centerCol - CENTER_COL;
    maxGap.centerAngle = -offsetFromCenter * ANGLE_STEP;
  }
}

/*
 * 初始化路径地图（复制TOF数据）
 */
void initializePathMap() {
  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      pathMap[row][col] = pointCloudGrid[row][col];
    }
  }
}

/*
 * 根据距离和角度计算对应的行列位置
 * 参数：distance - 距离（cm），angle - 角度（度，相对于正前方）
 * 返回：通过引用返回行和列
 */
void calculatePosition(float distance, float angle, int& row, int& col) {
  // 计算行（基于距离）
  // 距离5cm对应第0行，距离10cm对应第1行，以此类推
  // 公式：row = (distance / LAYER_HEIGHT) - 1
  row = (int)(distance / LAYER_HEIGHT) - 1;
  if (row < 0) row = 0;
  if (row >= ROWS) row = ROWS - 1;
  
  // 计算列（基于角度）
  // 中心列是15.5，角度0度对应中心列
  // 角度为正（右转）对应列增加（右边），角度为负（左转）对应列减少（左边）
  // 注意：列<16是右边，列>16是左边
  float colOffset = angle / ANGLE_STEP;
  float colFloat = CENTER_COL + colOffset;
  col = (int)round(colFloat);
  
  // 限制在有效范围内
  if (col < 0) col = 0;
  if (col >= COLS) col = COLS - 1;
}

/*
 * 计算在指定距离处，车身宽度对应的列数范围
 * 参数：distance - 距离（cm），centerCol - 中心列索引
 * 返回：通过引用返回起始列和结束列
 */
void calculateCarWidthColumns(float distance, int centerCol, int& startCol, int& endCol) {
  // 在距离d处，车身宽度CAR_WIDTH对应的角度范围
  // 使用公式：角度 = 2 * atan(宽度/(2*距离))
  // 距离越远，相同角度间隔对应的物理宽度越大，所以车身宽度对应的列数会减少
  
  // 对于非常近的距离，使用最小距离限制，避免角度过大
  // 但不要设置太大，否则会限制正常计算
  const float MIN_DISTANCE = 15.0;  // 最小距离15cm（避免在太近时标记过多列）
  if (distance < MIN_DISTANCE) {
    distance = MIN_DISTANCE;
  }
  
  // 计算车身宽度对应的角度（弧度）
  float angleRad = 2.0 * atan(CAR_WIDTH / (2.0 * distance));
  float angleDeg = angleRad * 180.0 / PI;
  
  // 计算列数范围（向上取整，确保覆盖整个车身宽度）
  // 距离越远，角度越小，列数越少
  float colRange = angleDeg / ANGLE_STEP;
  int halfCols = (int)ceil(colRange / 2.0);
  
  // 根据距离动态设置上限，距离越近上限越大，距离越远上限越小
  // 这样可以避免在近距离时标记过多列，同时保证远距离时列数能自然减少
  int maxHalfCols;
  if (distance < 20.0) {
    maxHalfCols = 6;  // 距离<20cm，最多每边6列（总共13列）
  } else if (distance < 40.0) {
    maxHalfCols = 5;  // 距离20-40cm，最多每边5列（总共11列）
  } else if (distance < 60.0) {
    maxHalfCols = 4;  // 距离40-60cm，最多每边4列（总共9列）
  } else if (distance < 80.0) {
    maxHalfCols = 3;  // 距离60-80cm，最多每边3列（总共7列）
  } else {
    maxHalfCols = 2;  // 距离>=80cm，最多每边2列（总共5列）
  }
  
  if (halfCols > maxHalfCols) {
    halfCols = maxHalfCols;
  }
  
  // 确保至少标记1列（中心列本身）
  if (halfCols < 1) {
    halfCols = 1;
  }
  
  startCol = centerCol - halfCols;
  endCol = centerCol + halfCols;
  
  // 限制在有效范围内
  if (startCol < 0) startCol = 0;
  if (endCol >= COLS) endCol = COLS - 1;
}

/*
 * 检查路径中心点两侧是否有足够空间容纳车身宽度
 * 参数：row - 行索引，centerCol - 中心列索引，distance - 距离（cm）
 * 返回：true表示有足够空间，false表示空间不足
 */
bool checkPathWidth(int row, int centerCol, float distance) {
  if (row < 0 || row >= ROWS || centerCol < 0 || centerCol >= COLS) {
    return false;
  }
  
  // 计算车身宽度对应的列范围
  int startCol, endCol;
  calculateCarWidthColumns(distance, centerCol, startCol, endCol);
  
  // 检查该列范围内是否都是无障碍物
  for (int col = startCol; col <= endCol; col++) {
    if (col < 0 || col >= COLS) {
      return false;  // 超出边界
    }
    if (pointCloudGrid[row][col] != 0) {
      return false;  // 有障碍物
    }
  }
  
  return true;  // 所有点都是无障碍物
}

/*
 * 标记路径上的点（从起点到终点）
 * 参数：startDist - 起点距离（cm），startAngle - 起点角度（度）
 *      endDist - 终点距离（cm），endAngle - 终点角度（度）
 */
void markPathSegment(float startDist, float startAngle, float endDist, float endAngle) {
  // 使用插值方法标记路径，确保路径连续
  // 计算路径上的多个中间点
  // 根据距离差计算步数，确保每层至少有一个点
  float distDiff = abs(endDist - startDist);
  int steps = max(ROWS, (int)(distDiff / LAYER_HEIGHT * 2));  // 每层至少2个点
  if (steps > 200) steps = 200;  // 最多200个点，避免过度计算
  
  // 计算空洞中心列（用于在空洞范围内保持路径在中心）
  float gapCenterCol = (maxGap.startCol + maxGap.endCol) / 2.0;
  
  // 记录已标记的行列组合，用于去重
  bool marked[ROWS][COLS];
  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      marked[r][c] = false;
    }
  }
  
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / (float)steps;
    float dist = startDist + (endDist - startDist) * t;
    
    // 使用角度插值计算路径点
    float angle = startAngle + (endAngle - startAngle) * t;
    int row, centerCol;
    calculatePosition(dist, angle, row, centerCol);
    
    // 检查当前距离是否在空洞范围内
    bool inGapRange = (dist >= maxGap.entranceDist && dist <= maxGap.distance);
    
    // 如果在空洞范围内，确保路径在空洞中心列
    if (inGapRange) {
      // 强制设置为空洞中心列，确保路径在两边障碍物的中间
      centerCol = (int)round(gapCenterCol);
      if (centerCol < 0) centerCol = 0;
      if (centerCol >= COLS) centerCol = COLS - 1;
      
      // 重新计算行（基于距离）
      row = (int)(dist / LAYER_HEIGHT) - 1;
      if (row < 0) row = 0;
      if (row >= ROWS) row = ROWS - 1;
    }
    
    // 检查路径中心点两侧是否有足够空间容纳车身宽度
    if (!checkPathWidth(row, centerCol, dist)) {
      // 如果空间不足，跳过这个点（不标记路径）
      continue;
    }
    
    // 计算车身宽度对应的列范围
    int startCol, endCol;
    calculateCarWidthColumns(dist, centerCol, startCol, endCol);
    
    // 标记车身宽度范围内的所有点
    for (int col = startCol; col <= endCol; col++) {
      if (row >= 0 && row < ROWS && col >= 0 && col < COLS) {
        // 只标记无障碍物的点，且避免重复标记
        if (pointCloudGrid[row][col] == 0 && !marked[row][col]) {
          pathMap[row][col] = 2;
          marked[row][col] = true;
        }
      }
    }
  }
}

/*
 * 规划路径并标记在地图上
 */
void planAndDisplayPath() {
  if (!gapFound) return;
  
  // 重新初始化路径地图
  initializePathMap();
  
  // 起点：小车当前位置（中心列，距离5cm）
  float startDist = LAYER_HEIGHT;
  float startAngle = 0.0;  // 正前方
  
  // 计算空洞中心位置和角度（用于路径地图）
  // 路径地图需要直接使用列偏移，不取反
  float gapCenterCol = (maxGap.startCol + maxGap.endCol) / 2.0;
  float gapCenterAngle = (gapCenterCol - CENTER_COL) * ANGLE_STEP;
  
  // 分段规划路径：只规划到洞口入口，不包含通过空洞的部分
  // 到达洞口入口后停止，重新检测规划下一条路径
  float entranceDist = maxGap.entranceDist;
  float entranceAngle = gapCenterAngle;  // 路径地图使用这个角度
  
  // 只标记到洞口入口的路径
  markPathSegment(startDist, startAngle, entranceDist, entranceAngle);
  
  // 计算转回角度：转到与洞口垂直的方向（考虑倾斜角度）
  // 转回角度 = -entranceAngle + tiltAngle
  float executionTurnBackAngle = -maxGap.centerAngle - maxGap.tiltAngle;
  // 只打印到洞口入口的步骤，不包含通过空洞
  printExecutionSteps(startDist, startAngle, entranceDist, maxGap.centerAngle, executionTurnBackAngle);
  
  // 打印地图
  printPathMap(); // 保持一次打印
}

/*
 * 分析路径并打印实际执行步骤
 */
void printExecutionSteps(float startDist, float startAngle, float entranceDist, float entranceAngle, 
                         float turnBackAngle) {
  Serial.println("\n【执行步骤】");
  
  // 显示倾斜角度信息
  // 注意：tiltAngle的符号：右边高时应该是负数（需要左转），左边高时应该是正数（需要右转）
  // 但当前代码中，如果右边高，tiltAngle可能是正数，需要检查
  if (abs(maxGap.tiltAngle) > 0.5) {
    Serial.print("  洞口倾斜角度: ");
    // 如果tiltAngle > 0，说明是正数，但从计算逻辑看，右边高应该是负数
    // 所以这里需要根据实际情况判断：如果tiltAngle > 0，可能是左边高（需要右转）
    // 如果tiltAngle < 0，可能是右边高（需要左转）
    if (maxGap.tiltAngle < 0) {
      Serial.print("右边高 ");
    } else {
      Serial.print("左边高 ");
    }
    Serial.print(abs(maxGap.tiltAngle), 1);
    Serial.println(" 度");
  }
  
  // 如果角度偏差很小（小于5度），直接直行到洞口入口
  if (abs(entranceAngle) < 5.0) {
    float distToEntrance = entranceDist - startDist;
    Serial.print("  步骤1: 直行 ");
    Serial.print(distToEntrance, 1);
    Serial.println(" cm (到达洞口入口)");
    return;
  }
  
  // 步骤1：转向空洞中心方向
  int stepNum = 1;
  if (entranceAngle < 0) {
    Serial.print("  步骤");
    Serial.print(stepNum);
    Serial.print(": 左转 ");
    Serial.print(abs(entranceAngle), 1);
    Serial.println(" 度 (朝向空洞中心)");
  } else {
    Serial.print("  步骤");
    Serial.print(stepNum);
    Serial.print(": 右转 ");
    Serial.print(entranceAngle, 1);
    Serial.println(" 度 (朝向空洞中心)");
  }
  
  // 步骤2：直行到洞口入口
  stepNum++;
  float distToEntrance = entranceDist - startDist;
  Serial.print("  步骤");
  Serial.print(stepNum);
  Serial.print(": 直行 ");
  Serial.print(distToEntrance, 1);
  Serial.println(" cm (到达洞口入口)");
  
  // 步骤3：转到与洞口垂直的方向（考虑倾斜角度）
  if (abs(turnBackAngle) > 0.5) {  // 如果角度大于0.5度，需要转回
    stepNum++;
    if (turnBackAngle < 0) {
      Serial.print("  步骤");
      Serial.print(stepNum);
      Serial.print(": 左转 ");
      Serial.print(abs(turnBackAngle), 1);
      Serial.print(" 度 (转到与洞口垂直");
      if (abs(maxGap.tiltAngle) > 0.5) {
        Serial.print("，考虑倾斜");
      }
      Serial.println(")");
    } else {
      Serial.print("  步骤");
      Serial.print(stepNum);
      Serial.print(": 右转 ");
      Serial.print(turnBackAngle, 1);
      Serial.print(" 度 (转到与洞口垂直");
      if (abs(maxGap.tiltAngle) > 0.5) {
        Serial.print("，考虑倾斜");
      }
      Serial.println(")");
    }
  }
  
  // 说明：当前策略在到达洞口并转正后会重新规划，不继续盲目穿过空洞
  Serial.println("  (到达洞口后将重新规划下一步，而不是直接穿过空洞)");
  Serial.println();
}

/*
 * 打印包含路径的地图
 * Serial输出文本格式（调试用）
 * Serial2输出二进制格式（专用串口）：帧头0xF5 + 数据(0x00/0x01/0x02) + 帧尾0xAF
 */
void printPathMap() {
  // Serial输出文本格式（调试用）
  Serial.println("\n【路径地图】");
  Serial.println("图例: 0=无障碍物, 1=障碍物, 2=小车路径");
  Serial.println("     行号=距离层数, 列号=角度方向(32列覆盖约76.8度)");
  Serial.println();
  
  // 打印列号（角度方向）- 简化显示，每5列显示一次
  Serial.print("行\\列 ");
  for (int col = 0; col < COLS; col++) {
    if (col % 5 == 0) {
      if (col < 10) {
        Serial.print(" ");
      }
      Serial.print(col);
    } else {
      Serial.print("  ");
    }
  }
  Serial.println();
  
  // 打印地图内容（文本格式）
  for (int row = 0; row < ROWS; row++) {
    // 打印行号
    if (row < 9) {
      Serial.print(" ");
    }
    Serial.print(row + 1);
    Serial.print("   ");
    
    // 打印该行的数据
    for (int col = 0; col < COLS; col++) {
      Serial.print(pathMap[row][col]);
      Serial.print(" ");
    }
    Serial.print("  (距离: ");
    Serial.print((row + 1) * LAYER_HEIGHT, 1);
    Serial.print("cm)");
    Serial.println();
  }
  
  Serial.println();
  
  // Serial2输出二进制格式（专用串口）
  // 帧头
  Serial2.write(0xF5);
  
  // 输出路径地图数据：0->0x00, 1->0x01, 2->0x02
  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      uint8_t data = pathMap[row][col];
      // 确保数据在有效范围内（0, 1, 2）
      if (data > 2) {
        data = 0;
      }
      Serial2.write(data);
    }
  }
  
  // 帧尾
  Serial2.write(0xAF);
}

/*
 * 计算到达空洞前方合适位置的直行距离
 */
float calculateForwardDistance(float angle) {
  const float SAFE_DISTANCE = 30.0;  // 转回正前方后，希望距离洞口的安全距离（cm）
  const float MIN_DISTANCE = 15.0;    // 最小直行距离15cm
  const float MAX_DISTANCE = 70.0;    // 最大直行距离70cm
  
  // 计算角度（弧度）
  float angleRad = radians(abs(angle));
  
  // 目标：转回正前方后，距离洞口还有SAFE_DISTANCE
  // 设直行距离为d，实际前进距离 = d * cos(angle)
  // 需要满足：maxGap.entranceDist - d * cos(angle) >= SAFE_DISTANCE
  // 因此：d <= (maxGap.entranceDist - SAFE_DISTANCE) / cos(angle)
  
  float maxAllowedDist = (maxGap.entranceDist - SAFE_DISTANCE) / cos(angleRad);
  
  // 如果角度很小，使用原来的70%策略（基于洞口入口距离）
  float baseDist = maxGap.entranceDist * 0.7;
  
  // 取两者中的较小值，确保不会太靠近洞口
  float forwardDist = min(maxAllowedDist, baseDist);
  
  // 应用最小和最大限制
  if (forwardDist < MIN_DISTANCE) {
    forwardDist = MIN_DISTANCE;
  } else if (forwardDist > MAX_DISTANCE) {
    forwardDist = MAX_DISTANCE;
  }
  
  return forwardDist;
}

/*
 * 导航到空洞方向（初始化导航状态机）
 */
void navigateToGap() {
  // 如果路径刚完成，先停止并延时5秒
  if (pathCompleted) {
    stopMotors();
    Serial.println("【路径执行完成】停止5秒，准备开始下一帧判断...");
    delay(5000);
    pathCompleted = false; // 清除标志
  }
  
  // 如果之前正在导航，先停止当前动作
  if (navState != NAV_IDLE && navState != NAV_MOVE_THROUGH) {
    stopMotors();
    delay(100);
  }
  
  if (!gapFound) {
    navState = NAV_IDLE;
    return;
  }
  
  // 如果角度偏差很小（小于5度），直接直行
  if (abs(maxGap.centerAngle) < 5.0) {
    navState = NAV_MOVE_THROUGH;
    if (!isMoving) {
      moveForward();
    }
    return;
  }
  
  // 计算直行距离
  navForwardDist = calculateForwardDistance(maxGap.centerAngle);
  
  // 计算转回角度：转回正前方（0度）
  navTurnBackAngle = 0.0 - maxGap.centerAngle;
  navTurnAngle = maxGap.centerAngle;
  
  // 初始化导航状态：开始转向空洞方向
  navState = NAV_TURN_TO_GAP;
  
  if (navTurnAngle < 0) {
    turnLeft(abs(navTurnAngle));
  } else {
    turnRight(navTurnAngle);
  }
}

/*
 * 更新导航状态机（需要在主循环中持续调用）
 */
void updateNavigation() {
  // 状态变更时打印一次（简化版）
  if (navState != lastLoggedNavState) {
    lastLoggedNavState = navState;
  }

  // 持续调用电机控制函数，让它们检查状态并完成动作
  if (navState == NAV_TURN_TO_GAP) {
    // 实时碰撞检测：即使在转向过程中也检测
    // 只检测中间列（正前方），避免侧边障碍物误判
    // 检测范围：中间1/6的列（很小的范围，只关注正中心）
    const int COLLISION_THRESHOLD_MM = 150; // 15cm = 150mm
    int centerStart = TOTAL_POINTS / 2 - TOTAL_POINTS / 12;  // 中心减去1/12
    int centerEnd = TOTAL_POINTS / 2 + TOTAL_POINTS / 12 - 1;  // 中心加上1/12（中间1/6的列）
    if (checkCollisionRisk(COLLISION_THRESHOLD_MM, centerStart, centerEnd)) {
      Serial.println("【警告】转向中检测到近距离障碍物，进入搜索模式！");
      stopMotors();
      navState = NAV_SEARCHING; // 进入搜索模式
      searchStep = 0;
      searchTotalAngle = 0.0; // 重置累计角度
      currentSearchReason = SEARCH_COLLISION; // 设置搜索原因：障碍物
      // 屏幕显示会在主循环的updateDisplay()中自动更新
      return;
    }
    
    // 持续调用转向函数，让它检查时间并完成转向
    if (navTurnAngle < 0) {
      turnLeft(abs(navTurnAngle));
    } else {
      turnRight(navTurnAngle);
    }
    // 检查转向是否完成
    if (!isTurning) {
      navState = NAV_MOVE_TO_ENTRANCE;
      // 确保移动标志已重置，准备启动前进
      if (isMoving) {
        stopMotors();
        delay(50);
      }
    }
  }
  else if (navState == NAV_MOVE_TO_ENTRANCE) {
    static bool wasMoving = false;
    
    // 实时碰撞检测：在前进到入口过程中检测
    // 只检测中间列（正前方），避免侧边障碍物误判
    // 检测范围：中间1/6的列（很小的范围，只关注正中心）
    const int COLLISION_THRESHOLD_MM = 150; // 15cm = 150mm
    int centerStart = TOTAL_POINTS / 2 - TOTAL_POINTS / 12;  // 中心减去1/12
    int centerEnd = TOTAL_POINTS / 2 + TOTAL_POINTS / 12 - 1;  // 中心加上1/12（中间1/6的列）
    if (checkCollisionRisk(COLLISION_THRESHOLD_MM, centerStart, centerEnd)) {
      Serial.println("【警告】前进中检测到近距离障碍物，进入搜索模式！");
      stopMotors();
      navState = NAV_SEARCHING; // 进入搜索模式
      searchStep = 0;
      searchTotalAngle = 0.0; // 重置累计角度
      currentSearchReason = SEARCH_COLLISION; // 设置搜索原因：障碍物
      // 屏幕显示会在主循环的updateDisplay()中自动更新
      wasMoving = false;
      return;
    }
    
    // 只在没有移动时才启动新的移动，避免重复启动
    if (!isMoving && !wasMoving) {
      moveForwardDistance(navForwardDist);
      if (isMoving) {
        wasMoving = true;
      }
    } else if (isMoving) {
      moveDistance(navForwardDist);
      wasMoving = true;
    }
    
    // 检查移动是否完成
    if (wasMoving && !isMoving) {
      // 前进到入口完成，继续到转回状态
      navState = NAV_TURN_BACK;
      wasMoving = false;
    }
  }
  else if (navState == NAV_TURN_BACK) {
    // 实时碰撞检测：在转回过程中检测
    // 只检测中间列（正前方），避免侧边障碍物误判
    // 检测范围：中间1/6的列（很小的范围，只关注正中心）
    const int COLLISION_THRESHOLD_MM = 150; // 15cm = 150mm
    int centerStart = TOTAL_POINTS / 2 - TOTAL_POINTS / 12;  // 中心减去1/12
    int centerEnd = TOTAL_POINTS / 2 + TOTAL_POINTS / 12 - 1;  // 中心加上1/12（中间1/6的列）
    if (checkCollisionRisk(COLLISION_THRESHOLD_MM, centerStart, centerEnd)) {
      Serial.println("【警告】转回中检测到近距离障碍物，进入搜索模式！");
      stopMotors();
      navState = NAV_SEARCHING; // 进入搜索模式
      searchStep = 0;
      searchTotalAngle = 0.0; // 重置累计角度
      currentSearchReason = SEARCH_COLLISION; // 设置搜索原因：障碍物
      // 屏幕显示会在主循环的updateDisplay()中自动更新
      return;
    }
    
    // 持续调用转向函数，让它检查时间并完成转向
    if (navTurnBackAngle < 0) {
      turnLeft(abs(navTurnBackAngle));
    } else {
      turnRight(navTurnBackAngle);
    }
    // 检查转向是否完成
    if (!isTurning) {
      // 转回正前方完成，现在传感器朝向正前方，数据更准确
      // 重置为IDLE，允许runGapTest()重新检测和规划路径
      stopMotors();
      navState = NAV_IDLE;
      pathCompleted = true; // 标记路径完成，需要在重新检测前延时
      // 注意：下次runGapTest()被调用时会重新规划路径
    }
  }
  else if (navState == NAV_MOVE_THROUGH) {
    // 实时碰撞检测：检查近距离障碍物
    // 只检测中间列（正前方），避免侧边障碍物误判
    // 检测范围：中间1/6的列（很小的范围，只关注正中心）
    const int COLLISION_THRESHOLD_MM = 150; // 15cm = 150mm
    int centerStart = TOTAL_POINTS / 2 - TOTAL_POINTS / 12;  // 中心减去1/12
    int centerEnd = TOTAL_POINTS / 2 + TOTAL_POINTS / 12 - 1;  // 中心加上1/12（中间1/6的列）
    if (checkCollisionRisk(COLLISION_THRESHOLD_MM, centerStart, centerEnd)) {
      Serial.println("【警告】检测到近距离障碍物，进入搜索模式！");
      stopMotors();
      navState = NAV_SEARCHING; // 进入搜索模式，旋转寻找可走路径
      searchStep = 0; // 从第一步开始搜索
      searchTotalAngle = 0.0; // 重置累计角度
      currentSearchReason = SEARCH_COLLISION; // 设置搜索原因：障碍物
      // 屏幕显示会在主循环的updateDisplay()中自动更新
      return;
    }
    
    // 持续前进，直到找到新的空洞或遇到障碍物
    // 只在没有移动时才启动新的前进动作，避免重复启动
    if (!isMoving) {
      moveForward();
    } else {
      // 如果正在移动，持续调用 moveDistance 让它检查时间
      moveDistance(CAR_SPEED_CM_PER_SEC);
    }
    // 导航会在下次 runGapTest() 找到新空洞时重新开始
  }
  else if (navState == NAV_SEARCHING) {
    // 搜索模式：单方向连续旋转搜索可走路径
    // searchStep: 0=旋转, 1=检测, 2=旋转, 3=检测...
    // 每次旋转固定角度（searchAngleStep），累计旋转角度不超过maxSearchAngle
    
    // 检查是否超过最大搜索角度
    if (abs(searchTotalAngle) >= maxSearchAngle) {
      // 已经旋转了一圈，重新开始搜索
      Serial.println("【搜索模式】已搜索一圈，重新开始搜索...");
      searchStep = 0;
      searchTotalAngle = 0.0;
      delay(500); // 短暂停顿后重新开始
    }
    
    // 偶数步骤：执行旋转
    if (searchStep % 2 == 0) {
      // 只在没有旋转时才启动新的旋转，避免重复启动
      static bool rotationStarted = false;
      if (!isTurning && !rotationStarted) {
        Serial.print("【搜索模式】");
        if (searchDirectionLeft) {
          Serial.print("左转 ");
        } else {
          Serial.print("右转 ");
        }
        Serial.print(searchAngleStep, 1);
        Serial.print(" 度 (累计: ");
        Serial.print(searchTotalAngle, 1);
        Serial.println(" 度)");
        
        if (searchDirectionLeft) {
          turnLeft(searchAngleStep);
          searchTotalAngle += searchAngleStep; // 累计角度
        } else {
          turnRight(searchAngleStep);
          searchTotalAngle += searchAngleStep; // 累计角度
        }
        rotationStarted = true;
      } else if (isTurning) {
        // 如果正在旋转，持续调用转向函数让它检查时间
        if (searchDirectionLeft) {
          turnLeft(searchAngleStep);
        } else {
          turnRight(searchAngleStep);
        }
      }
      
      // 检查旋转是否完成
      if (rotationStarted && !isTurning) {
        // 旋转完成，进入检测步骤
        searchStep++;
        rotationStarted = false;
        Serial.println("【搜索模式】旋转完成，等待检测...");
        // 等待一小段时间让传感器数据稳定
        delay(300);
      }
    }
    // 奇数步骤：等待检测（检测在 runGapTest() 中进行）
    // runGapTest() 会在检测步骤时自动调用并检测
    // 如果找到空洞，会自动退出搜索模式并开始导航
    // 如果没找到，等待一段时间后继续下一步搜索
    else {
      static unsigned long detectionStartTime = 0;
      static bool detectionStarted = false;
      static bool collisionWarningPrinted = false; // 防止重复打印碰撞警告
      
      // 初始化检测开始时间
      if (!detectionStarted) {
        detectionStartTime = millis();
        detectionStarted = true;
        collisionWarningPrinted = false; // 重置警告标志
        Serial.println("【搜索模式】正在检测当前方向...");
        // 打印当前地图和z值统计，帮助调试为什么没有找到空洞
        Serial.print("【搜索角度：");
        Serial.print(searchTotalAngle, 1);
        Serial.println("度】");
        printZValuesSummary();
      }
      
      // 在检测步骤时进行碰撞检测（只检测一次，避免重复打印）
      // 只检测中间列（正前方），避免侧边障碍物误判
      const int COLLISION_THRESHOLD_MM = 150; // 15cm = 150mm
      int centerStart = TOTAL_POINTS / 4;  // 从1/4处开始
      int centerEnd = TOTAL_POINTS * 3 / 4 - 1;  // 到3/4处结束（中间一半的列）
      if (!collisionWarningPrinted && checkCollisionRisk(COLLISION_THRESHOLD_MM, centerStart, centerEnd)) {
        Serial.println("【警告】搜索中检测到近距离障碍物，继续搜索其他方向！");
        collisionWarningPrinted = true; // 标记已打印，避免重复
        // 更新搜索原因显示（如果当前是未找到空洞，则更新为障碍物）
        if (currentSearchReason == SEARCH_NO_GAP) {
          currentSearchReason = SEARCH_COLLISION;
          // 屏幕显示会在主循环的updateDisplay()中自动更新
        }
        // 不停止搜索，继续尝试其他角度
      }
      
      // 等待至少500ms让 runGapTest() 有时间检测
      unsigned long currentTime = millis();
      if (currentTime - detectionStartTime >= 500) {
        // 检测时间已到，如果还没找到空洞，继续下一步搜索
        if (!gapFound) {
          // 打印地图和z值统计，帮助调试为什么没有找到空洞
          Serial.print("【搜索角度：");
          Serial.print(searchTotalAngle, 1);
          Serial.println("度 - 未找到空洞】");
          printZValuesSummary();
          
          searchStep++;
          detectionStarted = false;
          collisionWarningPrinted = false; // 重置警告标志，准备下次检测
          Serial.println("【搜索模式】当前方向未找到合适空洞，继续旋转搜索...");
        }
        // 如果找到空洞，gapFound 会被设置为 true，runGapTest() 会处理导航
      }
    }
  }
  else if (navState == NAV_IDLE) {
    // 空闲状态，什么都不做
  }
}

/*
 * 电机控制函数（空函数，待实现）
 */
void moveForward() {
  // TODO: 实现前进控制
  moveDistance(CAR_SPEED_CM_PER_SEC);
}

void moveForwardDistance(float distance) {
  // TODO: 实现按指定距离前进控制
  // 参数: distance - 前进距离（cm）
  moveDistance(distance);
}

void turnLeft(float angle) {
  // TODO: 实现左转控制
  // 参数: angle - 转向角度（度）
  turnAngle(angle);
}

void turnRight(float angle) {
  // TODO: 实现右转控制
  // 参数: angle - 转向角度（度）
  
  turnAngle(angle*(-1.0f));
}

void stopMotors() {
  setSpeed(0, 0);
  isMoving = false;
  isTurning = false;
}
