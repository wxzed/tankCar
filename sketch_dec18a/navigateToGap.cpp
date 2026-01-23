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
#include "tantable.h"
#include "motor.h"
#include "key.h"
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
float navForwardDist = 0.0;  // 到洞口入口的距离
float navThroughGapDist = 0.0;  // 通过空洞到洞底的距离
float navTurnAngle = 0.0;
float navTurnBackAngle = 0.0;
static NavigationState lastLoggedNavState = NAV_IDLE; // 调试：记录上一次打印的状态
static bool moveToEntranceStarted = false; // 标记是否已经启动了前进到入口的动作
static bool pathCompleted = false; // 标记路径是否刚完成，需要在重新检测前延时
static const unsigned long REPLAN_DELAY_MS = 200; // 重新规划前的短暂停顿

// 路径规划结果
static bool pathAvailable = false;
static int  goalRow = -1;
static int  goalCol = -1;
static float plannedHeadingDeg = 0.0f;
static float plannedDistanceCm = 0.0f;
static float plannedPathLengthCm = 0.0f;
static int preferredCols[ROWS]; // 每行偏好列（尽量走更宽裕的通道）
// 路径跟随：存储规划出的路径节点（行、列），以及执行进度
static int waypointRows[ROWS * COLS];
static int waypointCols[ROWS * COLS];
static int waypointCount = 0;
static int waypointIndex = 0;
static int currentPoseRow = 0;
static int currentPoseCol = (int)round(CENTER_COL);
static int executedSegments = 0;           // 每帧最多执行的航段数计数
static float lastSegmentTurnAngle = 0.0f;  // 最近航段的转向角
static bool pendingReplanLog = false;
static const char* pendingReplanReason = "";
static bool pendingBackThenStraight = false; // 回正后是否再执行一次直行
static bool macroPlanValid = false;
static bool macroActive = false;
static int macroPhase = 0; // 0=直行1,1=转向,2=直行2,3=回正,4=直行3
static bool stepPauseActive = false; // 步进暂停：每步完成后等待按键继续
static float macroStraight1Cm = 0.0f;
static float macroDiagDistCm = 0.0f;
static float macroStraight3Cm = 0.0f;
static float macroTurnDeg = 0.0f;
static int macroDiagRows = 0;
static const bool REPLAN_AFTER_TURNBACK = true; // 回正后立即重规划，不再执行最后直行
static char currentAction[10] = ""; // 当前执行动作：left/right/line
static float currentActionValue = 0.0f;
static const unsigned long ACTION_STEP_DELAY_MS = 250; // 每步完成后停顿，便于观察

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
  
  if (!pathAvailable || waypointIndex >= waypointCount) {
    // 没有可行路径或路径已执行完
    return;
  }
  
  // 只显示当前正在执行的一步
  if (state == NAV_TURN_TO_GAP || state == NAV_MOVE_TO_ENTRANCE || state == NAV_TURN_BACK) {
    if (strlen(currentAction) > 0) {
      strcpy(step1, currentAction);
      *value1 = currentActionValue;
      return;
    }
  }
  if (state == NAV_TURN_TO_GAP) {
    if (abs(navTurnAngle) > 0.5f) {
      strcpy(step1, navTurnAngle < 0 ? "left" : "right");
      *value1 = abs(navTurnAngle);
    }
    return;
  } else if (state == NAV_MOVE_TO_ENTRANCE) {
    strcpy(step1, "line");
    *value1 = navForwardDist;
    return;
  } else if (state == NAV_TURN_BACK) {
    if (abs(navTurnBackAngle) > 0.5f) {
      strcpy(step1, navTurnBackAngle < 0 ? "left" : "right");
      *value1 = abs(navTurnBackAngle);
    }
    return;
  } else if (state == NAV_IDLE) {
    // 待执行时展示当前航段
    if (abs(navTurnAngle) > 0.5f) {
      strcpy(step1, navTurnAngle < 0 ? "left" : "right");
      *value1 = abs(navTurnAngle);
    } else {
      strcpy(step1, "line");
      *value1 = navForwardDist;
    }
  }
}

/*
 * 获取完整计划步骤（用于测试模式显示）
 * 返回：实际步骤数量
 */
int getPlannedStepsForDisplay(char steps[][10], float values[], int maxSteps) {
  if (maxSteps <= 0) return 0;
  for (int i = 0; i < maxSteps; i++) {
    strcpy(steps[i], "");
    values[i] = 0.0f;
  }

  if (!pathAvailable || waypointIndex >= waypointCount) {
    return 0;
  }

  int count = 0;

  if (macroPlanValid) {
    if (macroStraight1Cm > 0.5f && count < maxSteps) {
      strcpy(steps[count], "line");
      values[count] = macroStraight1Cm;
      count++;
    }
    if (fabs(macroTurnDeg) > 0.5f && count < maxSteps) {
      strcpy(steps[count], macroTurnDeg < 0 ? "left" : "right");
      values[count] = fabs(macroTurnDeg);
      count++;
    }
    if (macroDiagDistCm > 0.5f && count < maxSteps) {
      strcpy(steps[count], "line");
      values[count] = macroDiagDistCm;
      count++;
    }
    if (fabs(macroTurnDeg) > 0.5f && count < maxSteps) {
      strcpy(steps[count], macroTurnDeg > 0 ? "left" : "right");
      values[count] = fabs(macroTurnDeg);
      count++;
    }
    if (macroStraight3Cm > 0.5f && count < maxSteps) {
      strcpy(steps[count], "line");
      values[count] = macroStraight3Cm;
      count++;
    }
  }

  if (count == 0) {
    char step1[10] = "", step2[10] = "", step3[10] = "";
    float value1 = 0.0f, value2 = 0.0f, value3 = 0.0f;
    getNextStepInfo(step1, &value1, step2, &value2, step3, &value3);
    if (strlen(step1) > 0 && count < maxSteps) {
      strcpy(steps[count], step1);
      values[count] = value1;
      count++;
    }
    if (strlen(step2) > 0 && count < maxSteps) {
      strcpy(steps[count], step2);
      values[count] = value2;
      count++;
    }
    if (strlen(step3) > 0 && count < maxSteps) {
      strcpy(steps[count], step3);
      values[count] = value3;
      count++;
    }
  }

  return count;
}

void findLargestGap();
void navigateToGap();
void planAndDisplayPath(); 
void printPathMap();
void printExecutionSteps(float startDist, float startAngle, float entranceDist, float entranceAngle, float turnBackAngle, float gapBottomDist);
void printZValuesSummary(); // 调试：打印z值统计
// void printCurrentScanMap();  // 调试：直接基于当前zValues生成并打印占据图（暂不打印）
void fillPointCloudGrid(); // 填充pointCloudGrid数据
void stopMotors(); 
void moveForward();
void moveForwardDistance(float distance);
void turnLeft(float angle);
void turnRight(float angle);
void initializePathMap();
bool checkCollisionRisk(int thresholdMm, int startCol, int endCol); // 前向声明供紧急刹停使用
bool checkPathWidth(int row, int centerCol, float distance); // 前向声明，供BFS使用
void calculateCarWidthColumns(float distance, int centerCol, int& startCol, int& endCol); // 前向声明，供BFS标记宽度
float calculateObstacleBalanceScore(int row, int col); // 计算当前点两侧障碍物的平衡分数（越接近0越好，表示两侧障碍物距离相等）
int calculateCorridorWidth(int row, int col); // 计算当前位置通道宽度（越大越宽）
bool planPathWithBFS();     // 基于BFS的路径规划
void markPathOnMap(int parentRow[ROWS][COLS], int parentCol[ROWS][COLS], int endR, int endC);
bool loadNextWaypoint();    // 计算下一个航段的转角与距离
void advanceWaypoint();     // 前进到下一个节点后更新当前位置
bool checkAndBrakeForCollisionImmediate(); // 最新帧的紧急刹停
bool tryReplanAndSwitchPath(bool stopImmediately); // 在行进阶段尝试重规划，支持平滑/急停切换
void printPlannedStepsDebug(); // 打印规划出的动作序列（供调试串口查看）
static void requestReplan(const char* reason); // 标记本轮结束并触发重规划
static bool buildMacroPlan(int& straightRows, int& diagSteps, int& straightAfterRows, int& diagSign);
static void startMacroExecution();
static void computeMacroTurnAndDist(int straightRows, int diagSteps, int diagRows, int startCol, int diagSign,
                                    float& turnDeg, float& diagDistCm);
static void setCurrentAction(const char* action, float value);
static void delayAfterAction();
static void requestStepPause(const char* reason);
static void computePreferredCols();
static bool buildPreferredPath(int startRow, int startCol,
                               int parentR[ROWS][COLS], int parentC[ROWS][COLS],
                               int& endR, int& endC);
static bool buildMacroPathDirect(int startRow, int startCol,
                                 int parentR[ROWS][COLS], int parentC[ROWS][COLS],
                                 int& endR, int& endC);

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
 * 基于最新 pointCloudGrid 做一次紧急刹停检测
 * 范围：中间 1/3 列；阈值 220mm
 */
bool checkAndBrakeForCollisionImmediate() {
  const int COLLISION_THRESHOLD_MM = 220; // 更敏感
  int centerStart = TOTAL_POINTS / 3;            // 中间 1/3
  int centerEnd   = TOTAL_POINTS * 2 / 3 - 1;
  if (checkCollisionRisk(COLLISION_THRESHOLD_MM, centerStart, centerEnd)) {
    Serial.println("【紧急刹停】检测到近距离障碍，进入搜索模式");
    stopMotors();
    navState = NAV_SEARCHING;
    searchStep = 0;
    searchTotalAngle = 0.0;
    currentSearchReason = SEARCH_COLLISION;
    return true;
  }
  return false;
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
  // 始终刷新占据网格，确保碰撞检测/显示使用最新数据
  fillPointCloudGrid();

  // 正在执行宏路径/航段时不刷新规划，避免中途改路径
  if (navState != NAV_IDLE && navState != NAV_SEARCHING) {
    return;
  }

  // 【测试模式】仅输出地图，不执行真实动作
  // 每次规划都输出地图到专用串口，测试刷新速度
  if (pendingReplanLog) {
    Serial.print("【重规划】");
    Serial.println(pendingReplanReason);
    pendingReplanLog = false;
  }
  pathAvailable = planPathWithBFS();
  gapFound = pathAvailable; // 兼容旧显示逻辑

  if (!pathAvailable) {
    Serial.println("【规划失败】未找到可行路径");
    printPathMap();
    return;
  }

  Serial.print("【地图刷新】目标行列: ");
  Serial.print(goalRow + 1);
  Serial.print(", ");
  Serial.print(goalCol);
  Serial.print(", 航向: ");
  Serial.print(plannedHeadingDeg, 1);
  Serial.print("度, 距离: ");
  Serial.print(plannedDistanceCm, 1);
  Serial.println("cm");

  // 每次规划成功都输出地图到专用串口
  printPathMap();
  // 调试：输出分解后的动作序列（转向/直行）
  printPlannedStepsDebug();

  // 初始化宏路径参数（用于执行/输出一致）
  int straightRows = 0;
  int diagSteps = 0;
  int straightAfterRows = 0;
  int diagSign = 0;
  if (buildMacroPlan(straightRows, diagSteps, straightAfterRows, diagSign)) {
    macroPlanValid = true;
    macroStraight1Cm = straightRows * LAYER_HEIGHT;
    macroStraight3Cm = straightAfterRows * LAYER_HEIGHT;
    int startCol = waypointCols[0];
    macroDiagRows = diagSteps; // 斜行段按侧移步数计算
    computeMacroTurnAndDist(straightRows, diagSteps, macroDiagRows, startCol, diagSign,
                            macroTurnDeg, macroDiagDistCm);
  } else {
    macroPlanValid = false;
  }
  
  // 执行导航：电机启用才执行，否则仅输出规划结果
  if (motorDriveState == MOTOR_DRIVE_ENABLED) {
    navigateToGap();
  } else {
    Serial.println("【测试模式】电机未启用，未执行导航");
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
 * 基于BFS的路径规划
 * - 起点：第1行中心列（若被占据则在附近寻找空点）
 * - 目标：可达的最远行，若行相同则优先靠近中心列
 * - 约束：仅在满足车宽通过(checkPathWidth)的网格上行走
 * - 输出：pathMap 标记路径(2)，并填充规划航向与距离
 */
bool planPathWithBFS() {
  initializePathMap();

  int startRow = 0;
  int startCol = (int)round(CENTER_COL);
  currentPoseRow = startRow;
  currentPoseCol = startCol;

  // 起点补偿：如果中心被占据，向两侧寻找最近的空点
  if (pointCloudGrid[startRow][startCol] != 0) {
    bool found = false;
    const int SPAN = 3;
    for (int off = 1; off <= SPAN && !found; off++) {
      if (startCol - off >= 0 && pointCloudGrid[startRow][startCol - off] == 0) {
        startCol -= off;
        found = true;
      } else if (startCol + off < COLS && pointCloudGrid[startRow][startCol + off] == 0) {
        startCol += off;
        found = true;
      }
    }
    if (!found) {
      Serial.println("【规划失败】起点被占据且附近无空隙");
      return false;
    }
  }

  // 起点车宽校验
  float startDistCm = (startRow + 1) * LAYER_HEIGHT;
  if (!checkPathWidth(startRow, startCol, startDistCm)) {
    Serial.println("【规划失败】起点宽度不足");
    return false;
  }


  // BFS 数据结构
  static int queueR[ROWS * COLS];
  static int queueC[ROWS * COLS];
  static int parentR[ROWS][COLS];
  static int parentC[ROWS][COLS];
  static bool visited[ROWS][COLS];

  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      visited[r][c] = false;
      parentR[r][c] = -1;
      parentC[r][c] = -1;
    }
  }

  // 计算每行偏好列，提前把路径引导到更宽裕通道
  computePreferredCols();

  int head = 0, tail = 0;
  queueR[tail] = startRow;
  queueC[tail] = startCol;
  visited[startRow][startCol] = true;
  tail++;

  // 邻接扩展顺序：优先向前（行+1）保持直行，其次前左/前右，再水平，再后向
  const int dr[8] = {1, 1, 1, 0, 0, -1, -1, -1};
  const int dc[8] = {0, -1, 1, -1, 1, -1, 0, 1};

  int bestR = startRow;
  int bestC = startCol;
  float bestScore = -1.0f;  // 最佳分数：行数*100 - 距离中心的列偏移*10（越大越好）
  bool usedPreferred = false;

  // 直接构造4段式路径：直行->转向->直行->回正->直行
  if (buildMacroPathDirect(startRow, startCol, parentR, parentC, bestR, bestC)) {
    usedPreferred = true;
  }

  if (!usedPreferred) {
    while (head < tail) {
    int r = queueR[head];
    int c = queueC[head];
    head++;

    // 计算当前点的分数：优先选择既远又走在障碍物中间的路径
    // 平衡度分数：绝对值越小越好，表示两侧障碍物距离相等
    float balanceRaw = calculateObstacleBalanceScore(r, c);
    float balanceScore = abs(balanceRaw);

    // 检查两侧是否都有障碍物（距离都不是最大值）
    int leftDist = 0, rightDist = 0;
    for (int c2 = c - 1; c2 >= 0; c2--) {
      if (pointCloudGrid[r][c2] != 0) break;
      leftDist++;
    }
    if (leftDist == c) leftDist = COLS;

    for (int c2 = c + 1; c2 < COLS; c2++) {
      if (pointCloudGrid[r][c2] != 0) break;
      rightDist++;
    }
    if (rightDist == COLS - c - 1) rightDist = COLS;

    bool hasLeftObstacle = (leftDist < COLS);
    bool hasRightObstacle = (rightDist < COLS);

    int preferredCol = preferredCols[r];
    float preferredOffset = abs(c - preferredCol);
    int minSideClear = min(leftDist, rightDist);
    const int CAR_HALF_COLS = 2; // 车身4列宽
    const int SAFE_MARGIN_COLS = 1;
    const int REQUIRED_CLEAR = CAR_HALF_COLS + SAFE_MARGIN_COLS;
    const int IDEAL_CLEAR = REQUIRED_CLEAR + 1; // 期望至少再留一列缓冲

    // 自适应评分：不再强偏直行，前段降低直行权重以便提前转向
    float straightWeight = 220.0f;
    if (r <= 8) {
      straightWeight *= 0.35f;
    } else if (r <= 12) {
      straightWeight *= 0.6f;
    }
    float distanceReward    = r * 210.0f;                  // 深度奖励（更高权重）
    float balancePenalty    = balanceScore * 16.0f;
    if (hasLeftObstacle && hasRightObstacle) {
      balancePenalty *= 0.7f;                              // 在两障碍之间时放宽平衡要求
    }

    // 当一侧空间明显更大时，鼓励提前朝开阔侧偏移，避免被迫向狭窄侧绕行
    int sideDiff = leftDist - rightDist;                   // 正值=左侧更开阔，负值=右侧更开阔
    float sideBias = 0.0f;
    if (abs(sideDiff) >= 2) {
      // 放宽“贴中线”惩罚，允许更早偏向开阔侧
      straightWeight *= 0.5f;
      // 朝开阔侧移动时给予奖励（限制奖励幅度，避免贴边）
      int towardOpen = (sideDiff > 0) ? (int)round(CENTER_COL - c) : (int)round(c - CENTER_COL);
      if (towardOpen > 0) {
        int cappedCols = min(towardOpen, 8);               // 最多奖励偏移8列
        sideBias = cappedCols * 160.0f;
      }
      // 单侧开阔时不必强制平衡
      if (!hasLeftObstacle || !hasRightObstacle) {
        balancePenalty *= 0.4f;
      }
    }

    float straightPenalty   = preferredOffset * straightWeight;
    float preferredPenalty = preferredOffset * ((r <= 8) ? 900.0f : 500.0f);

    // 离障碍太近时，提前给惩罚，促使更早转向到开阔侧
    float nearObstaclePenalty = 0.0f;
    if (minSideClear < REQUIRED_CLEAR) {
      int deficit = REQUIRED_CLEAR - minSideClear;
      nearObstaclePenalty = (deficit + 1) * 420.0f;
      straightPenalty *= 0.6f; // 靠近障碍时明显降低“硬直行”倾向
    }
    // 近距离阶段更严格：宁愿提前转向，也不要贴近障碍继续直走
    float earlyHazardPenalty = 0.0f;
    if (r <= 8 && minSideClear < REQUIRED_CLEAR) {
      int deficit = REQUIRED_CLEAR - minSideClear;
      earlyHazardPenalty = (deficit + 1) * 1200.0f;
    }
    // 紧贴障碍（只隔1列）强惩罚，避免沿障碍边滑行
    float adjacentPenalty = 0.0f;
    if (minSideClear <= 1) {
      adjacentPenalty = (2 - minSideClear) * 1800.0f;
    }
    // 净空奖励：鼓励走在更宽裕的区域（早期更敏感）
    float clearReward = 0.0f;
    if (minSideClear >= IDEAL_CLEAR) {
      clearReward = (minSideClear - IDEAL_CLEAR + 1) * (r <= 8 ? 180.0f : 120.0f);
    }

    // 转向平滑：惩罚与父节点的列跳变，鼓励小角度微调
    float turnPenalty = 0.0f;
    int pr = parentR[r][c];
    int pc = parentC[r][c];
    if (pr >= 0 && pc >= 0) {
      // 统一转向惩罚，避免前8行“强限制”
      turnPenalty = abs(c - pc) * 140.0f;
    }

    // 前向可达性：看未来若干行在当前列是否还能直走
    // 若前方很快被堵，鼓励提前侧移
    int forwardClearRows = 0;
    const int LOOKAHEAD_ROWS = 9;
    int forwardTightRows = 0;
    int firstTightStep = 0;
    for (int step = 1; step <= LOOKAHEAD_ROWS; step++) {
      int rr = r + step;
      if (rr >= ROWS) break;
      float distCm = (rr + 1) * LAYER_HEIGHT;
      if (!checkPathWidth(rr, c, distCm)) break;
      forwardClearRows++;
      // 统计前方“侧向净空不足”的行数，用于提前转向
      int l2 = 0, r2 = 0;
      for (int c2 = c - 1; c2 >= 0; c2--) {
        if (pointCloudGrid[rr][c2] != 0) break;
        l2++;
      }
      if (l2 == c) l2 = COLS;
      for (int c2 = c + 1; c2 < COLS; c2++) {
        if (pointCloudGrid[rr][c2] != 0) break;
        r2++;
      }
      if (r2 == COLS - c - 1) r2 = COLS;
      int minSide2 = min(l2, r2);
      if (minSide2 < REQUIRED_CLEAR) {
        forwardTightRows++;
        if (firstTightStep == 0) {
          firstTightStep = step;
        }
      }
    }
    float forwardClearReward = forwardClearRows * 90.0f;
    float forwardBlockPenalty = (forwardClearRows == 0) ? 180.0f : 0.0f;
    float forwardTightPenalty = forwardTightRows * 420.0f;
    float forwardNearPenalty = 0.0f;
    if (firstTightStep > 0) {
      forwardNearPenalty = (LOOKAHEAD_ROWS - firstTightStep + 1) * 420.0f;
    }
    // 早期行只要前方很快出现侧向净空不足，就强行压低直走倾向
    float earlyForwardPenalty = 0.0f;
    if (r <= 8 && firstTightStep > 0) {
      earlyForwardPenalty = (LOOKAHEAD_ROWS - firstTightStep + 1) * 1200.0f;
      straightPenalty *= 0.5f;
    }

    // 通道宽度偏好：满足车身宽度前提下，越宽越好，但深度权重更高
    int corridorWidth = leftDist + rightDist + 1;
    if (leftDist >= COLS || rightDist >= COLS) {
      corridorWidth = COLS * 2; // 一侧无遮挡时，视为非常宽
    }
    const int minPassCols = 6; // 车宽4列 + 左右安全边距1列
    int effectiveWidth = min(corridorWidth, COLS * 2);
    float corridorReward = 0.0f;
    if (effectiveWidth >= minPassCols) {
      // 仅在满足车身宽度时给宽度奖励
      corridorReward = (effectiveWidth - minPassCols) * 18.0f;
    }

    float score = distanceReward - straightPenalty - balancePenalty - turnPenalty
                  + sideBias + corridorReward + forwardClearReward - forwardBlockPenalty
                  + clearReward
                  - forwardTightPenalty - forwardNearPenalty - earlyForwardPenalty
                  - nearObstaclePenalty - earlyHazardPenalty - adjacentPenalty
                  - preferredPenalty;

    // 记录更优点；分数接近时优先中心、更平衡
    int bestCorridorWidth = calculateCorridorWidth(bestR, bestC);
    bool shouldUpdate = false;
    if (r > bestR) {
      // 深度优先：能走更深就直接选择
      shouldUpdate = true;
    } else if (r == bestR) {
      if (score > bestScore) {
        shouldUpdate = true;
      } else if (fabs(score - bestScore) < 1.0f && corridorWidth > bestCorridorWidth) {
        shouldUpdate = true;
      } else if (fabs(score - bestScore) < 1.0f &&
                 preferredOffset < abs(bestC - preferredCols[bestR])) {
        shouldUpdate = true;
      } else if (fabs(score - bestScore) < 1.0f &&
                 fabs(balanceScore) < fabs(calculateObstacleBalanceScore(bestR, bestC))) {
        shouldUpdate = true;
      }
    }

    if (shouldUpdate) {
      bestR = r;
      bestC = c;
      bestScore = score;
    }

    for (int k = 0; k < 8; k++) {
      int nr = r + dr[k];
      int nc = c + dc[k];
      if (nr < 0 || nr >= ROWS || nc < 0 || nc >= COLS) continue;
      if (visited[nr][nc]) continue;
      if (pointCloudGrid[nr][nc] != 0) continue;

      float distCm = (nr + 1) * LAYER_HEIGHT;
      if (!checkPathWidth(nr, nc, distCm)) continue;
      visited[nr][nc] = true;
      parentR[nr][nc] = r;
      parentC[nr][nc] = c;
      queueR[tail] = nr;
      queueC[tail] = nc;
      tail++;
      if (tail >= ROWS * COLS) break; // 防御
    }
    }
  }

  if (bestR == startRow && bestC == startCol) {
    Serial.println("【规划失败】仅原地可达，没有可行路径");
    return false;
  }

  goalRow = bestR;
  goalCol = bestC;

  // 回溯标记路径
  markPathOnMap(parentR, parentC, goalRow, goalCol);
  // 回溯收集路径节点（从起点到终点的有序列表）
  int tempR[ROWS * COLS];
  int tempC[ROWS * COLS];
  int tempCount = 0;
  int r = goalRow;
  int c = goalCol;
  while (r >= 0 && c >= 0 && tempCount < ROWS * COLS) {
    tempR[tempCount] = r;
    tempC[tempCount] = c;
    int pr = parentR[r][c];
    int pc = parentC[r][c];
    if (pr == -1 || pc == -1) break;
    r = pr;
    c = pc;
    tempCount++;
  }
  // 包含起点
  tempR[tempCount] = startRow;
  tempC[tempCount] = startCol;
  tempCount++;

  // 反转为从起点到终点
  waypointCount = 0;
  for (int i = tempCount - 1; i >= 0; i--) {
    waypointRows[waypointCount] = tempR[i];
    waypointCols[waypointCount] = tempC[i];
    waypointCount++;
    if (waypointCount >= ROWS * COLS) break;
  }
  waypointIndex = 1; // 第0个是起点，执行从第1个节点开始

  // 航向角：相对中心列的偏移，左为正
  float offsetCol = goalCol - CENTER_COL;
  plannedHeadingDeg = -offsetCol * ANGLE_STEP;

  // 路径长度估算（行步长5cm，斜向乘√2）
  plannedPathLengthCm = 0.0f;
  int cr = goalRow;
  int cc = goalCol;
  while (!(cr == startRow && cc == startCol)) {
    int pr = parentR[cr][cc];
    int pc = parentC[cr][cc];
    if (pr < 0 || pc < 0) break;
    int dRow = abs(cr - pr);
    int dCol = abs(cc - pc);
    float step = (dRow + dCol == 1) ? LAYER_HEIGHT : (LAYER_HEIGHT * 1.41421356f);
    plannedPathLengthCm += step;
    cr = pr;
    cc = pc;
  }

  plannedDistanceCm = plannedPathLengthCm;
  if (plannedDistanceCm < LAYER_HEIGHT) {
    plannedDistanceCm = LAYER_HEIGHT;
  }

  return true;
}

/*
 * 回溯父节点并在路径地图上标记 2
 */
void markPathOnMap(int parentRow[ROWS][COLS], int parentCol[ROWS][COLS], int endR, int endC) {
  int r = endR;
  int c = endC;
  while (r >= 0 && c >= 0) {
    // 根据距离换算行对应的实际距离（cm）
    float distCm = (r + 1) * LAYER_HEIGHT;

    // 计算在当前距离处车身宽度对应的列范围
    int startCol = 0;
    int endCol = 0;
    calculateCarWidthColumns(distCm, c, startCol, endCol);

    // 标记车身宽度覆盖的所有列为路径（值2），严格不覆盖障碍（以 pointCloudGrid 判定）
    for (int col = startCol; col <= endCol; col++) {
      if (col < 0 || col >= COLS) continue;
      if (pointCloudGrid[r][col] == 0) { // 障碍保持为1
        pathMap[r][col] = 2;
      }
    }

    int pr = parentRow[r][c];
    int pc = parentCol[r][c];
    if (pr == -1 || pc == -1) break;
    r = pr;
    c = pc;
  }
}

/*
 * 从当前航点索引装载下一段航向与距离
 * 返回 false 表示没有更多航点可执行
 */
bool loadNextWaypoint() {
  if (waypointIndex >= waypointCount) {
    return false;
  }

  int targetRow = waypointRows[waypointIndex];
  int targetCol = waypointCols[waypointIndex];

  float targetDist = (targetRow + 1) * LAYER_HEIGHT;
  float currentDist = (currentPoseRow + 1) * LAYER_HEIGHT;
  navForwardDist = targetDist - currentDist;
  if (navForwardDist < (LAYER_HEIGHT * 0.5f)) {
    // 距离太小，直接视为到达，跳到下一个
    advanceWaypoint();
    return loadNextWaypoint();
  }

  float colOffset = targetCol - currentPoseCol;
  navTurnAngle = -colOffset * ANGLE_STEP;
  navTurnBackAngle = 0.0f;
  navThroughGapDist = 0.0f;
  plannedHeadingDeg = navTurnAngle;
  plannedDistanceCm = navForwardDist;
  lastSegmentTurnAngle = navTurnAngle;
  return true;
}

/*
 * 前进到当前航点后，推进到下一个节点并更新当前位置
 */
void advanceWaypoint() {
  if (waypointIndex >= waypointCount) return;
  currentPoseRow = waypointRows[waypointIndex];
  currentPoseCol = waypointCols[waypointIndex];
  waypointIndex++;
}

/*
 * 行进中动态重规划：使用最新点云做 BFS，若找到新路径：
 * - 更新航点队列与索引
 * - 根据 stopImmediately 选择是否先停再转向/前进
 */
bool tryReplanAndSwitchPath(bool stopImmediately) {
  static unsigned long lastReplanMs = 0;

  if (navState != NAV_MOVE_TO_ENTRANCE && navState != NAV_SEARCHING) return false;
  // 行进中为平滑体验，若电机正处于移动则暂不切换，等停下后再评估
  if (isMoving) return false;

  // 节流：避免过于频繁重规划导致串口刷屏/顿挫
  unsigned long nowMs = millis();
  if (nowMs - lastReplanMs < 300) return false;
  lastReplanMs = nowMs;

  int oldGoalRow = goalRow;
  int oldGoalCol = goalCol;

  bool newPath = planPathWithBFS();
  if (!newPath || waypointCount < 2) return false;

  // 目标变化太小则不切换，避免抖动
  int dRow = abs(goalRow - oldGoalRow);
  int dCol = abs(goalCol - oldGoalCol);
  if (dRow < 2 && dCol < 2) {
    return false;
  }

  if (stopImmediately) {
    stopMotors();
    delay(20); // 短暂停顿，减少顿挫
  }

  // 载入新路径的首段
  waypointIndex = 1;
  if (!loadNextWaypoint()) {
    navState = NAV_IDLE;
    return true;
  }

  // 重新进入转向阶段
  navState = NAV_TURN_TO_GAP;
  Serial.printf("【动态重规划】切换到新路径 旧目标:(%d,%d) 新目标:(%d,%d) 航点数:%d\n",
                oldGoalRow + 1, oldGoalCol, goalRow + 1, goalCol, waypointCount);
  // 输出最新路径地图到专用串口，保证配套显示
  Serial.println("【动态重规划】刷新路径地图 -> Serial/Serial2");
  printPathMap();
  return true;
}

/*
 * 调试：打印规划出的动作序列（基于航点列表）
 * 输出格式：左/右转 X 度，直行 Y cm
 */
void printPlannedStepsDebug() {
  if (!pathAvailable || waypointCount < 2) {
    Serial.println("【规划动作序列】无可执行路径");
    return;
  }

  int straightRows = 0;
  int diagSteps = 0;
  int straightAfterRows = 0;
  int diagSign = 0; // 右为正，左为负
  if (buildMacroPlan(straightRows, diagSteps, straightAfterRows, diagSign)) {
    Serial.println("【规划动作序列】(简化4段式: 直行->转向->直行->回正->直行)");
    int step = 1;
    if (straightRows > 0) {
      Serial.printf("  步骤%d: 直行 %.1f cm\n", step++, straightRows * LAYER_HEIGHT);
    }
    if (diagSteps > 0) {
      float sideTurn = 0.0f;
      float diagDist = 0.0f;
      int startCol = waypointCols[0];
      int diagRows = diagSteps;
      computeMacroTurnAndDist(straightRows, diagSteps, diagRows, startCol, diagSign,
                              sideTurn, diagDist);
      Serial.printf("  步骤%d: %s转 %.1f 度\n", step++,
                    (sideTurn < 0) ? "左" : "右", fabs(sideTurn));
      Serial.printf("  步骤%d: 直行 %.1f cm\n", step++, diagDist);
      Serial.printf("  步骤%d: %s转 %.1f 度 (侧移后回正)\n", step++,
                    (sideTurn > 0) ? "左" : "右", fabs(sideTurn));
    }
    if (straightAfterRows > 0 && !REPLAN_AFTER_TURNBACK) {
      Serial.printf("  步骤%d: 直行 %.1f cm\n", step++, straightAfterRows * LAYER_HEIGHT);
    }
    Serial.printf("  终点: 行%d 列%d, 航段数:%d\n",
                  goalRow + 1, goalCol, waypointCount - 1);
    return;
  }

  Serial.println("【规划动作序列】(基于航点, 全路径, 本轮只执行首段)");

  int curR = waypointRows[0];
  int curC = waypointCols[0];
  float curHeading = 0.0f; // 初始朝向正前方
  int step = 1;

  int i = 1;
  bool nonForwardWarningPrinted = false;
  while (i < waypointCount) {
    int nr = waypointRows[i];
    int nc = waypointCols[i];
    int dr = nr - curR;
    int dc = nc - curC;
    int adr = abs(dr);
    int adc = abs(dc);

    // 仅用于调试输出：若出现后退/纯横移节点，打印提示并跳过
    if (dr <= 0) {
      if (!nonForwardWarningPrinted) {
        Serial.println("  【注意】路径含后退/横移节点，动作序列已跳过这些节点");
        nonForwardWarningPrinted = true;
      }
      curR = nr;
      curC = nc;
      i++;
      continue;
    }

    // 1) 纯前进：合并连续直行
    if (adr == 1 && adc == 0) {
      int straightCount = 1;
      while ((i + straightCount) < waypointCount) {
        int pr = waypointRows[i + straightCount - 1];
        int pc = waypointCols[i + straightCount - 1];
        int nr2 = waypointRows[i + straightCount];
        int nc2 = waypointCols[i + straightCount];
        if (nc2 != pc) break;
        if ((nr2 - pr) != dr) break;
        straightCount++;
      }
      float distCm = straightCount * LAYER_HEIGHT;
      Serial.printf("  步骤%d: 直行 %.1f cm\n", step++, distCm);
      curR = waypointRows[i + straightCount - 1];
      curC = waypointCols[i + straightCount - 1];
      i += straightCount;
      continue;
    }

    // 2) 斜向侧移：合并同方向连续侧移
    if (adr == 1 && adc == 1) {
      int lateralSign = (dc > 0) ? 1 : -1;
      int diagCount = 1;
      while ((i + diagCount) < waypointCount) {
        int pr = waypointRows[i + diagCount - 1];
        int pc = waypointCols[i + diagCount - 1];
        int nr2 = waypointRows[i + diagCount];
        int nc2 = waypointCols[i + diagCount];
        int dr2 = nr2 - pr;
        int dc2 = nc2 - pc;
        if (abs(dr2) != 1 || abs(dc2) != 1) break;
        int sign2 = (dc2 > 0) ? 1 : -1;
        if (sign2 != lateralSign) break;
        diagCount++;
      }

      float sideTurn = -dc * ANGLE_STEP; // 与转向逻辑保持一致：列>中心为左转(负角)
      float distCm = diagCount * LAYER_HEIGHT * 1.41421356f;
      Serial.printf("  步骤%d: %s转 %.1f 度\n", step++,
                    (sideTurn < 0) ? "左" : "右", fabs(sideTurn));
      Serial.printf("  步骤%d: 直行 %.1f cm\n", step++, distCm);
      // 侧移后回正：只有在确实发生了前进时才输出
      if (distCm > 0.5f) {
        Serial.printf("  步骤%d: %s转 %.1f 度 (侧移后回正)\n", step++,
                      (sideTurn > 0) ? "左" : "右", fabs(sideTurn));
      }

      curHeading = 0.0f;
      curR = waypointRows[i + diagCount - 1];
      curC = waypointCols[i + diagCount - 1];
      i += diagCount;
      continue;
    }

    // 3) 其他不常见情况，逐步输出
    float targetHeading = -(nc - CENTER_COL) * ANGLE_STEP;
    float turnDeg = targetHeading - curHeading;
    while (turnDeg > 180.0f) turnDeg -= 360.0f;
    while (turnDeg < -180.0f) turnDeg += 360.0f;
    if (fabs(turnDeg) > 0.5f) {
      Serial.printf("  步骤%d: %s转 %.1f 度\n", step++,
                    (turnDeg < 0) ? "左" : "右", fabs(turnDeg));
      curHeading = targetHeading;
    }
    float distCm = sqrtf((float)(adr * adr + adc * adc)) * LAYER_HEIGHT;
    Serial.printf("  步骤%d: 直行 %.1f cm\n", step++, distCm);
    curR = nr;
    curC = nc;
    i++;
  }

  // 终点默认不再强制“回正”，避免出现成对的 ±X° 输出
  // 如需回正，可放开下列代码：
  // if (fabs(curHeading) > 0.5f) {
  //   float backTurn = -curHeading;
  //   while (backTurn > 180.0f) backTurn -= 360.0f;
  //   while (backTurn < -180.0f) backTurn += 360.0f;
  //   Serial.printf("  步骤%d: %s转 %.1f 度 (终点回正)\n",
  //                 step++,
  //                 (backTurn < 0) ? "左" : "右",
  //                 fabs(backTurn));
  // }

  Serial.printf("  终点: 行%d 列%d, 航段数:%d\n",
                goalRow + 1, goalCol, waypointCount - 1);
}

static bool buildMacroPlan(int& straightRows, int& diagSteps, int& straightAfterRows, int& diagSign) {
  straightRows = 0;
  diagSteps = 0;
  straightAfterRows = 0;
  diagSign = 0;

  if (waypointCount < 2) return false;

  int i = 1;
  int curR = waypointRows[0];
  int curC = waypointCols[0];

  // 段1：起始直行（列不变）
  while (i < waypointCount) {
    int nr = waypointRows[i];
    int nc = waypointCols[i];
    int dr = nr - curR;
    int dc = nc - curC;
    if (dr == 1 && dc == 0) {
      straightRows++;
      curR = nr;
      curC = nc;
      i++;
      continue;
    }
    break;
  }

  if (i >= waypointCount) {
    return true; // 全程直行
  }

  // 段2：单一方向的斜向侧移
  int firstDr = waypointRows[i] - curR;
  int firstDc = waypointCols[i] - curC;
  if (abs(firstDr) != 1 || abs(firstDc) != 1) {
    return false; // 不是斜向，无法简化为4段式
  }
  diagSign = (firstDc > 0) ? 1 : -1;

  while (i < waypointCount) {
    int nr = waypointRows[i];
    int nc = waypointCols[i];
    int dr = nr - curR;
    int dc = nc - curC;
    if (abs(dr) == 1 && abs(dc) == 1 && ((dc > 0) ? 1 : -1) == diagSign) {
      diagSteps++;
      curR = nr;
      curC = nc;
      i++;
      continue;
    }
    break;
  }

  // 段3：回正后直行到终点行（列保持当前）
  int goalR = waypointRows[waypointCount - 1];
  if (goalR > curR) {
    straightAfterRows = goalR - curR;
  }

  return true;
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
  // 使用4列宽度：中心列左右各1列，再根据空隙情况额外扩展1列
  // 总宽度=4列，更贴近实际车宽
  const int BASE_HALF = 1;  // 先取中心列左右各1列（共3列）
  int row = (int)(distance / LAYER_HEIGHT) - 1;
  if (row < 0) row = 0;
  if (row >= ROWS) row = ROWS - 1;

  int leftSpace = 0;
  for (int c = centerCol - 1; c >= 0; c--) {
    if (pointCloudGrid[row][c] != 0) break;
    leftSpace++;
  }
  int rightSpace = 0;
  for (int c = centerCol + 1; c < COLS; c++) {
    if (pointCloudGrid[row][c] != 0) break;
    rightSpace++;
  }

  int extraRight = (rightSpace >= leftSpace) ? 1 : 0;
  int extraLeft = 1 - extraRight;

  startCol = centerCol - BASE_HALF - extraLeft;
  endCol = centerCol + BASE_HALF + extraRight;
  
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
  
  // 增加安全边距检查：路径左右各多1列也应无障碍物
  // 路径宽度5列 + 左右各1列安全边距 = 总共需要7列无障碍
  const int SAFETY_MARGIN = 1;  // 安全边距：左右各1列
  int safeStartCol = startCol - SAFETY_MARGIN;
  int safeEndCol = endCol + SAFETY_MARGIN;
  
  // 检查左侧安全边距（检查安全边距范围内的所有列）
  for (int col = safeStartCol; col < startCol; col++) {
    if (col >= 0 && col < COLS && pointCloudGrid[row][col] != 0) {
      return false;  // 左侧太靠近障碍物
    }
  }
  
  // 检查右侧安全边距（检查安全边距范围内的所有列）
  for (int col = endCol + 1; col <= safeEndCol; col++) {
    if (col >= 0 && col < COLS && pointCloudGrid[row][col] != 0) {
      return false;  // 右侧太靠近障碍物
    }
  }
  
  return true;  // 所有点都是无障碍物，且有安全边距
}

/*
 * 计算当前点两侧障碍物的平衡分数
 * 参数：row - 行索引，col - 列索引
 * 返回：平衡分数（绝对值越小越好，表示两侧障碍物距离相等）
 *       负数表示左侧障碍物更近，正数表示右侧障碍物更近
 */
float calculateObstacleBalanceScore(int row, int col) {
  if (row < 0 || row >= ROWS || col < 0 || col >= COLS) {
    return 999.0f;  // 无效点，返回很大的分数
  }
  
  // 向左查找最近的障碍物
  int leftDist = 0;
  for (int c = col - 1; c >= 0; c--) {
    if (pointCloudGrid[row][c] != 0) {
      break;  // 找到障碍物
    }
    leftDist++;
  }
  if (leftDist == col) {
    leftDist = COLS;  // 左侧没有障碍物，设为最大值
  }
  
  // 向右查找最近的障碍物
  int rightDist = 0;
  for (int c = col + 1; c < COLS; c++) {
    if (pointCloudGrid[row][c] != 0) {
      break;  // 找到障碍物
    }
    rightDist++;
  }
  if (rightDist == COLS - col - 1) {
    rightDist = COLS;  // 右侧没有障碍物，设为最大值
  }
  
  // 返回平衡分数：左右距离差（绝对值越小越好）
  return (float)(rightDist - leftDist);
}

/*
 * 计算当前位置的通道宽度（左右到障碍物的距离之和）
 * 返回值越大表示通道越宽
 */
int calculateCorridorWidth(int row, int col) {
  if (row < 0 || row >= ROWS || col < 0 || col >= COLS) {
    return 0;
  }

  int leftDist = 0;
  for (int c = col - 1; c >= 0; c--) {
    if (pointCloudGrid[row][c] != 0) {
      break;
    }
    leftDist++;
  }
  if (leftDist == col) {
    leftDist = COLS;  // 左侧无遮挡
  }

  int rightDist = 0;
  for (int c = col + 1; c < COLS; c++) {
    if (pointCloudGrid[row][c] != 0) {
      break;
    }
    rightDist++;
  }
  if (rightDist == COLS - col - 1) {
    rightDist = COLS;  // 右侧无遮挡
  }

  return leftDist + rightDist + 1;
}

/*
 * 计算每行偏好列：优先选择侧向净空最大的通道
 */
static void computePreferredCols() {
  int prev = (int)round(CENTER_COL);
  for (int r = 0; r < ROWS; r++) {
    int bestC = -1;
    int bestMin = -1;
    float bestAvg = -1.0f;
    int bestWidth = -1;
    int bestOffset = 9999;
    const int PREF_LOOKAHEAD = 8;

    for (int c = 0; c < COLS; c++) {
      int minClearMin = 9999;
      float minClearSum = 0.0f;
      int samples = 0;
      bool valid = true;

      for (int step = 0; step <= PREF_LOOKAHEAD; step++) {
        int rr = r + step;
        if (rr >= ROWS) break;
        float distCm = (rr + 1) * LAYER_HEIGHT;
        if (pointCloudGrid[rr][c] != 0) {
          valid = false;
          break;
        }
        if (!checkPathWidth(rr, c, distCm)) {
          valid = false;
          break;
        }

        int leftDist = 0;
        for (int c2 = c - 1; c2 >= 0; c2--) {
          if (pointCloudGrid[rr][c2] != 0) break;
          leftDist++;
        }
        if (leftDist == c) leftDist = COLS;

        int rightDist = 0;
        for (int c2 = c + 1; c2 < COLS; c2++) {
          if (pointCloudGrid[rr][c2] != 0) break;
          rightDist++;
        }
        if (rightDist == COLS - c - 1) rightDist = COLS;

        int minClear = min(leftDist, rightDist);
        minClearMin = min(minClearMin, minClear);
        minClearSum += (float)minClear;
        samples++;
      }

      if (!valid || samples == 0) continue;

      float avgClear = minClearSum / (float)samples;
      int offset = abs(c - prev);
      int width = calculateCorridorWidth(r, c);

      if (minClearMin > bestMin ||
          (minClearMin == bestMin && avgClear > bestAvg) ||
          (minClearMin == bestMin && fabs(avgClear - bestAvg) < 1e-3f && width > bestWidth) ||
          (minClearMin == bestMin && fabs(avgClear - bestAvg) < 1e-3f && width == bestWidth && offset < bestOffset)) {
        bestMin = minClearMin;
        bestAvg = avgClear;
        bestWidth = width;
        bestOffset = offset;
        bestC = c;
      }
    }

    if (bestC < 0) {
      preferredCols[r] = prev;
    } else {
      preferredCols[r] = bestC;
      prev = bestC;
    }
  }
}

/*
 * 根据偏好列构造一条平滑路径：逐行向偏好列靠近
 */
static bool buildPreferredPath(int startRow, int startCol,
                               int parentR[ROWS][COLS], int parentC[ROWS][COLS],
                               int& endR, int& endC) {
  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      parentR[r][c] = -1;
      parentC[r][c] = -1;
    }
  }

  int curR = startRow;
  int curC = startCol;
  endR = curR;
  endC = curC;

  for (int r = startRow + 1; r < ROWS; r++) {
    int targetC = (curC + preferredCols[r] * 2) / 3; // 更偏向目标列，提前转弯
    float distCm = (r + 1) * LAYER_HEIGHT;
    int bestC = -1;
    int bestScore = -999999;

    for (int dc = -2; dc <= 2; dc++) {
      int candC = curC + dc;
      if (candC < 0 || candC >= COLS) continue;
      if (pointCloudGrid[r][candC] != 0) continue;
      if (!checkPathWidth(r, candC, distCm)) continue;

      int leftDist = 0;
      for (int c2 = candC - 1; c2 >= 0; c2--) {
        if (pointCloudGrid[r][c2] != 0) break;
        leftDist++;
      }
      if (leftDist == candC) leftDist = COLS;

      int rightDist = 0;
      for (int c2 = candC + 1; c2 < COLS; c2++) {
        if (pointCloudGrid[r][c2] != 0) break;
        rightDist++;
      }
      if (rightDist == COLS - candC - 1) rightDist = COLS;

      int minClear = min(leftDist, rightDist);
      int width = leftDist + rightDist + 1;
      int toward = -abs(targetC - candC) * (r <= 10 ? 20 : 40);
      int earlyKeep = 0;
      if (r <= 10 && abs(candC - preferredCols[r]) > 1) {
        earlyKeep = -120;
      }
      int score = toward + minClear * 120 + width * 10 - abs(candC - curC) * 40 + earlyKeep;

      if (score > bestScore) {
        bestScore = score;
        bestC = candC;
      }
    }

    if (bestC < 0) {
      break;
    }

    parentR[r][bestC] = curR;
    parentC[r][bestC] = curC;
    curR = r;
    curC = bestC;
    endR = curR;
    endC = curC;
  }

  return (endR > startRow);
}

/*
 * 以原点为起点，选最深可达终点，构造“直行->转向->直行->回正->直行”路径
 */
static bool buildMacroPathDirect(int startRow, int startCol,
                                 int parentR[ROWS][COLS], int parentC[ROWS][COLS],
                                 int& endR, int& endC) {
  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      parentR[r][c] = -1;
      parentC[r][c] = -1;
    }
  }

  int bestRow = -1;
  int bestCol = -1;
  int bestMin = -1;
  int bestWidth = -1;

  for (int r = ROWS - 1; r >= startRow; r--) {
    float distCm = (r + 1) * LAYER_HEIGHT;
    for (int c = 0; c < COLS; c++) {
      if (pointCloudGrid[r][c] != 0) continue;
      if (!checkPathWidth(r, c, distCm)) continue;

      int leftDist = 0;
      for (int c2 = c - 1; c2 >= 0; c2--) {
        if (pointCloudGrid[r][c2] != 0) break;
        leftDist++;
      }
      if (leftDist == c) leftDist = COLS;

      int rightDist = 0;
      for (int c2 = c + 1; c2 < COLS; c2++) {
        if (pointCloudGrid[r][c2] != 0) break;
        rightDist++;
      }
      if (rightDist == COLS - c - 1) rightDist = COLS;

      int minClear = min(leftDist, rightDist);
      int width = leftDist + rightDist + 1;

      if (r > bestRow ||
          (r == bestRow && minClear > bestMin) ||
          (r == bestRow && minClear == bestMin && width > bestWidth)) {
        bestRow = r;
        bestCol = c;
        bestMin = minClear;
        bestWidth = width;
      }
    }
    if (bestRow == r && bestCol >= 0) {
      break; // 已找到最深行
    }
  }

  if (bestRow < 0) return false;

  int targetRow = bestRow;
  int targetCol = bestCol;
  int deltaCols = targetCol - startCol;
  int diagSteps = abs(deltaCols);
  int diagSign = (deltaCols >= 0) ? 1 : -1;

  for (int turnRow = startRow; turnRow <= targetRow; turnRow++) {
    if (turnRow + diagSteps > targetRow) break;

    bool ok = true;
    // 段1：直行到 turnRow
    for (int r = startRow + 1; r <= turnRow; r++) {
      float distCm = (r + 1) * LAYER_HEIGHT;
      if (pointCloudGrid[r][startCol] != 0 || !checkPathWidth(r, startCol, distCm)) {
        ok = false;
        break;
      }
    }
    if (!ok) continue;

    // 段2：斜行
    int curR = turnRow;
    int curC = startCol;
    for (int step = 0; step < diagSteps; step++) {
      curR += 1;
      curC += diagSign;
      float distCm = (curR + 1) * LAYER_HEIGHT;
      if (curR > targetRow || curC < 0 || curC >= COLS) {
        ok = false;
        break;
      }
      if (pointCloudGrid[curR][curC] != 0 || !checkPathWidth(curR, curC, distCm)) {
        ok = false;
        break;
      }
    }
    if (!ok) continue;

    // 段3：直行到终点行
    for (int r = curR + 1; r <= targetRow; r++) {
      float distCm = (r + 1) * LAYER_HEIGHT;
      if (pointCloudGrid[r][curC] != 0 || !checkPathWidth(r, curC, distCm)) {
        ok = false;
        break;
      }
    }
    if (!ok) continue;

    // 构造 parent 链
    curR = startRow;
    curC = startCol;
    for (int r = startRow + 1; r <= turnRow; r++) {
      parentR[r][startCol] = curR;
      parentC[r][startCol] = curC;
      curR = r;
      curC = startCol;
    }
    for (int step = 0; step < diagSteps; step++) {
      int nr = curR + 1;
      int nc = curC + diagSign;
      parentR[nr][nc] = curR;
      parentC[nr][nc] = curC;
      curR = nr;
      curC = nc;
    }
    for (int r = curR + 1; r <= targetRow; r++) {
      parentR[r][curC] = curR;
      parentC[r][curC] = curC;
      curR = r;
    }

    endR = targetRow;
    endC = curC;
    return true;
  }

  return false;
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
  
  // 分段规划路径：规划到洞底
  // 步骤1：转向空洞方向
  // 步骤2：直行到洞口入口
  // 步骤3：转回正前方（与洞口垂直）
  // 步骤4：直行通过空洞到洞底
  float entranceDist = maxGap.entranceDist;
  float gapBottomDist = maxGap.distance;  // 洞底距离
  float entranceAngle = gapCenterAngle;  // 路径地图使用这个角度
  
  // 标记到洞口入口的路径
  markPathSegment(startDist, startAngle, entranceDist, entranceAngle);
  
  // 标记通过空洞到洞底的路径（转回正前方后，角度为0）
  markPathSegment(entranceDist, 0.0, gapBottomDist, 0.0);
  
  // 计算转回角度：转到与洞口垂直的方向（考虑倾斜角度）
  // 转回角度 = -entranceAngle + tiltAngle
  float executionTurnBackAngle = -maxGap.centerAngle - maxGap.tiltAngle;
  // 打印完整步骤：包括通过空洞到洞底
  printExecutionSteps(startDist, startAngle, entranceDist, maxGap.centerAngle, executionTurnBackAngle, gapBottomDist);
  
  // 打印地图
  printPathMap(); // 保持一次打印
}

/*
 * 分析路径并打印实际执行步骤
 */
void printExecutionSteps(float startDist, float startAngle, float entranceDist, float entranceAngle, 
                         float turnBackAngle, float gapBottomDist) {
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
  
  // 如果角度偏差很小（小于5度），直接直行到洞口入口，然后通过空洞到洞底
  if (abs(entranceAngle) < 5.0) {
    float distToEntrance = entranceDist - startDist;
    float distThroughGap = gapBottomDist - entranceDist;
    Serial.print("  步骤1: 直行 ");
    Serial.print(distToEntrance, 1);
    Serial.println(" cm (到达洞口入口)");
    Serial.print("  步骤2: 直行 ");
    Serial.print(distThroughGap, 1);
    Serial.println(" cm (通过空洞到洞底)");
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
  
  // 步骤4：直行通过空洞到洞底
  stepNum++;
  float distThroughGap = gapBottomDist - entranceDist;
  Serial.print("  步骤");
  Serial.print(stepNum);
  Serial.print(": 直行 ");
  Serial.print(distThroughGap, 1);
  Serial.println(" cm (通过空洞到洞底)");
  
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
    Serial.println("【路径执行完成】短暂停顿，准备重新规划...");
    delay(REPLAN_DELAY_MS);
    pathCompleted = false; // 清除标志
    setCurrentAction("", 0.0f);
  }
  
  // 如果之前正在导航，先停止当前动作
  if (navState != NAV_IDLE && navState != NAV_MOVE_THROUGH) {
    stopMotors();
    delay(100);
  }
  
  if (!pathAvailable || waypointCount < 2) {
    navState = NAV_IDLE;
    setCurrentAction("", 0.0f);
    return;
  }

  executedSegments = 0;
  pendingBackThenStraight = false;
  macroActive = false;

  if (macroPlanValid) {
    startMacroExecution();
    return;
  }

  // 装载第一段航段（起点到第一个目标点）
  if (!loadNextWaypoint()) {
    navState = NAV_IDLE;
    setCurrentAction("", 0.0f);
    return;
  }

  navState = NAV_TURN_TO_GAP;
  
  Serial.print("【执行】步骤1: ");
  if (navTurnAngle < -0.5f) {
    Serial.print("左转 ");
    Serial.print(abs(navTurnAngle), 1);
    Serial.println(" 度");
    setCurrentAction("left", abs(navTurnAngle));
    turnLeft(abs(navTurnAngle));
  } else if (navTurnAngle > 0.5f) {
    Serial.print("右转 ");
    Serial.print(navTurnAngle, 1);
    Serial.println(" 度");
    setCurrentAction("right", navTurnAngle);
    turnRight(navTurnAngle);
  } else {
    Serial.println("无需转向，直接前进");
    navTurnAngle = 0.0f;
    navState = NAV_MOVE_TO_ENTRANCE;
    setCurrentAction("line", navForwardDist);
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

  // 如果电机未启用，直接停止并请求重规划
  if (motorDriveState != MOTOR_DRIVE_ENABLED &&
      navState != NAV_IDLE &&
      navState != NAV_SEARCHING) {
    stopMotors();
    navState = NAV_IDLE;
    requestReplan("电机未启用");
    return;
  }

  if (stepPauseActive) {
    return;
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
      setCurrentAction("left", abs(navTurnAngle));
      turnLeft(abs(navTurnAngle));
    } else {
      setCurrentAction("right", navTurnAngle);
      turnRight(navTurnAngle);
    }
    // 检查转向是否完成
    if (!isTurning) {
      delayAfterAction();
      if (macroActive && macroPhase == 1) {
        navForwardDist = macroDiagDistCm;
        navState = NAV_MOVE_TO_ENTRANCE;
        macroPhase = 2;
        Serial.print("【执行】宏路径: 直行 ");
        Serial.print(macroDiagDistCm, 1);
        Serial.println(" cm");
        setCurrentAction("line", navForwardDist);
        requestStepPause("转向完成");
        return;
      } else {
        Serial.print("【执行】步骤1完成，步骤2: 直行 ");
        Serial.print(navForwardDist, 1);
        Serial.println(" cm");
        navState = NAV_MOVE_TO_ENTRANCE;
        setCurrentAction("line", navForwardDist);
        requestStepPause("转向完成");
        return;
      }
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

    // 行进中尝试重规划（使用最新点云），平滑模式：不强制急停
    // 宏路径执行中不动态重规划，避免中途被打断
    if (!macroActive && tryReplanAndSwitchPath(false)) {
      Serial.println("【动态重规划】使用新路径，重新执行");
      return;
    }
    
    // 只在没有移动时才启动新的移动，避免重复启动
    if (!isMoving && !wasMoving) {
      setCurrentAction("line", navForwardDist);
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
      delayAfterAction();
      Serial.println("【执行】步骤2完成，节点到达");
      // 推进到下一个航点
      advanceWaypoint();
      wasMoving = false;
      executedSegments++;

      if (macroActive) {
        if (macroPhase == 0) {
          // 完成直行1
          if (fabs(macroTurnDeg) > 0.5f) {
            navTurnAngle = macroTurnDeg;
            navState = NAV_TURN_TO_GAP;
            macroPhase = 1;
            Serial.print("【执行】宏路径: ");
            Serial.print(macroTurnDeg < 0 ? "左转 " : "右转 ");
            Serial.print(fabs(macroTurnDeg), 1);
            Serial.println(" 度");
            setCurrentAction(macroTurnDeg < 0 ? "left" : "right", fabs(macroTurnDeg));
            requestStepPause("直行完成");
            return;
          } else {
            navForwardDist = macroStraight3Cm;
            navState = NAV_MOVE_TO_ENTRANCE;
            macroPhase = 4;
            Serial.print("【执行】宏路径: 直行 ");
            Serial.print(macroStraight3Cm, 1);
            Serial.println(" cm");
            setCurrentAction("line", navForwardDist);
            requestStepPause("直行完成");
            return;
          }
        } else if (macroPhase == 2) {
          // 完成直行2
          navTurnBackAngle = -macroTurnDeg;
          navState = NAV_TURN_BACK;
          macroPhase = 3;
          Serial.print("【执行】宏路径: ");
          Serial.print(macroTurnDeg > 0 ? "左转 " : "右转 ");
          Serial.print(fabs(macroTurnDeg), 1);
          Serial.println(" 度 (侧移后回正)");
          setCurrentAction(macroTurnDeg > 0 ? "left" : "right", fabs(macroTurnDeg));
          requestStepPause("直行完成");
          return;
        } else if (macroPhase == 4) {
          // 完成直行3
          stopMotors();
          navState = NAV_IDLE;
          macroActive = false;
          requestReplan("宏路径完成");
          setCurrentAction("", 0.0f);
          return;
        }
      }

      // 每帧只执行有限航段，执行完后回正再重新规划
      const int MAX_SEGMENTS_PER_CYCLE = 2; // 尽量形成“直行-转向-直行-回正-直行”节奏
      if (executedSegments >= MAX_SEGMENTS_PER_CYCLE) {
        if (fabs(lastSegmentTurnAngle) > 0.5f) {
          navTurnBackAngle = -lastSegmentTurnAngle;
          navState = NAV_TURN_BACK;
          pendingBackThenStraight = true;
          requestStepPause("直行完成");
        } else {
          stopMotors();
          navState = NAV_IDLE;
          requestReplan("本轮航段完成");
        }
        return;
      }

      // 判断是否还有后续航段
      if (waypointIndex >= waypointCount) {
        stopMotors();
        navState = NAV_IDLE;
        requestReplan("路径完成(本轮)");
      } else {
        // 装载下一段
        if (loadNextWaypoint()) {
          navState = NAV_TURN_TO_GAP;
          requestStepPause("直行完成");
          return;
        } else {
          navState = NAV_IDLE;
          pathCompleted = true;
        }
      }
    }
  }
  else if (navState == NAV_TURN_BACK) {
    if (fabs(navTurnBackAngle) > 0.5f) {
      setCurrentAction(navTurnBackAngle < 0 ? "left" : "right", fabs(navTurnBackAngle));
      if (navTurnBackAngle < 0) {
        turnLeft(abs(navTurnBackAngle));
      } else {
        turnRight(navTurnBackAngle);
      }
      if (!isTurning) {
        delayAfterAction();
        stopMotors();
        if (macroActive && macroPhase == 3) {
          if (REPLAN_AFTER_TURNBACK) {
            navState = NAV_IDLE;
            macroActive = false;
            requestStepPause("回正完成");
            requestReplan("回正完成");
            setCurrentAction("", 0.0f);
          } else if (macroStraight3Cm > 0.5f) {
            navForwardDist = macroStraight3Cm;
            navState = NAV_MOVE_TO_ENTRANCE;
            macroPhase = 4;
            Serial.print("【执行】宏路径: 直行 ");
            Serial.print(macroStraight3Cm, 1);
            Serial.println(" cm");
            setCurrentAction("line", navForwardDist);
            requestStepPause("回正完成");
          } else {
            navState = NAV_IDLE;
            macroActive = false;
            requestStepPause("回正完成");
            requestReplan("宏路径完成");
            setCurrentAction("", 0.0f);
          }
          return;
        }
        if (pendingBackThenStraight) {
          pendingBackThenStraight = false;
          // 尝试在回正后执行一次直行（仅当下一段无需转向）
          if (loadNextWaypoint() && fabs(navTurnAngle) <= 0.5f) {
            navTurnAngle = 0.0f;
            navState = NAV_MOVE_TO_ENTRANCE;
            setCurrentAction("line", navForwardDist);
            requestStepPause("回正完成");
          } else {
            navState = NAV_IDLE;
          requestStepPause("回正完成");
            requestReplan("回正完成");
            setCurrentAction("", 0.0f);
          }
        } else {
          navState = NAV_IDLE;
        requestStepPause("回正完成");
          requestReplan("回正完成");
          setCurrentAction("", 0.0f);
        }
      }
    } else {
      delayAfterAction();
      if (macroActive && macroPhase == 3) {
        if (REPLAN_AFTER_TURNBACK) {
          navState = NAV_IDLE;
          macroActive = false;
        requestStepPause("回正完成");
          requestReplan("回正完成");
          setCurrentAction("", 0.0f);
        } else if (macroStraight3Cm > 0.5f) {
          navForwardDist = macroStraight3Cm;
          navState = NAV_MOVE_TO_ENTRANCE;
          macroPhase = 4;
          Serial.print("【执行】宏路径: 直行 ");
          Serial.print(macroStraight3Cm, 1);
          Serial.println(" cm");
          setCurrentAction("line", navForwardDist);
          requestStepPause("回正完成");
        } else {
          navState = NAV_IDLE;
          macroActive = false;
        requestStepPause("回正完成");
          requestReplan("宏路径完成");
          setCurrentAction("", 0.0f);
        }
      } else if (pendingBackThenStraight) {
        pendingBackThenStraight = false;
        if (loadNextWaypoint() && fabs(navTurnAngle) <= 0.5f) {
          navTurnAngle = 0.0f;
          navState = NAV_MOVE_TO_ENTRANCE;
          setCurrentAction("line", navForwardDist);
          requestStepPause("回正完成");
        } else {
          navState = NAV_IDLE;
        requestStepPause("回正完成");
          requestReplan("回正完成");
          setCurrentAction("", 0.0f);
        }
      } else {
        navState = NAV_IDLE;
      requestStepPause("回正完成");
        requestReplan("回正完成");
        setCurrentAction("", 0.0f);
      }
    }
  }
  else if (navState == NAV_MOVE_THROUGH) {
    // 新规划不使用，防御留空
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
  if (motorDriveState != MOTOR_DRIVE_ENABLED) {
    stopMotors();
    return;
  }
  moveDistance(distance);
}

void turnLeft(float angle) {
  // TODO: 实现左转控制
  // 参数: angle - 转向角度（度）
  if (motorDriveState != MOTOR_DRIVE_ENABLED) {
    stopMotors();
    return;
  }
  turnAngle(angle);
}

void turnRight(float angle) {
  // TODO: 实现右转控制
  // 参数: angle - 转向角度（度）
  if (motorDriveState != MOTOR_DRIVE_ENABLED) {
    stopMotors();
    return;
  }
  turnAngle(angle*(-1.0f));
}

void stopMotors() {
  setSpeed(0, 0);
  isMoving = false;
  isTurning = false;
}

static void requestReplan(const char* reason) {
  pathCompleted = true;
  pendingReplanReason = reason;
  pendingReplanLog = true;
  Serial.print("【重规划请求】");
  Serial.println(reason);
}

static void setCurrentAction(const char* action, float value) {
  if (action == nullptr || strlen(action) == 0) {
    strcpy(currentAction, "");
    currentActionValue = 0.0f;
    return;
  }
  strcpy(currentAction, action);
  currentActionValue = value;
}

static void delayAfterAction() {
  if (ACTION_STEP_DELAY_MS > 0) {
    delay(ACTION_STEP_DELAY_MS);
  }
}

bool isStepPauseActive() {
  return stepPauseActive;
}

void resumeStepPause() {
  if (!stepPauseActive) return;
  stepPauseActive = false;
  Serial.println("【继续】按键继续执行");
}

static void requestStepPause(const char* reason) {
  if (motorDriveState != MOTOR_DRIVE_ENABLED) return;
  stepPauseActive = true;
  stopMotors();
  Serial.print("【暂停】");
  Serial.println(reason);
}

static void computeMacroTurnAndDist(int straightRows, int diagSteps, int diagRows, int startCol, int diagSign,
                                    float& turnDeg, float& diagDistCm) {
  turnDeg = 0.0f;
  diagDistCm = 0.0f;
  if (diagSteps <= 0) return;

  int endCol = startCol + diagSign * diagSteps;
  if (endCol < 0) endCol = 0;
  if (endCol >= COLS) endCol = COLS - 1;
  int deltaCols = endCol - startCol;

  int forwardRows = diagRows;
  if (forwardRows <= 0) forwardRows = diagSteps;
  float forwardCm = forwardRows * LAYER_HEIGHT;
  float lateralCm = 0.0f;
  int baseRow = straightRows;
  int stepCount = diagSteps;

  for (int i = 1; i <= stepCount; i++) {
    int rowIndex = baseRow + i;
    if (rowIndex < 0) rowIndex = 0;
    if (rowIndex >= TAN_TABLE_SIZE) rowIndex = TAN_TABLE_SIZE - 1;
    lateralCm += ROW_STEP_WIDTH[rowIndex] * diagSign;
  }

  float startDistCm = baseRow * LAYER_HEIGHT;
  float endDistCm = (baseRow + forwardRows) * LAYER_HEIGHT;
  float startAngleDeg = -((startCol - CENTER_COL) * ANGLE_STEP);
  float endAngleDeg = -((endCol - CENTER_COL) * ANGLE_STEP);
  float startX = startDistCm * tan(radians(startAngleDeg));
  float endX = endDistCm * tan(radians(endAngleDeg));
  float lateralFromOrigin = endX - startX;
  lateralCm = lateralFromOrigin;

  float angleRad = atan2(lateralCm, forwardCm);
  turnDeg = degrees(angleRad);
  diagDistCm = sqrtf(lateralCm * lateralCm + forwardCm * forwardCm);

  Serial.println("【宏路径角度计算】");
  Serial.printf("  直行行数=%d, 侧移步数=%d, 斜行行数=%d, 起始列=%d, 目标列=%d, 列偏移=%d\n",
                straightRows, diagSteps, diagRows, startCol, endCol, deltaCols);
  Serial.printf("  转向起点行=%d, 侧移累积行=%d\n", baseRow, baseRow + stepCount);
  Serial.println("  横向位移公式=终点x-起点x, x=距离*tan(列角)");
  Serial.printf("  起点: 距离=%.2fcm, 列角=%.2f°, x=%.2f\n",
                startDistCm, startAngleDeg, startX);
  Serial.printf("  终点: 距离=%.2fcm, 列角=%.2f°, x=%.2f\n",
                endDistCm, endAngleDeg, endX);
  Serial.printf("  横向位移=%.2f cm, 前向位移=%.2f cm\n", lateralCm, forwardCm);
  Serial.printf("  转向角=%.2f 度, 斜向直行=%.2f cm\n", turnDeg, diagDistCm);
}

static void startMacroExecution() {
  macroActive = true;
  macroPhase = 0;

  if (macroStraight1Cm > 0.5f) {
    Serial.print("【执行】宏路径: 直行 ");
    Serial.print(macroStraight1Cm, 1);
    Serial.println(" cm");
    navTurnAngle = 0.0f;
    navForwardDist = macroStraight1Cm;
    navState = NAV_MOVE_TO_ENTRANCE;
  } else if (fabs(macroTurnDeg) > 0.5f) {
    navTurnAngle = macroTurnDeg;
    navState = NAV_TURN_TO_GAP;
    macroPhase = 1;
    Serial.print("【执行】宏路径: ");
    Serial.print(macroTurnDeg < 0 ? "左转 " : "右转 ");
    Serial.print(fabs(macroTurnDeg), 1);
    Serial.println(" 度");
  } else if (macroStraight3Cm > 0.5f) {
    Serial.print("【执行】宏路径: 直行 ");
    Serial.print(macroStraight3Cm, 1);
    Serial.println(" cm");
    navTurnAngle = 0.0f;
    navForwardDist = macroStraight3Cm;
    navState = NAV_MOVE_TO_ENTRANCE;
    macroPhase = 4;
  } else {
    macroActive = false;
    navState = NAV_IDLE;
    requestReplan("宏路径为空");
  }
}
