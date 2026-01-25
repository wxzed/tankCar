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
#ifndef PI
#define PI 3.14159265358979323846
#endif

// 全局变量
bool gapFound = false;

// 路径地图（用于标记路径，2表示路径）
int pathMap[ROWS][COLS];

// 导航状态机（枚举定义在头文件中）
NavigationState navState = NAV_IDLE;
float navForwardDist = 0.0;  // 到洞口入口的距离
float navTurnAngle = 0.0;
float navTurnBackAngle = 0.0;
static NavigationState lastLoggedNavState = NAV_IDLE; // 调试：记录上一次打印的状态
static bool pathCompleted = false; // 标记路径是否刚完成，需要在重新检测前延时
static const unsigned long REPLAN_DELAY_MS = 0; // 重新规划前的短暂停顿

// 路径规划结果
static bool pathAvailable = false;
static int  goalRow = -1;
static int  goalCol = -1;
static float plannedHeadingDeg = 0.0f;
static float plannedDistanceCm = 0.0f;
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
static bool pendingTurnBackStraight = false; // 回正后插入一次直行，再重规划
static const float TURNBACK_STRAIGHT_CM = 20.0f;
static bool macroPlanValid = false;
static bool macroActive = false;
static int macroPhase = 0; // 0=直行1,1=转向,2=直行2,3=回正,4=直行3
static bool stepPauseActive = false; // 步进暂停：每步完成后等待按键继续
static bool stepPauseEnabled = true; // 步进模式总开关
static float macroStraight1Cm = 0.0f;
static float macroDiagDistCm = 0.0f;
static float macroStraight3Cm = 0.0f;
static float macroTurnDeg = 0.0f;
static const bool REPLAN_AFTER_TURNBACK = true; // 回正后立即重规划，不再执行最后直行
static char currentAction[10] = ""; // 当前执行动作：left/right/line
static float currentActionValue = 0.0f;
static const unsigned long ACTION_STEP_DELAY_MS = 0; // 每步完成后停顿，便于观察
static const float FRONT_BLOCK_DISTANCE_CM = 50.0f; // 前方阻塞检测距离
static const unsigned long BLOCKED_TURN_COOLDOWN_MS = 1200; // 避免频繁触发转向
static unsigned long lastBlockedTurnMs = 0;
static const int MIN_FORWARD_ROWS_FOR_KEEP = 15; // 至少可达10行(约50cm)才继续直行
static const float BLOCKED_TURN_ANGLE_DEG = 45.0f; // 阻塞转向角度
static const float DEADEND_TURN_ANGLE_DEG = 135.0f; // 死胡同转向角度
static const float DEADEND_BACK_CM = 20.0f; // 死胡同时先后退距离
static const float MAX_INITIAL_STRAIGHT_CM = 40.0f; // 仅在“纯直行开局”时限制

// 搜索模式相关变量
static int searchStep = 0;  // 搜索步骤：0=旋转, 1=检测, 2=旋转, 3=检测...
static float searchAngleStep = 30.0; // 每次搜索旋转的角度（度）
static float searchTotalAngle = 0.0;  // 累计旋转的总角度（度）
static float maxSearchAngle = 360.0;  // 最大搜索角度（度），360度表示旋转一圈
static bool searchDirectionLeft = true; // 搜索方向：true=左转，false=右转
static bool searchSingleTurn = false; // 只转一次（90度）后结束搜索
static bool deadendBackActive = false; // 死胡同后退中
static float singleTurnTargetDeg = 0.0f; // 单次转向角度
static bool deadendTurnOnly = false; // 死胡同后退完成后只转向不直行
static bool pendingDeadendReplan = false; // 死胡同后退完成，等待重规划判断
static bool deadendReturnActive = false; // 死胡同后退完成提示

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

bool isDeadendReturnActive() {
  return deadendReturnActive;
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

void navigateToGap();
void printPathMap();
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
static void smoothShortLateralRuns();
static bool isPlanningObstacle(int row, int col);
static void markWaypointsOnMap();
bool loadNextWaypoint();    // 计算下一个航段的转角与距离
void advanceWaypoint();     // 前进到下一个节点后更新当前位置
bool checkAndBrakeForCollisionImmediate(); // 最新帧的紧急刹停
bool tryReplanAndSwitchPath(bool stopImmediately); // 在行进阶段尝试重规划，支持平滑/急停切换
void printPlannedStepsDebug(); // 打印规划出的动作序列（供调试串口查看）
static void requestReplan(const char* reason); // 标记本轮结束并触发重规划
static bool buildMacroPlan(int& straightRows, int& diagSteps, int& diagRows,
                           int& straightAfterRows, int& diagSign);
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
static bool detectCollisionNarrow();
static bool detectCollisionWide();
static bool detectCollisionStop();
static bool isFrontFullyBlocked(float distanceCm);
static void enterSearchMode(SearchReason reason, const char* logMessage, bool singleTurn, float singleTurnAngleDeg);
static void handleTurnToGap();
static void handleMoveToEntrance();
static void handleMoveCompleted();
static void handleTurnBack();
static void handleSearch();

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

static bool isFrontFullyBlocked(float distanceCm) {
  if (distanceCm <= 0.0f) return false;
  int maxRow = (int)(distanceCm / LAYER_HEIGHT) - 1;
  if (maxRow < 0) maxRow = 0;
  if (maxRow >= ROWS) maxRow = ROWS - 1;

  for (int row = 0; row <= maxRow; row++) {
    float distCm = (row + 1) * LAYER_HEIGHT;
    bool hasPassable = false;
    for (int col = 1; col < COLS - 1; col++) {
      if (pointCloudGrid[row][col] != 0) continue;
      if (checkPathWidth(row, col, distCm)) {
        hasPassable = true;
        break;
      }
    }
    if (hasPassable) {
      return false;
    }
  }
  return true;
}

void runGapTest()  {
  // 始终刷新占据网格，确保碰撞检测/显示使用最新数据
  fillPointCloudGrid();

  // 步进暂停时不进行重规划
  if (stepPauseActive) {
    return;
  }

  // 正在执行宏路径/航段时不刷新规划，避免中途改路径
  if (navState != NAV_IDLE && navState != NAV_SEARCHING) {
    return;
  }

  // 前方近距离全阻塞：转向后重新测量再规划
  if (navState == NAV_IDLE &&
      motorDriveState == MOTOR_DRIVE_ENABLED &&
      !searchSingleTurn &&
      (millis() - lastBlockedTurnMs > BLOCKED_TURN_COOLDOWN_MS) &&
      isFrontFullyBlocked(FRONT_BLOCK_DISTANCE_CM)) {
    lastBlockedTurnMs = millis();
    searchSingleTurn = true;
    enterSearchMode(SEARCH_NO_GAP, "【前方阻塞】近距离全为障碍，转向45度后重新规划", true, BLOCKED_TURN_ANGLE_DEG);
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

  // 可达距离过短，视为前方死胡同，先转向再测量重规划
  if (navState == NAV_IDLE &&
      motorDriveState == MOTOR_DRIVE_ENABLED &&
      !searchSingleTurn &&
      (millis() - lastBlockedTurnMs > BLOCKED_TURN_COOLDOWN_MS) &&
      (goalRow + 1) < MIN_FORWARD_ROWS_FOR_KEEP) {
    float reachableCm = (goalRow + 1) * LAYER_HEIGHT;
    lastBlockedTurnMs = millis();
    if (motorDriveState == MOTOR_DRIVE_ENABLED) {
      deadendBackActive = true;
      navForwardDist = -DEADEND_BACK_CM;
      navState = NAV_MOVE_TO_ENTRANCE;
      setCurrentAction("line", navForwardDist);
      Serial.print("【前方死胡同】可达距离: ");
      Serial.print(reachableCm, 1);
      Serial.print(" cm，先后退 ");
      Serial.print(DEADEND_BACK_CM, 1);
      Serial.println(" cm");
    }
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
  int diagRows = 0;
  int straightAfterRows = 0;
  int diagSign = 0;
  if (buildMacroPlan(straightRows, diagSteps, diagRows, straightAfterRows, diagSign)) {
    macroPlanValid = true;
    macroStraight1Cm = straightRows * LAYER_HEIGHT;
    macroStraight3Cm = straightAfterRows * LAYER_HEIGHT;
    int startCol = waypointCols[0];
    computeMacroTurnAndDist(straightRows, diagSteps, diagRows, startCol, diagSign,
                            macroTurnDeg, macroDiagDistCm);
    if (fabs(macroTurnDeg) <= 0.5f && macroStraight1Cm > MAX_INITIAL_STRAIGHT_CM) {
      Serial.print("【直行上限】纯直行开局，距离 ");
      Serial.print(macroStraight1Cm, 1);
      Serial.print(" cm，限制为 ");
      Serial.print(MAX_INITIAL_STRAIGHT_CM, 1);
      Serial.println(" cm");
      macroStraight1Cm = MAX_INITIAL_STRAIGHT_CM;
    }
  } else {
    macroPlanValid = false;
  }

  if (pendingDeadendReplan) {
    pendingDeadendReplan = false;
    if ((goalRow + 1) < MIN_FORWARD_ROWS_FOR_KEEP) {
      deadendTurnOnly = true;
      navTurnAngle = -DEADEND_TURN_ANGLE_DEG;
      navState = NAV_TURN_TO_GAP;
      setCurrentAction("left", DEADEND_TURN_ANGLE_DEG);
      Serial.println("【前方死胡同】深度不足，转向45度后再规划");
      deadendReturnActive = false;
      return;
    }
    deadendReturnActive = false;
  }
  
  // 执行导航：电机启用才执行，否则仅输出规划结果
  if (motorDriveState == MOTOR_DRIVE_ENABLED) {
    navigateToGap();
  } else {
    Serial.println("【测试模式】电机未启用，未执行导航");
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
  if (isPlanningObstacle(startRow, startCol)) {
    bool found = false;
    const int SPAN = 3;
    for (int off = 1; off <= SPAN && !found; off++) {
      if (startCol - off >= 0 && !isPlanningObstacle(startRow, startCol - off)) {
        startCol -= off;
        found = true;
      } else if (startCol + off < COLS && !isPlanningObstacle(startRow, startCol + off)) {
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
      if (isPlanningObstacle(r, c2)) break;
      leftDist++;
    }
    if (leftDist == c) leftDist = COLS;

    for (int c2 = c + 1; c2 < COLS; c2++) {
      if (isPlanningObstacle(r, c2)) break;
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
    int effectiveMinSideClear = minSideClear;
    if (!hasLeftObstacle || !hasRightObstacle) {
      // 一侧无遮挡时，限制净空奖励，避免向开阔侧无限漂移
      effectiveMinSideClear = min(minSideClear, IDEAL_CLEAR);
    }

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
    if (abs(sideDiff) >= 3) {
      // 通道足够宽时，几乎禁用“朝开阔侧偏移”的倾向，避免因边缘细节偏线
      float sideBiasScale = (minSideClear >= IDEAL_CLEAR) ? 0.0f : 0.6f;
      if (minSideClear < IDEAL_CLEAR) {
        // 放宽“贴中线”惩罚，允许更早偏向开阔侧
        straightWeight *= 0.5f;
      }
      // 朝开阔侧移动时给予奖励（限制奖励幅度，避免贴边）
      int towardOpen = (sideDiff > 0) ? (int)round(CENTER_COL - c) : (int)round(c - CENTER_COL);
      if (towardOpen > 0) {
        int cappedCols = min(towardOpen, 8);               // 最多奖励偏移8列
        sideBias = cappedCols * 160.0f * sideBiasScale;
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
    if (effectiveMinSideClear >= IDEAL_CLEAR) {
      clearReward = (effectiveMinSideClear - IDEAL_CLEAR + 1) * (r <= 8 ? 180.0f : 120.0f);
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
      if (isPlanningObstacle(rr, c2)) break;
        l2++;
      }
      if (l2 == c) l2 = COLS;
      for (int c2 = c + 1; c2 < COLS; c2++) {
      if (isPlanningObstacle(rr, c2)) break;
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

    // 当前方宽裕且通道足够宽时，减弱对“偏好列”的追随，避免边缘小变化引发偏移
    if (minSideClear >= IDEAL_CLEAR && forwardClearRows >= 3) {
      preferredPenalty *= 0.05f;
    }

    // 通道宽且左右相对均衡时，额外惩罚偏离中心，减少不必要偏移
    float centerPenalty = 0.0f;
    if (minSideClear >= REQUIRED_CLEAR && abs(sideDiff) <= 3) {
      float centerOffset = abs(c - (int)round(CENTER_COL));
      centerPenalty = centerOffset * 600.0f;
    }
    // 单侧无遮挡时，压制“向开阔侧漂移”
    float openSidePenalty = 0.0f;
    if ((!hasLeftObstacle || !hasRightObstacle) && minSideClear >= REQUIRED_CLEAR) {
      float centerOffset = abs(c - (int)round(CENTER_COL));
      openSidePenalty = centerOffset * 520.0f;
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
                  - preferredPenalty - centerPenalty - openSidePenalty;

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
      if (isPlanningObstacle(nr, nc)) continue;

      // 避免“前后直，中间一行偏移”：当前方仍宽裕时延后侧移
      if (abs(nr - r) == 1 && abs(nc - c) == 1) {
        // 若本行前向直走仍安全，则直接禁止侧移
        float curDistCm = (r + 1) * LAYER_HEIGHT;
        if (checkPathWidth(r, c, curDistCm)) {
          int l0 = 0;
          for (int c2 = c - 1; c2 >= 0; c2--) {
            if (isPlanningObstacle(r, c2)) break;
            l0++;
          }
          if (l0 == c) l0 = COLS;
          int r0 = 0;
          for (int c2 = c + 1; c2 < COLS; c2++) {
            if (isPlanningObstacle(r, c2)) break;
            r0++;
          }
          if (r0 == COLS - c - 1) r0 = COLS;
          int minSide0 = min(l0, r0);
          if (minSide0 >= REQUIRED_CLEAR) {
            continue;
          }
        }

        bool forwardOpen = true;
        int nextR = r + 1;
        if (nextR >= ROWS) {
          forwardOpen = false;
        } else {
          float nextDistCm = (nextR + 1) * LAYER_HEIGHT;
          if (isPlanningObstacle(nextR, c) || !checkPathWidth(nextR, c, nextDistCm)) {
            forwardOpen = false;
          }
        }
        if (forwardOpen) {
          const int DIAG_AVOID_LOOKAHEAD = 3;
          bool nearTight = false;
          for (int step = 1; step <= DIAG_AVOID_LOOKAHEAD; step++) {
            int rr = r + step;
            if (rr >= ROWS) break;
            float distCm = (rr + 1) * LAYER_HEIGHT;
            if (isPlanningObstacle(rr, c) || !checkPathWidth(rr, c, distCm)) {
              nearTight = true;
              break;
            }
            int l2 = 0;
            for (int c2 = c - 1; c2 >= 0; c2--) {
              if (isPlanningObstacle(rr, c2)) break;
              l2++;
            }
            if (l2 == c) l2 = COLS;
            int r2 = 0;
            for (int c2 = c + 1; c2 < COLS; c2++) {
              if (isPlanningObstacle(rr, c2)) break;
              r2++;
            }
            if (r2 == COLS - c - 1) r2 = COLS;
            int minSide2 = min(l2, r2);
            if (minSide2 < REQUIRED_CLEAR) {
              nearTight = true;
              break;
            }
          }
          if (!nearTight) {
            continue;
          }
        }
      }

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
  smoothShortLateralRuns();
  goalRow = waypointRows[waypointCount - 1];
  goalCol = waypointCols[waypointCount - 1];
  // 重新标记路径地图，确保显示与修正后的航点一致
  initializePathMap();
  markWaypointsOnMap();
  waypointIndex = 1; // 第0个是起点，执行从第1个节点开始

  // 航向角：相对中心列的偏移，左为正
  float offsetCol = goalCol - CENTER_COL;
  plannedHeadingDeg = -offsetCol * ANGLE_STEP;

  // 路径长度估算（行步长5cm，斜向乘√2）
  float plannedPathLengthCm = 0.0f;
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

static void smoothShortLateralRuns() {
  if (waypointCount < 3) return;
  const int MAX_SHORT_RUN = 2; // 最多2行的短侧移会尝试拉直

  int i = 1;
  while (i < waypointCount - 1) {
    int baseCol = waypointCols[i - 1];
    if (waypointCols[i] == baseCol) {
      i++;
      continue;
    }

    int start = i;
    while (i < waypointCount && waypointCols[i] != baseCol) {
      i++;
    }
    if (i >= waypointCount) break; // 未回到原列，跳过
    int end = i - 1;
    int runLen = end - start + 1;

    if (runLen <= MAX_SHORT_RUN) {
      bool canStraight = true;
      for (int k = start; k <= end; k++) {
        int row = waypointRows[k];
        float distCm = (row + 1) * LAYER_HEIGHT;
        if (baseCol < 0 || baseCol >= COLS) {
          canStraight = false;
          break;
        }
        if (isPlanningObstacle(row, baseCol) || !checkPathWidth(row, baseCol, distCm)) {
          canStraight = false;
          break;
        }
      }
      if (canStraight) {
        for (int k = start; k <= end; k++) {
          waypointCols[k] = baseCol;
        }
      }
    }
  }
}

static void markWaypointsOnMap() {
  for (int i = 0; i < waypointCount; i++) {
    int r = waypointRows[i];
    int c = waypointCols[i];
    float distCm = (r + 1) * LAYER_HEIGHT;
    int startCol = 0;
    int endCol = 0;
    calculateCarWidthColumns(distCm, c, startCol, endCol);
    for (int col = startCol; col <= endCol; col++) {
      if (col < 0 || col >= COLS) continue;
      if (pointCloudGrid[r][col] == 0) {
        pathMap[r][col] = 2;
      }
    }
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
  int diagRows = 0;
  int straightAfterRows = 0;
  int diagSign = 0; // 右为正，左为负
  if (buildMacroPlan(straightRows, diagSteps, diagRows, straightAfterRows, diagSign)) {
    Serial.println("【规划动作序列】(简化4段式: 直行->转向->直行->回正->直行)");
    int step = 1;
    if (straightRows > 0) {
      Serial.printf("  步骤%d: 直行 %.1f cm\n", step++, straightRows * LAYER_HEIGHT);
    }
    if (diagSteps > 0) {
      float sideTurn = 0.0f;
      float diagDist = 0.0f;
      int startCol = waypointCols[0];
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

static bool buildMacroPlan(int& straightRows, int& diagSteps, int& diagRows,
                           int& straightAfterRows, int& diagSign) {
  straightRows = 0;
  diagSteps = 0;
  diagRows = 0;
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
  int diagStartRow = curR;
  int lastShiftRow = curR;

  while (i < waypointCount) {
    int nr = waypointRows[i];
    int nc = waypointCols[i];
    int dr = nr - curR;
    int dc = nc - curC;
    if (dr <= 0) {
      return false; // 非前进路径无法简化
    }
    if (dc == 0) {
      curR = nr;
      curC = nc;
      i++;
      continue;
    }
    int sign = (dc > 0) ? 1 : -1;
    if (sign != diagSign) {
      return false; // 侧移方向变化，无法简化
    }
    if (abs(dr) != 1 || abs(dc) != 1) {
      return false; // 仅支持45度斜向
    }
    diagSteps++;
    curR = nr;
    curC = nc;
    lastShiftRow = curR;
    i++;
  }

  if (diagSteps <= 0) {
    return false;
  }

  // 斜向阶段覆盖的总行数：从斜向开始到最后一次侧移的行
  diagRows = lastShiftRow - diagStartRow;

  // 段3：回正后直行到终点行（列保持当前）
  int goalR = waypointRows[waypointCount - 1];
  if (goalR > lastShiftRow) {
    straightAfterRows = goalR - lastShiftRow;
  }

  return true;
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
    if (isPlanningObstacle(row, c)) break;
    leftSpace++;
  }
  int rightSpace = 0;
  for (int c = centerCol + 1; c < COLS; c++) {
    if (isPlanningObstacle(row, c)) break;
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
static bool isPlanningObstacle(int row, int col) {
  if (row < 0 || row >= ROWS || col < 0 || col >= COLS) return true;
  if (col == 0 || col == COLS - 1) return true; // 规划时固定将两侧边界列视为障碍
  return pointCloudGrid[row][col] != 0;
}

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
    if (isPlanningObstacle(row, col)) {
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
    if (col >= 0 && col < COLS && isPlanningObstacle(row, col)) {
      return false;  // 左侧太靠近障碍物
    }
  }
  
  // 检查右侧安全边距（检查安全边距范围内的所有列）
  for (int col = endCol + 1; col <= safeEndCol; col++) {
    if (col >= 0 && col < COLS && isPlanningObstacle(row, col)) {
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
    if (isPlanningObstacle(row, c)) {
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
    if (isPlanningObstacle(row, c)) {
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
    if (isPlanningObstacle(row, c)) {
      break;
    }
    leftDist++;
  }
  if (leftDist == col) {
    leftDist = COLS;  // 左侧无遮挡
  }

  int rightDist = 0;
  for (int c = col + 1; c < COLS; c++) {
    if (isPlanningObstacle(row, c)) {
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
  const int CAR_HALF_COLS = 2; // 车身4列宽
  const int SAFE_MARGIN_COLS = 1;
  const int REQUIRED_CLEAR = CAR_HALF_COLS + SAFE_MARGIN_COLS;
  const int IDEAL_CLEAR = REQUIRED_CLEAR + 1;
  const int minPassCols = 6; // 车宽4列 + 左右安全边距1列
  for (int r = 0; r < ROWS; r++) {
    // 如果上一列在当前行仍可行，优先保持，避免因边缘小变化导致抖动
    if (prev >= 0 && prev < COLS) {
      float distCm = (r + 1) * LAYER_HEIGHT;
      if (!isPlanningObstacle(r, prev) && checkPathWidth(r, prev, distCm)) {
        int leftDist = 0;
        for (int c2 = prev - 1; c2 >= 0; c2--) {
          if (isPlanningObstacle(r, c2)) break;
          leftDist++;
        }
        if (leftDist == prev) leftDist = COLS;
        int rightDist = 0;
        for (int c2 = prev + 1; c2 < COLS; c2++) {
          if (isPlanningObstacle(r, c2)) break;
          rightDist++;
        }
        if (rightDist == COLS - prev - 1) rightDist = COLS;
        int minClear = min(leftDist, rightDist);
        if (minClear >= REQUIRED_CLEAR) {
          preferredCols[r] = prev;
          continue;
        }
      }
    }

    // 通道足够宽且中心列可行时，优先锁定中心，避免无必要的偏线
    const int centerC = (int)round(CENTER_COL);
    if (centerC >= 0 && centerC < COLS) {
      int minClearMin = 9999;
      int widthAtRow = calculateCorridorWidth(r, centerC);
      bool validCenter = true;
      for (int step = 0; step <= 8; step++) {
        int rr = r + step;
        if (rr >= ROWS) break;
        float distCm = (rr + 1) * LAYER_HEIGHT;
        if (isPlanningObstacle(rr, centerC)) {
          validCenter = false;
          break;
        }
        if (!checkPathWidth(rr, centerC, distCm)) {
          validCenter = false;
          break;
        }
        int leftDist = 0;
        for (int c2 = centerC - 1; c2 >= 0; c2--) {
          if (isPlanningObstacle(rr, c2)) break;
          leftDist++;
        }
        if (leftDist == centerC) leftDist = COLS;
        int rightDist = 0;
        for (int c2 = centerC + 1; c2 < COLS; c2++) {
          if (isPlanningObstacle(rr, c2)) break;
          rightDist++;
        }
        if (rightDist == COLS - centerC - 1) rightDist = COLS;
        int minClear = min(leftDist, rightDist);
        minClearMin = min(minClearMin, minClear);
      }
      if (validCenter && minClearMin >= IDEAL_CLEAR && widthAtRow >= minPassCols) {
        preferredCols[r] = centerC;
        prev = centerC;
        continue;
      }
    }

    int bestC = -1;
    int bestMin = -1;
    float bestAvg = -1.0f;
    int bestWidth = -1;
    int bestOffset = 9999;
    int bestCenterOffset = 9999;
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
        if (isPlanningObstacle(rr, c)) {
          valid = false;
          break;
        }
        if (!checkPathWidth(rr, c, distCm)) {
          valid = false;
          break;
        }

        int leftDist = 0;
        for (int c2 = c - 1; c2 >= 0; c2--) {
          if (isPlanningObstacle(rr, c2)) break;
          leftDist++;
        }
        if (leftDist == c) leftDist = COLS;

        int rightDist = 0;
        for (int c2 = c + 1; c2 < COLS; c2++) {
          if (isPlanningObstacle(rr, c2)) break;
          rightDist++;
        }
        if (rightDist == COLS - c - 1) rightDist = COLS;

        int minClear = min(leftDist, rightDist);
        if (leftDist >= COLS || rightDist >= COLS) {
          // 一侧无遮挡时，不让“极大净空”拉偏路径
          minClear = min(minClear, IDEAL_CLEAR);
        }
        minClearMin = min(minClearMin, minClear);
        minClearSum += (float)minClear;
        samples++;
      }

      if (!valid || samples == 0) continue;

      float avgClear = minClearSum / (float)samples;
      int offset = abs(c - prev);
      int centerOffset = abs(c - (int)round(CENTER_COL));
      int width = calculateCorridorWidth(r, c);

      if (minClearMin > bestMin ||
          (minClearMin == bestMin && avgClear > bestAvg) ||
          (minClearMin == bestMin && fabs(avgClear - bestAvg) < 1e-3f && width > bestWidth) ||
          (minClearMin == bestMin && fabs(avgClear - bestAvg) < 1e-3f && width == bestWidth &&
           ((minClearMin >= IDEAL_CLEAR && width >= minPassCols && centerOffset < bestCenterOffset) ||
            (minClearMin < IDEAL_CLEAR && offset < bestOffset)))) {
        bestMin = minClearMin;
        bestAvg = avgClear;
        bestWidth = width;
        bestOffset = offset;
        bestCenterOffset = centerOffset;
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
      if (isPlanningObstacle(r, candC)) continue;
      if (!checkPathWidth(r, candC, distCm)) continue;

      int leftDist = 0;
      for (int c2 = candC - 1; c2 >= 0; c2--) {
        if (isPlanningObstacle(r, c2)) break;
        leftDist++;
      }
      if (leftDist == candC) leftDist = COLS;

      int rightDist = 0;
      for (int c2 = candC + 1; c2 < COLS; c2++) {
        if (isPlanningObstacle(r, c2)) break;
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
      if (isPlanningObstacle(r, c)) continue;
      if (!checkPathWidth(r, c, distCm)) continue;

      int leftDist = 0;
      for (int c2 = c - 1; c2 >= 0; c2--) {
        if (isPlanningObstacle(r, c2)) break;
        leftDist++;
      }
      if (leftDist == c) leftDist = COLS;

      int rightDist = 0;
      for (int c2 = c + 1; c2 < COLS; c2++) {
        if (isPlanningObstacle(r, c2)) break;
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
      if (isPlanningObstacle(r, startCol) || !checkPathWidth(r, startCol, distCm)) {
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
      if (isPlanningObstacle(curR, curC) || !checkPathWidth(curR, curC, distCm)) {
        ok = false;
        break;
      }
    }
    if (!ok) continue;

    // 段3：直行到终点行
    for (int r = curR + 1; r <= targetRow; r++) {
      float distCm = (r + 1) * LAYER_HEIGHT;
      if (isPlanningObstacle(r, curC) || !checkPathWidth(r, curC, distCm)) {
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
    // 不再强制等待，避免走走停停
  }
  
  if (!pathAvailable || waypointCount < 2) {
    navState = NAV_IDLE;
    setCurrentAction("", 0.0f);
    return;
  }

  executedSegments = 0;
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

static bool detectCollisionNarrow() {
  const int COLLISION_THRESHOLD_MM = 150; // 15cm = 150mm
  int centerStart = TOTAL_POINTS / 2 - TOTAL_POINTS / 12;  // 中心减去1/12
  int centerEnd = TOTAL_POINTS / 2 + TOTAL_POINTS / 12 - 1;  // 中心加上1/12（中间1/6的列）
  return checkCollisionRisk(COLLISION_THRESHOLD_MM, centerStart, centerEnd);
}

static bool detectCollisionWide() {
  const int COLLISION_THRESHOLD_MM = 150; // 15cm = 150mm
  int centerStart = TOTAL_POINTS / 4;  // 从1/4处开始
  int centerEnd = TOTAL_POINTS * 3 / 4 - 1;  // 到3/4处结束（中间一半的列）
  return checkCollisionRisk(COLLISION_THRESHOLD_MM, centerStart, centerEnd);
}

static bool detectCollisionStop() {
  const int COLLISION_THRESHOLD_MM = 180; // 18cm 刹停阈值
  int centerStart = TOTAL_POINTS / 3;            // 中间 1/3
  int centerEnd   = TOTAL_POINTS * 2 / 3 - 1;
  return checkCollisionRisk(COLLISION_THRESHOLD_MM, centerStart, centerEnd);
}

static void enterSearchMode(SearchReason reason, const char* logMessage, bool singleTurn, float singleTurnAngleDeg) {
  if (logMessage != nullptr && strlen(logMessage) > 0) {
    Serial.println(logMessage);
  }
  stopMotors();
  navState = NAV_SEARCHING; // 进入搜索模式
  searchStep = 0;
  searchTotalAngle = 0.0f; // 重置累计角度
  currentSearchReason = reason; // 设置搜索原因
  searchSingleTurn = singleTurn;
  if (singleTurn) {
    singleTurnAngleDeg = max(5.0f, singleTurnAngleDeg);
    searchAngleStep = singleTurnAngleDeg;
    searchTotalAngle = 0.0f;
    singleTurnTargetDeg = singleTurnAngleDeg;
  } else {
    singleTurnTargetDeg = 0.0f;
  }
}

static void handleTurnToGap() {
  if (navTurnAngle < 0) {
    setCurrentAction("left", abs(navTurnAngle));
    turnLeft(abs(navTurnAngle));
  } else {
    setCurrentAction("right", navTurnAngle);
    turnRight(navTurnAngle);
  }

  if (!isTurning) {
    delayAfterAction();
    if (deadendTurnOnly) {
      deadendTurnOnly = false;
      navState = NAV_IDLE;
      requestReplan("死胡同转向完成");
      setCurrentAction("", 0.0f);
      return;
    }
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
    }

    Serial.print("【执行】步骤1完成，步骤2: 直行 ");
    Serial.print(navForwardDist, 1);
    Serial.println(" cm");
    navState = NAV_MOVE_TO_ENTRANCE;
    setCurrentAction("line", navForwardDist);
    requestStepPause("转向完成");
  }
}

static void handleMoveToEntrance() {
  static bool wasMoving = false;

  if (!deadendBackActive && navForwardDist < (LAYER_HEIGHT * 0.5f)) {
    handleMoveCompleted();
    wasMoving = false;
    return;
  }

  // 行进中尝试重规划（使用最新点云），平滑模式：不强制急停
  // 宏路径执行中不动态重规划，避免中途被打断
  if (!macroActive && tryReplanAndSwitchPath(false)) {
    Serial.println("【动态重规划】使用新路径，重新执行");
    return;
  }

  if (!deadendBackActive && navForwardDist > 0.5f && detectCollisionStop()) {
    stopMotors();
    navState = NAV_IDLE;
    requestReplan("近距离障碍，刹停重规划");
    setCurrentAction("", 0.0f);
    wasMoving = false;
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

  if (wasMoving && !isMoving) {
    handleMoveCompleted();
    wasMoving = false;
  }
}

static void handleMoveCompleted() {
  delayAfterAction();
  if (deadendBackActive) {
    deadendBackActive = false;
    navState = NAV_IDLE;
    requestReplan("死胡同后退完成，重新规划");
    setCurrentAction("", 0.0f);
    pendingDeadendReplan = true;
    deadendReturnActive = true;
    return;
  }
  if (pendingTurnBackStraight) {
    pendingTurnBackStraight = false;
    stopMotors();
    navState = NAV_IDLE;
    requestStepPause("直行完成");
    requestReplan("回正后直行完成");
    setCurrentAction("", 0.0f);
    return;
  }
  Serial.println("【执行】步骤2完成，节点到达");
  // 推进到下一个航点
  advanceWaypoint();
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

static void handleTurnBack() {
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
          navForwardDist = TURNBACK_STRAIGHT_CM;
          navState = NAV_MOVE_TO_ENTRANCE;
          pendingTurnBackStraight = true;
          Serial.print("【执行】回正完成，插入直行 ");
          Serial.print(navForwardDist, 1);
          Serial.println(" cm");
          setCurrentAction("line", navForwardDist);
          requestStepPause("回正完成");
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
      navForwardDist = TURNBACK_STRAIGHT_CM;
      navState = NAV_MOVE_TO_ENTRANCE;
      pendingTurnBackStraight = true;
      Serial.print("【执行】回正完成，插入直行 ");
      Serial.print(navForwardDist, 1);
      Serial.println(" cm");
      setCurrentAction("line", navForwardDist);
      requestStepPause("回正完成");
    }
  } else {
    delayAfterAction();
    if (macroActive && macroPhase == 3) {
      if (REPLAN_AFTER_TURNBACK) {
        navForwardDist = TURNBACK_STRAIGHT_CM;
        navState = NAV_MOVE_TO_ENTRANCE;
        pendingTurnBackStraight = true;
        Serial.print("【执行】回正完成，插入直行 ");
        Serial.print(navForwardDist, 1);
        Serial.println(" cm");
        setCurrentAction("line", navForwardDist);
        requestStepPause("回正完成");
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
    } else {
      navForwardDist = TURNBACK_STRAIGHT_CM;
      navState = NAV_MOVE_TO_ENTRANCE;
      pendingTurnBackStraight = true;
      Serial.print("【执行】回正完成，插入直行 ");
      Serial.print(navForwardDist, 1);
      Serial.println(" cm");
      setCurrentAction("line", navForwardDist);
      requestStepPause("回正完成");
    }
  }
}

static void handleSearch() {
  // 搜索模式：单方向连续旋转搜索可走路径
  // searchStep: 0=旋转, 1=检测, 2=旋转, 3=检测...
  // 每次旋转固定角度（searchAngleStep），累计旋转角度不超过maxSearchAngle

  float angleStep = searchSingleTurn ? singleTurnTargetDeg : searchAngleStep;
  float maxAngle = searchSingleTurn ? singleTurnTargetDeg : maxSearchAngle;

    if (abs(searchTotalAngle) >= maxAngle) {
      Serial.println("【搜索模式】已搜索一圈，重新开始搜索...");
      searchStep = 0;
      searchTotalAngle = 0.0;
    }

  if (searchStep % 2 == 0) {
    static bool rotationStarted = false;
    static unsigned long searchTurnStartMs = 0;
    static float lastSearchTurnAngle = 0.0f;
    if (!isTurning && !rotationStarted) {
      Serial.print("【搜索模式】");
      if (searchDirectionLeft) {
        Serial.print("左转 ");
      } else {
        Serial.print("右转 ");
      }
      Serial.print(angleStep, 1);
      Serial.print(" 度 (累计: ");
      Serial.print(searchTotalAngle, 1);
      Serial.println(" 度)");

      if (searchDirectionLeft) {
        turnLeft(angleStep);
        searchTotalAngle += angleStep;
        lastSearchTurnAngle = -angleStep;
      } else {
        turnRight(angleStep);
        searchTotalAngle += angleStep;
        lastSearchTurnAngle = angleStep;
      }
      searchTurnStartMs = millis();
      rotationStarted = true;
    } else if (isTurning) {
      if (searchDirectionLeft) {
        turnLeft(angleStep);
      } else {
        turnRight(angleStep);
      }
    }

    if (rotationStarted && !isTurning) {
      if (searchTurnStartMs > 0) {
        unsigned long elapsed = millis() - searchTurnStartMs;
        Serial.print("【动作】搜索转向完成，耗时 ");
        Serial.print(elapsed);
        Serial.print(" ms / 目标 ");
        Serial.print((unsigned long)((abs(lastSearchTurnAngle) / CAR_TURN_DEG_PER_SEC) * 1000.0f));
        Serial.println(" ms");
      }
      searchStep++;
      rotationStarted = false;
      Serial.println("【搜索模式】转向完成，等待检测...");
    }
  } else {
    static unsigned long detectionStartTime = 0;
    static bool detectionStarted = false;
    static bool collisionWarningPrinted = false;

    if (!detectionStarted) {
      detectionStartTime = millis();
      detectionStarted = true;
      collisionWarningPrinted = false;
      Serial.println("【搜索模式】正在检测当前方向...");
      Serial.print("【搜索角度：");
      Serial.print(searchTotalAngle, 1);
      Serial.println("度】");
      printZValuesSummary();
    }

    if (!collisionWarningPrinted && detectCollisionWide()) {
      Serial.println("【警告】搜索中检测到近距离障碍物，继续搜索其他方向！");
      collisionWarningPrinted = true;
      if (currentSearchReason == SEARCH_NO_GAP) {
        currentSearchReason = SEARCH_COLLISION;
      }
    }

    unsigned long currentTime = millis();
    if (currentTime - detectionStartTime >= 500) {
      if (!gapFound) {
        Serial.print("【搜索角度：");
        Serial.print(searchTotalAngle, 1);
        Serial.println("度 - 未找到空洞】");
        printZValuesSummary();

        if (searchSingleTurn) {
          searchSingleTurn = false;
          singleTurnTargetDeg = 0.0f;
          navState = NAV_IDLE;
          requestReplan("前方阻塞，转向后重规划");
          return;
        }

        searchStep++;
        detectionStarted = false;
        collisionWarningPrinted = false;
        Serial.println("【搜索模式】当前方向未找到合适空洞，继续旋转搜索...");
      }
    }
  }
}

/*
 * 更新导航状态机（需要在主循环中持续调用）
 */
void updateNavigation() {
  if (navState != NAV_SEARCHING && searchSingleTurn) {
    searchSingleTurn = false;
  }

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

  if (navState == NAV_TURN_TO_GAP) {
    handleTurnToGap();
  } else if (navState == NAV_MOVE_TO_ENTRANCE) {
    handleMoveToEntrance();
  } else if (navState == NAV_TURN_BACK) {
    handleTurnBack();
  } else if (navState == NAV_SEARCHING) {
    handleSearch();
  } else {
    // NAV_IDLE / NAV_MOVE_THROUGH 无动作
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
  Serial.println("【重规划】刷新路径地图 -> Serial/Serial2");
  printPathMap();
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

bool isStepPauseModeEnabled() {
  return stepPauseEnabled;
}

void resumeStepPause() {
  if (!stepPauseActive) return;
  stepPauseActive = false;
  Serial.println("【继续】按键继续执行");
}

void toggleStepPauseMode() {
  stepPauseEnabled = !stepPauseEnabled;
  if (!stepPauseEnabled) {
    stepPauseActive = false;
    Serial.println("【步进模式】已关闭，连续运行");
  } else {
    Serial.println("【步进模式】已开启，逐步执行");
  }
}

static void requestStepPause(const char* reason) {
  if (motorDriveState != MOTOR_DRIVE_ENABLED) return;
  if (!stepPauseEnabled) return;
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
  int baseRow = straightRows;
  float forwardCm = forwardRows * LAYER_HEIGHT + LAYER_HEIGHT; // 与终点距离同样强制+5
  float lateralCm = 0.0f;
  int stepCount = diagSteps;

  for (int i = 1; i <= stepCount; i++) {
    int rowIndex = baseRow + i;
    if (rowIndex < 0) rowIndex = 0;
    if (rowIndex >= TAN_TABLE_SIZE) rowIndex = TAN_TABLE_SIZE - 1;
    lateralCm += ROW_STEP_WIDTH[rowIndex] * diagSign;
  }

  float startDistCm = baseRow * LAYER_HEIGHT;
  float endDistCm = (baseRow + forwardRows) * LAYER_HEIGHT + LAYER_HEIGHT;
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
