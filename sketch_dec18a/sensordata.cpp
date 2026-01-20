#include "sensordata.h"
#include "tantable.h"
/*****点云二维数组 */
uint8_t pointCloudGrid[GRID_ROWS][TOTAL_POINTS]; // 1代表障碍物, 0代表空隙
/*数据帧*/
uint8_t frameBuffer[FRAME_SIZE];
int  frameIndex = 0;
bool inSyncMode = false; // 用于同步数据帧
bool sensorFlag = false;
/*z距离值*/
int16_t zValues[TOTAL_POINTS];
int16_t rawZBuffer[TOTAL_POINTS][FILTER_SAMPLES]; 
int16_t rawXBuffer[TOTAL_POINTS][FILTER_SAMPLES];
int     sampleCount = 0;

GapInfo foundGaps[10]; 
int totalGaps = 0;
int validRow = -1;
float angleDeg        = 0.0f ; //计算出来的转向角度
float driveDistanceCm = 0.0f;  //计算出来的直行距离
eMoveType_t  currentMoveType = MOVE_STOP;
eTurnState_t turnState       = TURN_STOPING; // 初始状态为直行
void calculateGapsForRow(int row, GapInfo *gapsOutput, int &gapCountOutput, int maxGaps)
{
  gapCountOutput = 0;
  int currentGapWidth = 0;
  int currentGapStart = -1;

  for (int col = 0; col < TOTAL_POINTS; col++)
  {
    if (pointCloudGrid[row][col] == 0)
    {
      if (currentGapWidth == 0)
        currentGapStart = col;
      currentGapWidth++;
    }
    else
    {
      if (currentGapWidth > 0 && gapCountOutput < maxGaps)
      {
        gapsOutput[gapCountOutput].start_index = currentGapStart;
        gapsOutput[gapCountOutput].width = currentGapWidth;
        gapCountOutput++;
        currentGapWidth = 0;
      }
    }
  }

  if (currentGapWidth > 0 && gapCountOutput < maxGaps)
  {
    gapsOutput[gapCountOutput].start_index = currentGapStart;
    gapsOutput[gapCountOutput].width = currentGapWidth;
    gapCountOutput++;
  }
}

void findBestRowAndAnalyzeGaps(GapInfo *gapsOutput, int &gapCountOutput, int maxGaps, int &validRowIndex,eMoveType_t &moveType)
{
  gapCountOutput = 0;
  validRowIndex = -1; // 用于输出找到的是哪一行
  for (int row = 0; row < GRID_ROWS; row++)
  {
    int threshold = row * ROW_INTERVAL_CM * 10; 
    for (int col = 0; col < TOTAL_POINTS; col++)
    {
       pointCloudGrid[row][col] = (zValues[col] <= threshold) ? 1 : 0;
    }
    // 这能解决“后面只有2个障碍物”的问题，它们会被直接抹去
    /***********************修正**************************** */
    int obstacleRun = 0;
    for (int col = 0; col < TOTAL_POINTS; col++)
    {
      if (pointCloudGrid[row][col] == 1)
      {
        obstacleRun++;
      }
      else
      {
        if (obstacleRun > 0 && obstacleRun < 3)
        {
          // 清除噪点  //可以直接跳过
          for (int k = 1; k <= obstacleRun; k++)
            pointCloudGrid[row][col - k] = 0;
        }
        obstacleRun = 0;
      }
    }
/**************************去噪******************************** */
    int obstacleTotalCount = 0;
    for (int col = 0; col < TOTAL_POINTS; col++) {
      if (pointCloudGrid[row][col] == 1) {
        obstacleTotalCount++;
      }
    }
    // 如果障碍物总数太少（小于10个），认为是无效行/噪声行，跳过
    if (obstacleTotalCount < 18) {
      continue;
    }

    // 因为我们已经把 <3 的都抹去了，所以只要现在还有 1，那就一定是 >=3 的有效障碍。
    bool hasObstacle = false;
    for (int col = 0; col < TOTAL_POINTS; col++)
    {
      if (pointCloudGrid[row][col] == 1)
      {
        hasObstacle = true;
        break; 
      }
    }
    //4. 找到有效行，停止搜索 
    validRowIndex = row;
    break;
  }

  // // 如果没有找到任何有效行，直接返回
  if (validRowIndex == -1)
  {
    moveType = MOVE_STRAIGHT;
    //Serial.println("No valid row found.");
    return;
  }
  
  // 当有效行小于等于3的时候,直接停车
  if (validRowIndex <= 3)
  {
    moveType = MOVE_STOP;
    return;
  }

  calculateGapsForRow(validRowIndex, gapsOutput, gapCountOutput, maxGaps);

    // --- 打印调试信息 ---
  #if DEBUG_ENABLE
  Serial.printf("\n--- Valid Scan at Row: %d ---\n", validRowIndex);
   // 1. 打印原始未去噪的 Map (Raw Map)
  Serial.print("RawMap: ");

  int rawMapThreshold = validRowIndex * ROW_INTERVAL_CM * 10;
  for (int col = 0; col < TOTAL_POINTS; col++) {
      // 重新计算原始状态，不修改 pointCloudGrid
      bool isObstacle = zValues[col] <= rawMapThreshold;
      Serial.print(isObstacle ? "# " : ". ");
  }
  Serial.println();

  // 2. 打印去噪后的 Map (Filtered Map, 也就是现在的 pointCloudGrid)
  Serial.print("FilMap: "); // Filtered Map
  for (int col = 0; col < TOTAL_POINTS; col++) {
      Serial.print(pointCloudGrid[validRowIndex][col] == 1 ? "# " : ". ");
  }
  Serial.println();
  #endif

  float stepWidthCm    = ROW_STEP_WIDTH[validRowIndex+1]; // 当前行的单点步长
  float verticalDistCm = (validRowIndex+1) * ROW_INTERVAL_CM;         // 当前行距离障碍物的垂直距离 (行号*10)
  int   pts;             //.的数量           
  float realWidthCm;     //实际宽度(间隙数*步长)
  for(int i = 0; i < gapCountOutput; i++) 
  {
      // 间隙数 = 点数 + 1 
      // 实际宽度 = 间隙数 * 单步长
      pts = gapsOutput[i].width;
      realWidthCm = (pts + 1) * stepWidthCm;

      #if DEBUG_ENABLE
      Serial.printf("Step Width: %.3f cm\n", stepWidthCm);
      Serial.printf("Gaps Found: %d\n", gapCountOutput);
      Serial.printf("  Gap %d: Start %02d | Pts %d | Intervals %d | Width %.2f cm\n", 
                        i + 1, 
                        gapsOutput[i].start_index, 
                        pts, 
                        pts + 1,
                        realWidthCm);
      #endif
  }
      // 2. 找出最大的空隙
    int   bestGapIndex = -1;
    float maxRealWidth = -1.0;

    for(int i = 0; i < gapCountOutput; i++) {
        // 间隙数 = 点数 + 1 (你的定义)
        // 实际宽度 = 间隙数 * 单点步长
        if (gapsOutput[i].width < MAX_GAPS) 
        {
            continue; // 跳过宽度不足的间隙
        }
        float realWidthCm = (gapsOutput[i].width + 1) * stepWidthCm; 
        
        if (realWidthCm > maxRealWidth) {
            maxRealWidth = realWidthCm;
            bestGapIndex = i;
        }
    }

    if (bestGapIndex != -1) 
    {
      GapInfo& bestGap = gapsOutput[bestGapIndex];

      // A. 计算空隙的几何中心索引 
      // 举例: Start=5, Width=3 (点5,6,7). 中心=(5+7)/2 = 6.0
      // 公式: Start + (Width - 1) / 2.0
      float gapCenterIndex = bestGap.start_index + (bestGap.width - 1) / 2.0f;

      // B. 计算视野(FOV)的中心索引
      float fovCenterIndex = (TOTAL_POINTS - 1) / 2.0f;

      // C. 计算水平偏差 (多少个步长)
      // 正数表示在右边，负数表示在左边
      float offsetInSteps = gapCenterIndex - fovCenterIndex;

      // D. 计算水平物理距离 (Horizontal Distance)
      float horizontalDistCm = offsetInSteps * stepWidthCm;

      // E. 计算转向角度 (Steering Angle)
      //tan(angle) = 水平 / 垂直
      //使用 atan2(y, x) -> atan2(水平, 垂直)
      float angleRad = atan2(horizontalDistCm, verticalDistCm);//弧度
      angleDeg = angleRad * RAD_TO_DEG;                       // 转为角度
      driveDistanceCm = verticalDistCm / cos(angleRad); // 计算实际行驶距离
      // angleDeg = stepWidthCm * offsetInSteps; 
      // float angleRad = angleDeg * DEG_TO_RAD;
      // driveDistanceCm = verticalDistCm / cos(angleRad);
  
      #if DEBUG_ENABLE
        // ...
      Serial.printf("Geometry: V_Dist %.1f cm | H_Dist %.1f cm\n", verticalDistCm, horizontalDistCm);
      Serial.printf("Turn&Drive: %.2f deg -> %.1f cm\n", angleDeg, driveDistanceCm); // 打印计算出的距离
        // ...
      #endif
      if (abs(angleDeg) < 10.0f) 
      {
          moveType = MOVE_STRAIGHT;
      } 
      else 
      {
          if (angleDeg > 0)
            moveType = MOVE_TURN_LEFT;
          else
            moveType = MOVE_TURN_RIGHT;
      }

      #if DEBUG_ENABLE
        // --- 打印导航结果 ---
      Serial.println("\n=== Navigation Decision ===");
      Serial.printf("Target: Gap %d (Width %.1f cm)\n", bestGapIndex + 1, maxRealWidth);
      Serial.printf("Center Offset: %.1f steps (GapCenter: %.1f, FOVCenter: %.1f)\n", 
                    offsetInSteps, gapCenterIndex, fovCenterIndex);
      
      Serial.printf("Geometry: V_Dist %.1f cm | H_Dist %.1f cm\n", verticalDistCm, horizontalDistCm);
      
      Serial.print("Action: Turn ");
      if (moveType == MOVE_STRAIGHT) 
      { 
          Serial.print("STRAIGHT");
      } 
      else 
      {
          Serial.print(moveType == MOVE_TURN_LEFT ? "LEFT" : "RIGHT");
      }
      Serial.printf(" : %.2f degrees\n", angleDeg); // 打印带符号的角度
      Serial.println("===========================");
      Serial.println();
      #endif
  } 
  else 
  {
      moveType = MOVE_STOP; // 如果没有找到合适的间隙，停车
      #if DEBUG_ENABLE
      Serial.println("Navigation: No Gap Found!");
      #endif
  }
}





void parsePointData(uint8_t* pointData,int16_t* zValue) 
{
  *zValue = (pointData[5] << 8) | pointData[4];
}
int16_t calculateAverage(int16_t *values, int count)
{
  if (count == 0)
    return 0;
  int32_t sum = 0;
  for (int i = 0; i < count; i++)
  {
    sum += values[i];
  }
  return (int16_t)(sum / count);
}

int16_t calculateTrimmedFilter(int16_t *values, int count)
{
  if (count <= 2)
  {
    return calculateAverage(values, count); // 如果样本太少，直接用普通平均
  }

  int16_t maxValue = values[0];
  int16_t minValue = values[0];
  int32_t sum = values[0];

  for (int i = 1; i < count; i++)
  {
    sum += values[i];
    if (values[i] > maxValue)
      maxValue = values[i];
    if (values[i] < minValue)
      minValue = values[i];
  }
  return (int16_t)((sum - maxValue - minValue) / (count - 2));
}
