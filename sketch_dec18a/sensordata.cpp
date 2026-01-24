#include "sensordata.h"
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
int     sampleCount = 0;
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
