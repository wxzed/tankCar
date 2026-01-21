#ifndef SENSORDATA_H
#define SENSORDATA_H
#include <Arduino.h>

#ifndef DEBUG_ENABLE
#define DEBUG_ENABLE    0
// 配置单线模式参数
#define LINE_NUM        4        // 第5行
#define START_POINT     16       // 起始点16
#define END_POINT       47       // 结束点48 
#define TOTAL_POINTS    (END_POINT - START_POINT + 1)  // 总点数
#define FILTER_SAMPLES  3        // 采集3次
#define ROW_INTERVAL_CM 5        // 行间隔，每行代表的实际距离增量 单位 cm
#define GRID_ROWS       20       // 网格行数，根据最大距离和行间隔计算得出
// 接收的数据参数
#define DATA_HEADER_SIZE 4  // OK\r\n的大小
#define POINT_DATA_SIZE  8  // 每个点的数据大小
#define FRAME_SIZE (DATA_HEADER_SIZE + TOTAL_POINTS * POINT_DATA_SIZE)  // 一帧数据的总大小
#define MAX_GAPS         7       // 最大空隙数量
extern uint8_t pointCloudGrid[GRID_ROWS][TOTAL_POINTS]; // 1代表障碍物, 0代表空隙

extern uint8_t frameBuffer[FRAME_SIZE];
extern int     frameIndex;
extern bool    inSyncMode; // 用于同步数据帧
extern bool    sensorFlag;
extern int16_t zValues[TOTAL_POINTS];
extern int16_t rawZBuffer[TOTAL_POINTS][FILTER_SAMPLES]; 
extern int16_t rawXBuffer[TOTAL_POINTS][FILTER_SAMPLES];
extern int     sampleCount;

struct GapInfo {
    int start_index; // 空隙起始点的索引
    int width;       // 空隙的宽度（点的数量）
};
extern GapInfo foundGaps[10]; 
extern int totalGaps;
extern int validRow;
typedef enum
{
    MOVE_STOP = 0,
    MOVE_STRAIGHT,
    MOVE_TURN_LEFT,
    MOVE_TURN_RIGHT
}eMoveType_t;
typedef enum
{
    TURN_STRAIGHT = 0,
    TURN_TURNING,
    TUNR_TURNBACK,
    TURN_WAIT,
    TURN_STOPING,
    TURN_LEAVE_HOLE,
    TURN_RETREAT
}eTurnState_t;
extern eTurnState_t turnState;
extern eMoveType_t  currentMoveType;
void calculateGapsForRow(int row, GapInfo* gapsOutput, int& gapCountOutput, int maxGaps);
void findBestRowAndAnalyzeGaps(GapInfo* gapsOutput, int& gapCountOutput, int maxGaps, int& validRowIndex,eMoveType_t &moveType);

int16_t calculateAverage(int16_t *values, int count);
int16_t calculateTrimmedFilter(int16_t *values, int count);
void parsePointData(uint8_t* pointData,int16_t* zValue); 


extern float angleDeg;
extern float driveDistanceCm;


#endif

#endif