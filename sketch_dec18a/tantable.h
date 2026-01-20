#ifndef TANTABLE_H
#define TANTABLE_H
#include <Arduino.h>

// 查找表大小: 0-19 共20行
#define TAN_TABLE_SIZE 20
// 声明外部只读数组，存储每行对应的单点步长 (单位: cm, 厘米)
// 计算公式: RowIndex * tan(2.4度)
extern const float ROW_STEP_WIDTH[TAN_TABLE_SIZE];
#endif