#ifndef KEY_H
#define KEY_H

// 定义按键引脚
#define KEY_PIN 8

// 定义按键检测相关参数
#define KEY_DEBOUNCE_TIME 10  // 按键消抖时间，单位：毫秒

// 电机驱动状态枚举
typedef enum {
  MOTOR_DRIVE_ENABLED,   // 电机驱动启用
  MOTOR_DRIVE_DISABLED   // 电机驱动禁用
} MotorDriveState;

// 外部变量声明
extern MotorDriveState motorDriveState;

// 函数声明
/**
 * @brief 初始化按键检测模块
 */
void keyInit(void);

/**
 * @brief 检测按键状态（带消抖）
 * @return bool 按键是否被按下（已消抖）
 */
bool keyCheck(void);

/**
 * @brief 切换电机驱动状态
 */
void toggleMotorDrive(void);

/**
 * @brief 检查并处理按键事件
 */
void handleKeyEvents(void);

#endif  // KEY_H