#include "Arduino.h"
#include "key.h"

// 声明setSpeed函数，以便在key.cpp中使用
extern void setSpeed(int leftSpeed, int rightSpeed);
MotorDriveState motorDriveState = MOTOR_DRIVE_DISABLED;  // 默认启用电机驱动

// 按键检测相关变量
unsigned long lastKeyPressTime = 0;  // 上次按键按下的时间，用于消抖
bool lastKeyState = HIGH;            // 上次按键状态（初始为高电平，因为使用上拉电阻）
bool keyPressed = false;             // 按键按下标志
unsigned long lastModeKeyPressTime = 0;  // 模式键消抖
bool lastModeKeyState = HIGH;            // 模式键状态
bool modeKeyPressed = false;             // 模式键按下标志

/**
 * @brief 初始化按键检测模块
 */
void keyInit(void) {
  pinMode(KEY_PIN, INPUT_PULLUP);  // 配置按键引脚为输入，启用内部上拉电阻
  pinMode(MODE_KEY_PIN, INPUT_PULLUP);
}

/**
 * @brief 检测按键状态（带消抖）
 * @return bool 按键是否被按下（已消抖）
 */
bool keyCheck(void) {
  bool currentKeyState = digitalRead(KEY_PIN);
  bool result = false;
  
  // 检查按键状态是否变化
  if (currentKeyState != lastKeyState) {
    lastKeyPressTime = millis();
    lastKeyState = currentKeyState;
  }
  
  // 等待消抖时间
  if (millis() - lastKeyPressTime > KEY_DEBOUNCE_TIME) {
    // 如果按键现在是按下状态（低电平）且之前没有按下标志
    if (currentKeyState == LOW && !keyPressed) {
      keyPressed = true;
      result = true;
    } else if (currentKeyState == HIGH && keyPressed) {
      // 按键释放
      keyPressed = false;
    }
  }
  
  return result;
}

/**
 * @brief 检测模式按键状态（带消抖）
 * @return bool 按键是否被按下（已消抖）
 */
bool modeKeyCheck(void) {
  bool currentKeyState = digitalRead(MODE_KEY_PIN);
  bool result = false;

  if (currentKeyState != lastModeKeyState) {
    lastModeKeyPressTime = millis();
    lastModeKeyState = currentKeyState;
  }

  if (millis() - lastModeKeyPressTime > KEY_DEBOUNCE_TIME) {
    if (currentKeyState == LOW && !modeKeyPressed) {
      modeKeyPressed = true;
      result = true;
    } else if (currentKeyState == HIGH && modeKeyPressed) {
      modeKeyPressed = false;
    }
  }

  return result;
}

/**
 * @brief 切换电机驱动状态
 */
void toggleMotorDrive(void) {
  if (motorDriveState == MOTOR_DRIVE_ENABLED) {
    // 禁用电机驱动
    motorDriveState = MOTOR_DRIVE_DISABLED;
    // 停止所有电机
    // setSpeed(0, 0);  // 假设setSpeed是电机控制函数，需要确保在主程序中可用
   // Serial.println("电机已关闭");
  } else {
    // 启用电机驱动
    motorDriveState = MOTOR_DRIVE_ENABLED;
    //Serial.println("电机已打开");
  }
}

/**
 * @brief 检查并处理按键事件
 */
void handleKeyEvents(void) {
  if (keyCheck()) {
    // 按键被按下，切换电机驱动状态
    toggleMotorDrive();
  }
}