#ifndef _DFROBOT_WY6005_h
#define _DFROBOT_WY6005_h

#include <Arduino.h>

class DFRobot_WY6005 {
  private:
    HardwareSerial* _serial;
    uint32_t _timeout;
    uint32_t _config;
    int8_t _rxPin;
    int8_t _txPin;
    
    // 发送AT指令并等待响应
    bool sendCommand(const String& command);
    
  public:
    // 构造函数
    DFRobot_WY6005(HardwareSerial *serial, uint32_t timeout ,uint32_t config , int8_t rxPin , int8_t txPin );
    
    // 初始化
    void begin(uint32_t baudRate);
    
    // 流控制
    bool setStreamControl(bool enable);
    
    // 设置帧模式
    bool setFrameMode(bool continuousMode);
    
    // 输出行数据配置
    bool setOutputLineData(uint8_t line, uint8_t startPoint, uint8_t pointCount);
    
    // 触发单帧数据输出
    bool triggerOneFrame(void);
    
    // 保存配置
    bool saveConfig(void);
    
    // 常用模式配置
    bool configSinglePointMode(uint8_t line, uint8_t point, bool enable = true);
    bool configSingleLineMode(uint8_t line, uint8_t startPoint, uint8_t endPoint, bool enable);
    bool configSingleFrameMode(bool enable = true);
    bool configContinuousMode(bool enable = true);
};

#endif