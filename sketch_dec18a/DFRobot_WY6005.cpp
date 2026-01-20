#include "DFRobot_WY6005.h"
DFRobot_WY6005::DFRobot_WY6005(HardwareSerial *serial, uint32_t timeout,uint32_t config, int8_t rxPin, int8_t txPin) {
  _serial = serial;
  _timeout = timeout;
  _config = config;
  _rxPin = rxPin;
  _txPin = txPin;
}

void DFRobot_WY6005::begin(uint32_t baudRate) {
  _serial->begin(baudRate, _config, _rxPin, _txPin);
  delay(100);
}

bool DFRobot_WY6005::sendCommand(const String& command) {
  _serial->print(command);
  _serial->print("\n"); // 添加换行符作为指令结束，只添加\n而不是\r\n
  return true;
}

bool DFRobot_WY6005::setStreamControl(bool enable) {
  String command = "AT+STREAM_CONTROL=" + String(enable ? "1" : "0");
  return sendCommand(command);
}

bool DFRobot_WY6005::setFrameMode(bool continuousMode) {
  String command = "AT+SPAD_FRAME_MODE=" + String(continuousMode ? "0" : "1");
  return sendCommand(command);
}

bool DFRobot_WY6005::setOutputLineData(uint8_t line, uint8_t startPoint, uint8_t pointCount) {
  String command = "AT+SPAD_OUTPUT_LINE_DATA=" + String(line) + "," + String(startPoint) + "," + String(pointCount);
  return sendCommand(command);
}

bool DFRobot_WY6005::triggerOneFrame(void) {
  String command = "AT+SPAD_TRIG_ONE_FRAME=1";
  return sendCommand(command);
}

bool DFRobot_WY6005::saveConfig(void) {
  String command = "AT+SAVE_CONFIG";
  return sendCommand(command);
}

bool DFRobot_WY6005::configSinglePointMode(uint8_t line, uint8_t point, bool enable) {
  if (!setStreamControl(false)) return false;
  if (!setOutputLineData(line, point, 1)) return false;
  if (enable) {
    return setStreamControl(true);
  }
  return true;
}

bool DFRobot_WY6005::configSingleLineMode(uint8_t line, uint8_t startPoint, uint8_t endPoint, bool enable) {
  if (!setStreamControl(false)) return false;
  if (!setOutputLineData(line, startPoint, endPoint)) return false;
  if (enable) {
    return setStreamControl(true);
  }
  return true;
}

bool DFRobot_WY6005::configSingleFrameMode(bool enable) {
  if (!setStreamControl(false)) return false;
  if (!setFrameMode(false)) return false;
  if (enable) {
    return setStreamControl(true);
  }
  return true;
}

bool DFRobot_WY6005::configContinuousMode(bool enable) {
  if (!setStreamControl(false)) return false;
  if (!setFrameMode(true)) return false;
  if (enable) {
    return setStreamControl(true);
  }
  return true;
}