#include "DFRobot_WY6005.h"
#include <Adafruit_SSD1306.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_struct.h" 
#include "soc/mcpwm_reg.h"
#include "esp_timer.h"
#include "key.h" 
#include "sensordata.h"
#include "motor.h"
#include "navigateToGap.h"
#include <string.h>
#define SDA_PIN 1
#define SCL_PIN 2
#define OLED_ADDR 0x3C
#define DEBUG_ENABLE 0  
#define PATH_MAP_SERIAL_RX 4  // IO4作为路径地图串口的RX
#define PATH_MAP_SERIAL_TX 5  // IO5作为路径地图串口的TX

Adafruit_SSD1306   display(128, 64, &Wire, -1);
DFRobot_WY6005     wy6005(&Serial1, 1000, SERIAL_8N1, 40, 41);
// Serial2已经在ESP32核心库中预定义，无需再次定义
esp_timer_handle_t periodic_timer;
void IRAM_ATTR onTimer(void* arg) {
  static uint32_t timer_count = 0;
  timer_count++;
  if (timer_count % 1 == 0) {
    timer_count = 0;
    wy6005.triggerOneFrame();
  }
}

// 初始化定时器函数
void timerInit(void) 
{
  // 配置定时器参数
  const esp_timer_create_args_t periodic_timer_args = {
    .callback = &onTimer,     // 绑定回调函数
    .name = "periodic_timer"  // 定时器名称（可选）
  };
  
  // 创建定时器
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  
  // 启动定时器，周期为100ms（100000微秒），重复执行
  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 100000));//us
}

void oledInit(void)
{
  Wire.begin(SDA_PIN, SCL_PIN);
  // OLED 初始化
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.setRotation(0);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("DFRobot");
  display.setCursor(100, 22);
  display.setTextSize(1);
  display.println("v1.0");
  display.display();
  delay(3000);
}

void updateDisplay()
{
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 200) return; // 刷新频率
  lastUpdate = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  
  // 检查是否在搜索模式
  NavigationState navState = getNavigationState();
  if (navState == NAV_SEARCHING) {
    // 搜索模式：显示搜索原因
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("Search:");
    display.setCursor(0, 20);
    
    SearchReason reason = getSearchReason();
    switch(reason) {
      case SEARCH_NO_GAP:
        display.print("S:1");
        display.setTextSize(1);
        display.setCursor(0, 45);
        display.print("No Gap");
        break;
      case SEARCH_COLLISION:
        display.print("S:2");
        display.setTextSize(1);
        display.setCursor(0, 45);
        display.print("Obstacle");
        break;
      default:
        display.print("S:0");
        display.setTextSize(1);
        display.setCursor(0, 45);
        display.print("Searching");
        break;
    }
  } else {
    if (motorDriveState != MOTOR_DRIVE_ENABLED) {
      // 测试模式：显示完整计划步骤
      char steps[6][10];
      float values[6];
      int count = getPlannedStepsForDisplay(steps, values, 6);
      if (count == 0) {
        display.print("IDLE");
      } else {
        for (int i = 0; i < count; i++) {
          display.setCursor(0, i * 10);
          display.print(steps[i]);
          display.print(":");
          display.print((int)values[i]);
        }
      }
    } else {
      // 正常模式：显示下一步执行步骤
      char step1[10] = "", step2[10] = "", step3[10] = "";
      float value1 = 0.0, value2 = 0.0, value3 = 0.0;

      getNextStepInfo(step1, &value1, step2, &value2, step3, &value3);

      // 显示步骤1
      if (strlen(step1) > 0) {
        display.print(step1);
        display.print(":");
        display.print((int)value1);
        if (strlen(step2) > 0 || strlen(step3) > 0) {
          display.print(",");
        }
      }

      // 显示步骤2
      if (strlen(step2) > 0) {
        display.print(step2);
        display.print(":");
        display.print((int)value2);
        if (strlen(step3) > 0) {
          display.print(",");
        }
      }

      // 显示步骤3
      if (strlen(step3) > 0) {
        display.print(step3);
        display.print(":");
        display.print((int)value3);
      }

      // 如果没有步骤信息，显示状态
      if (strlen(step1) == 0 && strlen(step2) == 0 && strlen(step3) == 0) {
        display.print("IDLE");
      }
    }
  }
  
  display.display();
}

bool checkCollision(int thresholdMm) {
  int count = 0;
  for (int i = 0; i < TOTAL_POINTS; i++) {
    if (zValues[i] > 0 && zValues[i] < thresholdMm) {
      count++;
    }
  }
  return count >= 5; // 至少2个点确认
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, PATH_MAP_SERIAL_RX, PATH_MAP_SERIAL_TX);  // 初始化路径地图串口
  mcpwm_init();         // 初始化电机控制模块
  setSpeed(0,0);
  wy6005.begin(921600);
  // 配置为单线输出模式：第4行，28点到46点
  bool configSuccess = wy6005.configSingleLineMode(LINE_NUM, START_POINT, END_POINT, true);
  #if DEBUG_ENABLE
  if (configSuccess) {
    Serial.printf("成功配置单线模式：Line %d, Points %d-%d\n", LINE_NUM, START_POINT, END_POINT);
    Serial.printf("共%d个点\n", TOTAL_POINTS);
    Serial.printf("一帧数据大小：%d字节\n", FRAME_SIZE);
    wy6005.saveConfig();
  } else {
    Serial.println("配置单线模式失败");
  }
  #endif
  oledInit();
  timerInit();          // 初始化定时器
  keyInit();            // 初始化按键检测模块

}

void loop() {

  static float targetAngle = 0.0f;
  static float targetDistance = 0.0f;
  static long  analysisCount = 0;
  static long  startWaitAnalysisCount = 0;
  updateDisplay();
  if (isStepPauseActive()) {
    if (keyCheck()) {
      resumeStepPause();
    }
  } else {
    handleKeyEvents();
  }
  updateNavigation();  // 更新导航状态机（持续调用电机控制函数）
  //test();
  // if(motorDriveState == MOTOR_DRIVE_ENABLED)
  // {
  //   turnAngle(90.0f);
  // }
  // else
  // {
  //   isMoving = false;
  // }
  while (Serial1.available()) 
  {
    uint8_t c = Serial1.read();
    if (inSyncMode) 
    {
      frameBuffer[frameIndex++] = c;
      if (frameIndex == FRAME_SIZE) 
      {
        for (int i = 0; i < TOTAL_POINTS; i++) 
        { 
          int16_t tempX, tempZ, tempI;
          int pointStartIndex = DATA_HEADER_SIZE + i * POINT_DATA_SIZE;
          parsePointData(&frameBuffer[pointStartIndex],&tempZ);
          rawZBuffer[i][sampleCount] = tempZ;
        }
        sampleCount++; // 增加采样计数
      if (sampleCount >= FILTER_SAMPLES) 
      {    
        for (int i = 0; i < TOTAL_POINTS; i++) 
        {
            zValues[i] = calculateTrimmedFilter(rawZBuffer[i], FILTER_SAMPLES);
        }
        sampleCount = 0; // 重置计数器
        
        runGapTest();    // <--- 在此处调用地图算法

        // if(turnState == TURN_STOPING)
        // {
        //   findBestRowAndAnalyzeGaps(foundGaps, totalGaps, 10, validRow, currentMoveType);
        //   sensorFlag = true; //数据采集+解析完成
        // }
        analysisCount++;
        #if DEBUG_ENABLE
        // --- 1. 打印顶部完整索引 ---
        Serial.print("    "); 
        for (int i = 0; i < TOTAL_POINTS; i++) {
           Serial.printf("%02d ", i);
        }
        Serial.println(); 
        for(int j = 0 ;j < GRID_ROWS; j++)
        {
          Serial.printf("%02d: ", j); // 打印行号
          
          for (int i = 0; i < TOTAL_POINTS; i++) {
               Serial.printf("%s ", zValues[i] > (j* ROW_INTERVAL_CM * 10) ? ". " : "# ");//1代表障碍物,0代表无障碍物,
          }
          Serial.println(); 
        }
        Serial.println();
        #endif
         
      }
      frameIndex = 0;
      inSyncMode = false;
    }
  } 
    // 检测同步序列
    else if (frameIndex == 0 && c == 0x0A) {  // 'O'
      frameBuffer[frameIndex++] = c;
    } 
    else if (frameIndex == 1 && c == 0x4F) {  // 'K'
      frameBuffer[frameIndex++] = c;
    } 
    else if (frameIndex == 2 && c == 0x4B) {  // '\r'
      frameBuffer[frameIndex++] = c;
    } 
    else if (frameIndex == 3 && c == 0x0A) {  // '\n'
      frameBuffer[frameIndex++] = c;
      inSyncMode = true;        // 找到同步序列，开始收集整个帧的数据
      sensorFlag = false;
    } 
    else {
      // 同步失败，重置索引
      frameIndex = 0;
      #if DEBUG_ENABLE
      Serial.println("同步失败");//调试的方法
      #endif
    }
  }
  
    // if(motorDriveState == MOTOR_DRIVE_ENABLED)
    // {
    //   // 当处于转向任务序列中时，强制维持转向状态，忽略传感器新数据导致的状态变化
    //   eMoveType_t logicMoveType = currentMoveType;
    //   if (turnState == TURN_WAIT)
    //   {
    //      logicMoveType = MOVE_TURN_LEFT; 
    //   }

    //   if (logicMoveType == MOVE_STOP) 
    //   {
    //     isMoving  =  false;
    //     isTurning =  false;
    //     setSpeed(0, 0);         // 停车
    //     turnState = TURN_STOPING;
    //   } 
    //   else if (logicMoveType == MOVE_STRAIGHT) 
    //   {
    //     setSpeed(LEFT_SPEED, RIGHT_SPEED);
    //     turnState = TURN_STOPING;
    //   } 

    //   else if (logicMoveType == MOVE_TURN_LEFT || logicMoveType == MOVE_TURN_RIGHT) 
    //   {
    //       switch(turnState)
    //       {
    //         case TURN_STOPING:

    //             targetAngle = angleDeg;
    //             targetDistance = driveDistanceCm;
    //             turnState = TURN_TURNING;
              
    //         break;

    //         case TURN_TURNING:
    //           turnAngle(targetAngle * 2.0f);
    //           if(isTurning == false)
    //           {
    //             turnState = TURN_STRAIGHT;
    //           }
    //           break;
              
    //         case TURN_STRAIGHT:
    //           moveDistance(targetDistance / 2.0f);
    //           // 检查障碍物防止撞击
    //           // if (isMoving) {
    //           //     if (checkCollision(100)) { // 15cm内有障碍物
    //           //         isMoving = false;
    //           //         setSpeed(0, 0); // 紧急停车
    //           //         turnState = TURN_RETREAT;
    //           //         break; 
    //           //     }
    //           // }

    //           if(isMoving == false)
    //           {
    //             turnState = TUNR_TURNBACK;
    //           }
    //           break;

    //         case TURN_RETREAT:
    //           moveDistance(-10.0f); // 后退10cm
    //           if(isMoving == false)
    //           {
    //              turnState = TURN_WAIT; 
    //              // 重新开始分析
    //              startWaitAnalysisCount = analysisCount;
    //              currentMoveType = MOVE_STRAIGHT; 
    //           }
    //           break;

    //         case TUNR_TURNBACK:
    //           turnAngle(-targetAngle * 2.0f);
    //           if(isTurning == false)
    //           {
    //             turnState = TURN_LEAVE_HOLE;
    //           }
    //           break; 
            
    //         case TURN_LEAVE_HOLE:
    //           moveDistance(10.0f);
    //           if(isMoving == false)
    //           {
    //             turnState = TURN_WAIT;
    //             startWaitAnalysisCount = analysisCount;
    //             currentMoveType = MOVE_STRAIGHT; // 回正完成后，先置为STOP，防止使用旧的转向指令再次进入转向逻辑
    //           }
    //           break;

    //         case TURN_WAIT:
    //           if (analysisCount - startWaitAnalysisCount == 0)
    //           {
    //              turnState = TURN_STOPING; // 切换回STOPING状态，允许下一帧数据进行计算分析
    //              analysisCount = 0;
    //           }
    //           break; 
    //       }
    //   }
    // }
    // else 
    // {
    //   setSpeed(0,0);
    //   carState = STOPING;
    //   isMoving = false;
    //   isTurning = false;
    // }
}
