#include "Arduino.h"
#include "motor.h"
#include "key.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_struct.h"
#include "soc/mcpwm_reg.h"

void mcpwm_init(void)
{
  // 左电机初始化
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, LEFT_MOTOR_EN_PIN);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, LEFT_MOTOR_DIR_PIN);

  // 右电机初始化
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, RIGHT_MOTOR_EN_PIN);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, RIGHT_MOTOR_DIR_PIN);

  // 配置MCPWM单元0（左电机）
  mcpwm_config_t pwm_config_unit0;
  pwm_config_unit0.frequency = 1000;
  pwm_config_unit0.cmpr_a = 0;
  pwm_config_unit0.cmpr_b = 0;
  pwm_config_unit0.counter_mode = MCPWM_UP_COUNTER;
  pwm_config_unit0.duty_mode = MCPWM_DUTY_MODE_0;

  // 配置MCPWM单元1（右电机）
  mcpwm_config_t pwm_config_unit1;
  pwm_config_unit1.frequency = 1000;
  pwm_config_unit1.cmpr_a = 0;
  pwm_config_unit1.cmpr_b = 0;
  pwm_config_unit1.counter_mode = MCPWM_UP_COUNTER;
  pwm_config_unit1.duty_mode = MCPWM_DUTY_MODE_0;

  // 初始化MCPWM
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config_unit0);
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config_unit1);
}

void setSpeed(int leftSpeed, int rightSpeed)
{
  // 左电机控制
  if (leftSpeed == 0)
  {
     // 刹车模式: 拉高两个引脚 (对于L298N/TB6612等常见驱动，IN1=1/IN2=1为急刹)
     mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A);
     mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B);
  }
  else if (leftSpeed > 0)
  {
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_0); // 恢复PWM模式
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, leftSpeed);
  }
  else
  {
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_0); // 恢复PWM模式
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, -leftSpeed);
  }

  // 右电机控制
  if (rightSpeed == 0)
  {
     // 刹车模式
     mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_A);
     mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_B);
  }
  else if (rightSpeed > 0)
  {
    mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_B);
    mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_A, rightSpeed);
  }
  else
  {
    mcpwm_set_signal_high(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_B);
    mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_A, -rightSpeed);
  }
}

bool isTurning = false;
bool isMoving = false;
eCarState_t carState = STOPING;

void moveDistance(float distanceCm)
{
  static unsigned long moveStartMs = 0;
  static unsigned long moveDurationMs = 0;
  static float lastDistanceCm = 0.0f;

  if (!isMoving) 
  {
      moveDurationMs = (unsigned long)((fabs(distanceCm) / CAR_SPEED_CM_PER_SEC) * 1000.0f);
      lastDistanceCm = distanceCm;

      if (distanceCm > 0)
      {
          setSpeed(LEFT_SPEED, RIGHT_SPEED); // 设定前进速度
          moveStartMs = millis();
          isMoving = true;
      }
      else if(distanceCm < 0)
      {
          setSpeed(-LEFT_SPEED, -RIGHT_SPEED); // 设定后退速度
          moveStartMs = millis();
          isMoving = true;
      }
      else
      {
          setSpeed(0, 0);
      }
  }
  else 
  {
      // 正在移动中，检查时间
      unsigned long elapsed = millis() - moveStartMs;
      if (elapsed >= moveDurationMs)
      {
          isMoving = false;
          setSpeed(0, 0); // 停车              

          // 只在动作完成时打印一次关键动作信息
          if (lastDistanceCm > 0)
          {
              Serial.print("【动作】前进完成，耗时 ");
              Serial.print(elapsed);
              Serial.print(" ms / 目标 ");
              Serial.print(moveDurationMs);
              Serial.println(" ms");
          }
          else if (lastDistanceCm < 0)
          {
              Serial.print("【动作】后退完成，耗时 ");
              Serial.print(elapsed);
              Serial.print(" ms / 目标 ");
              Serial.print(moveDurationMs);
              Serial.println(" ms");
          }
      }
  }
}

void turnAngle(float angleDeg)
{
  static unsigned long turnStartMs = 0;
  static unsigned long turnDurationMs = 0;
  static float lastAngleDeg = 0.0f;

  if (!isTurning) 
  {
      if (abs(angleDeg) > 1.0f) 
      {
          turnDurationMs = (unsigned long)((abs(angleDeg) / CAR_TURN_DEG_PER_SEC) * 1000.0f);
          lastAngleDeg = angleDeg;
          
          if (angleDeg > 0) 
          {
              setSpeed(-RIGHT_SPEED, RIGHT_SPEED); 
          }
          else 
          {
              setSpeed(RIGHT_SPEED, -RIGHT_SPEED); 
          }
          
          turnStartMs = millis();
          isTurning = true;
      }
      else
      {
         setSpeed(0, 0); // 角度太小，不动
      }
  }
  else 
  {
      unsigned long elapsed = millis() - turnStartMs;
      if (elapsed >= turnDurationMs)
      {
          setSpeed(0, 0); // 停止
          isTurning = false;

          // 只在动作完成时打印一次关键动作信息（带耗时）
          if (lastAngleDeg > 0)
          {
              Serial.print("【动作】左转完成，耗时 ");
              Serial.print(elapsed);
              Serial.print(" ms / 目标 ");
              Serial.print(turnDurationMs);
              Serial.println(" ms");
          }
          else if (lastAngleDeg < 0)
          {
              Serial.print("【动作】右转完成，耗时 ");
              Serial.print(elapsed);
              Serial.print(" ms / 目标 ");
              Serial.print(turnDurationMs);
              Serial.println(" ms");
      }
  }
}
}