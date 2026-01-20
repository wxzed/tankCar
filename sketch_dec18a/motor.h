#ifndef __MOTOR_H__
#define __MOTOR_H__
// 左电机
#define LEFT_MOTOR_EN_PIN   12       // 左电机EN引脚
#define LEFT_MOTOR_DIR_PIN  13      // 左电机方向引脚
// 右电机
#define RIGHT_MOTOR_EN_PIN  14      // 右电机EN引脚
#define RIGHT_MOTOR_DIR_PIN 21     // 右电机方向引脚

#define CAR_SPEED_CM_PER_SEC 22.0f 
#define CAR_TURN_DEG_PER_SEC 108.4f 

#define LEFT_SPEED  60
#define RIGHT_SPEED 60

typedef enum
{
    MOVEING,
    TURNING,
    BACKING,
    STOPING,
}eCarState_t;
void mcpwm_init(void);
void setSpeed(int leftSpeed, int rightSpeed);
void test(void);
void moveDistance(float distanceCm);
void turnAngle(float angleDeg);
extern eCarState_t carState;
extern bool isTurning ;
extern bool isMoving ;;
#endif