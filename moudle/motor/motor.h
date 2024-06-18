#ifndef MOTOR_H
#define MOTOR_H

#include "bsp_pwm.h"
#include "bsp_encoder.h"

#include "motor_def.h"

#define MOTOR_MAX 8

typedef struct 
{
    int16_t last_ecd;        // 上一次读取的编码器值
    int16_t ecd;             
    float speed_aps;          // 角速度,单位为:度/秒 
}Motor_Measure;

typedef struct 
{
    Motor_Measure measure;
    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;    // 电机控制器
    PWM_Instance *pwm_1;
    PWM_Instance *pwm_2;
    EncoderInstance *encoder;
    Motor_Working_Type_e stop_flag; // 启停标志
    uint8_t num;
}MotorInstance;

MotorInstance *MotorRegister(Motor_Init_Config_s *config);
void MotorEnable(MotorInstance *motor);
void MotorStop(MotorInstance *motor);
void MotorSet(MotorInstance *motor,float pule);
void MotorSetpule(MotorInstance *motor,float pule);
void MotorSetRef(MotorInstance *motor,float ref);
void MotorControl();


#endif // !MOTOR_H
