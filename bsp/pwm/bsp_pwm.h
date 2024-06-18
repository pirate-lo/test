#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "tim.h"
#include "stdint.h"

#define PWM_DEVICE_CNT 16 // 最大支持的PWM实例数量

typedef struct 
{
    TIM_HandleTypeDef *htim;                 // TIM句柄
    uint32_t channel;                        // 通道
    uint32_t period;                         // 周期
    uint32_t pulse;                          // 脉宽
    void *id;                                // 实例ID
}PWM_Instance;

typedef struct 
{
    TIM_HandleTypeDef *htim;                 // TIM句柄
    uint32_t channel;                        // 通道
    uint32_t period;                         // 周期
    uint32_t pulse;                          // 脉宽
    void *id;                                // 实例ID
}PWM_Init_Config_s;

PWM_Instance *PWMRegister(PWM_Init_Config_s *config);
void PWMStart(PWM_Instance *pwm);
void PWMStop(PWM_Instance *pwm);
void PWMSetPulse(PWM_Instance *pwm, uint32_t pulse);

#endif // !BSP_PWM_H
