#include "bsp_pwm.h"
#include "stdlib.h"
#include "memory.h"

static uint8_t idx;
static PWM_Instance *pwm_instance[PWM_DEVICE_CNT];


PWM_Instance *PWMRegister(PWM_Init_Config_s *config)
{
    if (idx >= PWM_DEVICE_CNT) // 超过最大实例数,考虑增加或查看是否有内存泄漏
        while (1)
            ;
    PWM_Instance *pwm = (PWM_Instance *)malloc(sizeof(PWM_Instance));
    memset(pwm, 0, sizeof(PWM_Instance));

    pwm->htim = config->htim;
    pwm->channel = config->channel;
    pwm->period = config->period;
    pwm->pulse = config->pulse;
    pwm->id = config->id;
    // 启动PWM
    HAL_TIM_PWM_Start(pwm->htim, pwm->channel);                
    __HAL_TIM_SetCompare(pwm->htim, pwm->channel, pwm->pulse); // 设置占空比

    pwm_instance[idx++] = pwm;
    return pwm;
}

void PWMStart(PWM_Instance *pwm)
{
    HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
    __HAL_TIM_SetCompare(pwm->htim, pwm->channel, pwm->pulse);
}


void PWMStop(PWM_Instance *pwm)
{
    HAL_TIM_PWM_Stop(pwm->htim, pwm->channel);
}

void PWMSetPulse(PWM_Instance *pwm, uint32_t pulse)
{
    pwm->pulse = pulse;
    __HAL_TIM_SetCompare(pwm->htim, pwm->channel, pwm->pulse);
}
