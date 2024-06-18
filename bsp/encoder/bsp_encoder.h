#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "tim.h"
#include "stdint.h"

#define ENCODER_MAX 8

typedef struct 
{
    TIM_HandleTypeDef *htim;                 // TIM句柄
    void (*module_callback)(struct _ *);
    void *id;
}EncoderInstance;

typedef struct 
{
    TIM_HandleTypeDef *htim;                 // TIM句柄
    void (*module_callback)(EncoderInstance *);
    void *id;
}Encoder_Config;

EncoderInstance *Encoder_Register(Encoder_Config *config);
short Readcounter(EncoderInstance *encoder);
void PeriodElapsedCallback(void);

#endif 

