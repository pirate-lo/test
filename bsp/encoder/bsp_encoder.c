#include "bsp_encoder.h"
#include "stdlib.h"
#include "memory.h"

static uint8_t idx;
static EncoderInstance *encoderinstance[ENCODER_MAX];

EncoderInstance *Encoder_Register(Encoder_Config *config)
{
    if(idx >= ENCODER_MAX)
        while(1)
            ;
    EncoderInstance *encoder = (EncoderInstance *)malloc(sizeof(EncoderInstance));
    memset(encoder,0,sizeof(EncoderInstance));

    encoder->htim = config->htim;
    encoder->module_callback = config->module_callback;
    encoder->id = config->id;

    HAL_TIM_Encoder_Start(encoder->htim,TIM_CHANNEL_ALL);

    encoderinstance[idx++] = encoder;
    return encoder;
}

short Readcounter(EncoderInstance *encoder)
{
    return (short)__HAL_TIM_GET_COUNTER(encoder->htim);
}

void PeriodElapsedCallback(void)
{
    for (size_t i = 0; i < idx; ++i)
    {
        encoderinstance[i]->module_callback(encoderinstance[i]);
    }
}
