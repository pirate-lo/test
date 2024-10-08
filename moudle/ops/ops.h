#ifndef _OPS_H
#define _OPS_H

#include "bsp_uart.h"
#include "stdint.h"

#define OPS_RECV_SIZE 22u

typedef struct
{
    uint8_t data[22];
} Union_OPS;

typedef struct 
{
    uint8_t code[9];
}Code;


Union_OPS *OPS_Init(UART_HandleTypeDef *_handle);
void OPS_Check();
void Cali_Ops(void);
void cali_angle_ops(float angle);
void Update_X(float posx);
void Update_Y(float posy);
void Update_Z(float posz);
void Stract(char str1[], uint8_t str2[], uint8_t num);

#endif 
