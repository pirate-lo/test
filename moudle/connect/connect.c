/*
 * @Author: pirate-lo @caoboxun8
 * @Date: 2024-07-31 09:47:09
 * @LastEditors: pirate-lo @caoboxun8
 * @LastEditTime: 2024-07-31 11:11:14
 * @FilePath: \play\moudle\connect\connect.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "connect.h"
#include "bsp_dwt.h"
#include "connect.h"

extern UART_HandleTypeDef huart2;
USARTInstance *connect_instance;
uint16_t con_code;

static void Connectback()
{
    if(connect_instance->recv_buff[0] == 0x2C &&connect_instance->recv_buff[2] == 0x5B)
    con_code = connect_instance->recv_buff[1];
     USARTServiceInit(connect_instance);
}


void ConnectInit()
{
    USART_Init_Config_s conuart;
    conuart.module_callback = Connectback;
    conuart.recv_buff_size = CONNECT_SIZE;
    conuart.usart_handle = &huart2;
    connect_instance = USARTRegister(&conuart);
}