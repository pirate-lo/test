#include "ops.h"
#include "bsp_dwt.h"
#include "gpio.h" //GPIO模块未建立bsp层，有待完善

uint8_t DMA_OPS_buf[38] = {0x5A, 0xA5,         33,          0x82, 0x03, 0x01,      0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,   0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  0x00, 0x00,   0x00, 0x00,  0x00, 0x00,   0x00, 0x00,  0x00, 0x00, 0x00, 0x00};
//数据前缀 (包含指令总数据长度)  指令  起始地址       第一个数据 0301 6-9    第二个数据 10-13        14-15       第四个数据 16-19           20-23              24-25        26-27      28-29        30-31       32-33      34-35
Union_OPS OPS;
uint8_t OPS_ready = 0;
static int X_for_DMA = 0, Y_for_DMA = 0, Z_for_DMA = 0;
static int X_FLASH_DMA = 0, Y_FLASH_DMA = 0, Z_FLASH_DMA = 0;

extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
static USARTInstance *ops_usatr_instance;
/*--------------------------------UART6的回调函数，接受ops数据并开启DMA传输数据----------------*/
static uint16_t get_ops_date(uint8_t *rx_buf,
                             uint16_t *flags_register,
                             uint8_t *rx_data)
{
    if(rx_buf[0] == 0x0d && rx_buf[1] == 0x0a)
    {
        *flags_register = (rx_buf[27] << 8) | rx_buf[26];
        memcpy(&rx_data[2], rx_buf, 28);
    }
    return 0;
}

static uint8_t i = 0,Y = 0;
extern uint8_t ch;

static void OPS_usart_back()
{   
    if(ch == 0xd)
    {   
        USARTServiceInit(ops_usatr_instance);
        HAL_TIM_Base_Start_IT(&htim5);
        OPS_ready = 1;
    }
    else 
    {
        HAL_UART_Receive_IT(&huart6, &ch, 1);
        DWT_Delay(0.1);
        Y ++;
    }
}

static void OPSCallback()
{
    uint16_t flag_register;
    OPS_usart_back();
    if(OPS_ready == 1)
    {
        get_ops_date(ops_usatr_instance->recv_buff,&flag_register,OPS.data);
        USARTServiceInit(ops_usatr_instance);
        if(i == 0)
        {
            Cali_Ops();
            i +=1;
        }
    }
}

static void OPSOfflineCallback()
{
    USARTServiceInit(ops_usatr_instance);
}

/*---------------------------------------ops初始化-----------------------------*/
Union_OPS *OPS_Init(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s opsuart;
    opsuart.module_callback = OPSCallback;
    opsuart.recv_buff_size = OPS_RECV_SIZE;
    opsuart.usart_handle = _handle;
    ops_usatr_instance = USARTRegister(&opsuart);

    return &OPS;
}

/*--------------------TIM4的回调函数检测ops数据是否正确-----------------------------*/
void OPS_Check(void)
{
    if(OPS.data[2] != 0x0D || OPS.data[3] != 0x0A || OPS.data[28] != 0x0A || OPS.data[29] != 0x0D)  //OPS返回数据帧头 帧尾检测
        {
            //若满足条件 进行数据对帧
            HAL_TIM_Base_Stop_IT(&htim5); //关闭检测定时器
            HAL_UART_Receive_IT(&huart6, ch, 1);    //开启串口中断
            __HAL_DMA_DISABLE(&hdma_usart6_rx); //DMA数据流关闭
            HAL_UART_DMAStop(&huart6);  //关闭DMA串口接收
    
           // HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_SET);
           // HAL_GPIO_WritePin(GPIOF,GPIO_PIN_11,GPIO_PIN_RESET);       //数据异常亮红灯
        }
    else
    {
            //HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_RESET);       //数据正常亮绿灯   
            //HAL_GPIO_WritePin(GPIOF,GPIO_PIN_11,GPIO_PIN_SET); 
    }

//......................................................................//
        X_for_DMA = -OPS.ActVal[4] * 10;
        Y_for_DMA = -OPS.ActVal[5] * 10;
        Z_for_DMA = -OPS.ActVal[1] * 100;

        DMA_OPS_buf[6] = X_for_DMA >> 24;     //取数据的高8位
        DMA_OPS_buf[7] = X_for_DMA >> 16;
        DMA_OPS_buf[8] = X_for_DMA >> 8;
        DMA_OPS_buf[9] = X_for_DMA;           //取数据的低8位

        DMA_OPS_buf[10] = Y_for_DMA >> 24;    //取数据的高8位
        DMA_OPS_buf[11] = Y_for_DMA >> 16;
        DMA_OPS_buf[12] = Y_for_DMA >> 8;
        DMA_OPS_buf[13] = Y_for_DMA;          //取数据的低8位

        DMA_OPS_buf[14] = Z_for_DMA >> 8;     //取数据的高8位
        DMA_OPS_buf[15] = Z_for_DMA;          //取数据的低8位

        DMA_OPS_buf[16] = X_FLASH_DMA >> 24;     //取数据的高8位
        DMA_OPS_buf[17] = X_FLASH_DMA >> 16;
        DMA_OPS_buf[18] = X_FLASH_DMA >> 8;
        DMA_OPS_buf[19] = X_FLASH_DMA;           //取数据的低8位

        DMA_OPS_buf[20] = Y_FLASH_DMA >> 24;    //取数据的高8位
        DMA_OPS_buf[21] = Y_FLASH_DMA >> 16;
        DMA_OPS_buf[22] = Y_FLASH_DMA >> 8;
        DMA_OPS_buf[23] = Y_FLASH_DMA;          //取数据的低8位

        DMA_OPS_buf[24] = Z_FLASH_DMA >> 8;     //取数据的高8位
        DMA_OPS_buf[25] = Z_FLASH_DMA;          //取数据的低8位
}

/*---------------------------------发送给ops的数据，包括清零、更新XY坐标----------------------*/
static void PC_SendChar(uint8_t DataToSend)
{
    while(USART_GetFlagStatus(&huart6, UART_FLAG_TXE) == RESET);
    HAL_UART_Transmit(&huart6,&DataToSend,1,1000);
    while(USART_GetFlagStatus(&huart6, UART_FLAG_TC) == RESET);
}

static void PC_SendString(uint8_t* str)
{
    while(*str)
    {
        PC_SendChar(*str);
        str++;
    }
}

void Cali_Ops()                /////////////清零
{
    PC_SendString((uint8_t*) "ACT0");
}

void cali_angle_ops(float angle)
{
    PC_SendString((uint8_t*) "ACTJangle");
}

void Stract(char str1[], uint8_t str2[], uint8_t num)
{
    int i = 0, j = 0;
    while(str1[i] != '\0')
        i++;

    for(j = 0; j < num; j++)
    {
        str1[i++] = str2[j];
    }
}

void Update_X(float posx)
{
    char update_x[8] = "ACTX";
    static union
    {
        float X;
        uint8_t data[4];
    } set;

    set.X = posx;
    Stract(update_x, set.data, 4);

    PC_SendString((uint8_t *)"ACTX");
}

void Update_Y(float posy)
{
    char update_y[8] = "ACTY";
    static union
    {
        float Y;
        uint8_t data[4];
    } set;

    set.Y = posy;
    Stract(update_y, set.data, 4);

    PC_SendString((uint8_t *)"ACTY");
}

void Update_Z(float posz)
{
    char update_z[8] = "ACTJ";
    static union
    {
        float Z;
        uint8_t data[4];
    } set;

    set.Z = posz;
    Stract(update_z, set.data, 4);

    PC_SendString((uint8_t *)"ACTZ");
}