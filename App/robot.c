#include "robot.h"
#include "robot_task.h"

extern UART_HandleTypeDef huart6;
extern uint8_t ch;
extern uint8_t OPS_ready;

#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#warning check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!
#endif // !ROBOT_DEF_PARAM_WARNING

void RobotInit()
{
     // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();

    Bsp_Init();
    //Ops_Init();
    ChassisInit();
    Ops_Init();

    OSTaskInit();

    __enable_irq();
   if(OPS_ready != 1)
   {
    HAL_UART_Receive_IT(&huart6, &ch, 1);
    DWT_Delay(0.1);
   }
}

void RobotTask()
{
    ChassisTask();
    OPSTask();
}