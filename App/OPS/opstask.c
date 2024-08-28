/*
 * @Author: pirate-lo @caoboxun8
 * @Date: 2024-05-22 23:23:11
 * @LastEditors: pirate-lo @caoboxun8
 * @LastEditTime: 2024-08-01 16:44:13
 * @FilePath: \play\App\OPS\opstask.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "opstask.h"
#include "message.h"


extern UART_HandleTypeDef huart6;
extern uint8_t OPS_ready;
extern uint8_t ch;
extern USARTInstance *ops_usatr_instance;
extern TIM_HandleTypeDef htim5;
extern USARTInstance *connect_instance;
extern uint16_t con_code;

static Union_OPS *OPS_rate;
static Code *code;
static PIDInstance pid_ops;
static uint16_t action_ok;

static Publisher_t *ops_pub;                    // 用于发布底盘的数据
static Subscriber_t *ops_sub;                   // 用于订阅底盘的控制命令 
static OPS_chassis ops_cmd_send;
static ServoInstance *servo;

static int servo_run;
static float X, Y, loc_x,loc_y;
static float V_a_y=0,V_a_x=0;
static float location_x_speed, location_y_speed, location_z_speed;


#define RUN_size 50
#define CON_size 9
static uint8_t RUN_date[RUN_size] = {0x2C,0,0,0,0X5B};
static uint8_t CON_date[CON_size] = {0x2C,0X11,0X44,0X65,0X01,0X55,0X41,0X01,0x5B};

void Ops_Init()
{
   OPS_rate = OPS_Init(&huart6);
   PID_Init_Config_s pid_config = {
        .Kp = 5.5,
        .Ki = 0.0005,
        .Kd = 0,
        .DeadBand = 2,
        .MaxOut = 800,
    };
    PIDInit(&pid_ops,&pid_config);

    Servo_Init_Config_s Servo_Init_Config = {
        .htim = &htim5,
        .Channel = TIM_CHANNEL_2,
        .Servo_Angle_Type = Start_mode,
        .Servo_type = Servo180,
    };
    servo = ServoInit(&Servo_Init_Config);
    Servo_Motor_StartSTOP_Angle_Set(servo,90,95);


    ops_sub = SubRegister("chassis_feed", sizeof(OPS_chassis));
    ops_pub = PubRegister("chassis_cmd", sizeof(OPS_chassis));




}

static float Ops_abs(float x)
{
    if(x < 0)
    x = -x;
    return x;
}


static int went_1,went_2, task1,task2;
static uint8_t run;

static void Clearn_ops()
{
    went_1 = 0;
    went_2 = 0;
    location_x_speed = 0;
    location_y_speed = 0;
    location_z_speed = 0;   
}

static void Openmv_get()
{
    memcpy(code->code,&OPS_rate->data[1],9);
    if( OPS_rate->data[10] == 0){X = -(float)(OPS_rate->data[12] *256 + OPS_rate->data[11]);}
    else X = (float)(OPS_rate->data[12] *256 + OPS_rate->data[11]) ;
    if( OPS_rate->data[13] == 0){Y = -(float)OPS_rate->data[14];}
    else Y = (float)OPS_rate->data[14];
    if( OPS_rate->data[15] == 0){loc_x = -(float)(OPS_rate->data[17] *256 + OPS_rate->data[16]);}
    else loc_x = (float)(OPS_rate->data[17] *256 + OPS_rate->data[16]);
    if( OPS_rate->data[18] == 0){loc_y = -(float)OPS_rate->data[19];}
    else loc_y = (float)OPS_rate->data[19] ;
}

/*
static connect_send()
{
    if(loc_x > 0){ if(loc_y >= 0) CON_date[4] = 0x11;else CON_date[4] = 0x10;}
    if(loc_x < 0){ if(loc_y >= 0) CON_date[4] = 0x01;else CON_date[4] = 0x00;}
    if(X > 0){ if(Y >= 0) CON_date[1] = 0x11;else CON_date[1] = 0x10;}
    if(X < 0){ if(Y >= 0) CON_date[1] = 0x01;else CON_date[1] = 0x00;}
    CON_date[2] = (uint8_t)X ;
    CON_date[3] = (uint8_t)Y; CON_date[5] = (uint8_t)loc_x ;CON_date[6] = (uint8_t)loc_y;CON_date[7] = 1;
}
*/

/*********************任务进行**************/
static void RUNTask(uint8_t qi,uint8_t di)
{
    if(OPS_rate->data[0] == 0)
    {   
        RUN_date[1] = 0;
        RUN_date[2] = 0;
        RUN_date[3] = 1;
        USARTSend(ops_usatr_instance,RUN_date,RUN_size, USART_TRANSFER_DMA);
        if(OPS_rate->data[0] != 0x2C || OPS_rate->data[21] != 0x5B )
            HAL_UART_Receive_IT(&huart6, ch, 1);
    }
    else if(OPS_rate->data[0] == 0x2C )
    {
        RUN_date[1] = qi;
        RUN_date[2] = di;
        RUN_date[3] = 1;
        USARTSend(ops_usatr_instance,RUN_date,RUN_size, USART_TRANSFER_DMA);
    }
    if(OPS_rate->data[20] == 1)
    {
        Openmv_get();
        if(Ops_abs(X) > 4 && went_1 == 0) {location_x_speed = PIDCalculate(&pid_ops,X,0);went_1++;}
        if(Ops_abs(Y) > 4 && went_1 == 1) {location_y_speed = PIDCalculate(&pid_ops,Y,0);went_1++;}
        if(went_1 == 2) {went_1++;}
        if(Ops_abs(loc_x) > 4 && went_1 == 3) {location_x_speed = PIDCalculate(&pid_ops,loc_x,0);went_1++;}
        if(Ops_abs(loc_y) > 4 && went_1 == 4) {location_y_speed = PIDCalculate(&pid_ops,loc_y,0);went_1++;}
        if(went_1 == 5) {went_1++;}
        if(went_1 == 6){RUN_date[3] = 2;
        for(int i = 0;i < 1000; i++)
        {USARTSend(ops_usatr_instance,RUN_date,RUN_size, USART_TRANSFER_DMA);}
        run ++;
        }
    }
}

uint8_t Servo0[] = {0x55, 0x55 ,0x055 ,0x06 ,0x00, 0x01 ,0x00 };
uint8_t Servo1[] = {0x55, 0x55 ,0x055 ,0x06 ,0x01, 0x01 ,0x00 };
 
void OPSTask()
{
    if(run == 0)
    {
        USARTSend(ops_usatr_instance,Servo0,sizeof(Servo0), USART_TRANSFER_DMA);
        run ++;
    }
    if(run == 1)
    {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET);
        run ++;
    }
    if(run == 2)
    {
        USARTSend(ops_usatr_instance,Servo1,sizeof(Servo1), USART_TRANSFER_DMA);
        run ++;
    }
    if(run == 3)
    {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET);
        run ++;
    }

}