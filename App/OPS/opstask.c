#include "opstask.h"
#include "message.h"

extern UART_HandleTypeDef huart6;
extern uint8_t OPS_ready;
extern uint8_t ch;
static Union_OPS *OPS_rate;
static PIDInstance pid_ops;
static uint16_t action_ok;

static Publisher_t *ops_pub;                    // 用于发布底盘的数据
static Subscriber_t *ops_sub;                   // 用于订阅底盘的控制命令 
static OPS_chassis ops_cmd_send;

static float Location_X, Location_Y, Locaton_Anger;
static float V_a_y=0,V_a_x=0;
static float location_x_speed, location_y_speed, location_z_speed;

void Ops_Init()
{
    OPS_rate = OPS_Init(&huart6);
    PID_Init_Config_s pid_config = {
        .Kp = 5.5,
        .Ki = 0.0005,
        .Kd = 0,
        .MaxOut = 190,
    };
    PIDInit(&pid_ops,&pid_config);

    ops_sub = SubRegister("chassis_feed", sizeof(OPS_chassis));
    ops_pub = PubRegister("chassis_cmd", sizeof(OPS_chassis));
}

static float Ops_abs(float x)
{
    if(x < 0)
    x = -x;
    return x;
}

static float Length = 0, cosx = 0, sinx = 0, V_slow = 0,lenth=0;
static float V_Length_max = 0, V_X_max = 0, V_Y_max = 0;
static float cosx_PID = 0, sinx_PID = 0;
static float V_a=0;
static int went_1,went_2;
static float turn_1;
static uint16_t  action_back = 0;

static void Clearn_ops()
{
    V_slow = 0;
    went_1 = 0;
    went_2 = 0;
    turn_1 = 0;
    location_x_speed = 0;
    location_y_speed = 0;
    location_z_speed = 0;   
}

static void Location_Plot(float x, float y, float z,float eero1)
{	
	  //x=-x;
	  //y=-y;
	  //z=-z;
    Location_X = x;           //PID X 目标参数赋值
    Location_Y = y;           //PID Y 目标参数赋值
    Locaton_Anger = z;        //PID Z 目标参数赋值

    Length = sqrt((x - OPS_rate->ActVal[4]) * (x - OPS_rate->ActVal[4]) + (y - OPS_rate->ActVal[5]) * (y - OPS_rate->ActVal[5])); //计算目标定位位移长度
    cosx = (x - OPS_rate->ActVal[4]) / Length;                            //计算目标定位航向角的余弦值
    sinx = (y - OPS_rate->ActVal[5]) / Length;                            //计算目标定位航向角的正弦值
    
    //短距离///##+(y-pos_y)

    if(V_slow < 100 && went_1 == 0)   //加速至运算最大速度
    {		
                V_slow += 10;            //(300/20)*10ms=150ms加速至峰值速度
                location_x_speed = V_slow * cosx;
                location_y_speed = V_slow * sinx;
                location_z_speed = PIDCalculate(&pid_ops,OPS_rate->ActVal[6],Locaton_Anger);
    }
    else
    {
        went_1 = 1;
    }
////////////定位目标点，误差 1mm, 1°///
    if(went_1 == 1 && Ops_abs(OPS_rate->ActVal[4] - x) > eero1*1.0f || Ops_abs(OPS_rate->ActVal[5] - y) > eero1*1.0f || Ops_abs(OPS_rate->ActVal[6] - z) > eero1*1.0f)
    {
        location_x_speed = PIDCalculate(&pid_ops,OPS_rate->ActVal[4],Location_X);
        location_y_speed = PIDCalculate(&pid_ops,OPS_rate->ActVal[5],Location_Y);
        location_z_speed = PIDCalculate(&pid_ops,OPS_rate->ActVal[6],Locaton_Anger);  
    }
    if(Ops_abs(OPS_rate->ActVal[4] - x) < eero1*1.0f && Ops_abs(OPS_rate->ActVal[5] - y) < eero1*1.0f && Ops_abs(OPS_rate->ActVal[6] - z) < eero1*1.0f)
    {
       Clearn_ops();
       action_back ++;
    }
}

static void Turn_anger(float turn_z)
{
    if (turn_z > 0)
    {
        if (turn_1 < 5)
        {
            location_z_speed += 75;
            turn_1 ++;
        }
        else
        {
            went_2 = 1;
        }
    }
    if (turn_z < 0)
    {
        if (turn_1 < 5)
        {
            location_z_speed -= 50;
            turn_1 ++;
        }
        else
        {
            went_2 =1;
        }
    }
    if(went_2 == 1 && Ops_abs(OPS_rate->ActVal[6] - turn_z) > 1.0f)
    {
        location_z_speed = 5*PIDCalculate(&pid_ops,OPS_rate->ActVal[6],turn_z);  
    }
    if(Ops_abs(OPS_rate->ActVal[6] - turn_z) < 1.0f)
    {

    }
}

void OPSTask()
{
        if(OPS_rate->data[2] != 0x0D || OPS_rate->data[3] != 0x0A || OPS_rate->data[28] != 0x0A || OPS_rate->data[29] != 0x0D)
        {
            action_ok = 0;
            HAL_UART_Receive_IT(&huart6, ch, 1);
        }
        else
        {
            action_ok = 1;
        }
        if(action_ok == 1)
        {
            switch (action_back)
            {
            case 0:
                Location_Plot(1000,1000,0,10);
                break;
            case 1:
                Location_Plot(1000,-1000,0,10);
                break;
            case 2:
                Location_Plot(-1000,-1000,0,10);
                break;    
            case 3:
                Location_Plot(-1000,1000,0,10);
                break;    
            case 4:
                Location_Plot(0,0,0,10);
                break;    
            default:
                break;
            }
        }
        else
        {
            location_x_speed = 0;
            location_y_speed = 0;
            location_z_speed = 0;
        }
        ops_cmd_send.chassis_vx = location_x_speed;
        ops_cmd_send.chassis_vy = location_y_speed;
        ops_cmd_send.chassis_vz = location_z_speed;
        PubPushMessage(ops_pub, (void *)&ops_cmd_send);           
}