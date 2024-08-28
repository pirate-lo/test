#include "chassis.h"
#include "robot_def.h"

static MotorInstance *motor1, *motor2 ,*motor3, *motor4; //从右上角逆时针开始
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令 
static OPS_chassis chassis_cmd_recv;

static float chassis_SPEED_PID[3] = {0};
static float chassis_vx =0, chassis_vy = 0, chassis_wz = 0;     
static float vt_1 = 1000, vt_2 = 1000, vt_3 = 1000, vt_4 = 1000;

void ChassisInit(void)
{   

    chassis_SPEED_PID[0]=10;
    chassis_SPEED_PID[1]=0.000025;
    chassis_SPEED_PID[2]=0.002;
    Motor_Init_Config_s config ={
        .controller_param_init_config.speed_PID = {
            .Kd = chassis_SPEED_PID[2],
            .Ki = chassis_SPEED_PID[1],
            .Kp = chassis_SPEED_PID[0],
            .IntegralLimit = 2000,
            .Output_LPF_RC = 0.02,
            .DeadBand = 20,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
            .MaxOut = 5000,
        },
        .controller_setting_init_config = {
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
        },
        .encoder_config.htim = &htim1,
        .pwm_1_config = {
            .htim = &htim10,
            .channel = TIM_CHANNEL_1,
        },
        .pwm_2_config = {
            .htim = &htim9,
            .channel = TIM_CHANNEL_1,
        },
    };

    config.num = 2;
    config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor2 = MotorRegister(&config);
    
    config.encoder_config.htim = &htim2;
    config.pwm_2_config.htim = &htim12;
    config.pwm_2_config.channel = TIM_CHANNEL_1;
    config.pwm_1_config.htim = &htim12;
    config.pwm_1_config.channel = TIM_CHANNEL_2;
    config.num = 3;
    config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor3 = MotorRegister(&config);

    config.encoder_config.htim = &htim3;
    config.pwm_2_config.htim = &htim11;
    config.pwm_2_config.channel = TIM_CHANNEL_1;
    config.pwm_1_config.htim = &htim14;
    config.pwm_1_config.channel = TIM_CHANNEL_1;
    config.num = 4;
    config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor4 = MotorRegister(&config);

    config.encoder_config.htim = &htim4;
    config.pwm_1_config.htim = &htim8;
    config.pwm_1_config.channel = TIM_CHANNEL_4;
    config.pwm_2_config.htim = &htim9;
    config.pwm_2_config.channel = TIM_CHANNEL_2;
    config.num = 1;
    config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor1 = MotorRegister(&config);

    chassis_sub = SubRegister("chassis_cmd", sizeof(OPS_chassis));
    chassis_pub = PubRegister("chassis_feed", sizeof(OPS_chassis));

    HAL_TIM_Base_Start_IT(&htim7);

}

#define CENTER (car_long+car_wide)

static void MecanumCalculate()
{
    vt_1 = (chassis_vx + chassis_vy + chassis_wz * CENTER);
    vt_2 = (chassis_vx + chassis_vy - chassis_wz * CENTER);
    vt_3 = (-chassis_vx + chassis_vy - chassis_wz * CENTER);
    vt_4 = (-chassis_vx + chassis_vy + chassis_wz * CENTER);
}

/*开环控制*/
static void Chassissetpule()
{
    MotorSetpule(motor1,vt_1);
    MotorSetpule(motor2,vt_2);
    MotorSetpule(motor3,vt_3);
    MotorSetpule(motor4,vt_4);
}

/*闭环控制**/
static void Chassissetref()
{
    MotorSetRef(motor1,vt_1);
    MotorSetRef(motor2,vt_2);
    MotorSetRef(motor3,vt_3);
    MotorSetRef(motor4,vt_4);
}

static void ChassisEnable()
{
    MotorEnable(motor1);
    MotorEnable(motor2);
    MotorEnable(motor3);
    MotorEnable(motor4);
}

static void ChassisStop()
{
    MotorStop(motor1);
    MotorStop(motor2);
    MotorStop(motor3);
    MotorStop(motor4);
}
static float time,dead,time_line = 0;

static void Chassis_dug()
{
    if(time_line == 0)
    {
        time  = DWT_GetTimeline_ms();
        dead = 8000;
        time_line = 1;
    }
    if(time + dead > DWT_GetTimeline_ms())
    {
        chassis_vy = 120;
    }
    else
    {
            chassis_vy = 0;
    }
}

void ChassisTask()
{

        SubGetMessage(chassis_sub, &chassis_cmd_recv);      
        ChassisEnable();
        chassis_vx = chassis_cmd_recv.chassis_vx;
        chassis_vy = chassis_cmd_recv.chassis_vy;
        chassis_wz = chassis_cmd_recv.chassis_vz;
        MecanumCalculate();
        Chassissetref();
        MotorSetRef(motor1,100);

}