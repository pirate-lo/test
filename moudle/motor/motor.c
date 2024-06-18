#include "motor.h"

static uint8_t idx;
static MotorInstance *motorinstance[MOTOR_MAX];
static float Send[8] = {0};

static void Enconderget(EncoderInstance *_instance)
{
    Motor_Measure *measure = &(((MotorInstance *)_instance->id)->measure);
    measure->last_ecd = measure->ecd;
    measure->ecd = Readcounter(_instance);

    __HAL_TIM_SET_COUNTER(_instance->htim,0);

    if(_instance->htim == &htim3 || _instance->htim == &htim4)
    {
        measure->speed_aps = -(float)measure->ecd*100/9.6/11/4;
    }
    else
    {
        measure->speed_aps = (float)measure->ecd*100/9.6/11/4;
    }
}

MotorInstance *MotorRegister(Motor_Init_Config_s *config)
{
    MotorInstance *motor = (MotorInstance*)malloc(sizeof(MotorInstance));
    memset(motor,0,sizeof(MotorInstance));

    motor->motor_settings = config->controller_setting_init_config;   
    PIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    // 后续增加电机前馈控制器(速度和电流)
    motor->motor_controller.speed_foward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
    motor->motor_controller.current_foward_ptr = config->controller_param_init_config.current_feedforward_ptr;

    config->encoder_config.id = motor;
    config->encoder_config.module_callback = Enconderget;
    motor->encoder = Encoder_Register(&config->encoder_config);
    motor->pwm_1 = PWMRegister(&config->pwm_1_config);
    motor->pwm_2 = PWMRegister(&config->pwm_2_config);
    motor->num = config->num;

    motorinstance[idx] = motor;
    idx ++;
    return motor;
}

void MotorEnable(MotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void MotorStop(MotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

/**
* @param  pule 的范围为-100~100，和占空比对应
*
*
*/
void MotorSet(MotorInstance *motor,float pule)
{   
    static float puler;
    puler = pule /100 *5600;
    if(pule >= 0)
    {
        PWMSetPulse(motor->pwm_1,puler);
        PWMSetPulse(motor->pwm_2,0);
    }
    else
    {
        PWMSetPulse(motor->pwm_1,0);
        PWMSetPulse(motor->pwm_2,-puler);
    }
}

void MotorSetpule(MotorInstance *motor,float pule)
{
    if(pule >= 0)
    {
        PWMSetPulse(motor->pwm_1,pule);
        PWMSetPulse(motor->pwm_2,0);
    }
    else
    {
        PWMSetPulse(motor->pwm_1,0);
        PWMSetPulse(motor->pwm_2,-pule);
    }
}

void MotorSetRef(MotorInstance *motor,float ref)
{
    motor->motor_controller.pid_ref = ref;
}

void MotorControl()
{
    // 直接保存一次指针引用从而减小访存的开销,同样可以提高可读性
    uint8_t num; // 电机组号和组内编号
    float set;        // 电机控制发送设定值
    MotorInstance *motor;
    Motor_Control_Setting_s *motor_setting; // 电机控制参数
    Motor_Controller_s *motor_controller;   // 电机控制器
    Motor_Measure *measure;           // 电机测量值
    float pid_measure, pid_ref;             // 电机PID测量值和设定值

    // 遍历所有电机实例,进行串级PID的计算并设置发送报文的值
    for (size_t i = 0; i < idx; ++i)
    { // 减小访存开销,先保存指针引用
        motor = motorinstance[i];
        motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;
        pid_ref = motor_controller->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1;
        // pid_ref会顺次通过被启用的闭环充当数据的载体

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->outer_loop_type & SPEED_LOOP) || (motor_setting->close_loop_type & SPEED_LOOP))
        {   
            if(motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor->motor_controller.speed_foward_ptr;

            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else // MOTOR_FEED
                pid_measure = measure->speed_aps;
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        // 获取最终输出
        set = (int16_t)pid_ref;
        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            set *= -1;
        // 分组填入发送数据
        num = motor->num;
        Send[num] = set;

        // 电机是否停止运行
        if (motor->stop_flag == MOTOR_STOP)
        { // 若该电机处于停止状态,直接将buff置零
            memset(&Send[motor->num], 0, 16u);
        }
    }

    // 遍历flag,检查是否要发送这一帧报文
    for (size_t i = 0; i < 6; ++i)
    {
        MotorSetpule(motorinstance[i],Send[motorinstance[i]->num]);
    }
}
