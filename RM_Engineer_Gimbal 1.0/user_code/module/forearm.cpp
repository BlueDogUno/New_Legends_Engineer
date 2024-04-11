
#include "forearm.h"

#include "Communicate.h"
#include "cmsis_os.h"
#include "arm_math.h"

#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{
#include "user_lib.h"
}
#endif

int32_t ROTATE_ANGLE[AUTO_MODE_NUM][CAN_FOREARM_SUCTION_MOTOR] = {
        {FOREARM_INIT_SPIN_ANGLE, FOREARM_INIT_YAW_ANGLE, FOREARM_INIT_SUCTION_ANGLE},
        {FOREARM_STANDARD_SPIN_ANGLE, FOREARM_STANDARD_YAW_ANGLE, FOREARM_STANDARD_SUCTION_ANGLE},
        {FOREARM_GROUND_SPIN_ANGLE, FOREARM_GROUND_YAW_ANGLE, FOREARM_GROUND_SUCTION_ANGLE},
        {FOREARM_DELIVERY_SPIN_ANGLE, FOREARM_DELIVERY_YAW_ANGLE, FOREARM_DELIVERY_SUCTION_ANGLE}
    };

Forearm forearm;

/**
 * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
 * @param          输入的遥控器值
 * @param          输出的死区处理后遥控器值
 * @param          死区值
 */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

void Forearm::init()
{
    forearm_RC = remote_control.get_remote_control_point();
    last_forearm_RC = remote_control.get_last_remote_control_point();

    for (uint8_t i = 0; i < 4; ++i)
    {
        //动力电机数据
        forearm_motive_motor[i].init(can_receive.get_forearm_motive_motor_measure_point(i));
        //初始化pid
        
        //设置初始值
        
        forearm_motive_motor[i].max_speed = NORMAL_MAX_FOREARM_SPEED;
        forearm_motive_motor[i].min_speed = -NORMAL_MAX_FOREARM_SPEED;

        motor_status[i] = WAIT;
    }

    //翻爪左
    fp32 flip_l_speed_pid_parm[5] = {FLIP_L_MOTOR_SPEED_PID_KP, FLIP_L_MOTOR_SPEED_PID_KI, FLIP_L_MOTOR_SPEED_PID_KD, FLIP_L_MOTOR_SPEED_PID_MAX_IOUT, FLIP_L_MOTOR_SPEED_PID_MAX_OUT};
    forearm_motive_motor[0].speed_pid.init(PID_SPEED, flip_l_speed_pid_parm, &forearm_motive_motor[0].speed, &forearm_motive_motor[0].speed_set, NULL);
    forearm_motive_motor[0].speed_pid.pid_clear();

    fp32 flip_l_angle_pid_parm[5] = {FLIP_L_MOTOR_ANGLE_PID_KP, FLIP_L_MOTOR_ANGLE_PID_KI, FLIP_L_MOTOR_ANGLE_PID_KD, FLIP_L_MOTOR_ANGLE_PID_MAX_IOUT, FLIP_L_MOTOR_ANGLE_PID_MAX_OUT};
    forearm_motive_motor[0].angle_pid.init(PID_ANGLE, flip_l_angle_pid_parm, &forearm_motive_motor[0].total_angle , &forearm_motive_motor[0].angle_set, 0);
    forearm_motive_motor[0].angle_pid.pid_clear();
    //翻爪右
    fp32 flip_r_speed_pid_parm[5] = {FLIP_R_MOTOR_SPEED_PID_KP, FLIP_R_MOTOR_SPEED_PID_KI, FLIP_R_MOTOR_SPEED_PID_KD, FLIP_R_MOTOR_SPEED_PID_MAX_IOUT, FLIP_R_MOTOR_SPEED_PID_MAX_OUT};
    forearm_motive_motor[1].speed_pid.init(PID_SPEED, flip_r_speed_pid_parm, &forearm_motive_motor[1].speed, &forearm_motive_motor[1].speed_set, NULL);
    forearm_motive_motor[1].speed_pid.pid_clear();

    fp32 flip_r_angle_pid_parm[5] = {FLIP_R_MOTOR_ANGLE_PID_KP, FLIP_R_MOTOR_ANGLE_PID_KI, FLIP_R_MOTOR_ANGLE_PID_KD, FLIP_R_MOTOR_ANGLE_PID_MAX_IOUT, FLIP_R_MOTOR_ANGLE_PID_MAX_OUT};
    forearm_motive_motor[1].angle_pid.init(PID_ANGLE, flip_r_angle_pid_parm, &forearm_motive_motor[1].total_angle , &forearm_motive_motor[1].angle_set, 0);
    forearm_motive_motor[1].angle_pid.pid_clear();

    //yaw轴
    fp32 yaw_speed_pid_parm[5] = {YAW_MOTOR_SPEED_PID_KP, YAW_MOTOR_SPEED_PID_KI, YAW_MOTOR_SPEED_PID_KD, YAW_MOTOR_SPEED_PID_MAX_IOUT, YAW_MOTOR_SPEED_PID_MAX_OUT};
    forearm_motive_motor[2].speed_pid.init(PID_SPEED, yaw_speed_pid_parm, &forearm_motive_motor[2].speed, &forearm_motive_motor[2].speed_set, NULL);
    forearm_motive_motor[2].speed_pid.pid_clear();

    fp32 yaw_angle_pid_parm[5] = {YAW_MOTOR_ANGLE_PID_KP, YAW_MOTOR_ANGLE_PID_KI, YAW_MOTOR_ANGLE_PID_KD, YAW_MOTOR_ANGLE_PID_MAX_IOUT, YAW_MOTOR_ANGLE_PID_MAX_OUT};
    forearm_motive_motor[2].angle_pid.init(PID_ANGLE, yaw_angle_pid_parm, &forearm_motive_motor[2].total_angle , &forearm_motive_motor[2].angle_set, 0);
    forearm_motive_motor[2].angle_pid.pid_clear();

    //roll轴
    fp32 roll_speed_pid_parm[5] = {PITCH_MOTOR_SPEED_PID_KP, PITCH_MOTOR_SPEED_PID_KI, PITCH_MOTOR_SPEED_PID_KD, PITCH_MOTOR_SPEED_PID_MAX_IOUT, PITCH_MOTOR_SPEED_PID_MAX_OUT};
    forearm_motive_motor[3].speed_pid.init(PID_SPEED, roll_speed_pid_parm, &forearm_motive_motor[3].speed, &forearm_motive_motor[3].speed_set, NULL);
    forearm_motive_motor[3].speed_pid.pid_clear();

    fp32 roll_angle_pid_parm[5] = {PITCH_MOTOR_ANGLE_PID_KP, PITCH_MOTOR_ANGLE_PID_KI, PITCH_MOTOR_ANGLE_PID_KD, PITCH_MOTOR_ANGLE_PID_MAX_IOUT, PITCH_MOTOR_ANGLE_PID_MAX_OUT};
    forearm_motive_motor[3].angle_pid.init(PID_ANGLE, roll_angle_pid_parm, &forearm_motive_motor[3].total_angle , &forearm_motive_motor[3].angle_set, 0);
    forearm_motive_motor[3].angle_pid.pid_clear();

    // 电机软件限位，需要测试后开启
    
    vTaskDelay(1000);
    //更新一下数据
    feedback_update();
    motor_start_angle[CAN_FOREARM_YAW_MOTOR] = forearm_motive_motor[CAN_FOREARM_YAW_MOTOR].total_angle;
    forearm_motive_motor[CAN_FOREARM_YAW_MOTOR].max_angle =  + YAW_LIMIT_ANGLE;
    forearm_motive_motor[CAN_FOREARM_YAW_MOTOR].min_angle =  - YAW_LIMIT_ANGLE;
    
    motor_start_angle[CAN_FOREARM_SUCTION_MOTOR] = forearm_motive_motor[CAN_FOREARM_SUCTION_MOTOR].total_angle;
    forearm_motive_motor[CAN_FOREARM_SUCTION_MOTOR].max_angle =  + SUCTION_LIMIT_ANGLE;
    forearm_motive_motor[CAN_FOREARM_SUCTION_MOTOR].min_angle =  - SUCTION_LIMIT_ANGLE;

    motor_start_angle[CAN_SPIN_L_MOTOR] = forearm_motive_motor[CAN_SPIN_L_MOTOR].total_angle;
    forearm_motive_motor[CAN_SPIN_L_MOTOR].max_angle = + SPIN_LIMIT_ANGLE;
    forearm_motive_motor[CAN_SPIN_L_MOTOR].min_angle =  - SPIN_LIMIT_ANGLE;

    motor_start_angle[CAN_SPIN_R_MOTOR] = forearm_motive_motor[CAN_SPIN_R_MOTOR].total_angle;
    forearm_motive_motor[CAN_SPIN_R_MOTOR].max_angle = + SPIN_LIMIT_ANGLE;
    forearm_motive_motor[CAN_SPIN_R_MOTOR].min_angle = - SPIN_LIMIT_ANGLE;
}

/**
 * @brief          状态更新函数
 * @param[in]
 * @retval         none
 */
void Forearm::feedback_update(){
    //记录上一次遥控器值
    forearm_last_key_v = forearm_RC->key.v;

    //更新电机数据
    for (uint8_t i = 0; i < 4; ++i)
    {
        //更新动力电机速度
        forearm_motive_motor[i].speed = FOREARM_MOTOR_RPM_TO_VECTOR_SEN * forearm_motive_motor[i].motor_measure->speed_rpm;
        forearm_motive_motor[i].total_angle = forearm_motive_motor[i].motor_measure->total_angle - motor_start_angle[i];
        forearm_motive_motor[i].angle_error = forearm_motive_motor[i].total_angle - forearm_motive_motor[i].angle_set;
        if (forearm_motive_motor[i].angle_error < ANGLE_ERR_TOLERANT &&  forearm_motive_motor[i].angle_error > -ANGLE_ERR_TOLERANT)
            motor_status[i] = READY;
        else
            motor_status[i] = WAIT;
    }

}

/**
 * @brief          行为切换设置
 * @param[in]
 * @retval         none
 */
void Forearm::set_mode(){
    behaviour_mode_set();
}

/**
 * @brief          通过逻辑判断，赋值"forearm_behaviour_mode"成哪种模式
 * @param[in]
 * @retval         none
 */
void Forearm::behaviour_mode_set()
{
    last_forearm_behaviour_mode = forearm_behaviour_mode;
    last_forearm_mode = forearm_mode;

    //遥控器设置模式
    if (switch_is_up(forearm_RC->rc.s[FOREARM_MODE_CHANNEL])) //右拨杆上 
    {
        forearm_behaviour_mode = FOREARM_OPEN;
    }
    else if (switch_is_mid(forearm_RC->rc.s[FOREARM_MODE_CHANNEL])) //右拨杆中  开启forearm手动
    {
        forearm_behaviour_mode = FOREARM_HOLD;
    }
    else if (switch_is_down(forearm_RC->rc.s[FOREARM_MODE_CHANNEL])) //右拨杆下
    {
        forearm_behaviour_mode = FOREARM_HOLD;
    }
    if (switch_is_down(forearm_RC->rc.s[0])) //左拨杆下
    {
        forearm_behaviour_mode = FOREARM_ZERO_FORCE;
    }


    //根据行为模式选择一个控制模式
    if (forearm_behaviour_mode == FOREARM_ZERO_FORCE || forearm_behaviour_mode == FOREARM_OPEN)
    {
        forearm_mode = FOREARM_HAND;
    }
    else if(forearm_behaviour_mode == FOREARM_CLOSE)
    {
        forearm_mode = FOREARM_AUTO;
    }
    else if(forearm_behaviour_mode == FOREARM_HOLD)
    {

        forearm_mode = FOREARM_HOLDM;
    }

}



/**
 * @brief          设置控制设置值, 运动控制值是通过behaviour_control_set函数设置的
 * @param[out]
 * @retval         none
 */
void Forearm::set_control()
{
    //TODO:暂时只用到两个通道值，分别控制拨矿电机和伸爪电机
    //vspin_set控制电机速度，vyaw_set控制电机速度, 
    fp32 vspin_set = 0.0f, vyaw_set = 0.0f, vsuction_set = 0.0f;

    //获取控制设置值
    behaviour_control_set(&vspin_set, &vyaw_set, &vsuction_set);

    if (forearm_mode == FOREARM_HAND)
    {
        //同轴有一个是相反的
        forearm_motive_motor[CAN_SPIN_L_MOTOR].angle_set += vspin_set;
        forearm_motive_motor[CAN_SPIN_R_MOTOR].angle_set += -vspin_set;
        forearm_motive_motor[CAN_FOREARM_YAW_MOTOR].angle_set += vyaw_set;
        forearm_motive_motor[CAN_FOREARM_SUCTION_MOTOR].angle_set += vsuction_set;

        forearm_hold_angle[CAN_SPIN_L_MOTOR] = forearm_motive_motor[CAN_SPIN_L_MOTOR].angle_set;
		forearm_hold_angle[CAN_SPIN_R_MOTOR] = forearm_motive_motor[CAN_SPIN_R_MOTOR].angle_set;
        forearm_hold_angle[CAN_FOREARM_YAW_MOTOR] = forearm_motive_motor[CAN_FOREARM_YAW_MOTOR].angle_set;
        forearm_hold_angle[CAN_FOREARM_SUCTION_MOTOR] = forearm_motive_motor[CAN_FOREARM_SUCTION_MOTOR].angle_set;
    }
    if(forearm_mode == FOREARM_HOLDM)
    {
        forearm_motive_motor[CAN_SPIN_L_MOTOR].angle_set =  forearm_hold_angle[CAN_SPIN_L_MOTOR];
        forearm_motive_motor[CAN_SPIN_R_MOTOR].angle_set =  forearm_hold_angle[CAN_SPIN_R_MOTOR];
        forearm_motive_motor[CAN_FOREARM_YAW_MOTOR].angle_set =  forearm_hold_angle[CAN_FOREARM_YAW_MOTOR];
        forearm_motive_motor[CAN_FOREARM_SUCTION_MOTOR].angle_set =  forearm_hold_angle[CAN_FOREARM_SUCTION_MOTOR];

    }
    // 做角度限幅
    for (int i = 0;i < 4;i++)
    {
        motor_angle_limit(&forearm_motive_motor[i]);
    }

}



/**
 * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
 * @param[out]     vforearm_set, 通常翻转机构纵向移动.
 * @param[out]     vstretch_set, 通常控制横向移动.
 * @param[out]     angle_set, 通常控制旋转运动.
 * @param[in]      包括底盘所有信息.
 * @retval         none
 */
void Forearm::behaviour_control_set(fp32 *vforearm_set, fp32 *vyaw_set, fp32 *vsuction_set)
{
    if (vforearm_set == NULL || vyaw_set == NULL || vsuction_set == NULL)
    {
        return;
    }
    //无力
    if (forearm_behaviour_mode == FOREARM_ZERO_FORCE)
    {
        *vforearm_set = 0.0f;
        *vyaw_set = 0.0f;
        *vsuction_set = 0.0f;
    }
    else
    {
        forearm_open_set_control(vforearm_set, vyaw_set, vsuction_set);

    }

    last_forearm_RC->key.v = forearm_RC->key.v;
}

/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      vx_set夹爪翻转的速度
 * @param[in]      vy_set抓取机构YAW轴的速度
 * @param[in]      wz_set吸盘旋转速度
 * @param[in]      数据
 * @retval         none
 */
void Forearm::forearm_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *vz_set)
{
    if (vx_set == NULL || vy_set == NULL || vz_set == NULL)
    {
        return;
    }
    static int16_t forearm_channel = 0, yaw_channel = 0, suction_channel = 0;

    rc_deadband_limit(forearm_RC->rc.ch[FOREARM_X_CHANNEL], forearm_channel, RC_DEADBAND);
    rc_deadband_limit(forearm_RC->rc.ch[FOREARM_Y_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(forearm_RC->rc.ch[FOREARM_Z_CHANNEL], suction_channel, RC_DEADBAND);

    *vx_set = forearm_RC->rc.ch[FOREARM_X_CHANNEL] / FOREARM_OPEN_RC_SCALE;
    *vy_set = -forearm_RC->rc.ch[FOREARM_Y_CHANNEL] / FOREARM_OPEN_RC_SCALE;
    *vz_set = forearm_RC->rc.ch[FOREARM_Z_CHANNEL] / FOREARM_OPEN_RC_SCALE;
}


/**
 * @brief          解算数据,并进行pid计算
 * @param[out]
 * @retval         none
 */
void Forearm::solve()
{      
    for (int i = 0; i < 4; i++)
    {
        motor_set_control(&forearm_motive_motor[i]);
    }

}

/**
 * @brief         输出电流
 * @param[in]
 * @retval         none
 */
void Forearm::output()
{
    if (forearm_behaviour_mode == FOREARM_ZERO_FORCE)
    {
        for (int i = 0; i < 4; i++)
        {
            forearm_motive_motor[i].current_give = 0.0f;
        }
    }
    can_receive.can_cmd_forearm_motive_motor(forearm_motive_motor[CAN_SPIN_L_MOTOR].current_give, forearm_motive_motor[CAN_SPIN_R_MOTOR].current_give,
                                          forearm_motive_motor[CAN_FOREARM_YAW_MOTOR].current_give, forearm_motive_motor[CAN_FOREARM_SUCTION_MOTOR].current_give);
    // can_receive.can_cmd_forearm_motive_motor(0,0,0,0);
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */

void Forearm::motor_set_control(Joint_motors *motor)
{
    if (motor == NULL)
    {
        return;
    }

    motor->speed_set = motor->angle_pid.pid_calc();
    if (motor->speed_set > motor->max_speed)
        motor->speed_set = motor->max_speed;
    if (motor->speed_set < motor->min_speed)
        motor->speed_set = motor->min_speed;
    motor->current_give = motor->speed_pid.pid_calc();
    
}

void Forearm::motor_angle_limit(Joint_motors *motor)
{
    if (motor->angle_set < motor->min_angle)
    {
        motor->angle_set = motor->min_angle;
    }
    else if (motor->angle_set > motor->max_angle)
    {
        motor->angle_set = motor->max_angle;
    }
}


/**
 * @brief          自动模式控制电机转动角度
 * @param[out]     add: 角度增加量
 * @retval         none
 */
void Forearm::auto_control(auto_mode_e *auto_mode)
{
    switch(*auto_mode)
    {
        case FOREARM_INIT:
        case FOREARM_STANDARD:
        case FOREARM_GROUND:
        case FOREARM_DELIVERY:
        {
            static int AUTO_MODE;
            AUTO_MODE = *auto_mode - FOREARM_INIT;
            forearm_motive_motor[CAN_SPIN_L_MOTOR].angle_set = + ROTATE_ANGLE[AUTO_MODE][SPIN_MOTOR];
            forearm_motive_motor[CAN_SPIN_R_MOTOR].angle_set = - ROTATE_ANGLE[AUTO_MODE][SPIN_MOTOR];
            forearm_motive_motor[CAN_FOREARM_YAW_MOTOR].angle_set = - ROTATE_ANGLE[AUTO_MODE][YAW_MOTOR];
            forearm_motive_motor[CAN_FOREARM_SUCTION_MOTOR].angle_set = + ROTATE_ANGLE[AUTO_MODE][SUCTION_MOTOR];
        }
    }
}