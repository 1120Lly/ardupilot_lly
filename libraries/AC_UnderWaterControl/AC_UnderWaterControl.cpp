#include "AC_UnderWaterControl.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#include "stdio.h"



extern const AP_HAL::HAL& hal;

//设置MP中可调参数
const AP_Param::GroupInfo AC_UnderWaterControl::var_info[]={
    AP_SUBGROUPINFO(_pid_roll, "ROL_", 1, AC_UnderWaterControl, AC_PID),

    AP_SUBGROUPINFO(_pid_yaw, "PIT_", 2, AC_UnderWaterControl, AC_PID),

    AP_SUBGROUPINFO(_pid_pitch, "YAW_", 3, AC_UnderWaterControl, AC_PID),

    AP_GROUPINFO("SVO_K", 4, AC_UnderWaterControl, transwater_servo_out_K, 1),

    AP_GROUPEND
};

AC_UnderWaterControl::AC_UnderWaterControl(AP_Motors* motors , AP_AHRS_View* ahrs)
    :_pid_roll(AC_underwater_ROLL_P, 0, AC_underwater_ROLL_D, 0, 0, 0, 0, 0),
    _pid_pitch(AC_underwater_Pit_P, 0, AC_underwater_Pit_D, 0, 0, 0, 0, 0),
    _pid_yaw(AC_underwater_Yaw_P, 0, AC_underwater_Yaw_D, 0, 0, 0, 0, 0)
{
    _dt = 200;
    AP_Param::setup_object_defaults(this, var_info);
    speed_low_pass_filter.set_cutoff_frequency(50.0f);
    speed_low_pass_filter.reset(0);
    angle_low_pass_filter.set_cutoff_frequency(0.1f);
    angle_low_pass_filter.reset(0);
    _movement_throttle = 0;
    _movement_roll = 0;
    _movement_yaw = 0;
    _movement_pitch = 0;
    _movement_propeller_angle = 1000;
    pwm_propeller_angle_now = 1000;
    U_JM_k = 0;
    mode_fly = false;
    mode_transwater = false;
    mode_underwater = false;
    UnderWaterMode = UnderWaterMode::fly;
    _motors = motors;
    _ahrs = ahrs;
}


void AC_UnderWaterControl::init()
{
    mode_fly = false;
    mode_transwater = false;
    mode_underwater = false;
    UnderWaterMode = UnderWaterMode::fly;
}

/**************************************************************************
函数功能：控制倾转螺旋桨舵机转速,控制量由遥控器输入，范围为1000-2000 ，函数作用
         是假设遥控器输入从1000变为了2000，那么返回值随着时间变化从1000到2000
入口参数：
返回  值：舵机转动角度_pwm_propeller_angle
**************************************************************************/
void AC_UnderWaterControl::propeller_servo_motor_plus()
{
    pwm_propeller_angle_now += U_JM_k;
    // gcs().send_text(MAV_SEVERITY_INFO, "当前角度=%d", pwm_propeller_angle_now);
}

void AC_UnderWaterControl::propeller_servo_motor_cut()
{
    pwm_propeller_angle_now -= U_JM_k;
    // gcs().send_text(MAV_SEVERITY_INFO, "当前角度=%d", pwm_propeller_angle_now);
}

void AC_UnderWaterControl::virtual_propeller_servo_motor_plus()
{
    virtual_propeller_angle_out += U_JM_k;
    // gcs().send_text(MAV_SEVERITY_INFO, "当前角度=%d", pwm_propeller_angle_now);
}

void AC_UnderWaterControl::virtual_propeller_servo_motor_cut()
{
    virtual_propeller_angle_out -= U_JM_k;
    // gcs().send_text(MAV_SEVERITY_INFO, "当前角度=%d", pwm_propeller_angle_now);
}

/************************************************************************************************
 * 函数：水下滚转控制
 * 入口参数：滚转目标角速度、陀螺仪z轴角速度
 * 返回值：转向控制pwm(-500~500)
 * 坐标系为自定义坐标系
 * **********************************************************************************************/
float AC_UnderWaterControl::Roll_control(float roll, float gyro_z)
{
    float roll_out = (float)roll * _pid_roll.kP() - gyro_z * _pid_roll.kD();

    return roll_out;
}

/************************************************************************************************
 * 函数：水下俯仰控制
 * 入口参数：俯仰目标角速度、陀螺仪y轴角速度
 * 返回值：转向控制pwm(-500~500)
 * **********************************************************************************************/
float AC_UnderWaterControl::Pitch_control(float Pit, float gyro_y)
{
    float gyro_y_filter = angle_low_pass_filter.apply(gyro_y, _dt);
    float Pit_out = (float)Pit * _pid_pitch.kP() - gyro_y_filter * _pid_pitch.kD();

    return Pit_out;
}

/************************************************************************************************
 * 函数：水下偏航控制
 * 入口参数：偏航目标角速度、陀螺仪x轴角速度
 * 返回值：转向控制pwm(-500~500)
 * **********************************************************************************************/
float AC_UnderWaterControl::Yaw_control(float Yaw, float gyro_x)
{
    float gyro_x_filter = speed_low_pass_filter.apply(gyro_x, _dt);
    float Yaw_out = (float)Yaw * _pid_yaw.kP() - gyro_x_filter * _pid_yaw.kD();

    return Yaw_out;
}

/************************************************************************************************
 * 函数：得到机体系速度速度
 * 入口参数：无
 * 返回值：无
 * **********************************************************************************************/
void AC_UnderWaterControl::get_Vbody()
{
    Vector3f Vground, Vbody;
    Matrix3f rot_ned_to_body = _ahrs->get_rotation_body_to_ned(); //将北东地坐标系转换为机体系坐标系
    if(_ahrs -> get_velocity_NED(Vground))
    {
        Vbody = rot_ned_to_body.transposed() * Vground; //将北东地速度转换为机体系速度
        _gyro_Vx = Vbody.x; //机体系x轴速度
        _gyro_Vy = Vbody.y; //机体系y轴速度
        _gyro_Vz = Vbody.z; //机体系z轴速度
    }
    else
    {
        gcs().send_text(MAV_SEVERITY_WARNING, "get_NorthEast_Velocity failed");
        _gyro_Vx = 0;
        _gyro_Vy = 0;
        _gyro_Vz = 0;
    }
}

/************************************************************************************************
 * 函数：得到螺旋桨折叠角度
 * 入口参数：无
 * 返回值：无
 * **********************************************************************************************/
void AC_UnderWaterControl::get_virtual_propeller_angle()
{
    virtual_propeller_angle = _gyro_Vx * 2;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void AC_UnderWaterControl::update(float U_T_ratio, float U_JM_K)
{
    if (_motors == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "_motors = nullptr");
        return;
    }
    
    if (_ahrs == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "_ahrs = nullptr");
        return;
    }

    U_T_Ratio = U_T_ratio;

    U_JM_k = U_JM_K;  //倾转螺旋桨舵机随时间变化的斜率

    pwm_read = hal.rcin->read(CH_6);
    pwm_throttle = hal.rcin->read(CH_3);
/////////////////////////////////////////////////////////////////////////////////////////
//设置模式
//////////////////////////////////////////////////////////////////////////////////////////
    if (pwm_read > telecontorl_low_position_min && pwm_read < telecontorl_low_position_max) 
    {
        if(pwm_throttle < 1850)
        {
            mode_fly = true;
            mode_transwater = false;
            mode_underwater = false;
            UnderWaterMode = UnderWaterMode::fly;
            gcs().send_text(MAV_SEVERITY_INFO,"飞行模式");
        }
        else 
        {
            mode_fly = false;
            mode_transwater = true;
            mode_underwater = false;
            UnderWaterMode = UnderWaterMode::transwater;
            gcs().send_text(MAV_SEVERITY_INFO,"穿越模式");            
        }
        // gcs().send_text(MAV_SEVERITY_INFO,"飞行模式");
    } 
    // else if (pwm_read > telecontorl_mid_position_min && pwm_read < telecontorl_mid_position_max)
    // {
    //     mode_fly = false;
    //     mode_transwater = true;
    //     mode_underwater = false;
    //     UnderWaterMode = UnderWaterMode::transwater;
    //     // gcs().send_text(MAV_SEVERITY_INFO,"穿越模式");
    // }
    else if (pwm_read > telecontorl_high_position_min && pwm_read < telecontorl_high_position_max)
    {
        mode_fly = false;
        mode_transwater = false;
        mode_underwater = true;
        UnderWaterMode = UnderWaterMode::underwater;
        gcs().send_text(MAV_SEVERITY_INFO,"水下模式");
    }
/////////////////////////////////////////////////////////////////////////////////////////////////
    //获得陀螺仪参数
    _gyro_z = _ahrs->get_gyro_latest()[2];
    _gyro_x = _ahrs->get_gyro_latest()[0];
    _gyro_y = _ahrs->get_gyro_latest()[1]; //坐标系为ardupilot自带坐标系

    //获得当前模式
    get_mode();

    // 遥控输入
    pilot_control();

    _movement_roll_out = Roll_control(_movement_roll, _gyro_x);
    _movement_yaw_out = Pitch_control(_movement_yaw, _gyro_y);
    _movement_pitch_out = Yaw_control(_movement_pitch, _gyro_z);

    //返回倾转螺旋桨舵机转角值pwm_propeller_angle_now
    if(pwm_propeller_angle_now < _movement_propeller_angle)
    {
        propeller_servo_motor_plus();
    }
    else if(pwm_propeller_angle_now > _movement_propeller_angle)
    {
        propeller_servo_motor_cut();
    }
    if(virtual_propeller_angle_out < virtual_propeller_angle)
    {
        virtual_propeller_servo_motor_plus();
    }
    else if(virtual_propeller_angle_out > virtual_propeller_angle)
    {
        virtual_propeller_servo_motor_cut();
    }

    //发送跨介质舵机放大因子
    get_K();
    
    //发送遥控输出
    set_servo_out();

    get_to_zero();

}



void AC_UnderWaterControl::pilot_control()
{
    int16_t pwm_roll = hal.rcin->read(CH_1) - 1500;
    int16_t pwm_yaw = hal.rcin->read(CH_2) - 1500;
    int16_t pwm_pitch = hal.rcin->read(CH_4) - 1500;
    int16_t pwm_propeller_angle = hal.rcin->read(CH_5); // 推杆舵机当油门大于1600时，向上推动，当油门小于1400时，向下推动

    if (pwm_throttle < 1150 && pwm_throttle > 1050) {
        _movement_throttle = 0;
    } else if (pwm_throttle < 1050 || pwm_throttle > 1950) {
        _movement_throttle = 0;
    } else {
        _movement_throttle = pwm_throttle;
    }

    if (pwm_roll < 50 && pwm_roll > -50) { 
        _movement_roll = 0;
    } else if (abs(pwm_roll) > 500) {
        _movement_roll = 0;
    } else {
        _movement_roll = pwm_roll;
    }

    if (pwm_yaw < 50 && pwm_yaw > -50) {
        _movement_yaw = 0;
    } else if (abs(pwm_yaw) > 500) {
        _movement_yaw = 0;
    } else {
        _movement_yaw = pwm_yaw;
    }

    if (pwm_pitch < 20 && pwm_pitch > -20) {
        _movement_pitch = 0;
    } else if (abs(pwm_pitch) > 500) {
        _movement_pitch = 0;
    } else {
        _movement_pitch = pwm_pitch;
    }

    if (pwm_propeller_angle < 1600 && pwm_propeller_angle > 1400) {
        _movement_propeller_angle = 1500;
    } else if (pwm_propeller_angle < 1000 || pwm_propeller_angle >2000) {
        _movement_pitch = 1500;
    } else {
        _movement_propeller_angle = pwm_propeller_angle;
    }
}

void AC_UnderWaterControl::get_mode()
{
    _motors->get_now_mode(UnderWaterMode);
}

void AC_UnderWaterControl::get_K()
{
    _motors->get_K(transwater_servo_out_K);
}

/***********************************************************************************************************
    函数：set_servo_out（）
    输入量：无
    作用：将遥控器的输入量线性映射到[0,1]，然后发送给AP_MotorsCoax.h
    movement_throttle:遥控器油门大于1200时，将油门[1200,1900]映射到[0,0.1]×U_T_Ratio，默认80
    movemeng_roll: 将遥控器输入的[1100,1900]映射到[-0.5,0.5]
    movement_yaw: 将遥控器输入的[1100,1900]映射到[-0.5,0.5],遥控器输入值增大，右转
    movement_pitch: 将遥控器输入的[1100,1900]映射到[-0.5,0.5]
    movement_push_pull: 将遥控器输入的[1100,1900]映射到[-0.5,0.5],推杆舵机当油门大于1600时，向上推动，当油门小于1400时，向下推动
*************************************************************************************************************/

void AC_UnderWaterControl::set_servo_out()
{
    float movement_throttle;
    float movement_roll = float(_movement_roll_out)/1000.0f;
    float movement_yaw;
    float movement_pitch = float(_movement_pitch_out)/1000.0f;
    float movement_propeller_angle = float(pwm_propeller_angle_now - 1500)/1000.0f;

    if(_movement_throttle >= 1200)
    {
        movement_throttle = float((((_movement_throttle - 1200))/70000.0f) * U_T_Ratio);
    }
    else{
        movement_throttle = 0.0f;
    }

    movement_yaw = float(_movement_yaw_out)/1000.0f;

    _motors->set_servo_out(movement_throttle, movement_roll, movement_yaw, movement_pitch, movement_propeller_angle);
}

void AC_UnderWaterControl::get_to_zero()
{
    if(UnderWaterMode != UnderWaterMode::underwater)
    {
    _movement_roll = 0;
    _movement_yaw = 0;
    _movement_pitch = 0;
    _movement_roll_out = 0;
    _movement_yaw_out = 0;
    _movement_pitch_out = 0;
    virtual_propeller_angle = 0;
    virtual_propeller_angle_out = 0;
    }

}

// void AC_BalanceControl::debug_info()
// {

//     // 调试用
//     static uint16_t cnt = 0;
//     cnt++;
//     if (cnt > 400) {
//         cnt = 0;
//         gcs().send_text(MAV_SEVERITY_NOTICE, "--------------------");
//         gcs().send_text(MAV_SEVERITY_NOTICE, "left_real_speed=%d", balanceCAN->getSpeed(0));
//         gcs().send_text(MAV_SEVERITY_NOTICE, "right_real_speed=%d", balanceCAN->getSpeed(1));
//         gcs().send_text(MAV_SEVERITY_NOTICE, "left_target_current=%d", balanceCAN->getCurrent(0));
//         gcs().send_text(MAV_SEVERITY_NOTICE, "right_target_current=%d", balanceCAN->getCurrent(1));
//         gcs().send_text(MAV_SEVERITY_NOTICE, "altok=%d, alt_cm=%f", alt_ok, alt_cm);
//         gcs().send_text(MAV_SEVERITY_NOTICE, "--------------------");
//     }
// }

