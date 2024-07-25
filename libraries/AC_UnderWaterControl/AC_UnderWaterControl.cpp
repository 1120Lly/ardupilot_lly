#include "AC_UnderWaterControl.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#include "stdio.h"



extern const AP_HAL::HAL& hal;

AC_UnderWaterControl::AC_UnderWaterControl(AP_Motors* motors , AP_AHRS_View* ahrs)
{
    speed_low_pass_filter.set_cutoff_frequency(50.0f);
    speed_low_pass_filter.reset(0);
    _movement_throttle = 0;
    _movement_roll = 0;
    _movement_yaw = 0;
    _movement_pitch = 0;
    _movement_propeller_angle = 1500;
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



void AC_UnderWaterControl::update(void)
{
    if (_motors == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "_motors = nullptr");
        return;
    }
    
    if (_ahrs == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "_ahrs = nullptr");
        return;
    }

    int pwm_read = hal.rcin->read(CH_6);
/////////////////////////////////////////////////////////////////////////////////////////
//设置模式
//////////////////////////////////////////////////////////////////////////////////////////
    if (pwm_read > telecontorl_low_position_min && pwm_read < telecontorl_low_position_max) 
    {
        mode_fly = true;
        mode_transwater = false;
        mode_underwater = false;
        UnderWaterMode = UnderWaterMode::fly;
        // gcs().send_text(MAV_SEVERITY_INFO,"飞行模式");
    } 
    else if (pwm_read > telecontorl_mid_position_min && pwm_read < telecontorl_mid_position_max)
    {
        mode_fly = false;
        mode_transwater = true;
        mode_underwater = false;
        UnderWaterMode = UnderWaterMode::transwater;
        // gcs().send_text(MAV_SEVERITY_INFO,"穿越模式");
    }
    else if (pwm_read > telecontorl_high_position_min && pwm_read < telecontorl_high_position_max)
    {
        mode_fly = false;
        mode_transwater = false;
        mode_underwater = true;
        UnderWaterMode = UnderWaterMode::underwater;
        // gcs().send_text(MAV_SEVERITY_INFO,"水下模式");
    }
/////////////////////////////////////////////////////////////////////////////////////////////////
    //获得当前模式
    get_mode();

    // 遥控输入
    pilot_control();

    //发送遥控输出
    set_servo_out();


}



void AC_UnderWaterControl::pilot_control()
{
    int16_t pwm_throttle = hal.rcin->read(CH_3);
    int16_t pwm_roll = hal.rcin->read(CH_1) - 1500;
    int16_t pwm_yaw = hal.rcin->read(CH_4) - 1500;
    int16_t pwm_pitch = hal.rcin->read(CH_2) - 1500;
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

/***********************************************************************************************************
    函数：set_servo_out（）
    输入量：无
    作用：将遥控器的输入量线性映射到[0,1]，然后发送给AP_MotorsCoax.h
    movement_throttle:遥控器油门大于1200时，将油门[1200,1900]映射到[0,0.5]
    movemeng_roll: 将遥控器输入的[1100,1900]映射到[-0.5,0.5]
    movement_yaw: 将遥控器输入的[1100,1900]映射到[-0.5,0.5],遥控器输入值增大，右转，左电机增大输出，右电机减少输出
    movement_pitch: 将遥控器输入的[1100,1900]映射到[-0.5,0.5]
    movement_push_pull: 将遥控器输入的[1100,1900]映射到[-0.5,0.5],推杆舵机当油门大于1600时，向上推动，当油门小于1400时，向下推动
*************************************************************************************************************/

void AC_UnderWaterControl::set_servo_out()
{
    float movement_throttle;
    float movement_roll = float(_movement_roll)/1000.0f;
    float movement_yaw;
    float movement_pitch = float(_movement_pitch)/1000.0f;
    float movement_propeller_angle = float(_movement_propeller_angle - 1500)/1000.0f;

    if(_movement_throttle >= 1200)
    {
        movement_throttle = float((_movement_throttle - 1200))/1400.0f;
    }
    else{
        movement_throttle = 0.0f;
    }

    movement_yaw = float(_movement_yaw)/1000.0f;

    _motors->set_servo_out(movement_throttle, movement_roll, movement_yaw, movement_pitch, movement_propeller_angle);
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

