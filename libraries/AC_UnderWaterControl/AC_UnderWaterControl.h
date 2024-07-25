#pragma once

/// @file    AC_AttitudeControl_Multi.h
/// @brief   ArduCopter attitude control library

#include <AC_PID/AC_PI.h>
#include <AC_PID/AC_PID.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Param/AP_Param.h>

//遥控器拨杆高，中，低位
#define telecontorl_high_position_min   1800
#define telecontorl_high_position_max   2000
#define telecontorl_mid_position_min    1400
#define telecontorl_mid_position_max    1600
#define telecontorl_low_position_min    1000
#define telecontorl_low_position_max    1200


class AC_UnderWaterControl {
public:
    AC_UnderWaterControl(AP_Motors* motors, AP_AHRS_View* ahrs); 

    // empty destructor to suppress compiler warning
    virtual ~AC_UnderWaterControl() { }

    void init();

    void pilot_control();

    void get_mode();

    void update(void);

    void set_servo_out();

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    AP_Motors*          _motors;
    const AP_AHRS_View* _ahrs;


    enum UnderWaterMode {
        underwater             = 0,
        transwater             = 1,
        fly                    = 2,
    };

    // bool Pick_Up(float Acceleration, float Angle, int16_t encoder_left, int16_t encoder_right);
    // bool Put_Down(float Angle, int encoder_left, int encoder_right);
    // void debug_info();


protected:

    LowPassFilterFloat speed_low_pass_filter;//一阶低通滤波器


    ///////////////////////////////////////////////////////
    int16_t _movement_throttle;
    int16_t _movement_roll;
    int16_t _movement_yaw;
    int16_t _movement_pitch;
    int16_t _movement_propeller_angle;

    bool mode_underwater;
    bool mode_transwater;
    bool mode_fly;
    bool set_underwater_stop;   //true是停下

    enum UnderWaterMode UnderWaterMode;
};
