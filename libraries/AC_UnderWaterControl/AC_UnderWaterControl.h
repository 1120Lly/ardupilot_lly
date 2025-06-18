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

#define AC_underwater_ROLL_P            0.3f
#define AC_underwater_ROLL_I            0.0f
#define AC_underwater_ROLL_D            0.1f

#define AC_underwater_Pit_P            0.3f
#define AC_underwater_Pit_I            0.0f
#define AC_underwater_Pit_D            0.1f

#define AC_underwater_Yaw_P            0.3f
#define AC_underwater_Yaw_I            0.0f
#define AC_underwater_Yaw_D            0.1f


class AC_UnderWaterControl {
public:
    AC_UnderWaterControl(AP_Motors* motors, AP_AHRS_View* ahrs); 

    // empty destructor to suppress compiler warning
    virtual ~AC_UnderWaterControl() { }

    void init();

    void pilot_control();

    void get_mode();
    void get_K();

    void update(float U_T_ratio, float U_JM_K);

    void set_servo_out();

    void propeller_servo_motor_plus();
    void propeller_servo_motor_cut();
    float Roll_control(float roll, float gyro_z);
    float Pitch_control(float pitch, float gyro_x);
    float Yaw_control(float yaw, float gyro_y);
    void get_to_zero();

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    AP_Motors*          _motors;
    const AP_AHRS_View* _ahrs;


    enum UnderWaterMode {
        underwater             = 0,
        transwater             = 1,
        fly                    = 2,
    };

    int pwm_read;

    enum UnderWaterMode UnderWaterMode;
    

    // bool Pick_Up(float Acceleration, float Angle, int16_t encoder_left, int16_t encoder_right);
    // bool Put_Down(float Angle, int encoder_left, int encoder_right);
    // void debug_info();
    int16_t _movement_throttle;
    int16_t _movement_roll;
    int16_t _movement_yaw;
    int16_t _movement_pitch;
    int16_t _movement_propeller_angle;
    float _movement_roll_out; //PD控制器输出值
    float _movement_pitch_out; //PD控制器输出值
    float _movement_yaw_out; //PD控制器输出值
    float _gyro_x; //陀螺仪x轴角速度
    float _gyro_y; //陀螺仪y轴角速度
    float _gyro_z; //陀螺仪z轴角速度


protected:

    LowPassFilterFloat speed_low_pass_filter;//一阶低通滤波器
    LowPassFilterFloat angle_low_pass_filter;//一阶低通滤波器


    bool mode_underwater;
    bool mode_transwater;
    bool mode_fly;
    bool set_underwater_stop;   //true是停下


    float U_T_Ratio;

    float U_JM_k;

    int pwm_propeller_angle_now;

    AP_Float transwater_servo_out_K; //跨介质时舵机放大比例

    //////////////////////////////////////////
    //PID参数 
    AC_PID _pid_roll;
    AC_PID _pid_yaw;
    AC_PID _pid_pitch;

    /////////////////////////////////////////////
    //水下滚转环参数
    float turn_target;
    float turn_out;

    float _dt;
};
