#include "AC_CustomControlPlane_config.h"

#if AP_CUSTOMCONTROLPLANE_XYZ_ENABLED

#include "AC_CustomControlPlane_XYZ.h"

#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControlPlane_XYZ::var_info[] = {
    // @Param: PARAM1
    // @DisplayName: XYZ param1
    // @Description: Dummy parameter for XYZ custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM1", 1, AC_CustomControlPlane_XYZ, param1, 0.0f),

    // @Param: PARAM2
    // @DisplayName: XYZ param2
    // @Description: Dummy parameter for XYZ custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM2", 2, AC_CustomControlPlane_XYZ, param2, 0.0f),

    // @Param: PARAM3
    // @DisplayName: XYZ param3
    // @Description: Dummy parameter for XYZ custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM3", 3, AC_CustomControlPlane_XYZ, param3, 0.0f),
    AP_GROUPEND
};

// initialize in the constructor
AC_CustomControlPlane_XYZ::AC_CustomControlPlane_XYZ(AC_CustomControlPlane& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControlPlane_Backend(frontend, ahrs, att_control, motors, dt)
//AC_CustomControl_XYZ::AC_CustomControl_XYZ(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt) :
   // AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt),
   // _p_angle_roll2(AC_ATTITUDE_CONTROL_ANGLE_P * 0.90f, AC_ATTITUDE_CONTROL_ANGLE_I * 0.90f, AC_ATTITUDE_CONTROL_ANGLE_D * 0.90f, 0.0f, AC_ATTITUDE_CONTROL_ANGLE_IMAX * 0.90f, AC_ATTITUDE_CONTROL_ANGLE_RPY_FILT_HZ * 0.90f, 0.0f, AC_ATTITUDE_CONTROL_ANGLE_RPY_FILT_HZ * 0.90f),
   // _p_angle_pitch2(AC_ATTITUDE_CONTROL_ANGLE_P * 0.90f, AC_ATTITUDE_CONTROL_ANGLE_I * 0.90f, AC_ATTITUDE_CONTROL_ANGLE_D * 0.90f, 0.0f, AC_ATTITUDE_CONTROL_ANGLE_IMAX * 0.90f, AC_ATTITUDE_CONTROL_ANGLE_RPY_FILT_HZ * 0.90f, 0.0f, AC_ATTITUDE_CONTROL_ANGLE_RPY_FILT_HZ * 0.90f),
   // _p_angle_yaw2(AC_ATTITUDE_CONTROL_ANGLE_P * 0.90f, AC_ATTITUDE_CONTROL_ANGLE_I * 0.90f, AC_ATTITUDE_CONTROL_ANGLE_D * 0.90f, 0.0f, AC_ATTITUDE_CONTROL_ANGLE_IMAX * 0.90f, AC_ATTITUDE_CONTROL_ANGLE_RPY_FILT_HZ * 0.90f, 0.0f, AC_ATTITUDE_CONTROL_ANGLE_RPY_FILT_HZ * 0.90f),
   // _pid_atti_rate_roll(AC_ATC_MULTI_RATE_RP_P * 0.90f, AC_ATC_MULTI_RATE_RP_I * 0.90f, AC_ATC_MULTI_RATE_RP_D * 0.90f, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX * 0.90f, AC_ATC_MULTI_RATE_RPY_FILT_HZ * 0.90f, 0.0f, AC_ATC_MULTI_RATE_RPY_FILT_HZ * 0.90f),
    //_pid_atti_rate_pitch(AC_ATC_MULTI_RATE_RP_P * 0.90f, AC_ATC_MULTI_RATE_RP_I * 0.90f, AC_ATC_MULTI_RATE_RP_D * 0.90f, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX * 0.90f, AC_ATC_MULTI_RATE_RPY_FILT_HZ * 0.90f, 0.0f, AC_ATC_MULTI_RATE_RPY_FILT_HZ * 0.90f),
    //_pid_atti_rate_yaw(AC_ATC_MULTI_RATE_YAW_P * 0.90f, AC_ATC_MULTI_RATE_YAW_I * 0.90f, AC_ATC_MULTI_RATE_YAW_D * 0.90f, 0.0f, AC_ATC_MULTI_RATE_YAW_IMAX * 0.90f, AC_ATC_MULTI_RATE_RPY_FILT_HZ * 0.90f, AC_ATC_MULTI_RATE_YAW_FILT_HZ * 0.90f, 0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);
    
    simulinkController.initialize();
}
Vector3f AC_CustomControlPlane_XYZ::update(void)
{
    // Reset controller based on spool state
    switch (_motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
        case AP_Motors::SpoolState::GROUND_IDLE:
            // We are still on the ground. Reset custom controller to avoid
            // build-up, e.g., integrator
            reset();
            break;

        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            // We are off the ground
            break;
    }

    // Run custom controller from here
    Quaternion attitude_body, attitude_target;
    _ahrs->get_quat_body_to_ned(attitude_body);
    attitude_target = _att_control->get_attitude_target_quat();

    // Calculate attitude error
    Vector3f attitude_error;
    float _thrust_angle, _thrust_error_angle;
    _att_control->thrust_heading_rotation_angles(attitude_target, attitude_body, attitude_error, _thrust_angle, _thrust_error_angle);

    // Calculate angle velocity feedforward
    Quaternion rotation_target_to_body = attitude_body.inverse() * attitude_target;
    Vector3f ang_vel_body_feedforward = rotation_target_to_body * _att_control->get_attitude_target_ang_vel();
    Vector3f gyro_latest = _ahrs->get_gyro_latest();

 // Prepare arguments for the simulink_controller
    float arg_attitude_error[3] = {attitude_error.x, attitude_error.y, attitude_error.z};
    float arg_rate_ff[3] = {ang_vel_body_feedforward.x, ang_vel_body_feedforward.y, ang_vel_body_feedforward.z};
    float arg_rate_meas[3] = {gyro_latest.x, gyro_latest.y, gyro_latest.z};
    float arg_out1[3];

    // Call the simulink_controller's step function
     simulinkController.step(arg_attitude_error, arg_rate_ff, arg_rate_meas, arg_out1);

    // Return the output from simulink_controller
    return Vector3f(arg_out1[0], arg_out1[1], arg_out1[2]);
}

void AC_CustomControlPlane_XYZ::reset(void)
{
}

#endif  // AP_CUSTOMCONTROLPLANE_XYZ_ENABLED
