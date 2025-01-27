#pragma once

/// @file    AC_CustomControlPlane.h
/// @brief   ArduCopter custom control library

#include "AC_CustomControlPlane_config.h"

#if AP_CUSTOMCONTROLPLANE_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AP_Motors/AP_MotorsMulticopter.h>

#ifndef CUSTOMCONTROLPLANE_MAX_TYPES
#define CUSTOMCONTROLPLANE_MAX_TYPES 3
#endif

class AC_CustomControlPlane_Backend;

class AC_CustomControlPlane {
public:
    AC_CustomControlPlane(AP_AHRS_View*& ahrs, AC_AttitudeControl*& _att_control, AP_MotorsMulticopter*& motors, float dt);

    CLASS_NO_COPY(AC_CustomControlPlane);  /* Do not allow copies */

    void init(void);
    void update(void);
    void motor_set(Vector3f motor_out);
    void set_custom_controller(bool enabled);
    void reset_main_att_controller(void);
    bool is_safe_to_run(void);
    void log_switch(void);

    // set the PID notch sample rates
    void set_notch_sample_rate(float sample_rate);

    // zero index controller type param, only use it to access _backend or _backend_var_info array
    uint8_t get_type() { return _controller_type > 0 ? (_controller_type - 1) : 0; };

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];
    static const struct AP_Param::GroupInfo *_backend_var_info[CUSTOMCONTROLPLANE_MAX_TYPES];

protected:
    // add custom controller here
    enum class CustomControlPlaneType : uint8_t {
        CONT_NONE            = 0,
        CONT_EMPTY           = 1,
        CONT_PID             = 2,
        CONT_XYZ             = 3,
    };            // controller that should be used     

    enum class  CustomControlPlaneOption {
        ROLL = 1 << 0,
        PITCH = 1 << 1,
        YAW = 1 << 2,
    };

    // Intersampling period in seconds
    float _dt;
    bool _custom_controller_active;

    // References to external libraries
    AP_AHRS_View*& _ahrs;
    AC_AttitudeControl*& _att_control;
    AP_MotorsMulticopter*& _motors;

    AP_Enum<CustomControlPlaneType> _controller_type;
    AP_Int8 _custom_controller_mask;

private:
    AC_CustomControlPlane_Backend *_backend;
};

#endif  // AP_CUSTOMCONTROLPLANE_ENABLED
