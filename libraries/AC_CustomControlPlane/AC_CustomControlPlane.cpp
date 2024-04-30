#include <AP_HAL/AP_HAL.h>

#include "AC_CustomControlPlane.h"

#if AP_CUSTOMCONTROLPLANE_ENABLED

#include "AC_CustomControlPlane_Backend.h"
//#include "AC_CustomControlPlane_Empty.h"
#include "AC_CustomControlPlane_PID.h"
#include "AC_CustomControlPlane_XYZ.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControlPlane::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: Custom control plane type
    // @Description: Custom control plane type to be used
    // @Values: 0:None, 1:Empty, 2:PID
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_TYPE", 1, AC_CustomControlPlane, _controller_type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _AXIS_MASK
    // @DisplayName: Custom Controller bitmask
    // @Description: Custom Controller bitmask to chose which axis to run
    // @Bitmask: 0:Roll, 1:Pitch, 2:Yaw
    // @User: Advanced
    AP_GROUPINFO("_AXIS_MASK", 2, AC_CustomControlPlane, _custom_controller_mask, 0),

    // parameters for empty controller. only used as a template, no need for param table 
    // AP_SUBGROUPVARPTR(_backend, "1_", 6, AC_CustomControlPlane, _backend_var_info[0]),

    // parameters for PID controller
    AP_SUBGROUPVARPTR(_backend, "2_", 7, AC_CustomControlPlane, _backend_var_info[1]),

    AP_SUBGROUPVARPTR(_backend, "3_", 8, AC_CustomControlPlane, _backend_var_info[2]),

    AP_GROUPEND
};

const struct AP_Param::GroupInfo *AC_CustomControlPlane::_backend_var_info[CUSTOMCONTROLPLANE_MAX_TYPES];

AC_CustomControlPlane::AC_CustomControlPlane(AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt) :
    _dt(dt),
    _ahrs(ahrs),
    _att_control(att_control),
    _motors(motors)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_CustomControlPlane::init(void)
{
    switch (CustomControlPlaneType(_controller_type))
    {
        case CustomControlPlaneType::CONT_NONE:
            break;
        case CustomControlPlaneType::CONT_EMPTY: // This is template backend. Don't initialize it.
            // This is template backend. Don't initialize it.
            // _backend = new AC_CustomControl_Empty(*this, _ahrs, _att_control, _motors, _dt);
            // _backend_var_info[get_type()] = AC_CustomControl_Empty::var_info;
            break;
        case CustomControlPlaneType::CONT_PID:
            _backend = new AC_CustomControlPlane_PID(*this, _ahrs, _att_control, _motors, _dt);
            _backend_var_info[get_type()] = AC_CustomControlPlane_PID::var_info;
            break;
        case CustomControlPlaneType::CONT_XYZ:
            _backend = new AC_CustomControlPlane_XYZ(*this, _ahrs, _att_control, _motors, _dt);
            _backend_var_info[get_type()] = AC_CustomControlPlane_XYZ::var_info;
            break;
        default:
            return;
    }

    if (_backend && _backend_var_info[get_type()]) {
        AP_Param::load_object_from_eeprom(_backend, _backend_var_info[get_type()]);
    }
}

// run custom controller if it is activated by RC switch and appropriate type is selected
void AC_CustomControlPlane::update(void)
{
    if (is_safe_to_run()) {
        Vector3f motor_out_rpy;

        motor_out_rpy = _backend->update();

        motor_set(motor_out_rpy);
    }
}

// choose which axis to apply custom controller output
void AC_CustomControlPlane::motor_set(Vector3f rpy) {
    if (_custom_controller_mask & (uint8_t)CustomControlPlaneOption::ROLL) {
        _motors->set_roll(rpy.x);
        _att_control->get_rate_roll_pid().set_integrator(0.0);
    }
    if (_custom_controller_mask & (uint8_t)CustomControlPlaneOption::PITCH) {
        _motors->set_pitch(rpy.y);
        _att_control->get_rate_pitch_pid().set_integrator(0.0);
    }
    if (_custom_controller_mask & (uint8_t)CustomControlPlaneOption::YAW) {
        _motors->set_yaw(rpy.z);
        _att_control->get_rate_yaw_pid().set_integrator(0.0);
    }
}

// move main controller's target to current states, reset filters,
// and move integrator to motor output
// to allow smooth transition to the primary controller
void AC_CustomControlPlane::reset_main_att_controller(void)
{
    // reset attitude and rate target, if feedforward is enabled
    if (_att_control->get_bf_feedforward()) {
        _att_control->relax_attitude_controllers();
    }

    _att_control->get_rate_roll_pid().set_integrator(0.0);
    _att_control->get_rate_pitch_pid().set_integrator(0.0);
    _att_control->get_rate_yaw_pid().set_integrator(0.0);
}

void AC_CustomControlPlane::set_custom_controller(bool enabled)
{
    // double logging switch makes the state change very clear in the log
    log_switch();

    _custom_controller_active = false;

    // don't allow accidental main controller reset without active custom controller
    if (_controller_type == CustomControlPlaneType::CONT_NONE) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Custom controller is not enabled");
        return;
    }

    // controller type is out of range
    if (_controller_type > CUSTOMCONTROLPLANE_MAX_TYPES) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Custom controller type is out of range");
        return;
    }

    // backend is not created
    if (_backend == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reboot to enable selected custom controller");
        return;
    }

    if (_custom_controller_mask == 0 && enabled) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Axis mask is not set");
        return;
    }

    // reset main controller
    if (!enabled) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Custom controller is OFF");
        // don't reset if the empty backend is selected
        if (_controller_type > CustomControlPlaneType::CONT_EMPTY) {
            reset_main_att_controller();
        }
    }

    if (enabled && _controller_type > CustomControlPlaneType::CONT_NONE) {
        // reset custom controller filter, integrator etc.
        _backend->reset();
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Custom controller is ON");
    }

    _custom_controller_active = enabled;

    // log successful switch
    log_switch();
}

// check that RC switch is on, backend is not changed mid flight and controller type is selected
bool AC_CustomControlPlane::is_safe_to_run(void) {
    if (_custom_controller_active && (_controller_type > CustomControlType::CONT_NONE)
        && (_controller_type <= CUSTOMCONTROLPLANE_MAX_TYPES) && _backend != nullptr)
    {
        return true;
    }

    return false;
}

// log when the custom controller is switch into
void AC_CustomControlPlane::log_switch(void) {
    AP::logger().Write("CC", "TimeUS,Type,Act","QBB",
                            AP_HAL::micros64(),
                            _controller_type,
                            _custom_controller_active);
}

void AC_CustomControlPlane::set_notch_sample_rate(float sample_rate)
{
#if AP_FILTER_ENABLED
    if (_backend != nullptr) {
        _backend->set_notch_sample_rate(sample_rate);
    }
#endif
}

#endif  // AP_CUSTOMCONTROLPLANE_ENABLED
