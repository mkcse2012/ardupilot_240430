#pragma once

#include "AC_CustomControlPlane_config.h"
#include <AC_Simulink/arduAttCont.h>
#if AP_CUSTOMCONTROLPLANE_XYZ_ENABLED

#include "AC_CustomControlPlane_Backend.h"

class AC_CustomControlPlane_XYZ : public AC_CustomControlPlane_Backend {
public:
    AC_CustomControlPlane_XYZ(AC_CustomControlPlane& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt);


    Vector3f update(void) override;
    void reset(void) override;
    
   arduAttContModelClass simulinkController;
    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // declare parameters here
    AP_Float param1;
    AP_Float param2;
    AP_Float param3;
     // angle P controller  objects
   // AC_P                _p_angle_roll2;
    //AC_P                _p_angle_pitch2;
    //AC_P                _p_angle_yaw2;

	// rate PID controller  objects
    //AC_PID _pid_atti_rate_roll;
    //AC_PID _pid_atti_rate_pitch;
    //AC_PID _pid_atti_rate_yaw;
};

#endif  // AP_CUSTOMCONTROLPLANE_XYZ_ENABLED
