#pragma once

#include "AC_CustomControlPlane_config.h"

#if AP_CUSTOMCONTROLPLANE_EMPTY_ENABLED

#include "AC_CustomControlPlane_Backend.h"

class AC_CustomControlPlane_Empty : public AC_CustomControlPlane_Backend {
public:
    AC_CustomControlPlane_Empty(AC_CustomControlPlane& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt);


    Vector3f update(void) override;
    void reset(void) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // declare parameters here
    AP_Float param1;
    AP_Float param2;
    AP_Float param3;
};

#endif  // AP_CUSTOMCONTROLPLANE_EMPTY_ENABLED
