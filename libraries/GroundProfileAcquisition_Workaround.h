/*
PeterSt

Temporary workaround for AC_GroundProfileAcquisition

*/

#pragma once

#error "This is not up to date, update it from Rangefinder.h"

// PeterSt

#include <AP_HAL/AP_HAL.h>
//#include "AC_PosControl.h"
#include <AP_Math/AP_Math.h>
//#include <DataFlash/DataFlash.h>

#include "DebugDefines.h"
#include "AnticipatingAltCtrlDefines.h"

#if 1   // CONTINUE HERE

class AC_GroundProfileAcquisition {

public:

    AC_GroundProfileAcquisition();
    bool init();

protected:

private:

    int16_t ground_profile[GROUND_PROFILE_ACQUISITION_PROFILE_ARRAY_SIZE];

};

#endif // 0