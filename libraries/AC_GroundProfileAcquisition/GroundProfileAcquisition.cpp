/*
PeterSt,    starting 2019-11-18

System for calculating and storing the ground profile that lies in front of a UAV,
using a forward facing rangefinder.

*/


#include "GroundProfileAcquisition.h"

extern const AP_HAL::HAL& hal;


AC_GroundProfileAcquisition::AC_GroundProfileAcquisition(void) {
    ;
}

bool AC_GroundProfileAcquisition::init(void) {
    // TODO: ...
    // init variables
    int i;
    for (i = 0; i < GROUND_PROFILE_ACQUISITION_PROFILE_ARRAY_SIZE; i++) {
        ground_profile[i] = GROUND_PROFILE_ACQUISITION_NO_DATA_VALUE;
    }

    #if IS_PRINT_GPA_TESTS
    hal.console->printf("GroundProfileAcquisition after init: ground_profile[0]: %" PRIi16 "\n",
        ground_profile[0]);
    #endif // IS_PRINT_GPA_TESTS

    // check rangefinders???

    // TODO: check return value
    return true;
}
