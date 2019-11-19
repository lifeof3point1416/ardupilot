/*
PeterSt

Temporary workaround for AC_GroundProfileAcquisition

*/

#include "GroundProfileAcquisition_Workaround.h"

#error "This is not up to date, update it from Rangefinder.cpp"


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

    return true;
}
