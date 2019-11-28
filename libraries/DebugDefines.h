#pragma once

/*
Debug defines added by PeterSt

This file can be included via ArduCopter/defines.h, which is included in many files

TODO (Prio 3): Find proper way to include this
*/

// #include <inttypes.h>                                        // for placeholders like PRIu64 


// debug switches
#define IS_DO_VERBOSE_DEBUG_PRINTOUTS               0           // these will be a lot of printouts!
#define IS_DO_TEMPORARY_DEBUG_PRINTOUTS             1           // replace with IS_DO_VERBOSE_DEBUG_PRINTOUTS, if done
#define IS_PRINT_REPEATET_GCS_MESSAGE               1           // print message via GCS message, repeatedly
#define IS_PRINT_INIT_GCS_MESSAGE                   1           // print init message via GCS message
#define IS_PRINT_INIT_PRINTF_MESSAGE                1           // print init message via printf
#define IS_PRINT_INIT_HAL_PRINTF_MESSAGE            1           // print init message via hal.console->printf
                                                                // works on sitl after Copter::init_ardupilot()
#define IS_PRINT_REPEATET_MESSAGE_IN_MEASUREMENT    1           // only sends gcs message in flightmode MEASUREMENT
#define IS_TEST_MEASUREMENT_INSTANCE_INIT           1           
#define IS_PRINT_ALT_CTRL_METHOD_IN_MEASUREMENT     1           // at start
// print some values in a mavlink message if desired in interval: PRINT_MESSAGE_VALUE_INTERVAL
#define IS_PRINT_MESSAGE_VALUE_RANGEFINDER_GAIN     00
#define IS_PRINT_MESSAGE_VALUE_RANGEFINDER_DIST     00
#define IS_PRINT_VALUE_LOITER_ALT_TARGET            00
#define IS_PRINT_MESSAGE_VALUE_RANGEFINDER_ALT_CM   00           // print rangefinder_alt_cm, variable that is manipulated
    // depending on the altitude control, eg. by Extended PID altitude control of MEASUREMENT flightmode

// more specific stuff
#define IS_PRINT_REPEATET_MESSAGE_1HZ_CONSOLE       0           // print message via GCS message in 1 Hz look
#define IS_TEST_FFC                                 1
#define IS_DEBUG_GPA                                1           // debug Ground Profile Acquisition
#define IS_PRINT_GROUND_PROFILE_ACQUISITION_MAP     1           // check rangefinder angle in SITL! // TODO: impl

// debug values
#define REPEATET_GCS_MESSAGE_INTERVAL               30          // print a custom gcs message every X seconds
#define REPEATET_MESSAGE_IN_MEASUREMENT_INTERVAL    30          // print a custom gcs message every X seconds
#define CALL_FREQUENCY_MEASUREMENT_RUN              400         // call frequency of Copter::ModeMeasurement::run()
#define LAST_CODE_CHANGE                    "2019-11-27 11:17+01:00"    // TODO: frequently update after changes
#define IS_LAST_CHANGE_DATE_DEPRECATED              0          // change to 1, if you changed code but
#define PRINT_MESSAGE_VALUE_INTERVAL                5
//                                                                  didn't update LAST_CODE_CHANGE


///// derived definitions, no need to change from here
//

#define IS_PRINT_GPA_TESTS                          (IS_TEST_FFC)   // for Ground Profile Acquisition