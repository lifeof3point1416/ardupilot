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
#define IS_DEBUG_MAX_HORIZONTAL_SPEED               0         // spamming console
// print some values in a mavlink message if desired in interval: PRINT_MESSAGE_VALUE_INTERVAL
#define IS_PRINT_MESSAGE_VALUE_RANGEFINDER_GAIN     0
#define IS_PRINT_MESSAGE_VALUE_RANGEFINDER_DIST     0
#define IS_PRINT_VALUE_LOITER_ALT_TARGET            0
#define IS_PRINT_MESSAGE_VALUE_RANGEFINDER_ALT_CM   0           // print rangefinder_alt_cm, variable that is manipulated
    // depending on the altitude control, eg. by Extended PID altitude control of MEASUREMENT flightmode

// more specific stuff
#define IS_PRINT_REPEATET_MESSAGE_1HZ_CONSOLE       0           // print message via GCS message in 1 Hz look
#define IS_TEST_FFC                                 1
#define IS_DEBUG_CHECK_FWD_RANGEFINDER_ANGLE        1           // assert 0° for sitl, 45° for others

// concerning Ground Profile Acquisition
#define IS_DISABLE_VERBOSE_GPA_PRINTOUTS            0
#define IS_DEBUG_GPA                                01           // debug Ground Profile Acquisition
#define IS_PRINT_GROUND_PROFILE_ACQUISITION_MAP     0           // print via console
#define IS_PRINT_GPA_MAP_AS_MESSAGE                 1           // print via MAVLink message
#define IS_PRINT_GPA_NEW_POINT                      0           // print each new point, too much io!
static_assert(!IS_PRINT_GPA_NEW_POINT, "must not use this, to prevent io from being spammed");
// print Ground Profile Acquisition map values as condensed numbers?
// hex numbers biased with a constant, so that there are only positive numbers (if the values are within range)
//  examples:
//  false: print "normally":                        [  -16   -5   +3  +47 +126]
//  true: print as biased hex (biased with 0x80)    [70 7B 83 AF FE]
#define IS_PRINT_GPA_MAP_CONDENSED                  1           
#define GPA_MAP_CONDENSED_BIAS                      (0x80)      // (values < -<this>) will still be negative
#define PRINT_GPA_MAP_INTERVAL                      30
#define IS_PRINT_GPA_MAIN_DIRECTION_COO             true        // print them in an interval
#define IS_USE_GPA_MAP_FROM_FILE                    000        // use predefined map instead (to avoid testflights each time we debug GPD)
// #define IS_USE_GPA_MAP_FROM_FILE                    false        // use predefined map instead (to avoid testflights each time we debug GPD)
// #define GPA_MAP_FROM_FILE_FILENAME                  "libraries/AP_RangeFinder/gpa_map_file_spline_mockup.txt"
// #define GPA_MAP_FROM_FILE_FILENAME                  "libraries/AP_RangeFinder/gpa_map_file_191206T1317P0100.txt"
// #define GPA_MAP_FROM_FILE_FILENAME                  "libraries/AP_RangeFinder/gpa_map_file_265_csma031.txt"
#define GPA_MAP_FROM_FILE_FILENAME                  "libraries/AP_RangeFinder/gpa_map_file_265_csma099.txt"
// #define GPA_MAP_FROM_FILE_FILENAME                  "libraries/AP_RangeFinder/gpa_map_file_265_csma201.txt"
#define GPA_MAP_LINE_BUFSIZ                         80          // MUST be longer than a line of the csv file
#define IS_PRINT_GPA_MAP_FROM_FILE_DATA             true        // print data when parsing, using printf

// concerning Ground Profile Derivator
#define IS_VERBOSE_DEBUG_GPD                        000        // very verbose debugs for Ground Profile Derivator
#define IS_RUN_GROUND_PROFILE_DERIVATOR_TESTS       false     
// #define IS_RUN_GROUND_PROFILE_DERIVATOR_TESTS       true
#define IS_DO_INTERMEDIATE_CLF_LOGGING              false        // not only before return, but also after every grade
#define IS_VERBOSE_CLF_LOGGING                      false        // log data actually used for GPD, tag "CLF2"
#define IS_TEST_INT32_INT16_LOGGING                 false       // run a test logging int32[] as int16[]
#define IS_VERBOSE_GPD_LOGGING                      000        // log GPD every time, not just once a second
#define IS_USE_FLOAT_ARITHMETIC_FOR_DERIVATION      true        // for CLF, as opposed to fixed integer arithmetic
#define IS_VERBOSE_DEBUG_SPF_PRINTOUTS              false        // a lot of printouts in xterm, very verbose

// concerning FFC itself
// #define IS_VERBOSE_THROTTLE_LOGGING_FFC             true        // deprecated, use IS_LOG_VERBOSE_PID_FFC_OUTPUT instead
// #define IS_FFC_ENABLED                              false       // false: disable already existing code, useful for tests not concerning untested FFC code
#define IS_FFC_ENABLED                              true       // false: disable already existing code, useful for tests not concerning untested FFC code
// #define IS_IGNORE_FFC_OUTPUT                        true        // true: we might have an ffc, giving output data, but we don't feed it onto altitude ctrl
#define IS_IGNORE_FFC_OUTPUT                        001        // true: we might have an ffc, giving output data, but we don't feed it onto altitude ctrl
#define IS_REVERSE_FLIGHT_SPEED_CHECK_LOG_TESTER    false        // do debug printout for horizontal velocities
#define IS_VERBOSE_DEBUG_FFC                        false         // at PosCtrl, where FFC comes into action
#define IS_VERY_VERBOSE_DEBUG_FFC_MCF               false         // find cause of Floating Point exception

// debug values
#define REPEATET_GCS_MESSAGE_INTERVAL               60          // print a custom gcs message every X seconds
#define REPEATET_MESSAGE_IN_MEASUREMENT_INTERVAL    30          // print a custom gcs message every X seconds
#define CALL_FREQUENCY_MEASUREMENT_RUN              400         // call frequency of Copter::ModeMeasurement::run()
#define LAST_CODE_CHANGE                    "2020-02-13 22:00+01:00"    // TODO: frequently update after changes
#define IS_LAST_CHANGE_DATE_DEPRECATED              00          // change to 1, if you changed code but not LAST_CODE_CHANGE
#define PRINT_MESSAGE_VALUE_INTERVAL                5
#define PRINT_GPA_MAP_UNTIL_INDEX                   2000         // print all ground_profile[0:<this value>


///// derived definitions, no need to change from here
//

// for Ground Profile Acquisition
#define IS_PRINT_GPA_TESTS                          (IS_TEST_FFC && !IS_DISABLE_VERBOSE_GPA_PRINTOUTS)
#define GET_BIASED_NUMBER(N, BIAS)                  ( (N) + (BIAS) )
#define GET_UNBIASED_NUMBER(N, BIAS)                ( (N) - (BIAS) )