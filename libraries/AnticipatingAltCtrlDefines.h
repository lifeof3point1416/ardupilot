/*
By Peter Steidten, (PeterSt)
2019-11-09

"Das tritt nach meiner Kenntnis ... ist das sofort, unverzüglich."
- Günter Schabowski, 09.11.1989

defines for the anticipating altitude control,
invoced in AC/defines.h
*/

#pragma once

#include <AP_Math/rotations.h>            // for rangefinder rotations
#include <DebugDefines.h>
//#include "rotations.h"            // for rangefinder rotations
// #include <AnticipatingAltCtrlModes.h>

enum AltCtrlMode : uint8_t {
    STANDARD_PID                = 1,                // as is of existing ArduCopter altitude control
    EXTENDED_PID                = 2,                // using interpolated value of dwn & fwd facing rangefinder
    FFC                         = 3,                // feeding forward anticipated ground profile
};

//
// for rangefinders
//

#define RANGEFINDER_ANGLE_FORWARD_FACING_DEG        45      // 0° is downwards

// rotation can be set in MissionPlanner: MP-->Config/Tuning/Full Param List
//                     in MAVProxy: param set RNGFND_ORIENT 0    # for ROTATION_NONE
//                                  param set RNGFND2_ORIENT 25  # for ROTATION_PITCH_270
#define RANGEFINDER_ORIENTATION_FORWARD_FACING      ROTATION_NONE               // "Forward" in MP
#define RANGEFINDER_ORIENTATION_DOWNWARD_FACING     ROTATION_PITCH_270          // "Down" in MP
// TODO: prio 7:    adjust orientations, 
// actually fwd should be ROTATION_PITCH_315 == 39 (45°backwards, there is not ROTATION_PITCH_45 yet, but 
//  prototype actually has it mounted backwards, because it was better to mount that way)

#define IS_ENABLE_SECOND_RANGEFINDER                true                        // for the fwd f rangefinder
#define IS_DO_TILT_COMPENSATION_SECOND_RANGEFINDER  true
#define IS_DO_HEALTH_CHECK_SECOND_RANGEFINDER       true

#define IS_MOCK_OSCILLATING_RANGEFINDER_DATA        false    // for proving, I edited the right code
#define IS_CHECK_MINIMUM_ALTITUDE_OVER_GROUND       true    // TODO: prio 9: test this
#define DIST_MINIMUM_ALTITUDE_OVER_GROUND_CM        50      // do not go below this altitude           

//
// for different altitude control methods
//

// define enum-mocks
// for some reason, Log.cpp doesn't recognize these AltCtrlMode enum defines and causes a compiler error
// therefore we add them as defines as a fix
#define ALT_CTRL_MODE_STANDARD_PID                  1
#define ALT_CTRL_MODE_EXTENDED_PID                  2
#define ALT_CTRL_MODE_FFC                           3

// specify behavior of MEASUREMENT flight mode
#define MEASUREMENT_BEHAVIOR_LOITER                 1       // as loiter, but with new altitude controller
#define MEASUREMENT_BEHAVIOR_SEMI_GUIDED            2       // initially as loiter, then forward flight is triggered manually
#define MEASUREMENT_BEHAVIOR_GUIDED                 3       // behave as in guided

// set max horizontal speed for MEASUREMENT flight mode
#define IS_OVERWRITE_LOIT_SPEED_IN_MEASUREMENT      true    // so that we can fly with max speed without a complex
                                                            //  GUIDED-like speed navigation
#define MAX_MEASUREMENT_HORIZONTAL_SPEED            200     // in cm/s (would be MEAS_SPEED analog. to LOIT_SPEED)

// for ground profile acquisition
#define IS_GROUND_PROFILE_ACQUISITION_ENABLED       true                // enable AC_GroundProfileAcquisition lib
#define IS_USE_WORKAROUND_GROUND_PROFILE_ACQUISITION        true        // problems with compiling new lib
#define IS_USE_WORKAROUND_HOST_FILE_GPA             true    // true: host file, false: GroundProfileAcquisition_Workaround.h
#define GROUND_PROFILE_ACQUISITION_PROFILE_ARRAY_SIZE       2000
#define GROUND_PROFILE_ACQUISITION_NO_DATA_VALUE            (0x8000)    // min { int16_t }

// actual parameter definitions

// #define MEASUREMENT_ALTITUDE_CONTROL_MODE           AltCtrlMode::EXTENDED_PID // used altitude control method
// #define MEASUREMENT_ALTITUDE_CONTROL_MODE           EXTENDED_PID // used altitude control method
// #define MEASUREMENT_ALTITUDE_CONTROL_MODE           ALT_CTRL_MODE_EXTENDED_PID
#define MEASUREMENT_ALTITUDE_CONTROL_MODE           ALT_CTRL_MODE_FFC               // FOR TESTING
#define MEASUREMENT_FLIGHTMODE_BEHAVIOR             MEASUREMENT_BEHAVIOR_LOITER

// physical model parameters
// TODO: use actual values of my flamewheel, these are taken from [Kam11] and [Kla12]
#define PHYSICAL_MODEL_TIME_CONSTANT_MICROS         149000  // PT1-time constant tau of the copter (including deadtime)

// for extended PID
#define EXTENDED_PID_PROJECTION_TAU_FACTOR          1       // this multiplied with tau will be the interpolated time for extended PID

//
///////////////////////////////////////////////////////////////////////////////
//
///// calculated parameters - DO NOT CHANGE from here on, if you just want to adjust the parameters
//

// how much do we want to shift the rangefinder value into the future
//  by interpolating dwn and fwd facing rangefinders with a weight
//  this time controls the weight
#define EXTENDED_PID_FUTURE_PROJECTION_TIME_MICROS  (EXTENDED_PID_PROJECTION_TAU_FACTOR * PHYSICAL_MODEL_TIME_CONSTANT_MICROS)

#define RANGEFINDER_SIN_ANGLE_FORWARD_FACING        (sinf(radians( RANGEFINDER_ANGLE_FORWARD_FACING_DEG )))
#define RANGEFINDER_COS_ANGLE_FORWARD_FACING        (cosf(radians( RANGEFINDER_ANGLE_FORWARD_FACING_DEG )))

/////

#if (MEASUREMENT_ALTITUDE_CONTROL_MODE == ALT_CTRL_MODE_FFC) && (!IS_TEST_FFC)
    #error ALT_CTRL_MODE_FFC is not implemented yet, only allowed for tests
#endif 
