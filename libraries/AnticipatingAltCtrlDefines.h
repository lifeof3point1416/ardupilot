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


//
// for different altitude control methods
//

// for some reason, Log.cpp doesn't recognize these AltCtrlMode enum defines and causes a compiler error
// therefore we add them as defines as a fix
#define ALT_CTRL_MODE_STANDARD_PID                  1
#define ALT_CTRL_MODE_EXTENDED_PID                  2
#define ALT_CTRL_MODE_FFC                           3

// #define MEASUREMENT_ALTITUDE_CONTROL_MODE           AltCtrlMode::EXTENDED_PID // used altitude control method
// #define MEASUREMENT_ALTITUDE_CONTROL_MODE           EXTENDED_PID // used altitude control method
#define MEASUREMENT_ALTITUDE_CONTROL_MODE           ALT_CTRL_MODE_EXTENDED_PID
