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

// for rangefinders
// rotation can be set in MissionPlanner: MP-->Config/Tuning/Full Param List
//                     in MAVProxy: param set RNGFND_ORIENT 0    # for ROTATION_NONE
//                                  param set RNGFND2_ORIENT 25  # for ROTATION_PITCH_270
#define RANGEFINDER_ORIENTATION_FORWARD_FACING      ROTATION_NONE               // "Forward" in MP
#define RANGEFINDER_ORIENTATION_DOWNWARD_FACING     ROTATION_PITCH_270          // "Down" in MP
// TODO: prio 7:    adjust orientations, 
// actually fwd should be ROTATION_PITCH_315 == 39 (45°backwards, there is not ROTATION_PITCH_45 yet, but 
//  prototype actually has it mounted backwards, because it was better to mount that way)
