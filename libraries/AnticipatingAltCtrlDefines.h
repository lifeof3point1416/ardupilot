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

#define IS_USE_SITL_CONFIGURATION                   true

#if !IS_USE_SITL_CONFIGURATION
#define RANGEFINDER_ANGLE_FORWARD_FACING_DEG        45      // 0° is downwards
#else 
// for test in SITL, b/c SITL can only simulate 0° rangefinders
#define RANGEFINDER_ANGLE_FORWARD_FACING_DEG        0      // 0° is downwards, 
#endif

// rotation can be set in MissionPlanner: MP-->Config/Tuning/Full Param List
//                     in MAVProxy: param set RNGFND_ORIENT 0    # for ROTATION_NONE
//                                  param set RNGFND2_ORIENT 25  # for ROTATION_PITCH_270
#define RANGEFINDER_ORIENTATION_FORWARD_FACING      ROTATION_NONE               // "Forward" in MP
#define RANGEFINDER_ORIENTATION_DOWNWARD_FACING     ROTATION_PITCH_270          // "Down" in MP
// TODO: prio 5:    adjust orientations, 
// actually fwd should be ROTATION_PITCH_315 == 39 (45°backwards, there is not ROTATION_PITCH_45 yet, but 
//  prototype actually has it mounted backwards, because it was better to mount that way)

#define IS_ENABLE_SECOND_RANGEFINDER                true                        // for the fwd rangefinder
#define IS_DO_TILT_COMPENSATION_SECOND_RANGEFINDER  true
#define IS_DO_HEALTH_CHECK_SECOND_RANGEFINDER       true
#define IS_REVERSE_GPA_MAIN_DIRECTION               true        // in the case that fwd rangefinder is actually mounted at the back

#define IS_MOCK_OSCILLATING_RANGEFINDER_DATA        false    // for proving, I edited the right code
#define IS_CHECK_MINIMUM_ALTITUDE_OVER_GROUND       true    // TODO: prio 7: test this
#define DIST_MINIMUM_ALTITUDE_OVER_GROUND_CM        50      // do not go below this altitude           

#define CALL_FREQUENCY_UPDATE_GPA                   50     // in Hz for the scheduler

//
// for different altitude control methods
//

///// define enum-mocks
// for some reason, Log.cpp doesn't recognize these AltCtrlMode enum defines and causes a compiler error
// therefore we add them as defines as a fix
#define ALT_CTRL_MODE_STANDARD_PID                  1
#define ALT_CTRL_MODE_EXTENDED_PID                  2
#define ALT_CTRL_MODE_FFC                           3
// alt ctrl mode names for debug printouts
#define ALT_CTRL_MODE_NAME_STANDARD_PID             "STANDARD_PID"
#define ALT_CTRL_MODE_NAME_EXTENDED_PID             "EXTENDED_PID"
#define ALT_CTRL_MODE_NAME_FFC                      "FFC"

#define GROUND_PROFILE_DERIVATOR_SINGLE_POLYNOME_FITTING    1
#define GROUND_PROFILE_DERIVATOR_CONSECUTIVE_LINEAR_FITTING 2

// specify behavior of MEASUREMENT flight mode
#define MEASUREMENT_BEHAVIOR_LOITER                         1       // as loiter, but with new altitude controller
#define MEASUREMENT_BEHAVIOR_SEMI_GUIDED                    2       // initially as loiter, then forward flight is triggered manually
#define MEASUREMENT_BEHAVIOR_GUIDED                         3       // behave as in guided
///// end of define enum-mocks

// set max horizontal speed for MEASUREMENT flight mode
#define IS_OVERWRITE_LOIT_SPEED_IN_MEASUREMENT              0001    // so that we can fly with max speed without a complex
                                                            //  GUIDED-like speed navigation
#define IS_SEND_MESSAGE_LOIT_SPEED_IN_MEASUREMENT           0
#define MAX_MEASUREMENT_HORIZONTAL_SPEED                    100     // in cm/s (would be MEAS_SPEED analog. to LOIT_SPEED)

// for ground profile acquisition (GPA)

//  overwrite IS_GROUND_PROFILE_ACQUISITION_ENABLED with GROUND_PROFILE_ACQUISITION_VALUE_TO_BE_FORCED ?
#define IS_FORCE_GROUND_PROFILE_ACQUISITION_WITH_VALUE      false       
#define GROUND_PROFILE_ACQUISITION_VALUE_TO_BE_FORCED       true

#define IS_USE_WORKAROUND_GROUND_PROFILE_ACQUISITION        true        // had problems with compiling new lib
// true: host file, false: GroundProfileAcquisition_Workaround.h
#define IS_USE_WORKAROUND_HOST_FILE_GPA                     true

#define GROUND_PROFILE_ACQUISITION_PROFILE_ARRAY_SIZE       2000
#define GROUND_PROFILE_ACQUISITION_NO_DATA_VALUE            INT16_MIN    // write this value if no data available, b/c 0 cm is a valid datum
#define GROUND_PROFILE_ACQUISITION_INVALID_X_VALUE          INT16_MIN
//#define GROUND_PROFILE_ACQUISITION_NO_DATA_VALUE            (0x8000)    // smallest possible value for int16_t
#define IS_USE_GPA_MAP_OFFSET                               true        // so that first GPA map value is set to 0

#define GPA_MAX_DEVIATION_FROM_MAIN_DIRECTION_CM            100     // otherwise: don't store in GPA, send warning
#define IS_SEND_MESSAGE_IF_GPA_NOT_SUCCESSFUL               true
//#define MESSAGE_IF_GPA_NOT_SUCCESSFUL_TIMEOUT_USEC        (10*1000000)    // wait at least this timespan for next msg
#define MESSAGE_IF_GPA_NOT_SUCCESSFUL_TIMEOUT_USEC          (2*1000000)    // shorter for debug mode
#define IS_LOG_GPA                                          true            // within GPA method
#define IS_LOG_EXTRA_XF_ZF_CONSTRAINT                       true            // to prevent log review window zooming out for -32k values
#define IS_CONVERT_FLOAT_LOGS_TO_DOUBLE                     true
#define GPA_MAP_LOG_CHUNK_SIZE                              32      // constrained by 'a' arrays: int16_t[32]
#define INVALID_RANGEFINDER_VALUE                           (-1)    // if some rangefinder value is not available for any reason

// for ground profile derivator (GPD)

#define IS_SMOOTHEN_GROUND_PROFILE_DERIVATION_VALUES        true
// #define IS_SMOOTHEN_GROUND_PROFILE_DERIVATION_VALUES        false
#define GROUND_PROFILE_DERIVATION_FILTER_WINDOW_SIZE        3
#define IS_CHECK_HEADING_FOR_HORIZONTAL_SPEED_COMPENSATION  true        // check for validity in get_deviations
#define GROUND_PROFILE_DERIVATOR_DX_APPROX                  10          // step size for derivations [cm]
// #define GROUND_PROFILE_DERIVATOR_FITTING                    GROUND_PROFILE_DERIVATOR_CONSECUTIVE_LINEAR_FITTING
#define GROUND_PROFILE_DERIVATOR_FITTING                    GROUND_PROFILE_DERIVATOR_SINGLE_POLYNOME_FITTING
#define DERIVATIONS_NO_DATA_INIT_VALUE                      0           // had some trouble with NAN
#define IS_DO_GPD2_DEBUGGING_LOGGING                        true        // attention! this is very verbose
#define GPD2_LOGGING_FREQUENCY                              100         // [Hz]
#define IS_DO_HSC_LOGGING                                   true
// for CLF
#define GROUND_PROFILE_DERIVATOR_MULTIPLICATOR_EXPONENT     4           // fixed comma values will be multiplied by 2^<this number> for better precision
#define IS_DO_CLF_DEBUGGING_LOGGING                         true        // for Consecutive Linear Fitting Derivation
// #define CLF_LOGGING_FREQUENCY                               400         // [Hz]
// for SPF
#define LINEAR_EQUATION_SYSTEM_SOLVER_0_TOLERANCE           (1.0e-10f)  // values with an abs below this are considered 0
#define IS_DO_SPF_DEBUGGING_LOGGING                         true        // for Single Polynome Fitting Derivation
#define SPF_MINIMUM_N_VALUES                                4           // 4 is absolute minimum
#define IS_DO_VERBOSE_SPF_DEBUGGING_LOGGING                 false        // log intermediate results for SPF ("SPF2")
#define SPF_LES_N_VARIABLES_CUBIC                           4           // a cubic curve has 4 parameters, which we need to find

// actual parameter definitions

// #define MEASUREMENT_ALTITUDE_CONTROL_MODE           ALT_CTRL_MODE_STANDARD_PID
// #define MEASUREMENT_ALTITUDE_CONTROL_MODE           ALT_CTRL_MODE_EXTENDED_PID
#define MEASUREMENT_ALTITUDE_CONTROL_MODE           ALT_CTRL_MODE_FFC               // only FOR TESTING yet
#define MEASUREMENT_FLIGHTMODE_BEHAVIOR             MEASUREMENT_BEHAVIOR_LOITER

// physical model parameters
// TODO: use actual values of my flamewheel, these are taken from [Kam11] and [Kla12]
#define PHYSICAL_MODEL_TIME_CONSTANT_MICROS                 149000  // PT1-time constant tau of the copter (including deadtime) [us]
#define PHYSICAL_MODEL_SIMPLIFIED_AIR_RESISTANCE_CONST      (10000) // [1e6/s] == [1/Ms]
#define PHYSICAL_MODEL_COPTER_MASS                            1500  // [g]
#define PHYSICAL_MODEL_GRAVITATION_CONST                       981  // [cm/s/s]

// for extended PID
#define EXTENDED_PID_PROJECTION_TAU_FACTOR          1       // this multiplied with tau will be the interpolated time for extended PID
#define IS_DO_XPID_DEBUGGING_LOGGING                false   // do logging for all PIDs, especially useful for Extended PID, very verbose!
// #define XPID_LOGGING_FREQUENCY                      100     // [Hz]
#define IS_DO_XPI2_DEBUGGING_LOGGING                true    // verbose additional info to XPID
#define EXTENDED_PID_ZERO_EPSILON                   (1e-6f)
#define EXTENDED_PID_MAX_PROJECTION_FACTOR          (5.0f)       // don't extrapolate more than this

//
///////////////////////////////////////////////////////////////////////////////
//
///// calculated parameters - DO NOT CHANGE from here on, if you just want to adjust the parameters
//

// returns true after (N*1.05) s (exactly 1<<20 us), but only of the tick before hasn't been triggering
//  usage for triggering eventy every ca. N seconds, eg. debug printout 
//  using barrel shifter instead of int division to be faster on arm processors
//  sometimes it is triggered twice, but not very often ==> not reliable
//  usage: if (IS_TRIGGER_EVENT_ROUGHLY_EVERY_N_SEC_MICROS(1, AP_HAL::micros(), 400)) {...}
#define IS_TRIGGER_EVENT_ROUGHLY_EVERY_N_SEC_MICROS(N, micros, freq) (   \
    ( ((( (micros) >> 10) >> 7) & 0b111) / N == 0 ) && ( ((( (micros - ((1<<20)/freq)) >> 10) >> 7) & 0b111) / N != 0 ) \
)

// keep some reserve: window might include 1 element to the left and to the right, also we might add 1 element for each
//  derivation, to counter that the rightmost element is consumed by diffing
#define GROUND_PROFILE_DERIVATOR_VECTOR_ARRAY_SIZE  (GROUND_PROFILE_DERIVATOR_DX_APPROX + 5)

// use analogue to ~_MICROS, but this is not as reliable
#define IS_TRIGGER_EVENT_ROUGHLY_EVERY_N_SEC_MILLIS(N, millis, freq) (   \
    ( (( (millis) >> 7) & 0b111) / N == 0 ) && ( (( (millis - ((1<<10)/freq)) >> 7) & 0b111) / N != 0 ) \
)

// number of GPA map chunks that is necessary to log one GPA map
// round number up. if there is a single value which is left, we need to have another chunk
#define GPA_MAP_LOG_N_CHUNKS (GROUND_PROFILE_ACQUISITION_PROFILE_ARRAY_SIZE / GPA_MAP_LOG_CHUNK_SIZE + \
    ( (GROUND_PROFILE_ACQUISITION_PROFILE_ARRAY_SIZE % GPA_MAP_LOG_CHUNK_SIZE) != 0) )

// how much do we want to shift the rangefinder value into the future
//  by interpolating dwn and fwd facing rangefinders with a weight
//  this time controls the weight
#define EXTENDED_PID_FUTURE_PROJECTION_TIME_MICROS  (EXTENDED_PID_PROJECTION_TAU_FACTOR * PHYSICAL_MODEL_TIME_CONSTANT_MICROS)

#define RANGEFINDER_SIN_ANGLE_FORWARD_FACING        (sinf(radians( RANGEFINDER_ANGLE_FORWARD_FACING_DEG )))
#define RANGEFINDER_COS_ANGLE_FORWARD_FACING        (cosf(radians( RANGEFINDER_ANGLE_FORWARD_FACING_DEG )))

#define IS_USE_GPA_MAP_FREEZE_MODE                  (IS_USE_SITL_CONFIGURATION && IS_RUN_GROUND_PROFILE_DERIVATOR_TESTS)

// set name string of alt ctrl
#if MEASUREMENT_ALTITUDE_CONTROL_MODE == ALT_CTRL_MODE_STANDARD_PID
    #define MEASUREMENT_NAME_OF_ALTITUDE_CONTROL_MODE   ALT_CTRL_MODE_NAME_STANDARD_PID
#elif MEASUREMENT_ALTITUDE_CONTROL_MODE == ALT_CTRL_MODE_EXTENDED_PID
    #define MEASUREMENT_NAME_OF_ALTITUDE_CONTROL_MODE   ALT_CTRL_MODE_NAME_EXTENDED_PID
#elif MEASUREMENT_ALTITUDE_CONTROL_MODE == ALT_CTRL_MODE_FFC
    #define MEASUREMENT_NAME_OF_ALTITUDE_CONTROL_MODE   ALT_CTRL_MODE_NAME_FFC
#else
    #error "UNKNOWN MEASUREMENT_ALTITUDE_CONTROL_MODE"
#endif // MEASUREMENT_ALTITUDE_CONTROL_MODE == ALT_CTRL_MODE_STANDARD_PID  

// enable AC_GroundProfileAcquisition lib?
#if IS_FORCE_GROUND_PROFILE_ACQUISITION_WITH_VALUE
    #define IS_GROUND_PROFILE_ACQUISITION_ENABLED       GROUND_PROFILE_ACQUISITION_VALUE_TO_BE_FORCED                
#else
    #define IS_GROUND_PROFILE_ACQUISITION_ENABLED       (MEASUREMENT_ALTITUDE_CONTROL_MODE == ALT_CTRL_MODE_FFC)           
#endif // IS_FORCE_GROUND_PROFILE_ACQUISITION_WITH_VALUE
//
///// asserts
//
// TODO: remove the following assert if FFC is fully implemented
#if ((MEASUREMENT_ALTITUDE_CONTROL_MODE == ALT_CTRL_MODE_FFC) && (!IS_TEST_FFC))
    #error "ALT_CTRL_MODE_FFC is not implemented yet, only allowed for tests"
#endif 

#if ((MEASUREMENT_ALTITUDE_CONTROL_MODE == ALT_CTRL_MODE_FFC) && !IS_GROUND_PROFILE_ACQUISITION_ENABLED)
    #error "for Feed Forward Control, Ground Profile Acquisition must be enabled, but it is not enabled."
#endif

#define BIASED_GPA_VALUE(N)                             ( (N) > GROUND_PROFILE_ACQUISITION_NO_DATA_VALUE ? \
    GET_BIASED_NUMBER((N), (GPA_MAP_CONDENSED_BIAS)) : (0x00) )
#define UNBIASED_GPA_VALUE(N)                           ( (N) != 0x00 ? \
    GET_UNBIASED_NUMBER((N), (GPA_MAP_CONDENSED_BIAS)) : (GROUND_PROFILE_ACQUISITION_NO_DATA_VALUE) ) 

#if IS_DEBUG_CHECK_FWD_RANGEFINDER_ANGLE
    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    static_assert(RANGEFINDER_ANGLE_FORWARD_FACING_DEG == 0, 
        "in SITL only  dwn rangefinders can be simulated, fwd rangefinder angle should be 0°!");
    #else
    static_assert(RANGEFINDER_ANGLE_FORWARD_FACING_DEG > 0+10, 
        "check fwd rangefinder angle, should be much bigger than 0°!");
    #endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
#endif // IS_DEBUG_CHECK_FWD_RANGEFINDER_ANGLE

static_assert( (GROUND_PROFILE_DERIVATOR_FITTING == GROUND_PROFILE_DERIVATOR_SINGLE_POLYNOME_FITTING) ||
    (GROUND_PROFILE_DERIVATOR_FITTING == GROUND_PROFILE_DERIVATOR_CONSECUTIVE_LINEAR_FITTING),
    "Unknown value for GROUND_PROFILE_DERIVATOR_FITTING" );

// #if (GROUND_PROFILE_DERIVATOR_FITTING == GROUND_PROFILE_DERIVATOR_SINGLE_POLYNOME_FITTING)
// #error GROUND_PROFILE_DERIVATOR_SINGLE_POLYNOME_FITTING is not implemented yet
// #endif

static_assert(GPA_MAP_LOG_CHUNK_SIZE == 32, "GPA map chunk sizes must be 32, in order to match logging formatter 'a'");

#if IS_DO_CLF_DEBUGGING_LOGGING && (!IS_DO_GPD2_DEBUGGING_LOGGING)
#error CLF logging requires GDP2 logging!
#endif

static_assert(IS_VERBOSE_THROTTLE_LOGGING_FFC <= IS_LOG_GPA,
    "For logging within Rangefinder.cpp IS_LOG_GPA must be true.");

static_assert(IS_VERBOSE_CLF_LOGGING <= IS_USE_SITL_CONFIGURATION,
    "Verbose CLF logging may only be used in SITL configuration.");
