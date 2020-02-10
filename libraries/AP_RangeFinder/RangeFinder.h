/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>

// Maximum number of range finder instances available on this platform
#define RANGEFINDER_MAX_INSTANCES 2
#define RANGEFINDER_GROUND_CLEARANCE_CM_DEFAULT 10
#define RANGEFINDER_PREARM_ALT_MAX_CM           200
#define RANGEFINDER_PREARM_REQUIRED_CHANGE_CM   50

class AP_RangeFinder_Backend;

class RangeFinder
{
    friend class AP_RangeFinder_Backend;

public:
    RangeFinder(AP_SerialManager &_serial_manager, enum Rotation orientation_default);

    /* Do not allow copies */
    RangeFinder(const RangeFinder &other) = delete;
    RangeFinder &operator=(const RangeFinder&) = delete;

    // RangeFinder driver types
    enum RangeFinder_Type {
        RangeFinder_TYPE_NONE   = 0,
        RangeFinder_TYPE_ANALOG = 1,
        RangeFinder_TYPE_MBI2C  = 2,
        RangeFinder_TYPE_PLI2C  = 3,
        RangeFinder_TYPE_PX4    = 4,
        RangeFinder_TYPE_PX4_PWM= 5,
        RangeFinder_TYPE_BBB_PRU= 6,
        RangeFinder_TYPE_LWI2C  = 7,
        RangeFinder_TYPE_LWSER  = 8,
        RangeFinder_TYPE_BEBOP  = 9,
        RangeFinder_TYPE_MAVLink = 10,
        RangeFinder_TYPE_ULANDING= 11,
        RangeFinder_TYPE_LEDDARONE = 12,
        RangeFinder_TYPE_MBSER  = 13,
        RangeFinder_TYPE_TRI2C  = 14,
        RangeFinder_TYPE_PLI2CV3= 15,
        RangeFinder_TYPE_VL53L0X = 16,
        RangeFinder_TYPE_NMEA = 17,
        RangeFinder_TYPE_WASP = 18,
        RangeFinder_TYPE_BenewakeTF02 = 19,
        RangeFinder_TYPE_BenewakeTFmini = 20,
        RangeFinder_TYPE_PLI2CV3HP = 21,
        RangeFinder_TYPE_BenewakeTFminiPlus = 25,
    };

    enum RangeFinder_Function {
        FUNCTION_LINEAR    = 0,
        FUNCTION_INVERTED  = 1,
        FUNCTION_HYPERBOLA = 2
    };

    enum RangeFinder_Status {
        RangeFinder_NotConnected = 0,
        RangeFinder_NoData,
        RangeFinder_OutOfRangeLow,
        RangeFinder_OutOfRangeHigh,
        RangeFinder_Good
    };

    // The RangeFinder_State structure is filled in by the backend driver
    struct RangeFinder_State {
        uint16_t               distance_cm; // distance: in cm
        uint16_t               voltage_mv;  // voltage in millivolts,
                                            // if applicable, otherwise 0
        enum RangeFinder_Status status;     // sensor status
        uint8_t                range_valid_count;   // number of consecutive valid readings (maxes out at 10)
        bool                   pre_arm_check;   // true if sensor has passed pre-arm checks
        uint16_t               pre_arm_distance_min;    // min distance captured during pre-arm checks
        uint16_t               pre_arm_distance_max;    // max distance captured during pre-arm checks

        AP_Int8  type;
        AP_Int8  pin;
        AP_Int8  ratiometric;
        AP_Int8  stop_pin;
        AP_Int16 settle_time_ms;
        AP_Float scaling;
        AP_Float offset;
        AP_Int8  function;
        AP_Int16 min_distance_cm;
        AP_Int16 max_distance_cm;
        AP_Int8  ground_clearance_cm;
        AP_Int8  address;
        AP_Vector3f pos_offset; // position offset in body frame
        AP_Int8  orientation;
        const struct AP_Param::GroupInfo *var_info;
    };

    static const struct AP_Param::GroupInfo *backend_var_info[RANGEFINDER_MAX_INSTANCES];
    
    AP_Int16 _powersave_range;

    // parameters for each instance
    static const struct AP_Param::GroupInfo var_info[];
    
    // Return the number of range finder instances
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    // detect and initialise any available rangefinders
    void init(void);

    // update state of all rangefinders. Should be called at around
    // 10Hz from main loop
    void update(void);

    // Handle an incoming DISTANCE_SENSOR message (from a MAVLink enabled range finder)
    void handle_msg(mavlink_message_t *msg);

    // return true if we have a range finder with the specified orientation
    bool has_orientation(enum Rotation orientation) const;

    // find first range finder instance with the specified orientation
    // PSt: this to get 2nd rangefinder?
    AP_RangeFinder_Backend *find_instance(enum Rotation orientation) const;

    AP_RangeFinder_Backend *get_backend(uint8_t id) const;

    // methods to return a distance on a particular orientation from
    // any sensor which can current supply it
    uint16_t distance_cm_orient(enum Rotation orientation) const;
    uint16_t voltage_mv_orient(enum Rotation orientation) const;
    int16_t max_distance_cm_orient(enum Rotation orientation) const;
    int16_t min_distance_cm_orient(enum Rotation orientation) const;
    int16_t ground_clearance_cm_orient(enum Rotation orientation) const;
    MAV_DISTANCE_SENSOR get_mav_distance_sensor_type_orient(enum Rotation orientation) const;
    RangeFinder_Status status_orient(enum Rotation orientation) const;
    bool has_data_orient(enum Rotation orientation) const;
    uint8_t range_valid_count_orient(enum Rotation orientation) const;
    const Vector3f &get_pos_offset_orient(enum Rotation orientation) const;

    /*
      set an externally estimated terrain height. Used to enable power
      saving (where available) at high altitudes.
     */
    void set_estimated_terrain_height(float height) {
        estimated_terrain_height = height;
    }

    /*
      returns true if pre-arm checks have passed for all range finders
      these checks involve the user lifting or rotating the vehicle so that sensor readings between
      the min and 2m can be captured
     */
    bool pre_arm_check() const;

    static RangeFinder *get_singleton(void) { return _singleton; }


private:
    static RangeFinder *_singleton;

    RangeFinder_State state[RANGEFINDER_MAX_INSTANCES];
    AP_RangeFinder_Backend *drivers[RANGEFINDER_MAX_INSTANCES];
    uint8_t num_instances:3;
    float estimated_terrain_height;
    AP_SerialManager &serial_manager;
    Vector3f pos_offset_zero;   // allows returning position offsets of zero for invalid requests

    void detect_instance(uint8_t instance, uint8_t& serial_instance);
    void update_instance(uint8_t instance);  

    bool _add_backend(AP_RangeFinder_Backend *driver);
};

///// Temporary workaround for AC_GroundProfileAcquisition here
// couldn't add AC_GroundProfileAcquisition to waf successfully, yet
// definition part is in RangeFinder.cpp

#include "AnticipatingAltCtrlDefines.h"                 // PeterSt

#define HEADING_CENTIDEGREES_FROM_MATH_ANGLE_RADIANS(angle) ( ( (9000 - ((angle) * DEGX100)) >= 0) ? \
    (9000 - ((angle) * DEGX100)) : (9000 - ((angle) * DEGX100) + 36000) )

#if IS_USE_WORKAROUND_GROUND_PROFILE_ACQUISITION && IS_USE_WORKAROUND_HOST_FILE_GPA
#if 1   // in case we want to disable this definition manually


class AC_GroundProfileAcquisition {

public:

    AC_GroundProfileAcquisition(void);
    bool init(void);
#if IS_USE_GPA_MAP_FROM_FILE
    bool read_gpa_from_file(void);
#endif // IS_USE_GPA_MAP_FROM_FILE
    // start scanning with the current orientation and set position_neu_cm as reference point for absolute position
    bool start(uint16_t _heading, Vector3f position_neu_cm);                      
    #if 0 
    bool start(int _heading, Vector3f position_neu_cm);
    #endif
    int scan_point(int16_t fwd_rangefinder_dist_cm, Vector3f position_neu_cm);
    // int scan_point(int16_t dwn_rangefinder_dist_cm, int16_t fwd_rangefinder_dist_cm, Vector3f position_neu_cm);

    //int get_1d_x(Vector3f position_neu_cm);
    Vector2<int> get_main_direction_coo(Vector3f position_neu_cm);

    // note that these enums must be negative, because any positive return value of scan_point(...) is
    //  considered valid
    enum ScanPointInvalidReturnValue {
        ScanPointInvalidReturnValue_NOT_INITIALIZED = -1,
        ScanPointInvalidReturnValue_GROUND_PROFILE_INDEX_NEGATIVE = -2,
        ScanPointInvalidReturnValue_GROUND_PROFILE_INDEX_TOO_HIGH = -3,
        ScanPointInvalidReturnValue_DEVIATION_FROM_MAIN_DIRECTION_EXCEEDED = -4,
        ScanPointInvalidReturnValue_VALUE_OUT_OF_RANGE = -5,
        ScanPointInvalidReturnValue_FWD_RANGEFINDER_NOT_HEALTHY = -6,       // actually no Scan Point return value
        ScanPointInvalidReturnValue_GROUND_PROFILE_ACQUISITION_FROZEN = -7,
        ScanPointInvalidReturnValue_USE_GROUND_PROFILE_ACQUISITION_DATA_FROM_FILE = -8,  // no scanning necessary
    };
    bool is_scan_point_index_valid(int scan_point_return_value) { return scan_point_return_value >= 0;}
    inline int16_t get_ground_profile_datum(int index) {return ground_profile[index];}
    inline bool has_ground_profile_datum_no_index_check(int index) {        // only check if the ground_profile value not the invalid value
        return get_ground_profile_datum(index) != GROUND_PROFILE_ACQUISITION_NO_DATA_VALUE;}
    inline bool has_ground_profile_datum(int index) {                       // check whether index is valid, and if ground_profile value is not invalid
        return ((0 <= index) && (index < GROUND_PROFILE_ACQUISITION_PROFILE_ARRAY_SIZE)) &&
        (get_ground_profile_datum(index) != GROUND_PROFILE_ACQUISITION_NO_DATA_VALUE);}

    uint16_t get_main_direction(void) {return main_direction;}
#if IS_PRINT_GPA_NEW_POINT
    struct ScannedPoint {
        uint32_t time_us;
        // input
        int16_t fwd_rangefinder_dist_cm;
        Vector3f position_neu_cm;
        // calculated values, all in cm
        int16_t x_f;                            // for ground_profile index
        int y_p;                                // expresses deviation from main_direction line
        int16_t z_f;                            // absolute altitiude
    };
    struct ScannedPoint last_scanned_point;
#endif // IS_PRINT_GPA_NEW_POINT

    // concerning logging
#if IS_LOG_GPA 
    bool log_scan_point(uint64_t TimeUS, int16_t FwdRF, float PosX, float PosY, float PosZ,
        int32_t XP, int32_t YP, int32_t ZP, int16_t XF, int16_t ZF, bool IsValid, int Ret);
    bool scan_point_unhealthy_fwd_rangefinder(int16_t fwd_rangefinder_dist_cm, Vector3f position_neu_cm);
#endif // IS_LOG_GPA
    bool log_ground_profile(void);
    inline uint32_t get_ground_profile_map_seq_no(void) {
        return ground_profile_map_seq_no;
    }
#if IS_USE_GPA_MAP_FREEZE_MODE
    inline void set_freeze_map(bool is_frozen) {    // freeze/unfreeze GPA ground_profile map
        _is_frozen = is_frozen;
    }
    inline bool is_frozen(void) {return _is_frozen;} 
#endif // IS_USE_GPA_MAP_FREEZE_MODE

protected:

private:

    int16_t ground_profile[GROUND_PROFILE_ACQUISITION_PROFILE_ARRAY_SIZE];
    uint16_t main_direction = 36000;            // heading in centi degrees [c°]
    // int main_direction = 36000;              // heading in centi degrees [c°]
    // position_neu_cm of starting point, this will be 0 for ground_profile
    Vector3f start_position_cm;                                 
#if IS_USE_GPA_MAP_OFFSET
    int16_t ground_profile_offset = 0;          // [cm] so that first value in map is 0
    bool is_ground_profile_offset_set = false;
#endif // IS_USE_GPA_MAP_OFFSET
    uint32_t ground_profile_map_seq_no = 0;     // sequential number of the gpa ground_profile, for identifying logs
#if IS_USE_GPA_MAP_FREEZE_MODE
    bool _is_frozen = false;                    // is ground_profile frozen?
#endif // IS_USE_GPA_MAP_FREEZE_MODE
};

#endif // 1 OR 0
#endif // IS_USE_WORKAROUND_GROUND_PROFILE_ACQUISITION && IS_USE_WORKAROUND_HOST_FILE_GPA

///// Temporary workaround for AC_GroundProfileDerivator here
// couldn't add files to waf successfully, yet
// definition part is in RangeFinder.cpp

class AC_GroundProfileDerivator {

public:

    // AC_GroundProfileDerivator(void) {;}          // don't use this ==> force initing gpa for gpd
    AC_GroundProfileDerivator(AC_GroundProfileAcquisition *_ground_profile_acquisition);
    bool init(void);                                // is this necessary?
    void set_ground_profile(AC_GroundProfileAcquisition *_ground_profile_acquisition) {
        ground_profile_acquisition = _ground_profile_acquisition;
    }

    // the first three derivates of distance over space or time, depending on context
    struct DistanceDerivations {                    
        float first;                                // [cm/cm]          or [cm/s]
        float second;                               // [cm/cm/cm]       or [cm/s/s]
        float third;                                // [cm/cm/cm/cm]    or [cm/s/s/s]
        bool is_valid;                              // was it possible to calculate the derivations
    };

    // return states for CLF and SPF, respectively
    enum ConsecutiveLinearFittingReturnState {
        ConsecutiveLinearFittingReturnState_VALID_RESULT = 0,       // got a valid result; deviations.is_valid == true
        ConsecutiveLinearFittingReturnState_N_VALUES_EQ_ZERO = 1,
        ConsecutiveLinearFittingReturnState_N_VALUES_LT_TWO = 2,
        ConsecutiveLinearFittingReturnState_XX_DIFF_SUM_EQ_ZERO = 3,
        ConsecutiveLinearFittingReturnState_NOT_DONE_YET = 10,      // no return state, only intermediate value!
    };
    enum SinglePolynomeFittingReturnState {
        SinglePolynomeFittingReturnState_NOT_IMPLEMENTED_YET = -1,  // not impl
        SinglePolynomeFittingReturnState_VALID_RESULT = 0,          // deviations.is_valid == true
        // ...
        SinglePolynomeFittingReturnState_PIVOT_ELEMENT_EQ_ZERO = 4,
        SinglePolynomeFittingReturnState_N_VALUES_TOO_LOW = 5,
        SinglePolynomeFittingReturnState_NOT_DONE_YET = 10,         // no return state, only intermediate value!
    };
    enum LinearEquationSystemState {
        LinearEquationSystemState_NOT_INITIALIZED       = 1,        
        LinearEquationSystemState_ORIGINAL_MATRIX       = 2,        // original LES, containing sums of x_i^p*z_i^q, for q = 0..3, p = 0 .. 6
        LinearEquationSystemState_ECHELON               = 3,
        LinearEquationSystemState_DIAGONAL              = 4,
        LinearEquationSystemState_NORMALIZED_DIAGONAL   = 5,
    };

    // get first 3 derivations of ground profile at position_neu_cm
    //  using IS_SMOOTHEN_GROUND_PROFILE_DERIVATION_VALUES
    //  internally use int, because it is faster
    DistanceDerivations get_profile_derivations(Vector3f position_neu_cm, float horiz_speed, int32_t heading, bool is_log);
    //DistanceDerivations get_profile_derivations(Vector3f position_neu_cm, float horiz_speed, bool is_log);
    int32_t get_opposite_heading_cd(int32_t heading_cd);
    int32_t get_heading_diff_cd(int32_t heading1, int32_t heading2);
    // logging functions
    inline bool log_ground_profile(void) {return ground_profile_acquisition->log_ground_profile();}
    // curve fitting related functions for deriving ground profile derivations
#if     GROUND_PROFILE_DERIVATOR_FITTING == GROUND_PROFILE_DERIVATOR_CONSECUTIVE_LINEAR_FITTING
    DistanceDerivations get_consecutive_linear_fitting(int x_target_left, int x_target_right);
 #if IS_USE_FLOAT_ARITHMETIC_FOR_DERIVATION
  #if IS_DO_CLF_DEBUGGING_LOGGING
    // log data under tag "CLF" for before function return, or intermediate
    void log_consecutive_linear_fitting(int n_values, int8_t validity_status,
        float x_sum, float z_sum_i, int grade_i, float xx_diff_sum, float xz_diff_sum,
        AC_GroundProfileDerivator::DistanceDerivations derivations);
  #endif // IS_DO_CLF_DEBUGGING_LOGGING
  #if IS_VERBOSE_CLF_LOGGING
    // log the data which will actually be used to calculate the derivations (where invalid data had been removed)
    //  tag "CLF2" or "CLF3"; note that if a filter is used, unfiltered values are logged, too
    //  ==> two CLF2 logs with GrdI==0, first with unfiltered, second with filtered values
    // "CLF2" for int arithmetic
    // "CLF3" for float arithmetic, inherent conversion, example:
    // <full> <fraction*10000> ==> [4.5 42.0 3.1416 0.12345] ==> [4 5000 42 0 3 1416 0 1235]
    void log_consecutive_linear_fitting2(int n_values, float *x_vector, float *z_vector, float *dx_vector, int grade_i);
   #if IS_TEST_INT32_INT16_LOGGING
    // this will be used for log_consecutive_linear_fitting2
    void test_logging_int32ar_as_int16ar(void);
   #endif // IS_TEST_INT32_INT16_LOGGING
  #endif // IS_VERBOSE_CLF_LOGGING
 #else // IS_USE_FLOAT_ARITHMETIC_FOR_DERIVATION
  #if IS_DO_CLF_DEBUGGING_LOGGING
    // log data under tag "CLF" for before function return, or intermediate
    void log_consecutive_linear_fitting(int n_values, int8_t validity_status,
        int x_sum, int z_sum_mult_i, int grade_i, int xx_diff_sum_mult, int xz_diff_sum_mult,
        AC_GroundProfileDerivator::DistanceDerivations derivations);
  #endif // IS_DO_CLF_DEBUGGING_LOGGING
  #if IS_VERBOSE_CLF_LOGGING
    // log the data which will actually be used to calculate the derivations (where invalid data had been removed)
    //  tag "CLF2"; note that if a filter is used, unfiltered values are logged, too
    //  ==> two CLF2 logs with GrdI==0, first with unfiltered, second with filtered values
    void log_consecutive_linear_fitting2(int n_values, int *x_vector, int *z_vector_mult, int *dx_vector, int grade_i);
   #if IS_TEST_INT32_INT16_LOGGING
    // this will be used for log_consecutive_linear_fitting2
    void test_logging_int32ar_as_int16ar(void);
   #endif // IS_TEST_INT32_INT16_LOGGING
  #endif // IS_VERBOSE_CLF_LOGGING
 #endif // IS_USE_FLOAT_ARITHMETIC_FOR_DERIVATION
#elif     GROUND_PROFILE_DERIVATOR_FITTING == GROUND_PROFILE_DERIVATOR_SINGLE_POLYNOME_FITTING
    DistanceDerivations get_single_polynome_fitting(int x_target_left, int x_target_right, int x_p);
 #if IS_DO_SPF_DEBUGGING_LOGGING
    void log_single_polynome_fitting(int x_p, int n_values, float coeff_a, float coeff_b, float coeff_c, float coeff_d, 
        AC_GroundProfileDerivator::DistanceDerivations derivations, int8_t validity_status);
    void log_single_polynome_fitting_profile_data(int x_p, int n_values, int *x_vector, 
        int *z_vector_raw, float *z_vector_filtered, bool is_filter_enabled);
  #if IS_DO_VERBOSE_SPF_DEBUGGING_LOGGING
    void log_single_polynome_fitting_linear_equation_sys(float A[SPF_LES_N_VARIABLES_CUBIC][SPF_LES_N_VARIABLES_CUBIC],
        float *b, int8_t les_status);
  #endif // IS_DO_VERBOSE_SPF_DEBUGGING_LOGGING
 #endif // IS_DO_SPF_DEBUGGING_LOGGING
#else   // GROUND_PROFILE_DERIVATOR_FITTING == GROUND_PROFILE_DERIVATOR_CONSECUTIVE_LINEAR_FITTING
    #error Unknown value for GROUND_PROFILE_DERIVATOR_FITTING
#endif  // GROUND_PROFILE_DERIVATOR_FITTING == GROUND_PROFILE_DERIVATOR_CONSECUTIVE_LINEAR_FITTING

#if IS_DO_HSC_LOGGING
    void log_horizontal_speed_compensation(int32_t heading, int32_t main_direction, int32_t heading_deviation,
        float horizontal_speed_compensation_factor, float horiz_speed_before, float horiz_speed_after);
#endif // IS_DO_HSC_LOGGING

protected:

#if IS_DO_GPD2_DEBUGGING_LOGGING
    int call_gpd2_log_counter = 0;                  // for reducing GDP2 logging from 400 to 100 Hz
#endif // IS_DO_GPD2_DEBUGGING_LOGGING

private:

    AC_GroundProfileAcquisition *ground_profile_acquisition = nullptr;        // use reference or pointer?
};


#if IS_RUN_GROUND_PROFILE_DERIVATOR_TESTS 
// use this like an FFC controller, only for testing the GPD
class AC_GroundProfileDerivatorTester {

public:

    AC_GroundProfileDerivatorTester(AC_GroundProfileDerivator *_ground_profile_derivator) {
        ground_profile_derivator = _ground_profile_derivator;
    }
    // void log_profile_derivations(Vector3f position_neu_cm, float horiz_speed,
        // AC_GroundProfileDerivator::DistanceDerivations derivations);
#if IS_RUN_GROUND_PROFILE_DERIVATOR_TESTS
    bool test_using_gpa(Vector3f position_neu_cm, float horiz_speed, int32_t heading, bool is_log);       // run GPD as the FFC would
#endif // IS_RUN_GROUND_PROFILE_DERIVATOR_TESTS

protected:

    AC_GroundProfileDerivator *ground_profile_derivator = nullptr;

private:

};
#endif // IS_RUN_GROUND_PROFILE_DERIVATOR_TESTS

class AC_FeedForwardController {

public:
    AC_FeedForwardController(AC_GroundProfileDerivator *_ground_profile_derivator) {
        ground_profile_derivator = _ground_profile_derivator;
    }

    float get_throttle_from_thrust(float thrust_N);     // inverse motor control function f_m^-1
    float get_thrust_from_throttle(float throttle);     // motor control function f_m
    float get_thrust_output_from_derivations(AC_GroundProfileDerivator::DistanceDerivations altitude_over_ground_derivations);
    // float get_throttle_output(AC_GroundProfileDerivator::DistanceDerivations altitude_over_ground_derivations);
    float get_thrust_output(void);                      // use inherent derivation data
#if IS_VERBOSE_THROTTLE_LOGGING_FFC
    void log_throttle(float throttle_out, float throttle_pid, float throttle_ffc, bool is_ffc_active);
#endif // IS_VERBOSE_THROTTLE_LOGGING_FFC
    inline AC_GroundProfileDerivator *get_gpd(void) {return ground_profile_derivator;}
    void update_last_derivation(AC_GroundProfileDerivator::DistanceDerivations new_derivations);

    const float copter_mass = PHYSICAL_MODEL_COPTER_MASS;                                   // [g]
    const float copter_time_const = PHYSICAL_MODEL_TIME_CONSTANT_MICROS;                    // [us]
    const float copter_air_resist_const = PHYSICAL_MODEL_SIMPLIFIED_AIR_RESISTANCE_CONST;   // [1e6 1/s] = [1/us]
    const float copter_gravitation_const = PHYSICAL_MODEL_GRAVITATION_CONST;                // [cm/s/s]

protected:
    AC_GroundProfileDerivator *ground_profile_derivator = nullptr;  // also contains GroundProfileAcquisition instance
    AC_GroundProfileDerivator::DistanceDerivations last_derivations = {
        DERIVATIONS_NO_DATA_INIT_VALUE, DERIVATIONS_NO_DATA_INIT_VALUE, DERIVATIONS_NO_DATA_INIT_VALUE, false};
    uint64_t last_derivations_update = 0;                           // time of last update for last_derivatios [us]

private:
};
