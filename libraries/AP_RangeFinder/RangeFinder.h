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
    // start scanning with the current orientation and set position_neu_cm as reference point for absolute position
    bool start(uint16_t _heading, Vector3f position_neu_cm);                      
    #if 0 
    bool start(int _heading, Vector3f position_neu_cm);
    #endif
    int scan_point(int16_t fwd_rangefinder_dist_cm, Vector3f position_neu_cm);
    // int scan_point(int16_t dwn_rangefinder_dist_cm, int16_t fwd_rangefinder_dist_cm, Vector3f position_neu_cm);

    //int get_1d_x(Vector3f position_neu_cm);
    Vector2<int> get_main_direction_coo(Vector3f position_neu_cm);
    // get first 3 derivations of ground profile at position_neu_cm
    //  using IS_SMOOTHEN_GROUND_PROFILE_DERIVATION_VALUES
    // TODO: think about return value: int or float?
    //  internally use int, because it is faster
    Vector3f get_profile_derivations(Vector3f position_neu_cm, float horiz_speed); // TODO: implement

    // note that these enums must be negative, because any positive return value of scan_point(...) is
    //  considered valid
    enum ScanPointInvalidReturnValue {
        ScanPointInvalidReturnValue_NOT_INITIALIZED = -1,
        ScanPointInvalidReturnValue_GROUND_PROFILE_INDEX_NEGATIVE = -2,
        ScanPointInvalidReturnValue_GROUND_PROFILE_INDEX_TOO_HIGH = -3,
        ScanPointInvalidReturnValue_DEVIATION_FROM_MAIN_DIRECTION_EXCEEDED = -4,
        ScanPointInvalidReturnValue_VALUE_OUT_OF_RANGE = -5,
    };
    bool is_scan_point_index_valid(int scan_point_return_value) { return scan_point_return_value >= 0;}
    int16_t get_ground_profile_datum(int index) {return ground_profile[index];}

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

protected:

private:

    int16_t ground_profile[GROUND_PROFILE_ACQUISITION_PROFILE_ARRAY_SIZE];
    uint16_t main_direction = 36000;                                    // heading in centi degrees [c°]
    // int main_direction = 36000;                                    // heading in centi degrees [c°]
    // position_neu_cm of starting point, this will be 0 for ground_profile
    Vector3f start_position_cm;                                 
#if IS_USE_GPA_MAP_OFFSET
    int16_t ground_profile_offset = 0;                              // [cm] so that first value in map is 0
    bool is_ground_profile_offset_set = false;
#endif // IS_USE_GPA_MAP_OFFSET
};

#endif // 1 OR 0
#endif // IS_USE_WORKAROUND_GROUND_PROFILE_ACQUISITION && IS_USE_WORKAROUND_HOST_FILE_GPA