#include "Copter.h"

// return barometric altitude in centimeters
void Copter::read_barometer(void)
{
    barometer.update();

    baro_alt = barometer.get_altitude() * 100.0f;
    baro_climbrate = barometer.get_climb_rate() * 100.0f;

    motors->set_air_density_ratio(barometer.get_air_density_ratio());
}

void Copter::init_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
   rangefinder.init();
   rangefinder_state.alt_cm_filt.set_cutoff_frequency(RANGEFINDER_WPNAV_FILT_HZ);
   rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);
#endif
}

// return rangefinder altitude in centimeters
void Copter::read_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.update();

    if (rangefinder.num_sensors() > 0 &&
        should_log(MASK_LOG_CTUN)) {
        DataFlash.Log_Write_RFND(rangefinder);
    }

    // PSt: choosing the 1 and only particular rangefinder happens here
    //  downward rangefinder is hardcoded with ROTATION_PITCH_270

    rangefinder_state.alt_healthy = ((rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::RangeFinder_Good) && (rangefinder.range_valid_count_orient(ROTATION_PITCH_270) >= RANGEFINDER_HEALTH_MAX));

    int16_t temp_alt = rangefinder.distance_cm_orient(ROTATION_PITCH_270);

 #if RANGEFINDER_TILT_CORRECTION == ENABLED
    // correct alt for angle of the rangefinder
    temp_alt = (float)temp_alt * MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);
 #endif

    // PSt: rangefinder_state carries only the state of the downward facing rangefinder (tilt compensated)
    rangefinder_state.alt_cm = temp_alt;

 #if IS_ENABLE_SECOND_RANGEFINDER
    // added by PeterSt - do tilt compensation for forward facing rangefinder
    rangefinder2_state.dist_healthy = (
        (rangefinder.status_orient(RANGEFINDER_ORIENTATION_FORWARD_FACING) == RangeFinder::RangeFinder_Good)
        && (rangefinder.range_valid_count_orient(RANGEFINDER_ORIENTATION_FORWARD_FACING) >= RANGEFINDER_HEALTH_MAX));

    int16_t temp_dist_rangefinder2 = rangefinder.distance_cm_orient(RANGEFINDER_ORIENTATION_FORWARD_FACING);

  #if (RANGEFINDER_TILT_CORRECTION == ENABLED) && IS_DO_TILT_COMPENSATION_SECOND_RANGEFINDER
        // correct alt for angle of the rangefinder
        // compensate maximum 45° tilt (sin 45° = 1/sqrt(2) ~== 0.707)
        temp_dist_rangefinder2 = (float)temp_dist_rangefinder2 * MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);
  #endif // (RANGEFINDER_TILT_CORRECTION == ENABLED) && IS_DO_TILT_COMPENSATION_SECOND_RANGEFINDER

    rangefinder2_state.dist_cm = temp_dist_rangefinder2;
 #endif // IS_ENABLE_SECOND_RANGEFINDER

    // filter rangefinder for use by AC_WPNav
    // PSt comment: don't need to do that for second rangefinder, as it wont't be used for AC_WPNav
    uint32_t now = AP_HAL::millis();

    if (rangefinder_state.alt_healthy) {
        if (now - rangefinder_state.last_healthy_ms > RANGEFINDER_TIMEOUT_MS) {
            // reset filter if we haven't used it within the last second
            rangefinder_state.alt_cm_filt.reset(rangefinder_state.alt_cm);
        } else {
            rangefinder_state.alt_cm_filt.apply(rangefinder_state.alt_cm, 0.05f);
        }
        rangefinder_state.last_healthy_ms = now;
    }

    // send rangefinder altitude and health to waypoint navigation library
    wp_nav->set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());

#else
    rangefinder_state.enabled = false;
    rangefinder_state.alt_healthy = false;
    rangefinder_state.alt_cm = 0;

    // code for 2nd rangefinder added by PeterSt
 #if IS_ENABLE_SECOND_RANGEFINDER
    rangefinder2_state.enabled = false;
    rangefinder2_state.dist_healthy = false;
    rangefinder2_state.dist_cm = 0;
 #endif // IS_ENABLE_SECOND_RANGEFINDER

#endif // RANGEFINDER_ENABLED == ENABLED
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_alt_ok()
{
    return (rangefinder_state.enabled && rangefinder_state.alt_healthy);
}

// Ground Profile Acquisition related function by PeterSt

void Copter::update_ground_profile_acquisition(void) {
#if IS_GROUND_PROFILE_ACQUISITION_ENABLED
    call_update_gpa_counter++;
    // GPA is only necessary in MEASUREMENT flightmode
    if (copter.control_mode != control_mode_t::MEASUREMENT) {
        return;
    }
    // NEU relative to home position ("absolute position" with origin in home position, in contrast to
    //  altitude over ground), all in cm
    Vector3f position_neu;
    position_neu = inertial_nav.get_position();

    // start Ground Profile Acquisition, if not started yet
    if (!is_started_ground_profile_acquisition) {
        copter.ground_profile_acquisition->start(ahrs.yaw_sensor, position_neu);
        is_started_ground_profile_acquisition = true;

        #if IS_DEBUG_GPA
        gcs().send_text(MAV_SEVERITY_DEBUG, "GPA start. main_direction: %hu c°",
            copter.ground_profile_acquisition->get_main_direction());
        #endif // IS_DEBUG_GPA
    }

    // CONTINUE HERE
    #if !IS_TEST_FFC
        #error "not implemented yet"
    #endif // IS_TEST_FFC

    #if IS_PRINT_GPA_TESTS
        if (copter.call_update_gpa_counter % (PRINT_MESSAGE_VALUE_INTERVAL * CALL_FREQUENCY_UPDATE_GPA) == 1) {
            hal.console->printf("position_neu: x: %8f, y: %8f, z: %8f\n", 
                position_neu.x, position_neu.y, position_neu.z);
        }
    #endif // IS_PRINT_GPA_TESTS

    #if IS_PRINT_GPA_MAIN_DIRECTION_COO
    if (copter.call_update_gpa_counter % (PRINT_MESSAGE_VALUE_INTERVAL * CALL_FREQUENCY_UPDATE_GPA) == 1) {
        //int x_f, y_f;
        Vector2<int> main_direction_coo;
        main_direction_coo = copter.ground_profile_acquisition->get_main_direction_coo(position_neu);
        gcs().send_text(MAV_SEVERITY_DEBUG, "main_direction coo: x_f: %d, y_f: %d",
            main_direction_coo.x, main_direction_coo.y);
    }
    #endif // IS_PRINT_GPA_MAIN_DIRECTION_COO

    #if IS_PRINT_GROUND_PROFILE_ACQUISITION_MAP
        if (copter.call_run_counter % (PRINT_GPA_MAP_INTERVAL * CALL_FREQUENCY_MEASUREMENT_RUN) == 1) {
            hal.console->printf("map, ground_profile: [");
            int i;
            for (i = 0; i < GROUND_PROFILE_ACQUISITION_PROFILE_ARRAY_SIZE; i++) {
                hal.console->printf("%hd, ", copter.ground_profile_acquisition->get_ground_profile_datum(i));
            }
            hal.console->printf("]\n\n");
        }
    #endif // IS_PRINT_GROUND_PROFILE_ACQUISITION_MAP

    #if IS_PRINT_GPA_MAP_AS_MESSAGE
    if (copter.call_update_gpa_counter % (PRINT_GPA_MAP_INTERVAL * CALL_FREQUENCY_UPDATE_GPA) == 1) {
        #if IS_PRINT_GPA_MAP_CONDENSED
        gcs().send_text(MAV_SEVERITY_INFO, "sending GPA map as hex");
        gcs().send_text(MAV_SEVERITY_INFO, "map is biased with %02X, 00 means empty [", GPA_MAP_CONDENSED_BIAS);
        // print only first PRINT_GPA_MAP_UNTIL_INDEX cm
        int i;
        for (i = 0; i < PRINT_GPA_MAP_UNTIL_INDEX; i+=10) {
            // 50 chars can be displayed in mavlink message
            gcs().send_text(MAV_SEVERITY_INFO, "%04d: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X%s",
                i,
                BIASED_GPA_VALUE(copter.ground_profile_acquisition->get_ground_profile_datum(i)),
                BIASED_GPA_VALUE(copter.ground_profile_acquisition->get_ground_profile_datum(i+1)),
                BIASED_GPA_VALUE(copter.ground_profile_acquisition->get_ground_profile_datum(i+2)),
                BIASED_GPA_VALUE(copter.ground_profile_acquisition->get_ground_profile_datum(i+3)),
                BIASED_GPA_VALUE(copter.ground_profile_acquisition->get_ground_profile_datum(i+4)),
                BIASED_GPA_VALUE(copter.ground_profile_acquisition->get_ground_profile_datum(i+5)),
                BIASED_GPA_VALUE(copter.ground_profile_acquisition->get_ground_profile_datum(i+6)),
                BIASED_GPA_VALUE(copter.ground_profile_acquisition->get_ground_profile_datum(i+7)),
                BIASED_GPA_VALUE(copter.ground_profile_acquisition->get_ground_profile_datum(i+8)),
                BIASED_GPA_VALUE(copter.ground_profile_acquisition->get_ground_profile_datum(i+9)),
                i < PRINT_GPA_MAP_UNTIL_INDEX-10 ? ";" : "]");
        }
        #else // IS_PRINT_GPA_MAP_CONDENSED
        gcs().send_text(MAV_SEVERITY_INFO, "sending GPA map [");
        // print only first PRINT_GPA_MAP_UNTIL_INDEX cm
        int i;
        for (i = 0; i < PRINT_GPA_MAP_UNTIL_INDEX; i+=5) {
            // 50 chars can be displayed in mavlink message
            gcs().send_text(MAV_SEVERITY_INFO, "%+4d, %+4d, %+4d, %+4d, %+4d%s",
                copter.ground_profile_acquisition->get_ground_profile_datum(i),
                copter.ground_profile_acquisition->get_ground_profile_datum(i+1),
                copter.ground_profile_acquisition->get_ground_profile_datum(i+2),
                copter.ground_profile_acquisition->get_ground_profile_datum(i+3),
                copter.ground_profile_acquisition->get_ground_profile_datum(i+4),
                i < PRINT_GPA_MAP_UNTIL_INDEX-5 ? ", " : "]");
        }
        #endif // IS_PRINT_GPA_MAP_CONDENSED
    }
    #endif // IS_PRINT_GPA_MAP_AS_MESSAGE
    if (copter.rangefinder2_state.dist_healthy) {
        last_scan_point_return_value = copter.ground_profile_acquisition->scan_point(
            copter.rangefinder2_state.dist_cm, position_neu);
        #if IS_SEND_MESSAGE_IF_GPA_NOT_SUCCESSFUL
        if (!copter.ground_profile_acquisition->is_scan_point_index_valid(last_scan_point_return_value)) {
                copter.mode_measurement.handle_invalid_ground_profile_acquisition_index(last_scan_point_return_value);
        }
        #endif // IS_SEND_MESSAGE_IF_GPA_NOT_SUCCESSFUL
        
        #if IS_USE_GPA_MAP_FREEZE_MODE
        // freeze GPA if map is full
        if (last_scan_point_return_value == 
                AC_GroundProfileAcquisition::ScanPointInvalidReturnValue_GROUND_PROFILE_INDEX_TOO_HIGH) {
            if (!copter.ground_profile_acquisition->is_frozen()) {
                gcs().send_text(MAV_SEVERITY_WARNING, "GPAMap is full, freezing GPA!");
                copter.ground_profile_acquisition->set_freeze_map(true);
            }
        }
        #endif // IS_USE_GPA_MAP_FREEZE_MODE
    } else {
        #if IS_LOG_GPA
        // this is actually not within ground_profile_acquisition->scan_point, because the method doesn't get called
        //  if fwd rangefinder is not healthy, but there is a scan_point-return value especially for this case
        ground_profile_acquisition->scan_point_unhealthy_fwd_rangefinder(
            copter.rangefinder2_state.dist_cm, position_neu);
        #endif 
    }

    #if IS_PRINT_GPA_NEW_POINT
    // messages too slow!
     #if 0
    gcs().send_text(MAV_SEVERITY_DEBUG, "Last scanned point at %" PRIu32 "", 
        copter.ground_profile_acquisition->last_scanned_point.time_us);
    gcs().send_text(MAV_SEVERITY_DEBUG, "in: fwd: %3hd cm; pos: x: %4.1f, y: %4.1f, z: %4.1f",
    //gcs().send_text(MAV_SEVERITY_DEBUG, "in:fwd: %3hd; pos:x: %4f,y: %4f,z: %4f",
        copter.ground_profile_acquisition->last_scanned_point.fwd_rangefinder_dist_cm,
        copter.ground_profile_acquisition->last_scanned_point.position_neu_cm.x,
        copter.ground_profile_acquisition->last_scanned_point.position_neu_cm.y,
        copter.ground_profile_acquisition->last_scanned_point.position_neu_cm.z);
    gcs().send_text(MAV_SEVERITY_DEBUG, "out: x_f: %3hd cm; y_p: %3d cm; z_f: %3hd cm",
        copter.ground_profile_acquisition->last_scanned_point.x_f,
        copter.ground_profile_acquisition->last_scanned_point.y_p,
        copter.ground_profile_acquisition->last_scanned_point.z_f);
     #else // 0 or 1
    hal.console->printf("Last scanned point at %" PRIu32 "\n", 
        copter.ground_profile_acquisition->last_scanned_point.time_us);
    hal.console->printf("in: fwd: %3hd cm; pos: x: %4.1f, y: %4.1f, z: %4.1f\n",
    //gcs().send_text(MAV_SEVERITY_DEBUG, "in:fwd: %3hd; pos:x: %4f,y: %4f,z: %4f",
        copter.ground_profile_acquisition->last_scanned_point.fwd_rangefinder_dist_cm,
        copter.ground_profile_acquisition->last_scanned_point.position_neu_cm.x,
        copter.ground_profile_acquisition->last_scanned_point.position_neu_cm.y,
        copter.ground_profile_acquisition->last_scanned_point.position_neu_cm.z);
    hal.console->printf("out: x_f: %3hd cm; y_p: %3d cm; z_f: %3hd cm\n",
        copter.ground_profile_acquisition->last_scanned_point.x_f,
        copter.ground_profile_acquisition->last_scanned_point.y_p,
        copter.ground_profile_acquisition->last_scanned_point.z_f);
     #endif // 0
    #endif // IS_PRINT_GPA_NEW_POINT

    // no need for dwn rangefinder
#endif // IS_GROUND_PROFILE_ACQUISITION_ENABLED
}

/*
  update RPM sensors
 */
void Copter::rpm_update(void)
{
#if RPM_ENABLED == ENABLED
    rpm_sensor.update();
    if (rpm_sensor.enabled(0) || rpm_sensor.enabled(1)) {
        if (should_log(MASK_LOG_RCIN)) {
            DataFlash.Log_Write_RPM(rpm_sensor);
        }
    }
#endif
}

// initialise compass
void Copter::init_compass()
{
    if (!g.compass_enabled) {
        return;
    }

    if (!compass.init() || !compass.read()) {
        // make sure we don't pass a broken compass to DCM
        hal.console->printf("COMPASS INIT ERROR\n");
        Log_Write_Error(ERROR_SUBSYSTEM_COMPASS,ERROR_CODE_FAILED_TO_INITIALISE);
        return;
    }
    ahrs.set_compass(&compass);
}

/*
  if the compass is enabled then try to accumulate a reading
  also update initial location used for declination
 */
void Copter::compass_accumulate(void)
{
    if (!g.compass_enabled) {
        return;
    }

    compass.accumulate();

    // update initial location used for declination
    if (!ap.compass_init_location) {
        Location loc;
        if (ahrs.get_position(loc)) {
            compass.set_initial_location(loc.lat, loc.lng);
            ap.compass_init_location = true;
        }
    }
}

// initialise optical flow sensor
void Copter::init_optflow()
{
#if OPTFLOW == ENABLED
    // initialise optical flow sensor
    optflow.init();
#endif      // OPTFLOW == ENABLED
}

// called at 200hz
#if OPTFLOW == ENABLED
void Copter::update_optical_flow(void)
{
    static uint32_t last_of_update = 0;

    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }

    // read from sensor
    optflow.update();

    // write to log and send to EKF if new data has arrived
    if (optflow.last_update() != last_of_update) {
        last_of_update = optflow.last_update();
        uint8_t flowQuality = optflow.quality();
        Vector2f flowRate = optflow.flowRate();
        Vector2f bodyRate = optflow.bodyRate();
        const Vector3f &posOffset = optflow.get_pos_offset();
        ahrs.writeOptFlowMeas(flowQuality, flowRate, bodyRate, last_of_update, posOffset);
        if (g.log_bitmask & MASK_LOG_OPTFLOW) {
            Log_Write_Optflow();
        }
    }
}
#endif  // OPTFLOW == ENABLED

void Copter::compass_cal_update()
{
    static uint32_t compass_cal_stick_gesture_begin = 0;

    if (!hal.util->get_soft_armed()) {
        compass.compass_cal_update();
    }

    if (compass.is_calibrating()) {
        if (channel_yaw->get_control_in() < -4000 && channel_throttle->get_control_in() > 900) {
            compass.cancel_calibration_all();
        }
    } else {
        bool stick_gesture_detected = compass_cal_stick_gesture_begin != 0 && !motors->armed() && channel_yaw->get_control_in() > 4000 && channel_throttle->get_control_in() > 900;
        uint32_t tnow = millis();

        if (!stick_gesture_detected) {
            compass_cal_stick_gesture_begin = tnow;
        } else if (tnow-compass_cal_stick_gesture_begin > 1000*COMPASS_CAL_STICK_GESTURE_TIME) {
#ifdef CAL_ALWAYS_REBOOT
            compass.start_calibration_all(true,true,COMPASS_CAL_STICK_DELAY,true);
#else
            compass.start_calibration_all(true,true,COMPASS_CAL_STICK_DELAY,false);
#endif
        }
    }
}

void Copter::accel_cal_update()
{
    if (hal.util->get_soft_armed()) {
        return;
    }
    ins.acal_update();
    // check if new trim values, and set them
    float trim_roll, trim_pitch;
    if(ins.get_new_trim(trim_roll, trim_pitch)) {
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }

#ifdef CAL_ALWAYS_REBOOT
    if (ins.accel_cal_requires_reboot()) {
        hal.scheduler->delay(1000);
        hal.scheduler->reboot(false);
    }
#endif
}

// initialise proximity sensor
void Copter::init_proximity(void)
{
#if PROXIMITY_ENABLED == ENABLED
    g2.proximity.init();
    g2.proximity.set_rangefinder(&rangefinder);
#endif
}

// update error mask of sensors and subsystems. The mask
// uses the MAV_SYS_STATUS_* values from mavlink. If a bit is set
// then it indicates that the sensor or subsystem is present but
// not functioning correctly.
void Copter::update_sensor_status_flags(void)
{
    // default sensors present
    control_sensors_present = MAVLINK_SENSOR_PRESENT_DEFAULT;

    // first what sensors/controllers we have
    if (g.compass_enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG; // compass present
    }
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#if OPTFLOW == ENABLED
    if (optflow.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
#if PRECISION_LANDING == ENABLED
    if (precland.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    if (g2.visual_odom.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif
    if (ap.rc_receiver_present) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }
    if (copter.DataFlash.logging_present()) { // primary logging only (usually File)
        control_sensors_present |= MAV_SYS_STATUS_LOGGING;
    }
#if PROXIMITY_ENABLED == ENABLED
    if (copter.g2.proximity.sensor_present()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
#endif
#if AC_FENCE == ENABLED
    if (copter.fence.sys_status_present()) {
        control_sensors_present |= MAV_SYS_STATUS_GEOFENCE;
    }
#endif
#if RANGEFINDER_ENABLED == ENABLED
    if (rangefinder.has_orientation(ROTATION_PITCH_270)) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    }
#endif

    // all sensors are present except these, which may be set as enabled below:
    control_sensors_enabled = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL &
                                                         ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL &
                                                         ~MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS &
                                                         ~MAV_SYS_STATUS_LOGGING &
                                                         ~MAV_SYS_STATUS_SENSOR_BATTERY &
                                                         ~MAV_SYS_STATUS_GEOFENCE &
                                                         ~MAV_SYS_STATUS_SENSOR_LASER_POSITION &
                                                         ~MAV_SYS_STATUS_SENSOR_PROXIMITY);

    switch (control_mode) {
    case AUTO:
    case AVOID_ADSB:
    case GUIDED:
    case LOITER:
    case RTL:
    case CIRCLE:
    case LAND:
    case POSHOLD:
    case BRAKE:
    case THROW:
    case SMART_RTL:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        break;
    case ALT_HOLD:
    case GUIDED_NOGPS:
    case SPORT:
    case AUTOTUNE:
    case FLOWHOLD:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        break;
    default:
        // stabilize, acro, drift, and flip have no automatic x,y or z control (i.e. all manual)
        break;
    }

    // set motors outputs as enabled if safety switch is not disarmed (i.e. either NONE or ARMED)
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }

    if (copter.DataFlash.logging_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_LOGGING;
    }

    if (battery.num_instances() > 0) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_BATTERY;
    }
#if AC_FENCE == ENABLED
    if (copter.fence.sys_status_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_GEOFENCE;
    }
#endif
#if PROXIMITY_ENABLED == ENABLED
    if (copter.g2.proximity.sensor_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
#endif


    // default to all healthy
    control_sensors_health = control_sensors_present;

    if (!barometer.all_healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    }
    if (!g.compass_enabled || !compass.healthy() || !ahrs.use_compass()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (!gps.is_healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_GPS;
    }
    if (!ap.rc_receiver_present || failsafe.radio) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }
#if OPTFLOW == ENABLED
    if (!optflow.healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
#if PRECISION_LANDING == ENABLED
    if (precland.enabled() && !precland.healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    if (g2.visual_odom.enabled() && !g2.visual_odom.healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif
    if (!ins.get_gyro_health_all() || !ins.gyro_calibrated_ok_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_GYRO;
    }
    if (!ins.get_accel_health_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    }

    if (ahrs.initialised() && !ahrs.healthy()) {
        // AHRS subsystem is unhealthy
        control_sensors_health &= ~MAV_SYS_STATUS_AHRS;
    }

    if (copter.DataFlash.logging_failed()) {
        control_sensors_health &= ~MAV_SYS_STATUS_LOGGING;
    }

#if PROXIMITY_ENABLED == ENABLED
    if (copter.g2.proximity.sensor_failed()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
#endif

#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    switch (terrain.status()) {
    case AP_Terrain::TerrainStatusDisabled:
        break;
    case AP_Terrain::TerrainStatusUnhealthy:
        // To-Do: restore unhealthy terrain status reporting once terrain is used in copter
        //control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        //control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        //break;
    case AP_Terrain::TerrainStatusOK:
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_health  |= MAV_SYS_STATUS_TERRAIN;
        break;
    }
#endif

#if RANGEFINDER_ENABLED == ENABLED
    if (rangefinder_state.enabled) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (!rangefinder.has_data_orient(ROTATION_PITCH_270)) {
            control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
    }
#endif

    if (!ap.initialised || ins.calibrating()) {
        // while initialising the gyros and accels are not enabled
        control_sensors_enabled &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
        control_sensors_health &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
    }

    if (!copter.battery.healthy() || copter.battery.has_failsafed()) {
         control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_BATTERY;
    }
#if AC_FENCE == ENABLED
    if (copter.fence.sys_status_failed()) {
        control_sensors_health &= ~MAV_SYS_STATUS_GEOFENCE;
    }
#endif

#if FRSKY_TELEM_ENABLED == ENABLED
    // give mask of error flags to Frsky_Telemetry
    frsky_telemetry.update_sensor_status_flags(~control_sensors_health & control_sensors_enabled & control_sensors_present);
#endif
}

// init visual odometry sensor
void Copter::init_visual_odom()
{
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    g2.visual_odom.init();
#endif
}

// update visual odometry sensor
void Copter::update_visual_odom()
{
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    // check for updates
    if (g2.visual_odom.enabled() && (g2.visual_odom.get_last_update_ms() != visual_odom_last_update_ms)) {
        visual_odom_last_update_ms = g2.visual_odom.get_last_update_ms();
        float time_delta_sec = g2.visual_odom.get_time_delta_usec() / 1000000.0f;
        ahrs.writeBodyFrameOdom(g2.visual_odom.get_confidence(),
                                g2.visual_odom.get_position_delta(),
                                g2.visual_odom.get_angle_delta(),
                                time_delta_sec,
                                visual_odom_last_update_ms,
                                g2.visual_odom.get_pos_offset());
        // log sensor data
        DataFlash.Log_Write_VisualOdom(time_delta_sec,
                                       g2.visual_odom.get_angle_delta(),
                                       g2.visual_odom.get_position_delta(),
                                       g2.visual_odom.get_confidence());
    }
#endif
}

// winch and wheel encoder initialisation
void Copter::winch_init()
{
#if WINCH_ENABLED == ENABLED
    g2.wheel_encoder.init();
    g2.winch.init(&g2.wheel_encoder);
#endif
}

// winch and wheel encoder update
void Copter::winch_update()
{
#if WINCH_ENABLED == ENABLED
    g2.wheel_encoder.update();
    g2.winch.update();
#endif
}
