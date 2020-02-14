// New flightmode MEASUREMENT, added entire file (PeterSt)

#include "Copter.h"

/*
 * Init and run calls for guided_nogps flight mode
 */


// initialise guided_nogps controller
// TODO: prio 6: reconsider how to start
bool Copter::ModeMeasurement::init(bool ignore_checks)
{
    // TODO: check if there is a fwd facing rangefinder
    // TODO: init future ground profile detection
    // TODO: init anticipating altitude control

#if IS_TEST_MEASUREMENT_INSTANCE_INIT
    call_conter_Copter_ModeMeasurement_init++;
#endif // IS_TEST_MEASUREMENT_INSTANCE_INIT

#if IS_PRINT_ALT_CTRL_METHOD_IN_MEASUREMENT
    gcs().send_text(MAV_SEVERITY_INFO, "%s in " MEASUREMENT_NAME_OF_ALTITUDE_CONTROL_MODE " alt ctrl mode", name4());
#endif // IS_PRINT_ALT_CTRL_METHOD_IN_MEASUREMENT

    bool ret = false;

#if MEASUREMENT_FLIGHTMODE_BEHAVIOR == MEASUREMENT_BEHAVIOR_LOITER
    ret = Copter::ModeLoiter::init(ignore_checks);
    #if IS_DEBUG_MAX_HORIZONTAL_SPEED
    hal.console->printf("set_speed_xy from ModeMeasurement::init()\n");
    #endif // IS_DEBUG_MAX_HORIZONTAL_SPEED
    pos_control->set_speed_xy((float) MAX_MEASUREMENT_HORIZONTAL_SPEED);
#elif MEASUREMENT_FLIGHTMODE_BEHAVIOR == MEASUREMENT_BEHAVIOR_SEMI_GUIDED
    #error This behavior for flightmode MEASUREMENT is not implemented
    // pos_control->set_speed_xy(MAX_MEASUREMENT_HORIZONTAL_SPEED);
    // use wp_nav->set_speed_xy ?
#elif MEASUREMENT_FLIGHTMODE_BEHAVIOR == MEASUREMENT_BEHAVIOR_GUIDED
    #error This behavior for flightmode MEASUREMENT is not implemented
    // start in angle control mode
    // TODO: really start in this mode angle control mode?
    // pos_control->set_speed_xy(MAX_MEASUREMENT_HORIZONTAL_SPEED);
    // use wp_nav->set_speed_xy ?
    Copter::ModeGuided::angle_control_start();
#else 
    #error This behavior for flightmode MEASUREMENT is not implemented
#endif // MEASUREMENT_FLIGHTMODE_BEHAVIOR == MEASUREMENT_BEHAVIOR_LOITER 

#if MEASUREMENT_ALTITUDE_CONTROL_MODE == ALT_CTRL_MODE_FFC
    // init FFC
    #if !IS_TEST_FFC
        // for now: only for testing
        #error not implemented yet
    #endif // !IS_TEST_FFC

    copter.ground_profile_acquisition = new AC_GroundProfileAcquisition();
    if (copter.ground_profile_acquisition == nullptr) {
        AP_HAL::panic("Unable to allocate GroundProfileAcquisition");
    }

    copter.last_scan_point_return_value = AC_GroundProfileAcquisition::ScanPointInvalidReturnValue_NOT_INITIALIZED;
    copter.ground_profile_acquisition->init();

    copter.is_started_ground_profile_acquisition = false;      // force restart after fresh switch to MEASUREMENT
    
    // init GPD
    #if IS_VERBOSE_DEBUG_GPD
    printf("MEAS line %d ok.\n", __LINE__);
    #endif // IS_VERBOSE_DEBUG_GPD
    copter.ground_profile_derivator = new AC_GroundProfileDerivator(copter.ground_profile_acquisition);
    if (copter.ground_profile_derivator == nullptr) {
        AP_HAL::panic("Unable to allocate GroundProfileDerivator");
    }
    #if IS_VERBOSE_DEBUG_GPD
    printf("MEAS line %d ok.\n", __LINE__);
    if (copter.ground_profile_derivator == nullptr) {
        hal.console->printf("init copter.ground_profile_derivator didn't work!!!\n");
    } else {
        hal.console->printf("init copter.ground_profile_derivator ok.\n");  // ok
    }
    #endif // IS_VERBOSE_DEBUG_GPD

    // init GroundProfileDerivatorTester
    #if IS_RUN_GROUND_PROFILE_DERIVATOR_TESTS
    copter.ground_profile_derivator_tester = new AC_GroundProfileDerivatorTester(copter.ground_profile_derivator);
    if (copter.ground_profile_derivator_tester == nullptr) {
        AP_HAL::panic("Unable to allocate GroundProfileDerivatorTester");
    }

     #if IS_VERBOSE_DEBUG_GPD
    printf("MEAS line %d ok.\n", __LINE__);
    if (copter.ground_profile_derivator_tester == nullptr) {
        hal.console->printf("init copter.ground_profile_derivator_tester didn't work!!!\n");
    } else {
        hal.console->printf("init copter.ground_profile_derivator_tester ok.\n");   // ok
    }
    printf("MEAS line %d ok.\n", __LINE__);
     #endif // IS_VERBOSE_DEBUG_GPD
    #endif // IS_RUN_GROUND_PROFILE_DERIVATOR_TESTS

    // init FFC

    copter.pos_control->set_ffc(new AC_FeedForwardController(copter.ground_profile_derivator));
    if (copter.pos_control->get_ffc() == nullptr) {
        AP_HAL::panic("Unable to allocate FeedForwardController");
    }

    // set rangefinder_state pointer to pos_control, because FFC needs it and is called from there
    copter.pos_control->set_rangefinder_state_alt_cm_ptr(&(copter.rangefinder_state.alt_cm));
    // impossible, because we can't get an address of a bitfield:
    // copter.pos_control->set_rangefinder_state(&(copter.rangefinder_state.alt_cm), &(copter.rangefinder_state.alt_healthy))

#if IS_FFC_ENABLED
    // #error CONTINUE HERE !!
    // TODO: prio 7: start working ffc from scheduler
 #if IS_LOG_FFC_THRUST_CURTAILMENTS
    // do "FFC2" log once
    copter.get_ffc()->log_ffc_thrust_curtailments_parameters(FFC_IS_ENABLE_THRUST_CAPPING, FFC_IS_ENABLE_ALTITUDE_SAFETY_THRUST_CURTAIL,
            FFC_THRUST_CAPPING_MIN_THRUST, FFC_THRUST_CAPPING_MAX_THRUST, 
            FFC_ALTITUDE_THRUST_CURTAIL_LOWER_THRESHOLD_CM, FFC_ALTITUDE_THRUST_CURTAIL_UPPER_THRESHOLD_CM);
 #endif // IS_LOG_FFC_THRUST_CURTAILMENTS
#endif // IS_FFC_ENABLED

#endif // MEASUREMENT_ALTITUDE_CONTROL_MODE == ALT_CTRL_MODE_FFC

#if IS_DO_GPD2_DEBUGGING_LOGGING
    gcs().send_text(MAV_SEVERITY_WARNING, "GDP2 logging enabled, very high rate!!");
#endif // IS_DO_GPD2_DEBUGGING_LOGGING
#if IS_USE_GPA_MAP_FREEZE_MODE 
    gcs().send_text(MAV_SEVERITY_WARNING, "GPA Map freeze mode is enabled!");
#endif // IS_USE_GPA_MAP_FREEZE_MODE 

    return ret;
}

// return values of AC_GroundProfileAcquisition::scan_point(...) must be >= 0, of not: handle return value
//  by sending a MAVLink message
// return: is this value known?
bool Copter::ModeMeasurement::handle_invalid_ground_profile_acquisition_index(int scan_point_return_value) {
    // first check timeout for invalid return value messages, to avoid spamming
    if (AP_HAL::micros() < send_message_scan_point_error_timeout) {
        // still in message timeout, check if scan_point_return_value is known
        switch (scan_point_return_value) {
        case AC_GroundProfileAcquisition::ScanPointInvalidReturnValue_NOT_INITIALIZED:
        case AC_GroundProfileAcquisition::ScanPointInvalidReturnValue_GROUND_PROFILE_INDEX_NEGATIVE:
        case AC_GroundProfileAcquisition::ScanPointInvalidReturnValue_GROUND_PROFILE_INDEX_TOO_HIGH:
        case AC_GroundProfileAcquisition::ScanPointInvalidReturnValue_DEVIATION_FROM_MAIN_DIRECTION_EXCEEDED:
        case AC_GroundProfileAcquisition::ScanPointInvalidReturnValue_GROUND_PROFILE_ACQUISITION_FROZEN:
            return true;
        default:
            return false;    
        }
    }

    switch (scan_point_return_value) {
    case AC_GroundProfileAcquisition::ScanPointInvalidReturnValue_NOT_INITIALIZED:
        gcs().send_text(MAV_SEVERITY_ERROR, "GPA: %d, variable uninitialized", 
            scan_point_return_value);
        break;
    case AC_GroundProfileAcquisition::ScanPointInvalidReturnValue_GROUND_PROFILE_INDEX_NEGATIVE:
        gcs().send_text(MAV_SEVERITY_ERROR, "GPA: %d, couldn't store point, index < 0", 
            scan_point_return_value);
        break;
    case AC_GroundProfileAcquisition::ScanPointInvalidReturnValue_GROUND_PROFILE_INDEX_TOO_HIGH:
        gcs().send_text(MAV_SEVERITY_ERROR, "GPA: %d, couldn't store point, index too high", 
            scan_point_return_value);
        break;
    case AC_GroundProfileAcquisition::ScanPointInvalidReturnValue_DEVIATION_FROM_MAIN_DIRECTION_EXCEEDED:
        gcs().send_text(MAV_SEVERITY_ERROR, "GPA: %d, didn't store point, deviation from main_direction too high", 
            scan_point_return_value);
        break;
    case AC_GroundProfileAcquisition::ScanPointInvalidReturnValue_GROUND_PROFILE_ACQUISITION_FROZEN:
        gcs().send_text(MAV_SEVERITY_WARNING, "GPA: frozen, didn't store point", 
            scan_point_return_value);
        break;
    case AC_GroundProfileAcquisition::ScanPointInvalidReturnValue_USE_GROUND_PROFILE_ACQUISITION_DATA_FROM_FILE:
        gcs().send_text(MAV_SEVERITY_WARNING, "GPA: using old GPA data from file", 
            scan_point_return_value);
        break;
    default:
        gcs().send_text(MAV_SEVERITY_ERROR, "GPA: %d, unknown error status", 
            scan_point_return_value);
        return false;
    }
    // here only if default case didn't match ==> here if scan_point_return_value was known
    // message has been sent ==> set timeout
    send_message_scan_point_error_timeout = AP_HAL::micros() + MESSAGE_IF_GPA_NOT_SUCCESSFUL_TIMEOUT_USEC;
    return true;
}

// for MEASUREMENT mode, run function copied from LOITER, with some features added
// should be called at 100hz or more
void Copter::ModeMeasurement::loiterlike_run()
{
    // begin    added by PeterSt
    // copter.call_run_counter++;                                  // for debug messages
    // loiter navi needs to now, if it's in MEASUREMENT flightmode, so we can restrict maximum horizontal speed
#if IS_OVERWRITE_LOIT_SPEED_IN_MEASUREMENT
    loiter_nav->is_measurement_mode = copter.control_mode == control_mode_t::MEASUREMENT;
#endif // IS_OVERWRITE_LOIT_SPEED_IN_MEASUREMENT
    // end

    LoiterModeState loiter_state;

    float target_roll, target_pitch;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speed and acceleration
    pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_accel_z(g.pilot_accel_z);

    // by PeterSt
    #if IS_RUN_GROUND_PROFILE_DERIVATOR_TESTS
    // to prevent floating point error
    // TODO: prio 8: fill or remove this
    #endif // IS_RUN_GROUND_PROFILE_DERIVATOR_TESTS

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    if (ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    if (!motors->armed() || !motors->get_interlock()) {
        loiter_state = Loiter_MotorStopped;
    } else if (takeoff.running() || takeoff.triggered(target_climb_rate)) {
        loiter_state = Loiter_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        loiter_state = Loiter_Landed;
    } else {
        loiter_state = Loiter_Flying;
    }

#if IS_DEBUG_MAX_HORIZONTAL_SPEED
    uint32_t _micros = AP_HAL::micros();
    if (IS_TRIGGER_EVENT_ROUGHLY_EVERY_N_SEC_MICROS(10, _micros, 400)) {
        hal.console->printf("MEAS: pos_control->get_speed_xy(): %f\n", pos_control->get_speed_xy());
    }
#endif // IS_DEBUG_MAX_HORIZONTAL_SPEED
#if IS_SEND_MESSAGE_LOIT_SPEED_IN_MEASUREMENT
    if (copter.call_run_counter % (PRINT_MESSAGE_VALUE_INTERVAL * CALL_FREQUENCY_MEASUREMENT_RUN) == 1) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "MEAS: pos_control->get_speed_xy(): %f\n", pos_control->get_speed_xy());
        //hal.console->printf("MEAS: pos_control->get_speed_xy(): %f\n", pos_control->get_speed_xy());
    }
#endif // IS_SEND_MESSAGE_LOIT_SPEED_IN_MEASUREMENT

    // Loiter State Machine
    switch (loiter_state) {

    case Loiter_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
#if FRAME_CONFIG == HELI_FRAME
        if (!motors->get_interlock() && ap.land_complete) {
            loiter_nav->init_target();
            attitude_control->reset_rate_controller_I_terms();
            attitude_control->set_yaw_target_to_current_heading();
            pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        } else {
        // force descent rate and call position controller
            pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
            if (ap.land_complete_maybe) {
                pos_control->relax_alt_hold_controllers(0.0f);
            }
        }
#else
        loiter_nav->init_target();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        loiter_nav->update(ekfGndSpdLimit, ekfNavVelGainScaler);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);
        pos_control->update_z_controller(false);
        break;

    case Loiter_Takeoff:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }

        // get takeoff adjusted pilot and takeoff climb rates
        takeoff.get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // run loiter controller
        loiter_nav->update(ekfGndSpdLimit, ekfNavVelGainScaler);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

        // update altitude target and call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control->update_z_controller(false);
        break;

    case Loiter_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }
        loiter_nav->init_target();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0);
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->update_z_controller(false);
        break;

    case Loiter_Flying:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if PRECISION_LANDING == ENABLED
        if (do_precision_loiter()) {
            precision_loiter_xy();
        }
#endif

        // run loiter controller
        loiter_nav->update(ekfGndSpdLimit, ekfNavVelGainScaler);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

        // adjust climb rate using rangefinder
        #if IS_PRINT_VALUE_LOITER_ALT_TARGET    // PeterSt added some debug printouts
            if (copter.call_run_counter % (PRINT_MESSAGE_VALUE_INTERVAL * CALL_FREQUENCY_MEASUREMENT_RUN) == 1) {
                hal.console->printf("%s pos_control->get_alt_target(): %f\n", name4(), pos_control->get_alt_target());
                hal.console->printf("target_climb_rate before:           %f\n", target_climb_rate);
            }
        #endif // IS_PRINT_VALUE_LOITER_ALT_TARGET
        // PSt: pos_control->get_alt_target() is probably an absolute altitude (ref to home position)
        //  adjust target_climb_rate according to altitude over ground
        target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
        
        #if IS_PRINT_VALUE_LOITER_ALT_TARGET    // PeterSt added some debug printouts
            if (copter.call_run_counter % (PRINT_MESSAGE_VALUE_INTERVAL * CALL_FREQUENCY_MEASUREMENT_RUN) == 1) {
                hal.console->printf("%s pos_control->get_alt_target(): %f\n", name4(), pos_control->get_alt_target());
                hal.console->printf("target_climb_rate after:            %f\n", target_climb_rate);
            }
        #endif // IS_PRINT_VALUE_LOITER_ALT_TARGET

        // PeterSt:
    #if IS_RUN_GROUND_PROFILE_DERIVATOR_TESTS && (MEASUREMENT_ALTITUDE_CONTROL_MODE == ALT_CTRL_MODE_FFC)
        // run GPD tests
        bool is_log_GPDTester, is_GPDTester_return_valid = false; 
     #if IS_VERBOSE_GPD_LOGGING
        is_log_GPDTester = true;
     #else // IS_VERBOSE_GPD_LOGGING
        // a lot of logs (whole GPA map eg.) ==> log only once per second for samples
        is_log_GPDTester = (copter.call_run_counter % (1 * CALL_FREQUENCY_MEASUREMENT_RUN)) == 1;
     #endif // IS_VERBOSE_GPD_LOGGING
     #if IS_VERBOSE_DEBUG_GPD
        // for verbose debugging: also log first call
        is_log_GPDTester = is_log_GPDTester || (copter.call_update_gpa_counter == 1);
     #endif

     #if IS_VERBOSE_DEBUG_GPD
            printf("mode_MEAS.cpp line %d ok.\n", __LINE__);  // ok
     #endif // IS_VERBOSE_DEBUG_GPD   
        
        float horiz_speed;
        int32_t heading;
        horiz_speed = inertial_nav.get_velocity_xy();   // [cm/s]
        heading = ahrs.yaw_sensor;                      // [cdeg]
        #if IS_REVERSE_FLIGHT_SPEED_CHECK_LOG_TESTER
        uint64_t _micros;
        _micros = AP_HAL::micros64();
        if (IS_TRIGGER_EVENT_ROUGHLY_EVERY_N_SEC_MICROS(1, _micros, 400)) {
            hal.console->printf("MEAS: before reverse flight check: horiz_speed: %f\n", horiz_speed);
        }
        #endif // 0
        #if IS_REVERSE_GPA_MAIN_DIRECTION               // declare vehicles "backward" as flying "forward"
            // velocity_xy seems to be positive, even when flying backwards
            // horiz_speed = -horiz_speed; // velocity_xy is always measured in vehicle-forward direction
            heading = copter.ground_profile_derivator->get_opposite_heading_cd(heading);
        #endif // IS_REVERSE_GPA_MAIN_DIRECTION
        #if IS_REVERSE_FLIGHT_SPEED_CHECK_LOG_TESTER
        _micros = AP_HAL::micros64();
        if (IS_TRIGGER_EVENT_ROUGHLY_EVERY_N_SEC_MICROS(1, _micros, 400)) {
            hal.console->printf("MEAS: after reverse flight check: horiz_speed: %f\n\n", horiz_speed);
        }
        #endif // 0
        
        #if IS_VERBOSE_DEBUG_GPD
            printf("mode_MEAS.cpp line %d ok.\n", __LINE__);  // ok
        #endif // IS_VERBOSE_DEBUG_GPD   
        is_GPDTester_return_valid = copter.ground_profile_derivator_tester->test_using_gpa(
            inertial_nav.get_position(), horiz_speed, heading, is_log_GPDTester);
        #if IS_VERBOSE_DEBUG_GPD
            printf("mode_MEAS.cpp line %d ok.\n", __LINE__);  // ???
        #endif // IS_VERBOSE_DEBUG_GPD   

     #if 0  // temporary debug
        printf("%d", is_GPDTester_return_valid);
     #else
        is_GPDTester_return_valid;      // have to use variable to prevent compiler error for unused variable
     #endif // 1
     #if IS_VERBOSE_DEBUG_GPD
        printf("mode_MEAS.cpp line %d ok.\n", __LINE__);  // not ok!!!
     #endif // IS_VERBOSE_DEBUG_GPD 

    #endif // IS_RUN_GROUND_PROFILE_DERIVATOR_TESTS

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update altitude target and call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        #if IS_FFC_ENABLED
            pos_control->update_z_controller(true);
        #else // IS_FFC_ENABLED
            pos_control->update_z_controller(false);
        #endif // IS_FFC_ENABLED
        break;
    }
}

// guided_run - runs the guided controller
// should be called at 100hz or more
// TODO: prio 6: reconsider how to run
//  add Anticipating AltCtrl here?
// PeterSt: according to http://ardupilot.org/dev/docs/apmcopter-adding-a-new-flight-mode.html
//  run() is called with 400 Hz
void Copter::ModeMeasurement::run()
{
    // float target_climb_rate = 0.0f;             // for altitude over ground
    // TODO: get some input? (eg. for setting altitude)
    copter.call_run_counter++;
    // call_run_counter++;
    #if (IS_PRINT_REPEATET_MESSAGE_IN_MEASUREMENT)
        // if (call_run_counter % (REPEATET_MESSAGE_IN_MEASUREMENT_INTERVAL * CALL_FREQUENCY_MEASUREMENT_RUN) == 1) {
        if (copter.call_run_counter % (REPEATET_MESSAGE_IN_MEASUREMENT_INTERVAL * CALL_FREQUENCY_MEASUREMENT_RUN) == 1) {
            gcs().send_text(MAV_SEVERITY_DEBUG, "Copter is in flightmode %s (27)", this->name());
            // print system time for checking intervals
            hal.console->printf("(%s:) Time since start: %" PRIu32 " us\n", this->name4(), AP_HAL::micros());
        }
        // TODO: CONTINUE HERE
    #endif

    #if IS_TEST_MEASUREMENT_INSTANCE_INIT
    if (copter.call_run_counter % (REPEATET_MESSAGE_IN_MEASUREMENT_INTERVAL * CALL_FREQUENCY_MEASUREMENT_RUN) == 1) {
        gcs().send_text(MAV_SEVERITY_DEBUG, "call_conter_Copter_ModeMeasurement_init: %d", 
            call_conter_Copter_ModeMeasurement_init);
        // print system time for checking intervals
        hal.console->printf("(%s:) Time since start: %" PRIu32 " us\n", this->name4(), AP_HAL::micros());
        printf("call_conter_Copter_ModeMeasurement_init: %d\n", 
            call_conter_Copter_ModeMeasurement_init);
    }
    #endif // IS_TEST_MEASUREMENT_INSTANCE_INIT

#if MEASUREMENT_FLIGHTMODE_BEHAVIOR == MEASUREMENT_BEHAVIOR_LOITER
    // Copter::ModeLoiter::run();
    Copter::ModeMeasurement::loiterlike_run();
#elif MEASUREMENT_FLIGHTMODE_BEHAVIOR == MEASUREMENT_BEHAVIOR_SEMI_GUIDED
    #error This behavior for flightmode MEASUREMENT is not implemented
#elif MEASUREMENT_FLIGHTMODE_BEHAVIOR == MEASUREMENT_BEHAVIOR_GUIDED
    // TODO: implement perhaps
    #error This behavior for flightmode MEASUREMENT is not implemented
    // run angle controller
 #if 1   // begin Copter::ModeGuided::angle_control_run(), 0 --> direct call; 1 --> reimplementation
    Copter::ModeGuided::angle_control_run();
 #else 
    // copied Copter::ModeGuided::angle_control_run() from mode_guided.cpp, so we can adjust things
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || (ap.land_complete && guided_angle_state.climb_rate_cms <= 0.0f)) {
#if FRAME_CONFIG == HELI_FRAME
        attitude_control->set_yaw_target_to_current_heading();
#endif
        zero_throttle_and_relax_ac();
        pos_control->relax_alt_hold_controllers(0.0f);
        return;
    }

    // constrain desired lean angles
    float roll_in = guided_angle_state.roll_cd;
    float pitch_in = guided_angle_state.pitch_cd;
    float total_in = norm(roll_in, pitch_in);
    float angle_max = MIN(attitude_control->get_althold_lean_angle_max(), copter.aparm.angle_max);
    if (total_in > angle_max) {
        float ratio = angle_max / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // wrap yaw request
    float yaw_in = wrap_180_cd(guided_angle_state.yaw_cd);
    float yaw_rate_in = wrap_180_cd(guided_angle_state.yaw_rate_cds);

    // constrain climb rate
    float climb_rate_cms = constrain_float(guided_angle_state.climb_rate_cms, -fabsf(wp_nav->get_speed_down()), wp_nav->get_speed_up());

    // get avoidance adjusted climb rate
    climb_rate_cms = get_avoidance_adjusted_climbrate(climb_rate_cms);

    // check for timeout - set lean angles and climb rate to zero if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - guided_angle_state.update_time_ms > GUIDED_ATTITUDE_TIMEOUT_MS) {
        roll_in = 0.0f;
        pitch_in = 0.0f;
        climb_rate_cms = 0.0f;
        yaw_rate_in = 0.0f;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // call attitude controller
    if (guided_angle_state.use_yaw_rate) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll_in, pitch_in, yaw_rate_in);
    } else {
        attitude_control->input_euler_angle_roll_pitch_yaw(roll_in, pitch_in, yaw_in, true);
    }

    // adding altitude over ground HERE
    // adjust climb rate using rangefinder
    target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
    // Anticipating Altitude Control HERE

    // call position controller
    pos_control->set_alt_target_from_climb_rate_ff(climb_rate_cms, G_Dt, false);
    pos_control->update_z_controller();
 #endif // end Copter::ModeGuided::angle_control_run()

#else
    #error This behavior for flightmode MEASUREMENT is unknown
#endif  // MEASUREMENT_FLIGHTMODE_BEHAVIOR == MEASUREMENT_BEHAVIOR_LOITER
}