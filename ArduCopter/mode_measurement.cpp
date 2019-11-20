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

    bool ret = false;

#if MEASUREMENT_FLIGHTMODE_BEHAVIOR == MEASUREMENT_BEHAVIOR_LOITER
    ret = Copter::ModeLoiter::init(ignore_checks);
#elif MEASUREMENT_FLIGHTMODE_BEHAVIOR == MEASUREMENT_BEHAVIOR_SEMI_GUIDED
    #error This behavior for flightmode MEASUREMENT is not implemented
#elif MEASUREMENT_FLIGHTMODE_BEHAVIOR == MEASUREMENT_BEHAVIOR_GUIDED
    #error This behavior for flightmode MEASUREMENT is not implemented
    // start in angle control mode
    // TODO: really start in this mode angle control mode?
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

    copter.ground_profile_acquisition->init();

    is_started_ground_profile_acquisition = false;      // force restart after fresh switch to MEASUREMENT
    // CONTINUE HERE
#endif // MEASUREMENT_ALTITUDE_CONTROL_MODE == ALT_CTRL_MODE_FFC

    return ret;
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
        pos_control->update_z_controller();
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
        pos_control->update_z_controller();
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
        pos_control->update_z_controller();
        break;

    case Loiter_Flying:

        // PeterSt:
        
#if IS_GROUND_PROFILE_ACQUISITION_ENABLED
        if (!is_started_ground_profile_acquisition) {
            // start Ground Profile Acquisition
            copter.ground_profile_acquisition->start(ahrs.yaw_sensor);
            is_started_ground_profile_acquisition = true;
        }

        //  TODO: prio 8: next point for Ground Profile Acquisition here
        // CONTINUE HERE
        #if !IS_TEST_FFC
            #error "not implemented yet"
        #endif // IS_TEST_FFC

        // NEU relative to home position ("absolute position" with origin in home position, in contrast to
        //  altitude over ground), all in cm
        Vector3f position_neu;
        position_neu = inertial_nav.get_position();

        #if IS_PRINT_GPA_TESTS
            if (copter.call_run_counter % (PRINT_MESSAGE_VALUE_INTERVAL * CALL_FREQUENCY_MEASUREMENT_RUN) == 1) {
                hal.console->printf("position_neu: x: %8f, y: %8f, z: %8f\n", 
                    position_neu.x, position_neu.y, position_neu.z);
            }
        #endif // IS_PRINT_GPA_TESTS
        copter.ground_profile_acquisition->scan_point(copter.rangefinder2_state.dist_cm, position_neu);
#endif // IS_GROUND_PROFILE_ACQUISITION_ENABLED

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if PRECISION_LANDING == ENABLED
        if (do_precision_loiter()) {
            precision_loiter_xy();
        }
#endif

        // run loiter controller
        loiter_nav->update(ekfGndSpdLimit, ekfNavVelGainScaler);
        // CONTINUE HERE

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

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update altitude target and call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller();
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
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Copter is in flightmode %s (27)", this->name());
            // print system time for checking intervals
            hal.console->printf("(%s:) Time since start: %" PRIu32 " us\n", this->name4(), AP_HAL::micros());
        }
        // TODO: CONTINUE HERE
    #endif

    #if IS_TEST_MEASUREMENT_INSTANCE_INIT
    if (copter.call_run_counter % (REPEATET_MESSAGE_IN_MEASUREMENT_INTERVAL * CALL_FREQUENCY_MEASUREMENT_RUN) == 1) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "call_conter_Copter_ModeMeasurement_init: %d", 
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