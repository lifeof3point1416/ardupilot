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

    return ret;
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
    // #error This behavior for flightmode MEASUREMENT is not implemented
    Copter::ModeLoiter::run();
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
    // TODO: throw compiler error
    #error This behavior for flightmode MEASUREMENT is unknown
#endif  // MEASUREMENT_FLIGHTMODE_BEHAVIOR == MEASUREMENT_BEHAVIOR_LOITER
}