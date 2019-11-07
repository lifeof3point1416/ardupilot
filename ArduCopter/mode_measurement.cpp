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

    // start in angle control mode
    // TODO: really start in this mode angle control mode?
    Copter::ModeGuided::angle_control_start();
    return true;
}

// guided_run - runs the guided controller
// should be called at 100hz or more
// TODO: prio 6: reconsider how to run
//  add Anticipating AltCtrl here?
// PeterSt: according to http://ardupilot.org/dev/docs/apmcopter-adding-a-new-flight-mode.html
//  run() is called with 400 Hz
void Copter::ModeMeasurement::run()
{
    // TODO: get some input? (eg. for setting altitude)

    call_run_counter++;
    #if (IS_PRINT_REPEATET_MESSAGE_IN_MEASUREMENT)
        if (call_run_counter % (REPEATET_MESSAGE_IN_MEASUREMENT_INTERVAL * CALL_FREQUENCY_MEASUREMENT_RUN) == 1) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Copter is in flightmode %s (27)", this->name());
            // print system time for checking intervals
            hal.console->printf("(%s:) Time since start: %" PRIu32 " us\n", this->name4(), AP_HAL::micros());
        }
        // TODO: CONTINUE HERE
    #endif

    // run angle controller
    Copter::ModeGuided::angle_control_run();
}