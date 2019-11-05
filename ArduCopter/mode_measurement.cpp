// New flightmode MEASUREMENT by PeterSt

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

    // run angle controller
    Copter::ModeGuided::angle_control_run();
}