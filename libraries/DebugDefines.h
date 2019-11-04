#pragma once

/*
Debug defines added by PeterSt

This file can be included via ArduCopter/defines.h, which is included in many files

TODO (Prio 3): Find proper way to include this
*/

// #include <inttypes.h>                                           // for placeholders like PRIu64 


// debug switches
#define IS_DO_VERBOSE_DEBUG_PRINTOUTS           0
#define IS_PRINT_REPEATET_GCS_MESSAGE           1               // print message via GCS message, repeatedly
#define IS_PRINT_INIT_GCS_MESSAGE               1               // print init message via GCS message
#define IS_PRINT_INIT_PRINTF_MESSAGE            1               // print init message via printf
#define IS_PRINT_INIT_HAL_PRINTF_MESSAGE        1               // print init message via hal.console->printf
                                                                // works on sitl after Copter::init_ardupilot()
// more specific stuff
#define IS_PRINT_REPEATET_MESSAGE_1HZ_CONSOLE   0               // print message via GCS message in 1 Hz look

// debug values
#define REPEATET_GCS_MESSAGE_INTERVAL           30              // print a custom gcs message every X seconds