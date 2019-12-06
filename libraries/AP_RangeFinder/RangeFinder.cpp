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

#include "RangeFinder.h"
#include "AP_RangeFinder_analog.h"
#include "AP_RangeFinder_PulsedLightLRF.h"
#include "AP_RangeFinder_MaxsonarI2CXL.h"
#include "AP_RangeFinder_MaxsonarSerialLV.h"
#include "AP_RangeFinder_PX4_PWM.h"
#include "AP_RangeFinder_BBB_PRU.h"
#include "AP_RangeFinder_LightWareI2C.h"
#include "AP_RangeFinder_LightWareSerial.h"
#if (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP || \
     CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO) &&      \
    defined(HAVE_LIBIIO)
#include "AP_RangeFinder_Bebop.h"
#endif
#include "AP_RangeFinder_MAVLink.h"
#include "AP_RangeFinder_LeddarOne.h"
#include "AP_RangeFinder_uLanding.h"
#include "AP_RangeFinder_TeraRangerI2C.h"
#include "AP_RangeFinder_VL53L0X.h"
#include "AP_RangeFinder_NMEA.h"
#include "AP_RangeFinder_Wasp.h"
#include "AP_RangeFinder_Benewake.h"
#include "AP_RangeFinder_Benewake_TFMiniPlus.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#if IS_LOG_GPA
#include <DataFlash/DataFlash.h>
#endif // IS_LOG_GPA

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo RangeFinder::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: Rangefinder type
    // @Description: What type of rangefinder device that is connected
    // @Values: 0:None,1:Analog,2:MaxbotixI2C,3:LidarLiteV2-I2C,5:PX4-PWM,6:BBB-PRU,7:LightWareI2C,8:LightWareSerial,9:Bebop,10:MAVLink,11:uLanding,12:LeddarOne,13:MaxbotixSerial,14:TeraRangerI2C,15:LidarLiteV3-I2C,16:VL53L0X,17:NMEA,18:WASP-LRF,19:BenewakeTF02,20:BenewakeTFmini,21:LidarLiteV3HP-I2C,25:BenewakeTFMiniPlus
    // @User: Standard
    AP_GROUPINFO("_TYPE",    0, RangeFinder, state[0].type, 0),

    // @Param: _PIN
    // @DisplayName: Rangefinder pin
    // @Description: Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
    // @Values: -1:Not Used, 0:APM2-A0, 1:APM2-A1, 2:APM2-A2, 3:APM2-A3, 4:APM2-A4, 5:APM2-A5, 6:APM2-A6, 7:APM2-A7, 8:APM2-A8, 9:APM2-A9, 11:PX4-airspeed port, 15:Pixhawk-airspeed port, 64:APM1-airspeed port
    // @User: Standard
    AP_GROUPINFO("_PIN",     1, RangeFinder, state[0].pin, -1),

    // @Param: _SCALING
    // @DisplayName: Rangefinder scaling
    // @Description: Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
    // @Units: m/V
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("_SCALING", 2, RangeFinder, state[0].scaling, 3.0f),

    // @Param: _OFFSET
    // @DisplayName: rangefinder offset
    // @Description: Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM and I2C Lidars
    // @Units: V
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("_OFFSET",  3, RangeFinder, state[0].offset, 0.0f),

    // @Param: _FUNCTION
    // @DisplayName: Rangefinder function
    // @Description: Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
    // @Values: 0:Linear,1:Inverted,2:Hyperbolic
    // @User: Standard
    AP_GROUPINFO("_FUNCTION", 4, RangeFinder, state[0].function, 0),

    // @Param: _MIN_CM
    // @DisplayName: Rangefinder minimum distance
    // @Description: Minimum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_MIN_CM",  5, RangeFinder, state[0].min_distance_cm, 20),

    // @Param: _MAX_CM
    // @DisplayName: Rangefinder maximum distance
    // @Description: Maximum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_MAX_CM",  6, RangeFinder, state[0].max_distance_cm, 700),

    // @Param: _STOP_PIN
    // @DisplayName: Rangefinder stop pin
    // @Description: Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
    // @Values: -1:Not Used,50:Pixhawk AUXOUT1,51:Pixhawk AUXOUT2,52:Pixhawk AUXOUT3,53:Pixhawk AUXOUT4,54:Pixhawk AUXOUT5,55:Pixhawk AUXOUT6,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    // @User: Standard
    AP_GROUPINFO("_STOP_PIN", 7, RangeFinder, state[0].stop_pin, -1),

    // @Param: _SETTLE
    // @DisplayName: Rangefinder settle time
    // @Description: The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
    // @Units: ms
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_SETTLE", 8, RangeFinder, state[0].settle_time_ms, 0),

    // @Param: _RMETRIC
    // @DisplayName: Ratiometric
    // @Description: This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
    // @Values: 0:No,1:Yes
    // @User: Standard
    AP_GROUPINFO("_RMETRIC", 9, RangeFinder, state[0].ratiometric, 1),

    // @Param: _PWRRNG
    // @DisplayName: Powersave range
    // @Description: This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode (if available). A value of zero means power saving is not enabled
    // @Units: m
    // @Range: 0 32767
    // @User: Standard
    AP_GROUPINFO("_PWRRNG", 10, RangeFinder, _powersave_range, 0),

    // @Param: _GNDCLEAR
    // @DisplayName: Distance (in cm) from the range finder to the ground
    // @Description: This parameter sets the expected range measurement(in cm) that the range finder should return when the vehicle is on the ground.
    // @Units: cm
    // @Range: 5 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_GNDCLEAR", 11, RangeFinder, state[0].ground_clearance_cm, RANGEFINDER_GROUND_CLEARANCE_CM_DEFAULT),

    // @Param: _ADDR
    // @DisplayName: Bus address of sensor
    // @Description: This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ADDR", 23, RangeFinder, state[0].address, 0),

    // @Param: _POS_X
    // @DisplayName:  X position offset
    // @Description: X position of the first rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: _POS_Y
    // @DisplayName: Y position offset
    // @Description: Y position of the first rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: _POS_Z
    // @DisplayName: Z position offset
    // @Description: Z position of the first rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("_POS", 49, RangeFinder, state[0].pos_offset, 0.0f),

    // @Param: _ORIENT
    // @DisplayName: Rangefinder orientation
    // @Description: Orientation of rangefinder
    // @Values: 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up, 25:Down
    // @User: Advanced
    AP_GROUPINFO("_ORIENT", 53, RangeFinder, state[0].orientation, ROTATION_PITCH_270),

    // @Group: _
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[0], "_",  57, RangeFinder, backend_var_info[0]),

#if RANGEFINDER_MAX_INSTANCES > 1
    // @Param: 2_TYPE
    // @DisplayName: Second Rangefinder type
    // @Description: What type of rangefinder device that is connected
    // @Values: 0:None,1:Analog,2:MaxbotixI2C,3:LidarLiteV2-I2C,5:PX4-PWM,6:BBB-PRU,7:LightWareI2C,8:LightWareSerial,9:Bebop,10:MAVLink,11:uLanding,12:LeddarOne,13:MaxbotixSerial,14:TeraRangerI2C,15:LidarLiteV3-I2C,16:VL53L0X,17:NMEA,18:WASP-LRF,19:BenewakeTF02,20:BenewakeTFmini,21:LidarLiteV3HP-I2C,25:BenewakeTFMiniPlus
    // @User: Advanced
    AP_GROUPINFO("2_TYPE",    12, RangeFinder, state[1].type, 0),

    // @Param: 2_PIN
    // @DisplayName: Rangefinder pin
    // @Description: Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
    // @Values: -1:Not Used, 0:APM2-A0, 1:APM2-A1, 2:APM2-A2, 3:APM2-A3, 4:APM2-A4, 5:APM2-A5, 6:APM2-A6, 7:APM2-A7, 8:APM2-A8, 9:APM2-A9, 11:PX4-airspeed port, 15:Pixhawk-airspeed port, 64:APM1-airspeed port
    // @User: Advanced
    AP_GROUPINFO("2_PIN",     13, RangeFinder, state[1].pin, -1),

    // @Param: 2_SCALING
    // @DisplayName: Rangefinder scaling
    // @Description: Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
    // @Units: m/V
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("2_SCALING", 14, RangeFinder, state[1].scaling, 3.0f),

    // @Param: 2_OFFSET
    // @DisplayName: rangefinder offset
    // @Description: Offset in volts for zero distance
    // @Units: V
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("2_OFFSET",  15, RangeFinder, state[1].offset, 0.0f),

    // @Param: 2_FUNCTION
    // @DisplayName: Rangefinder function
    // @Description: Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
    // @Values: 0:Linear,1:Inverted,2:Hyperbolic
    // @User: Advanced
    AP_GROUPINFO("2_FUNCTION",  16, RangeFinder, state[1].function, 0),

    // @Param: 2_MIN_CM
    // @DisplayName: Rangefinder minimum distance
    // @Description: Minimum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2_MIN_CM",  17, RangeFinder, state[1].min_distance_cm, 20),

    // @Param: 2_MAX_CM
    // @DisplayName: Rangefinder maximum distance
    // @Description: Maximum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2_MAX_CM",  18, RangeFinder, state[1].max_distance_cm, 700),

    // @Param: 2_STOP_PIN
    // @DisplayName: Rangefinder stop pin
    // @Description: Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
    // @Values: -1:Not Used,50:Pixhawk AUXOUT1,51:Pixhawk AUXOUT2,52:Pixhawk AUXOUT3,53:Pixhawk AUXOUT4,54:Pixhawk AUXOUT5,55:Pixhawk AUXOUT6,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    // @User: Advanced
    AP_GROUPINFO("2_STOP_PIN", 19, RangeFinder, state[1].stop_pin, -1),

    // @Param: 2_SETTLE
    // @DisplayName: Sonar settle time
    // @Description: The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
    // @Units: ms
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2_SETTLE", 20, RangeFinder, state[1].settle_time_ms, 0),

    // @Param: 2_RMETRIC
    // @DisplayName: Ratiometric
    // @Description: This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
    // @Values: 0:No,1:Yes
    // @User: Advanced
    AP_GROUPINFO("2_RMETRIC", 21, RangeFinder, state[1].ratiometric, 1),

    // @Param: 2_GNDCLEAR
    // @DisplayName: Distance (in cm) from the second range finder to the ground
    // @Description: This parameter sets the expected range measurement(in cm) that the second range finder should return when the vehicle is on the ground.
    // @Units: cm
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2_GNDCLEAR", 22, RangeFinder, state[1].ground_clearance_cm, RANGEFINDER_GROUND_CLEARANCE_CM_DEFAULT),

    // @Param: 2_ADDR
    // @DisplayName: Bus address of second rangefinder
    // @Description: This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2_ADDR", 24, RangeFinder, state[1].address, 0),

    // @Param: 2_POS_X
    // @DisplayName:  X position offset
    // @Description: X position of the second rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: 2_POS_Y
    // @DisplayName: Y position offset
    // @Description: Y position of the second rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: 2_POS_Z
    // @DisplayName: Z position offset
    // @Description: Z position of the second rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("2_POS", 50, RangeFinder, state[1].pos_offset, 0.0f),

    // @Param: 2_ORIENT
    // @DisplayName: Rangefinder 2 orientation
    // @Description: Orientation of 2nd rangefinder
    // @Values: 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up, 25:Down
    // @User: Advanced
    AP_GROUPINFO("2_ORIENT", 54, RangeFinder, state[1].orientation, ROTATION_PITCH_270),

    // @Group: 2_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[1], "2_", 58, RangeFinder, backend_var_info[1]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 2

    // @Param: 3_TYPE
    // @DisplayName: Third Rangefinder type
    // @Description: What type of rangefinder device that is connected
    // @Values: 0:None,1:Analog,2:MaxbotixI2C,3:LidarLiteV2-I2C,5:PX4-PWM,6:BBB-PRU,7:LightWareI2C,8:LightWareSerial,9:Bebop,10:MAVLink,11:uLanding,12:LeddarOne,13:MaxbotixSerial,14:TeraRangerI2C,15:LidarLiteV3-I2C,16:VL53L0X,17:NMEA,18:WASP-LRF,19:BenewakeTF02,20:BenewakeTFmini,21:LidarLiteV3HP-I2C,25:BenewakeTFMiniPlus
    // @User: Advanced
    AP_GROUPINFO("3_TYPE",    25, RangeFinder, state[2].type, 0),

    // @Param: 3_PIN
    // @DisplayName: Rangefinder pin
    // @Description: Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
    // @Values: -1:Not Used, 0:APM2-A0, 1:APM2-A1, 2:APM2-A2, 3:APM2-A3, 4:APM2-A4, 5:APM2-A5, 6:APM2-A6, 7:APM2-A7, 8:APM2-A8, 9:APM2-A9, 11:PX4-airspeed port, 15:Pixhawk-airspeed port, 64:APM1-airspeed port
    // @User: Advanced
    AP_GROUPINFO("3_PIN",     26, RangeFinder, state[2].pin, -1),

    // @Param: 3_SCALING
    // @DisplayName: Rangefinder scaling
    // @Description: Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
    // @Units: m/V
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("3_SCALING", 27, RangeFinder, state[2].scaling, 3.0f),

    // @Param: 3_OFFSET
    // @DisplayName: rangefinder offset
    // @Description: Offset in volts for zero distance
    // @Units: V
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("3_OFFSET",  28, RangeFinder, state[2].offset, 0.0f),

    // @Param: 3_FUNCTION
    // @DisplayName: Rangefinder function
    // @Description: Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
    // @Values: 0:Linear,1:Inverted,2:Hyperbolic
    // @User: Advanced
    AP_GROUPINFO("3_FUNCTION",  29, RangeFinder, state[2].function, 0),

    // @Param: 3_MIN_CM
    // @DisplayName: Rangefinder minimum distance
    // @Description: Minimum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("3_MIN_CM",  30, RangeFinder, state[2].min_distance_cm, 20),

    // @Param: 3_MAX_CM
    // @DisplayName: Rangefinder maximum distance
    // @Description: Maximum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("3_MAX_CM",  31, RangeFinder, state[2].max_distance_cm, 700),

    // @Param: 3_STOP_PIN
    // @DisplayName: Rangefinder stop pin
    // @Description: Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
    // @Values: -1:Not Used,50:Pixhawk AUXOUT1,51:Pixhawk AUXOUT2,52:Pixhawk AUXOUT3,53:Pixhawk AUXOUT4,54:Pixhawk AUXOUT5,55:Pixhawk AUXOUT6,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    // @User: Advanced
    AP_GROUPINFO("3_STOP_PIN", 32, RangeFinder, state[2].stop_pin, -1),

    // @Param: 3_SETTLE
    // @DisplayName: Sonar settle time
    // @Description: The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
    // @Units: ms
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("3_SETTLE", 33, RangeFinder, state[2].settle_time_ms, 0),

    // @Param: 3_RMETRIC
    // @DisplayName: Ratiometric
    // @Description: This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
    // @Values: 0:No,1:Yes
    // @User: Advanced
    AP_GROUPINFO("3_RMETRIC", 34, RangeFinder, state[2].ratiometric, 1),

    // @Param: 3_GNDCLEAR
    // @DisplayName: Distance (in cm) from the third range finder to the ground
    // @Description: This parameter sets the expected range measurement(in cm) that the third range finder should return when the vehicle is on the ground.
    // @Units: cm
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("3_GNDCLEAR", 35, RangeFinder, state[2].ground_clearance_cm, RANGEFINDER_GROUND_CLEARANCE_CM_DEFAULT),

    // @Param: 3_ADDR
    // @DisplayName: Bus address of third rangefinder
    // @Description: This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("3_ADDR", 36, RangeFinder, state[2].address, 0),

    // @Param: 3_POS_X
    // @DisplayName:  X position offset
    // @Description: X position of the third rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: 3_POS_Y
    // @DisplayName: Y position offset
    // @Description: Y position of the third rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: 3_POS_Z
    // @DisplayName: Z position offset
    // @Description: Z position of the third rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("3_POS", 51, RangeFinder, state[2].pos_offset, 0.0f),

    // @Param: 3_ORIENT
    // @DisplayName: Rangefinder 3 orientation
    // @Description: Orientation of 3rd rangefinder
    // @Values: 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up, 25:Down
    // @User: Advanced
    AP_GROUPINFO("3_ORIENT", 55, RangeFinder, state[2].orientation, ROTATION_PITCH_270),

    // @Group: 3_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[2], "3_", 59, RangeFinder, backend_var_info[2]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 3

    // @Param: 4_TYPE
    // @DisplayName: Fourth Rangefinder type
    // @Description: What type of rangefinder device that is connected
    // @Values: 0:None,1:Analog,2:MaxbotixI2C,3:LidarLiteV2-I2C,5:PX4-PWM,6:BBB-PRU,7:LightWareI2C,8:LightWareSerial,9:Bebop,10:MAVLink,11:uLanding,12:LeddarOne,13:MaxbotixSerial,14:TeraRangerI2C,15:LidarLiteV3-I2C,16:VL53L0X,17:NMEA,18:WASP-LRF,19:BenewakeTF02,20:BenewakeTFmini,21:LidarLiteV3HP-I2C,25:BenewakeTFMiniPlus
    // @User: Advanced
    AP_GROUPINFO("4_TYPE",    37, RangeFinder, state[3].type, 0),

    // @Param: 4_PIN
    // @DisplayName: Rangefinder pin
    // @Description: Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
    // @Values: -1:Not Used, 0:APM2-A0, 1:APM2-A1, 2:APM2-A2, 3:APM2-A3, 4:APM2-A4, 5:APM2-A5, 6:APM2-A6, 7:APM2-A7, 8:APM2-A8, 9:APM2-A9, 11:PX4-airspeed port, 15:Pixhawk-airspeed port, 64:APM1-airspeed port
    // @User: Advanced
    AP_GROUPINFO("4_PIN",     38, RangeFinder, state[3].pin, -1),

    // @Param: 4_SCALING
    // @DisplayName: Rangefinder scaling
    // @Description: Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
    // @Units: m/V
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("4_SCALING", 39, RangeFinder, state[3].scaling, 3.0f),

    // @Param: 4_OFFSET
    // @DisplayName: rangefinder offset
    // @Description: Offset in volts for zero distance
    // @Units: V
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("4_OFFSET",  40, RangeFinder, state[3].offset, 0.0f),

    // @Param: 4_FUNCTION
    // @DisplayName: Rangefinder function
    // @Description: Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
    // @Values: 0:Linear,1:Inverted,2:Hyperbolic
    // @User: Advanced
    AP_GROUPINFO("4_FUNCTION",  41, RangeFinder, state[3].function, 0),

    // @Param: 4_MIN_CM
    // @DisplayName: Rangefinder minimum distance
    // @Description: Minimum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("4_MIN_CM",  42, RangeFinder, state[3].min_distance_cm, 20),

    // @Param: 4_MAX_CM
    // @DisplayName: Rangefinder maximum distance
    // @Description: Maximum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("4_MAX_CM",  43, RangeFinder, state[3].max_distance_cm, 700),

    // @Param: 4_STOP_PIN
    // @DisplayName: Rangefinder stop pin
    // @Description: Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
    // @Values: -1:Not Used,50:Pixhawk AUXOUT1,51:Pixhawk AUXOUT2,52:Pixhawk AUXOUT3,53:Pixhawk AUXOUT4,54:Pixhawk AUXOUT5,55:Pixhawk AUXOUT6,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    // @User: Advanced
    AP_GROUPINFO("4_STOP_PIN", 44, RangeFinder, state[3].stop_pin, -1),

    // @Param: 4_SETTLE
    // @DisplayName: Sonar settle time
    // @Description: The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
    // @Units: ms
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("4_SETTLE", 45, RangeFinder, state[3].settle_time_ms, 0),

    // @Param: 4_RMETRIC
    // @DisplayName: Ratiometric
    // @Description: This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
    // @Values: 0:No,1:Yes
    // @User: Advanced
    AP_GROUPINFO("4_RMETRIC", 46, RangeFinder, state[3].ratiometric, 1),

    // @Param: 4_GNDCLEAR
    // @DisplayName: Distance (in cm) from the fourth range finder to the ground
    // @Description: This parameter sets the expected range measurement(in cm) that the fourth range finder should return when the vehicle is on the ground.
    // @Units: cm
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("4_GNDCLEAR", 47, RangeFinder, state[3].ground_clearance_cm, RANGEFINDER_GROUND_CLEARANCE_CM_DEFAULT),

    // @Param: 4_ADDR
    // @DisplayName: Bus address of fourth rangefinder
    // @Description: This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("4_ADDR", 48, RangeFinder, state[3].address, 0),

    // @Param: 4_POS_X
    // @DisplayName:  X position offset
    // @Description: X position of the fourth rangefinder in body frame. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: 4_POS_Y
    // @DisplayName: Y position offset
    // @Description: Y position of the fourth rangefinder in body frame. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: 4_POS_Z
    // @DisplayName: Z position offset
    // @Description: Z position of the fourth rangefinder in body frame. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("4_POS", 52, RangeFinder, state[3].pos_offset, 0.0f),

    // @Param: 4_ORIENT
    // @DisplayName: Rangefinder 4 orientation
    // @Description: Orientation of 4th range finder
    // @Values: 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up, 25:Down
    // @User: Advanced
    AP_GROUPINFO("4_ORIENT", 56, RangeFinder, state[3].orientation, ROTATION_PITCH_270),

    // @Group: 4_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[3], "4_", 60, RangeFinder, backend_var_info[3]),
#endif

    AP_GROUPEND
};

const AP_Param::GroupInfo *RangeFinder::backend_var_info[RANGEFINDER_MAX_INSTANCES];

RangeFinder::RangeFinder(AP_SerialManager &_serial_manager, enum Rotation orientation_default) :
    num_instances(0),
    estimated_terrain_height(0),
    serial_manager(_serial_manager)
{
    AP_Param::setup_object_defaults(this, var_info);

    // set orientation defaults
    for (uint8_t i=0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        state[i].orientation.set_default(orientation_default);
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("Rangefinder must be singleton");
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _singleton = this;
}

/*
  initialise the RangeFinder class. We do detection of attached range
  finders here. For now we won't allow for hot-plugging of
  rangefinders.
 */
void RangeFinder::init(void)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0, serial_instance = 0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        // serial_instance will be increased inside detect_instance
        // if a serial driver is loaded for this instance
        detect_instance(i, serial_instance);
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        }
        // initialise pre-arm check variables
        state[i].pre_arm_check = false;
        state[i].pre_arm_distance_min = 9999;  // initialise to an arbitrary large value
        state[i].pre_arm_distance_max = 0;

        // initialise status
        state[i].status = RangeFinder_NotConnected;
        state[i].range_valid_count = 0;
    }
}

/*
  update RangeFinder state for all instances. This should be called at
  around 10Hz by main loop
 */
void RangeFinder::update(void)
{
    // PeterSt debug printout, num_instances is 1
    #if 0
        printf("in RangeFinder::update(void) num_instances: %hhu\n", num_instances);
    #endif
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            if (state[i].type == RangeFinder_TYPE_NONE) {
                // allow user to disable a rangefinder at runtime
                state[i].status = RangeFinder_NotConnected;
                state[i].range_valid_count = 0;
                continue;
            }
            drivers[i]->update();
            // PeterSt, debug printout
            #if 0
                // only 0 in sitl, because there is only 1 rangefinder
                printf("in RangeFinder::update(void) drivers[%hhu]->update();\n", i);
            #endif
            drivers[i]->update_pre_arm_check();
        }
    }
}

bool RangeFinder::_add_backend(AP_RangeFinder_Backend *backend)
{
    if (!backend) {
        return false;
    }
    if (num_instances == RANGEFINDER_MAX_INSTANCES) {
        AP_HAL::panic("Too many RANGERS backends");
    }

    drivers[num_instances++] = backend;
    return true;
}

/*
  detect if an instance of a rangefinder is connected. 
 */
void RangeFinder::detect_instance(uint8_t instance, uint8_t& serial_instance)
{
    enum RangeFinder_Type _type = (enum RangeFinder_Type)state[instance].type.get();
    switch (_type) {
    case RangeFinder_TYPE_PLI2C:
    case RangeFinder_TYPE_PLI2CV3:
    case RangeFinder_TYPE_PLI2CV3HP:
        for (int8_t i=3; i>=0; i--) {
            if (_add_backend(AP_RangeFinder_PulsedLightLRF::detect(i, state[instance], _type))) {
                break;
            }
        }
        break;
    case RangeFinder_TYPE_MBI2C:
        for (int8_t i=3; i>=0; i--) {
            if (_add_backend(AP_RangeFinder_MaxsonarI2CXL::detect(state[instance],
                                                                  hal.i2c_mgr->get_device(i, AP_RANGE_FINDER_MAXSONARI2CXL_DEFAULT_ADDR)))) {
                break;
            }
        }
        break;
    case RangeFinder_TYPE_LWI2C:
        if (state[instance].address) {
#ifdef HAL_RANGEFINDER_LIGHTWARE_I2C_BUS
            _add_backend(AP_RangeFinder_LightWareI2C::detect(state[instance],
                hal.i2c_mgr->get_device(HAL_RANGEFINDER_LIGHTWARE_I2C_BUS, state[instance].address)));
#else
            for (int8_t i=3; i>=0; i--) {
                if (_add_backend(AP_RangeFinder_LightWareI2C::detect(state[instance],
                                                                     hal.i2c_mgr->get_device(i, state[instance].address)))) {
                    break;
                }
            }
#endif
        }
        break;
    case RangeFinder_TYPE_TRI2C:
        if (state[instance].address) {
            for (int8_t i=3; i>=0; i--) {
                if (_add_backend(AP_RangeFinder_TeraRangerI2C::detect(state[instance],
                                                                      hal.i2c_mgr->get_device(i, state[instance].address)))) {
                    break;
                }
            }
        }
        break;
    case RangeFinder_TYPE_VL53L0X:
        for (int8_t i=3; i>=0; i--) {
            if (_add_backend(AP_RangeFinder_VL53L0X::detect(state[instance],
                                                            hal.i2c_mgr->get_device(i, 0x29)))) {
                break;
            }
        }
        break;
    case RangeFinder_TYPE_BenewakeTFminiPlus:
        for (int8_t i=3; i>=0; i--) {
            if (_add_backend(AP_RangeFinder_Benewake_TFMiniPlus::detect(state[instance],
                                                                        hal.i2c_mgr->get_device(i, state[instance].address)))) {
                break;
            }
        }
        break;
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    case RangeFinder_TYPE_PX4_PWM:
        if (AP_RangeFinder_PX4_PWM::detect()) {
            drivers[instance] = new AP_RangeFinder_PX4_PWM(state[instance], _powersave_range, estimated_terrain_height);
        }
        break;
#endif
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
    case RangeFinder_TYPE_BBB_PRU:
        if (AP_RangeFinder_BBB_PRU::detect()) {
            drivers[instance] = new AP_RangeFinder_BBB_PRU(state[instance]);
        }
        break;
#endif
    case RangeFinder_TYPE_LWSER:
        if (AP_RangeFinder_LightWareSerial::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_LightWareSerial(state[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_LEDDARONE:
        if (AP_RangeFinder_LeddarOne::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_LeddarOne(state[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_ULANDING:
        if (AP_RangeFinder_uLanding::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_uLanding(state[instance], serial_manager, serial_instance++);
        }
        break;
#if (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP || \
     CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO) && defined(HAVE_LIBIIO)
    case RangeFinder_TYPE_BEBOP:
        if (AP_RangeFinder_Bebop::detect()) {
            drivers[instance] = new AP_RangeFinder_Bebop(state[instance]);
        }
        break;
#endif
    case RangeFinder_TYPE_MAVLink:
        if (AP_RangeFinder_MAVLink::detect()) {
            drivers[instance] = new AP_RangeFinder_MAVLink(state[instance]);
        }
        break;
    case RangeFinder_TYPE_MBSER:
        if (AP_RangeFinder_MaxsonarSerialLV::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_MaxsonarSerialLV(state[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_ANALOG:
        // note that analog will always come back as present if the pin is valid
        if (AP_RangeFinder_analog::detect(state[instance])) {
            drivers[instance] = new AP_RangeFinder_analog(state[instance]);
        }
        break;
    case RangeFinder_TYPE_NMEA:
        if (AP_RangeFinder_NMEA::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_NMEA(state[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_WASP:
        if (AP_RangeFinder_Wasp::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_Wasp(state[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_BenewakeTF02:
        if (AP_RangeFinder_Benewake::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_Benewake(state[instance], serial_manager, serial_instance++, AP_RangeFinder_Benewake::BENEWAKE_TF02);
        }
        break;
    case RangeFinder_TYPE_BenewakeTFmini:
        if (AP_RangeFinder_Benewake::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_Benewake(state[instance], serial_manager, serial_instance++, AP_RangeFinder_Benewake::BENEWAKE_TFmini);
        }
        break;
    default:
        break;
    }

    // if the backend has some local parameters then make those available in the tree
    if (drivers[instance] && state[instance].var_info) {
        backend_var_info[instance] = state[instance].var_info;
        AP_Param::load_object_from_eeprom(drivers[instance], backend_var_info[instance]);
    }
}

AP_RangeFinder_Backend *RangeFinder::get_backend(uint8_t id) const {
    if (id >= num_instances) {
        return nullptr;
    }
    if (drivers[id] != nullptr) {
        if (drivers[id]->type() == RangeFinder_TYPE_NONE) {
            // pretend it isn't here; disabled at runtime?
            return nullptr;
        }
    }
    return drivers[id];
};

RangeFinder::RangeFinder_Status RangeFinder::status_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return RangeFinder_NotConnected;
    }
    return backend->status();
}

void RangeFinder::handle_msg(mavlink_message_t *msg)
{
    uint8_t i;
    for (i=0; i<num_instances; i++) {
        if ((drivers[i] != nullptr) && (state[i].type != RangeFinder_TYPE_NONE)) {
          drivers[i]->handle_msg(msg);
        }
    }
}

// return true if we have a range finder with the specified orientation
bool RangeFinder::has_orientation(enum Rotation orientation) const
{
    return (find_instance(orientation) != nullptr);
}

// find first range finder instance with the specified orientation
AP_RangeFinder_Backend *RangeFinder::find_instance(enum Rotation orientation) const
{
    for (uint8_t i=0; i<num_instances; i++) {
        AP_RangeFinder_Backend *backend = get_backend(i);
        if (backend == nullptr) {
            continue;
        }
        // PeterSt: debug printout rangefinder orientation
        #if 0
            // sitl with emulated rangefinder:
            //  25 == ROTATION_PITCH_270, why is that!?
            printf("*RangeFinder::find_instance(...): backend->orientation(): %d; ", backend->orientation());
            printf("orientation: %d\n", orientation);
        #endif
        if (backend->orientation() != orientation) {
            continue;
        }
        return backend;
    }
    return nullptr;
}

uint16_t RangeFinder::distance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->distance_cm();
}

uint16_t RangeFinder::voltage_mv_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->voltage_mv();
}

int16_t RangeFinder::max_distance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->max_distance_cm();
}

int16_t RangeFinder::min_distance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->min_distance_cm();
}

int16_t RangeFinder::ground_clearance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->ground_clearance_cm();
}

bool RangeFinder::has_data_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return false;
    }
    return backend->has_data();
}

uint8_t RangeFinder::range_valid_count_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->range_valid_count();
}

/*
  returns true if pre-arm checks have passed for all range finders
  these checks involve the user lifting or rotating the vehicle so that sensor readings between
  the min and 2m can be captured
 */
bool RangeFinder::pre_arm_check() const
{
    for (uint8_t i=0; i<num_instances; i++) {
        // if driver is valid but pre_arm_check is false, return false
        if ((drivers[i] != nullptr) && (state[i].type != RangeFinder_TYPE_NONE) && !state[i].pre_arm_check) {
            return false;
        }
    }
    return true;
}

const Vector3f &RangeFinder::get_pos_offset_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return pos_offset_zero;
    }
    return backend->get_pos_offset();
}

MAV_DISTANCE_SENSOR RangeFinder::get_mav_distance_sensor_type_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }
    return backend->get_mav_distance_sensor_type();
}

RangeFinder *RangeFinder::_singleton;





///// Temporary workaround for AC_GroundProfileAcquisition here
// couldn't add AC_GroundProfileAcquisition to waf successfully, yet
// definition part is in RangeFinder.cpp

#if IS_USE_WORKAROUND_GROUND_PROFILE_ACQUISITION && IS_USE_WORKAROUND_HOST_FILE_GPA
#if 1  // in case we want to disable this definition manually

// extern const AP_HAL::HAL& hal;   // already defined in host file RangeFinder.cpp


AC_GroundProfileAcquisition::AC_GroundProfileAcquisition(void) {
    ;
}

// init array map for ground profile points to be scanned
// return: successfully initialized?
bool AC_GroundProfileAcquisition::init(void) {
    // TODO: ...
    // init variables
    int i;
    for (i = 0; i < GROUND_PROFILE_ACQUISITION_PROFILE_ARRAY_SIZE; i++) {
        ground_profile[i] = GROUND_PROFILE_ACQUISITION_NO_DATA_VALUE;
    }

    #if IS_PRINT_GPA_TESTS
    hal.console->printf("GroundProfileAcquisition after init: ground_profile[0]: %" PRIi16 "\n",
        ground_profile[0]);
    #endif // IS_PRINT_GPA_TESTS

    // init last_scanned_point
    #if IS_PRINT_GPA_NEW_POINT
    last_scanned_point.fwd_rangefinder_dist_cm = INT16_MIN;
    last_scanned_point.position_neu_cm.x = NAN;
    last_scanned_point.position_neu_cm.y = NAN;
    last_scanned_point.position_neu_cm.z = NAN;
    last_scanned_point.x_f = INT16_MIN;
    last_scanned_point.y_p = INT32_MIN;
    last_scanned_point.z_f = INT16_MIN;
    #endif

    // check rangefinders???

    return true;
}

// start scanning, use UAV's current yaw as main main_direction (in our 2D world)
// _heading in centi degrees
// return: successfully started?
bool AC_GroundProfileAcquisition::start(uint16_t _heading, Vector3f position_neu_cm) {
// bool AC_GroundProfileAcquisition::start(int _heading, Vector3f position_neu_cm) {
#if IS_DEBUG_GPA
    hal.console->printf("Calling start!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n!!!!!\n");
#endif // IS_DEBUG_GPA

    // check value
    if (_heading > 36000) {
        // invalid value
        return false;
    }
// set main direction of the current measurement flight
#if IS_REVERSE_GPA_MAIN_DIRECTION                           
    // in case we fly backwards (if "fwd rangefinder" is mounted in the back due to practicability reasons)
    main_direction = (18000 + _heading) % 36000;
# else // IS_REVERSE_GPA_MAIN_DIRECTION
    main_direction = _heading;
#endif // IS_REVERSE_GPA_MAIN_DIRECTION

#if IS_PRINT_GPA_TESTS
    hal.console->printf("main_direction: %" PRIu16 " (hal.console)\n", main_direction);
    // printf("main_direction: %" PRIu16 " (printf)\n", main_direction);
#endif // IS_PRINT_GPA_TESTS

    // also set absolute position to "start" position
    start_position_cm = position_neu_cm;
    // add offset to altitude if applicable, so that the first value of the GPA map is 0
    //  ==> ground_profile[0] should be 0
    //  usually it starts with the distance of initial altitude and rangefinder value 
    //  (distance of rangefinder to arbitrary origin of body frame)
// #if IS_USE_GPA_MAP_OFFSET
//     start_position_cm.z
// #endif // IS_USE_GPA_MAP_OFFSET

#if IS_DEBUG_GPA
    hal.console->printf("GPA: called start()\n");
    hal.console->printf("GPA: start_position_cm (NEU): x: %8f cm, y: %8f cm, z: %8f cm\n",
        start_position_cm.x, start_position_cm.y, start_position_cm.z);
    hal.console->printf("GPA: main_direction: %hu c°\n", main_direction);
#endif 

    return true;
}

// get coordinates in main direction space,
//  of position_neu_cm along this->main_direction, origin at this->start_position_cm
//  can be interpreted as 1-dimensional position, y should be as low as possible
// return: x-coordiate [cm]
//int AC_GroundProfileAcquisition::get_1d_x(Vector3f position_neu_cm) {
Vector2<int> AC_GroundProfileAcquisition::get_main_direction_coo(Vector3f position_neu_cm) {
    // goal is to get x (1D variable) of a point P is specified by position_neu_cm
    
    // angle of the point P to North (CCW), of main_direction to point P, in ° respectively
    float alpha_p, alpha_x;
    // wrong:
    //alpha_p = HEADING_CENTIDEGREES_FROM_MATH_ANGLE_RADIANS(atan2f(position_neu_cm.y, position_neu_cm.x)) / 100;
    // correct: "x is y, and y is x", because of NEU coo: x - North (up | abscisse) , y - East (right | ordinate)
    // wrong: using "absolute value", not difference!
    //alpha_p = HEADING_CENTIDEGREES_FROM_MATH_ANGLE_RADIANS(atan2f(position_neu_cm.x, position_neu_cm.y)) / 100;
    alpha_p = HEADING_CENTIDEGREES_FROM_MATH_ANGLE_RADIANS(atan2f(
        position_neu_cm.x - start_position_cm.x, position_neu_cm.y - start_position_cm.y)) / 100.0f;
    alpha_x = ( ((float) main_direction)/100.0f) - alpha_p;

    float dist_o_p; // distance between origin (start_position_cm) and P
    dist_o_p = hypotf(position_neu_cm.x - start_position_cm.x, position_neu_cm.y - start_position_cm.y);

    #if IS_DEBUG_GPA
    uint32_t _micros = AP_HAL::micros();
    if (IS_TRIGGER_EVENT_ROUGHLY_EVERY_N_SEC_MICROS(5, _micros, 400)) {
        hal.console->printf("GPA: alpha_p: %6.2f deg, alpha_x: %6.2f deg\n", alpha_p, alpha_x);
        hal.console->printf("GPA: main_direction/100.0f: %6.2f deg\n", main_direction/100.0f);
        hal.console->printf("GPA: dist_o_p: %8f cm\n", dist_o_p);
    }
    #endif // IS_DEBUG_GPA

    float x_p, y_p; // x coordinate in 1D (requested output), y coo. (error or distance to x-axis)
    x_p = cosf(radians(alpha_x)) * dist_o_p;
    y_p = sinf(radians(alpha_x)) * dist_o_p;

    //return (int) x_p;
    Vector2<int> ret;
    ret.x = x_p;
    ret.y = y_p;
    return ret;
}

#if IS_LOG_GPA
bool AC_GroundProfileAcquisition::log_scan_point(uint64_t TimeUS, int16_t FwdRF, float PosX, float PosY, float PosZ,
    int32_t XP, int32_t YP, int32_t ZP, int16_t XF, int16_t ZF, bool IsValid, int Ret) {
    // TODO: prio 8: implement logging
    // dataflash tag "GPA" is already preoccupied by GPS-Auxilliary - use GPAQ instead
    // TODO: prio 7: log ground_profile instead of sending it via gcs message

    // see https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/LogStructure.h#L6 for specifiers
    #if IS_LOG_EXTRA_XF_ZF_CONSTRAINT
    DataFlash_Class::instance()->Log_Write("GPAQ",                  // tag for Ground Profile AQuisition (must be unique)
        "TimeUS,FwdRF,PosX,PosY,PosZ,XP,YP,ZP,XF,ZF,XFOk,ZFOk,IsOk,Ret",   // variable names
        "smmmmmmmmmmm--",                                             // base units
        "FBBBBBBBBBBB00",                                             // exponents
        "QhfffiiihhhhBi",                                             // types
        TimeUS,
        FwdRF,
        #if 0
        PosX, PosY, PosZ,
        #else 
        (double) PosX, (double) PosY, (double) PosZ,        // as in libraries/AC_AttitudeControl/AC_PosControl.cpp, ln 860
        #endif // 1
        XP, YP, ZP,
        XF, ZF,
        IsValid ? XF : (int16_t) 0, IsValid ? (int16_t) ZF : 0, // to prevent from INVALID_VALUE (-32k) busting log review graphs
        ((uint8_t) IsValid), Ret                            // no bool available, sizeof(bool) is 1
    );
    #else
    DataFlash_Class::instance()->Log_Write("GPAQ",                  // tag for Ground Profile AQuisition (must be unique)
        "TimeUS,FwdRF,PosX,PosY,PosZ,XP,YP,ZP,XF,ZF,IsValid,Ret",   // variable names
        "smmmmmmmmm--",                                             // base units
        "FBBBBBBBBB00",                                             // exponents
        "QhfffiiihhBi",                                             // types
        TimeUS,
        FwdRF,
        #if 0
        PosX, PosY, PosZ,
        #else 
        (double) PosX, (double) PosY, (double) PosZ,        // as in libraries/AC_AttitudeControl/AC_PosControl.cpp, ln 860
        #endif // 1
        XP, YP, ZP,
        XF, ZF, ((uint8_t) IsValid), Ret                            // no bool available, sizeof(bool) is 1
    );
    #endif // IS_LOG_EXTRA_XF_ZF_CONSTRAINT
    return true;    // this is redundant
}
#endif // IS_LOG_GPA

// scan a point of the ground profile, using forward facing rangefinder value and "absolute position"
//  (with regard to the position of start())
// fwd_rangefinder_dist_cm in cm
// position_neu: .x: north of home position in cm, .y: east of home position in cm, .z: up of home
// return: ground_profile index of the new point that has been scanned, negative if it hasn't been stored
int AC_GroundProfileAcquisition::scan_point(int16_t fwd_rangefinder_dist_cm, Vector3f position_neu_cm) {

    // cf. prototype simulator (in python): AnticipatingFFC.set_future_profile_point

    // transform UAV coordinates (point P) from NEU-home-position-space into main_direction-space
    Vector2<int> main_dir_coo;
    main_dir_coo = get_main_direction_coo(position_neu_cm);
    int x_p, y_p;               // current position of the UAV at point P(x_p|y_p) in main_direction-space
    x_p = main_dir_coo.x;
    y_p = main_dir_coo.y;
    int z_p;                    // altitude in home-position-space
    z_p = (int) roundf(position_neu_cm.z);
    
    #if IS_DEBUG_GPA
    uint32_t _micros = AP_HAL::micros();
    if (IS_TRIGGER_EVENT_ROUGHLY_EVERY_N_SEC_MICROS(1, _micros, 400)) {
        hal.console->printf("----\nGPA: AP_HAL::micros: %" PRIu32 "\n", _micros);
        hal.console->printf("GPA: scan_point:\n");
        hal.console->printf("GPA: x_p: %d, y_p: %d\n", x_p, y_p);
        //
        hal.console->printf("position_neu: x: %8f, y: %8f, z: %8f\n", 
                    position_neu_cm.x, position_neu_cm.y, position_neu_cm.z);
    }
    #endif // IS_DEBUG_GPA

    // check y_p (distance from main_direction line), feedback will be handled from within update_<gpa>
    if (abs(y_p) > GPA_MAX_DEVIATION_FROM_MAIN_DIRECTION_CM) {

        #if IS_DEBUG_GPA
        #if 0
        gcs().send_text(MAV_SEVERITY_ERROR, "far from GPA axis! %d cm", 
            y_p);
        #endif // 1
        if (IS_TRIGGER_EVENT_ROUGHLY_EVERY_N_SEC_MICROS(1, _micros, 400)) {
            hal.console->printf("GPA: far from GPA axis! %d cm\n", y_p);
        }
        #endif // IS_DEBUG_GPA

        #if IS_LOG_GPA
        log_scan_point(AP_HAL::micros64(), fwd_rangefinder_dist_cm, 
            position_neu_cm.x, position_neu_cm.y, position_neu_cm.z,
            x_p, y_p, z_p, 
            GROUND_PROFILE_ACQUISITION_NO_DATA_VALUE, GROUND_PROFILE_ACQUISITION_NO_DATA_VALUE, false,
            ScanPointInvalidReturnValue_DEVIATION_FROM_MAIN_DIRECTION_EXCEEDED);
        #endif 

        return ScanPointInvalidReturnValue_DEVIATION_FROM_MAIN_DIRECTION_EXCEEDED;
    }

    /// calculate absolute position of the future point to be scanned (called F)
    // int16_t h1;     // altitude over ground for current position                [cm]
    int16_t h2;     // altitude over ground for projected future point F        [cm]
    int16_t dx2;    // horizontal distance from current position P to F         [cm]
    // h1  = dwn_rangefinder_dist_cm;
    h2  = RANGEFINDER_COS_ANGLE_FORWARD_FACING * fwd_rangefinder_dist_cm;
    dx2 = RANGEFINDER_SIN_ANGLE_FORWARD_FACING * fwd_rangefinder_dist_cm;
    int16_t x_f, z_f;   // position of F with respect to start_position_neu in  [cm] in main_direction space
    x_f = (x_p + dx2) / 1;      // 1 cm is for 1 array cell    

    // pre check if value is in range (as overflow couldn't be detected)
    if (((z_p - h2) <= GROUND_PROFILE_ACQUISITION_NO_DATA_VALUE) || (INT16_MAX < (z_p - h2))) {
        #if IS_DEBUG_GPA
        if (IS_TRIGGER_EVENT_ROUGHLY_EVERY_N_SEC_MICROS(1, _micros, 400)) {
            hal.console->printf("GPA debug: (z_p - h2) out of range, (z_p - h2) = %d\n", (z_p - h2));
        }
        #endif // IS_DEBUG_GPA

        #if IS_LOG_GPA
        log_scan_point(AP_HAL::micros64(), fwd_rangefinder_dist_cm, 
            position_neu_cm.x, position_neu_cm.y, position_neu_cm.z,
            x_p, y_p, z_p, 
            x_f, GROUND_PROFILE_ACQUISITION_NO_DATA_VALUE, false,
            ScanPointInvalidReturnValue_VALUE_OUT_OF_RANGE);
        #endif 
        
        return ScanPointInvalidReturnValue_VALUE_OUT_OF_RANGE;
    }
    // y_f = y_p - h2;  WRONG! 
    z_f = z_p - h2;
#if IS_USE_GPA_MAP_OFFSET
    // an additional conditional branch for every cycle :/
    if (!is_ground_profile_offset_set) {
        ground_profile_offset = z_f;
        is_ground_profile_offset_set = true;
    }
    z_f -= ground_profile_offset;
#endif // IS_USE_GPA_MAP_OFFSET
    // not sure why sometimes index 0 has a value != 0... perhaps

    // calculate array index (for ground_profile)
    int ground_profile_index;

    // use "hard" insert
    ground_profile_index = x_f;

    // TODO: prio 7: implement "soft" insert

    // check if index is in array range
    if (x_f < 0) {
        // vorwaerts immer, rueckwaerts nimmer ;)

        #if IS_DEBUG_GPA
        if (IS_TRIGGER_EVENT_ROUGHLY_EVERY_N_SEC_MICROS(1, _micros, 400)) {
            hal.console->printf("GPA debug: x_f < 0, x_f = %d\n", x_f);
        }
        #endif // IS_DEBUG_GPA

        #if IS_LOG_GPA
        log_scan_point(AP_HAL::micros64(), fwd_rangefinder_dist_cm, 
            position_neu_cm.x, position_neu_cm.y, position_neu_cm.z,
            x_p, y_p, z_p, 
            x_f, z_f, false,
            ScanPointInvalidReturnValue_GROUND_PROFILE_INDEX_NEGATIVE);
        #endif 

        return ScanPointInvalidReturnValue_GROUND_PROFILE_INDEX_NEGATIVE;
    } else if (x_f >= GROUND_PROFILE_ACQUISITION_PROFILE_ARRAY_SIZE) {
        // too big for the array
        
        #if IS_DEBUG_GPA
        if (IS_TRIGGER_EVENT_ROUGHLY_EVERY_N_SEC_MICROS(1, _micros, 400)) {
            hal.console->printf("GPA debug: x_f too big for array, x_f = %d\n", x_f);
        }
        #endif // IS_DEBUG_GPA

        #if IS_LOG_GPA
        log_scan_point(AP_HAL::micros64(), fwd_rangefinder_dist_cm, 
            position_neu_cm.x, position_neu_cm.y, position_neu_cm.z,
            x_p, y_p, z_p, 
            x_f, z_f, false,
            ScanPointInvalidReturnValue_GROUND_PROFILE_INDEX_TOO_HIGH);
        #endif 

        return ScanPointInvalidReturnValue_GROUND_PROFILE_INDEX_TOO_HIGH;
    }

    // check if value is in range
    //if ((z_f <= GROUND_PROFILE_ACQUISITION_NO_DATA_VALUE) || (INT16_MAX < z_f)) {
    // int16_t is implicitly constraining to INT16_MAX, if value is too high, precheck will detect this
    if (z_f <= GROUND_PROFILE_ACQUISITION_NO_DATA_VALUE) {      
        #if IS_DEBUG_GPA
        if (IS_TRIGGER_EVENT_ROUGHLY_EVERY_N_SEC_MICROS(1, _micros, 400)) {
            hal.console->printf("GPA debug: z_f out of range, z_f = %d\n", z_f);
        }
        #endif // IS_DEBUG_GPA

        #if IS_LOG_GPA
        log_scan_point(AP_HAL::micros64(), fwd_rangefinder_dist_cm, 
            position_neu_cm.x, position_neu_cm.y, position_neu_cm.z,
            x_p, y_p, z_p, 
            x_f, z_f, false,
            ScanPointInvalidReturnValue_VALUE_OUT_OF_RANGE);
        #endif 

        return ScanPointInvalidReturnValue_VALUE_OUT_OF_RANGE;
    }

    /// insert value (hard or soft, using a filter, check for outliers)
    // hard insert, not robust against outliers!
    // TODO: prio 7: implement soft insert
    ground_profile[ground_profile_index] = z_f;

    #if IS_PRINT_GPA_NEW_POINT
    last_scanned_point.time_us = AP_HAL::micros();
    last_scanned_point.fwd_rangefinder_dist_cm = fwd_rangefinder_dist_cm;
    last_scanned_point.position_neu_cm = position_neu_cm;
    last_scanned_point.x_f = x_f;
    last_scanned_point.y_p = y_p;
    last_scanned_point.z_f = z_f;
    #endif // IS_PRINT_GPA_NEW_POINT

    #if IS_LOG_GPA
    log_scan_point(AP_HAL::micros64(), fwd_rangefinder_dist_cm, 
        position_neu_cm.x, position_neu_cm.y, position_neu_cm.z,
        x_p, y_p, z_p, 
        x_f, z_f, true,
        ground_profile_index);
    #endif 

    return ground_profile_index;
}

#endif // 1 OR 0
#endif // IS_USE_WORKAROUND_GROUND_PROFILE_ACQUISITION && IS_USE_WORKAROUND_HOST_FILE_GPA