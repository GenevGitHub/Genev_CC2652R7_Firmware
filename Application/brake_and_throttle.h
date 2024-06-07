/*
 * brake_and_throttle.h
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */

#ifndef APPLICATION_BRAKE_AND_THROTTLE_H_
#define APPLICATION_BRAKE_AND_THROTTLE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <stdint.h>
#include <math.h>
#include "Hardware/gGo_device_params.h"
#include "Application/motor_control.h"


//#include "motor_control.h"
/*********************************************************************
*  EXTERNAL VARIABLES
*/
/*********************************************************************
 * CONSTANTS
 */
// Speed limit protection selection
// * Direct Law -> speed limit protection is deactivated
// * Normal law -> speed limit protection is activated
#define BRAKE_AND_THROTTLE_NORMALLAW                              1
#define BRAKE_AND_THROTTLE_DIRECTLAW                              0

// The parameters GPT_TIME & BRAKE_AND_THROTTLE_SAMPLES control the sensitivity of the throttle input to motor output
#define BRAKE_AND_THROTTLE_SAMPLES                                3     // 3 samples seem ideal, 5 is okay, 8 is too laggy

//Speed modes
#define BRAKE_AND_THROTTLE_SPEED_MODE_AMBLE                       0x00
#define BRAKE_AND_THROTTLE_SPEED_MODE_LEISURE                     0x01
#define BRAKE_AND_THROTTLE_SPEED_MODE_SPORTS                      0x02

//Speed mode TORQUEIQ reduction ratio
#define BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_AMBLE       60        //60%   Pout = 240 W
#define BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_LEISURE     70        //70%   Pout = 270 W
#define BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_SPORTS      80        //80%   Pout = 300 W

//Speed mode ramp rate (acceleration) in milliseconds
#define BRAKE_AND_THROTTLE_RAMPRATE_AMBLE                         3000      // 4000
#define BRAKE_AND_THROTTLE_RAMPRATE_LEISURE                       2250      // 3000
#define BRAKE_AND_THROTTLE_RAMPRATE_SPORTS                        1500      // 2000

//Speed mode Torque IQ value
#define BRAKE_AND_THROTTLE_TORQUEIQ_MAX                           20000     // IQ 16000 = 14.222 Amp

//Hard braking definition   (What is Hard Braking? why is this necessary?)
#define HARD_BRAKING_THROTTLE_PERCENTAGE                          5
#define HARD_BRAKING_BRAKE_PERCENTAGE                             5
#define BRAKEPERCENTTHRESHOLD                                     30
#define THROTTLEPERCENTTHRESHOLD                                  30

//Throttle calibration values = value range the throttle ADC is conditioned to be within
#define THROTTLE_ADC_CALIBRATE_H                                  2350
#define THROTTLE_ADC_CALIBRATE_L                                  850

//Throttle error thresholds = values that should not be possible under nominal operation
#define THROTTLE_ADC_THRESHOLD_H                                  2700
#define THROTTLE_ADC_THRESHOLD_L                                  600

//Brake calibration values = value range the Brake ADC is conditioned to be within
#define BRAKE_ADC_CALIBRATE_H                                     2350
#define BRAKE_ADC_CALIBRATE_L                                     850

//Brake error thresholds = values that should not be possible under nominal operation
#define BRAKE_ADC_THRESHOLD_H                                     2700
#define BRAKE_ADC_THRESHOLD_L                                     600

//Error message
#define BRAKE_AND_THROTTLE_NORMAL                                 0x00
#define THROTTLE_ERROR                                            0x0C
#define BRAKE_ERROR                                               0x0E
#define HARD_BRAKING_ERROR                                        0x0F

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */
/*
 * Task creation function for the Simple Peripheral.
 */
static void brake_and_throttle_normalLawControl();
extern void brake_and_throttle_motorControl_rpm(uint16_t *ptrBrakeAndThrottleRPM);

extern void brake_and_throttle_init();
extern uint8_t* bat_dashboardErrorCodeStatusRegister();
extern void brake_and_throttle_MCUArrayRegister(MCUD_t *ptrMCUDArray);

extern void brake_and_throttle_setSpeedMode(uint8_t speed_Mode);
extern uint8_t brake_and_throttle_getSpeedMode();
extern uint8_t brake_and_throttle_toggleSpeedMode();

extern void brake_and_throttle_ADC_conversion();
extern uint16_t brake_and_throttle_getThrottlePercent();
extern uint16_t brake_and_throttle_getBrakePercent();

extern uint8_t brake_and_throttle_getControlLaw();
extern void brake_and_throttle_setControlLaw(uint8_t newControlLaw);

/*********************************************************************
*********************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_BRAKE_AND_THROTTLE_H_ */