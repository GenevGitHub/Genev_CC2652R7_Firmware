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
#include "Hardware/gGo_debug_config.h"
#include "Hardware/STM32MCP.h"

#include "Application/motor_control.h"
#include "Application/data_analytics.h"


//#include "motor_control.h"
/*********************************************************************
*  EXTERNAL VARIABLES
*/
/*********************************************************************
 * CONSTANTS
 */
#define RPM_SAMPLES                                               4
#define BRAKE_AND_THROTTLE_SAMPLES                                3     //
#define THROTTLE_INDEX_SAMPLES                                    10
#define KICKSTART_STATE                                         0x00
#define BRAKESTART_STATE                                        0x01
#define THROTTLE_CONTROL_STATE                                  0x02

// Direct mode controls IQ value by throttle percentage
/* Direct Mode Params   */
//#define RPM_REF_DIRECTMODE                                        BRAKE_AND_THROTTLE_MAXSPEED_SPORTS
#define MIN_KICK_START_THROTTLE_PERCENT                           50    // throttle% != IQ%. Note: Throttle% and IQ is not linear. See table below
#define MIN_BRAKE_START_THROTTLE_PERCENT                          33
#define THROTTLE_INCREMENT_LIMIT                                  1     // in percentage, the maximum increase in applied throttle per GPT cycle
#define DIRECT_MODE_AMBLE_ACCELERATION_THRESHOLD                  36    // rpm per second (27 rpm per second = 1km per second)
#define DIRECT_MODE_LEISURE_ACCELERATION_THRESHOLD                48    // rpm per second
#define DIRECT_MODE_SPORTS_ACCELERATION_THRESHOLD                 60    // rpm per second
#define DIRECT_MODE_IQ_EXPONENT                                   1
/*********************************************
 * Throttle%    IQ% @ DIRECT_MODE_IQ_EXPONENT=1
 *      0               0.00
 *     10               0.01
 *     20               0.05
 *     30               0.11
 *     40               0.19
 *     50               0.29
 *     60               0.41
 *     70               0.55
 *     80               0.69
 *     90               0.84
 *     100              1.00
 ********************************************/
// Normal mode controls IQ value by RPM-demand
/* Normal Mode Params   */
#define MIN_KICK_START_IQ_PERCENT                                 33    // IQ%
#define MIN_BRAKE_START_IQ_PERCENT                                15    //15
#define MIN_THROTTLE_START_IQ_PERCENT                             18    //13
#define NORMAL_MODE_AMBLE_ACCELERATION_THRESHOLD                  55    //52 rpm per second
#define NORMAL_MODE_LEISURE_ACCELERATION_THRESHOLD                60    //52 rpm per second
#define NORMAL_MODE_SPORTS_ACCELERATION_THRESHOLD                 70    //60 rpm per second
#define NN_IQ_INCREMENTS_AMBLE                                    100   //120    IQ_increment = speedModeIQmax / NN_IQ_INCREMENTS
#define NN_IQ_INCREMENTS_LEISURE                                  100   //120    IQ_increment = speedModeIQmax / NN_IQ_INCREMENTS
#define NN_IQ_INCREMENTS_SPORTS                                   80   //138   // IQ_increment = speedModeIQmax / NN_IQ_INCREMENTS

#define MAX_ACCELERATION_FACTOR                                   6
#define MIN_ACCELERATION_FACTOR                                   -1.5     //2
#define K_prop_COEFF01                                            2   //
#define K_prop_COEFF02                                            15    //13
#define K_prop_EXPONENT                                           1.3
#define MAX_IQ_DECREMENT_PERCENT                                  2

//Speed modes
#define BRAKE_AND_THROTTLE_SPEED_MODE_AMBLE                       0x00
#define BRAKE_AND_THROTTLE_SPEED_MODE_LEISURE                     0x01
#define BRAKE_AND_THROTTLE_SPEED_MODE_SPORTS                      0x02

// Direct Law max IQ percentage
// Note: IQ 15750 is approximately 14000 milli-Amp.  @ 9600mAh -> 14000mA = 1.458C
// 50.625% IQmax = 100125 -> 9000 mA
// 56.25% IQmax = 11250 -> 10000 mA
// 58% IQmax = 11600 -> 10311 mA
// 60% IQmax = 12000 -> 10667 mA
// 61.875% IQmax = 12375 -> 11000 mA
// 64.7% IQmax = 12937.5 -> 11500 mA
// 67.5% IQmax = 13500 -> 12000 mA
// 73.125% IQmax = 14625 -> 13000 mA
#define BRAKE_AND_THROTTLE_DIRECT_MODE_REDUCTION_RATIO_AMBLE      50        //56%   IQmax = 11250 -> 10000 mA -> 1.04C
#define BRAKE_AND_THROTTLE_DIRECT_MODE_REDUCTION_RATIO_LEISURE    55        //68%   IQmax = 13600 -> 12000 mA -> 1.25C
#define BRAKE_AND_THROTTLE_DIRECT_MODE_REDUCTION_RATIO_SPORTS     60        //79%   IQmax = 15800 -> 14044 mA -> 1.46C

// Normal Law TORQUEIQ reduction ratio
#define BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_AMBLE       58        //70%   IQmax = 13600 -> 12444 mA -> 1.25C
#define BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_LEISURE     58        //74%   IQmax = 14600 -> 13155 mA -> 1.35C
#define BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_SPORTS      58        //79%   IQmax = 15800 -> 14044 mA -> 1.46C

//Speed mode ramp rate (acceleration) in milliseconds
#define BRAKE_AND_THROTTLE_RAMPRATE_AMBLE                         3000      // ramp rates are not used
#define BRAKE_AND_THROTTLE_RAMPRATE_LEISURE                       2250      // ramp rates are not used
#define BRAKE_AND_THROTTLE_RAMPRATE_SPORTS                        1500      // ramp rates are not used

//Brake and Throttle dynamic threshold in percentage - used to trigger brake status and throttle status during dynamic conditions
#define BRAKEPERCENTTHRESHOLD                                     30    //%
#define THROTTLEPERCENTTHRESHOLD                                  30    //%

//Throttle calibration values = value range the throttle ADC is conditioned to be within
#define THROTTLE_ADC_CALIBRATE_H                                  2250  // @ 3.32V
#define THROTTLE_ADC_CALIBRATE_L                                  870   // @ 3.32V

//Throttle error thresholds = values that should not be possible under nominal operation
#define THROTTLE_ADC_THRESHOLD_H                                  2800
#define THROTTLE_ADC_THRESHOLD_L                                  500

//Brake calibration values = value range the Brake ADC is conditioned to be within
#define BRAKE_ADC_CALIBRATE_H                                     2300  // @ 3.32V
#define BRAKE_ADC_CALIBRATE_L                                     830   // @ 3.32V

//Brake error thresholds = values that should not be possible under nominal operation
#define BRAKE_ADC_THRESHOLD_H                                     2800
#define BRAKE_ADC_THRESHOLD_L                                     500

//Error message
#define BRAKE_AND_THROTTLE_NORMAL                                 0x00
#define THROTTLE_ERROR                                            0x0C
#define BRAKE_ERROR                                               0x0E

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Typedef
 */

/*********************************************************************
 * FUNCTIONS
 */
/*
 * Task creation function for the Simple Peripheral.
 */
extern void brake_and_throttle_STM32MCDArrayRegister(STM32MCPD_t *ptrSTM32MCPDArray);

extern void brake_and_throttle_init();
extern uint8_t* bat_dashboardErrorCodePriorityRegister();
extern void brake_and_throttle_MCUArrayRegister(MCUD_t *ptrMCUDArray);
extern void brake_and_throttle_ADArrayRegister(AD_t *ptrADArray);

extern void brake_and_throttle_setSpeedMode(uint8_t speed_mode);
extern uint8_t brake_and_throttle_getSpeedMode();
extern uint8_t brake_and_throttle_toggleSpeedMode();
extern void brake_and_throttle_getSpeedModeParams();

extern uint8_t brake_and_throttle_ADC_conversion(uint32_t gptCounter);
extern uint16_t brake_and_throttle_getThrottlePercent();
extern uint16_t brake_and_throttle_getBrakePercent();

extern uint8_t brake_and_throttle_getControlLaw();
extern void brake_and_throttle_setControlLaw(uint8_t control_law);
extern void bat_powerOnRegister(bool *ptrpowerOn);

extern uint8_t* bat_shutDownReadyRegister();

extern void bat_dashboard_speedmode_service();
extern uint8_t bat_auxiliaryLightStatusRegister(uint8_t *ptrAuxiliaryLightStatus);

static void throttleErrorHandler();
/*********************************************************************
*********************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_BRAKE_AND_THROTTLE_H_ */
