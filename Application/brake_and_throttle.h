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

/*********************************************************************
*  EXTERNAL VARIABLES
*/
/*********************************************************************
 * CONSTANTS
 */
#define RPM_SAMPLES                                               3     // Moving average sampling to smooth out rpm measurements. Min. 1 - Max 5
#define BRAKE_AND_THROTTLE_SAMPLES                                10//8     // Moving average sampling to smooth out throttle and brake adc input (3-10 recommended)

#define KICKSTART_STATE                                         0x00
#define BRAKESTART_STATE                                        0x01
#define THROTTLE_CONTROL_STATE                                  0x02
#define CHARGING_STATE                                          0x03

// Direct mode controls IQ value by throttle percentage
/* Direct Mode Params   */
#define MIN_KICK_START_THROTTLE_PERCENT                           35    // throttle% != IQ%. Note: Throttle% and IQ is not linear. See table below
#define MIN_BRAKE_START_THROTTLE_PERCENT                          18
#define THROTTLE_INCREMENT_LIMIT                                  0.15     // in decimal, the maximum increase in applied throttle per GPT cycle
#define DIRECT_MODE_AMBLE_ACCELERATION_THRESHOLD                  27    // rpm per second (27 rpm per second = 1km per second)
#define DIRECT_MODE_LEISURE_ACCELERATION_THRESHOLD                36    // rpm per second
#define DIRECT_MODE_SPORTS_ACCELERATION_THRESHOLD                 48    // rpm per second
#define DIRECT_MODE_IQ_EXPONENT                                   1
/*********************************************
 * Throttle%    IQ% @ DIRECT_MODE_IQ_EXPONENT=1
 *      0               00
 *     10               01
 *     20               05
 *     30               11
 *     40               19
 *     50               29
 *     60               41
 *     70               55
 *     80               69
 *     90               84
 *     100              100
 ********************************************/
// Normal mode controls IQ value by RPM-demand
/* Normal Mode Params   */
#define KICK_START_IQ_PERCENT_BIAS                                40    //35
#define BRAKE_START_IQ_PERCENT_BIAS                               25    //20

#define NORMAL_MODE_AMBLE_ACCELERATION_THRESHOLD                  40//40    // rpm per second (185.7 rpm/s = 3km/hr to 10 km/hr in 1 sec)
#define NORMAL_MODE_LEISURE_ACCELERATION_THRESHOLD                40//40    // rpm per second
#define NORMAL_MODE_SPORTS_ACCELERATION_THRESHOLD                 40//40    // rpm per second

#define MAX_IQ_DECREMENT_PERCENT                                  8
#define MAX_IQ_INCREMENT_PERCENT                                  15//12
#define DELTA_ERROR_THRESHOLD                                     80// delta error = delta rpm
#define K_VOLT_EXP                                                0.5
#define THROTTLE_FACTOR_CEILING                                   2.7//2.5
#define THROTTLE_FACTOR_EXP1                                      1.5//1.5
#define THROTTLE_FACTOR_EXP2                                      1.0//1.0
//Speed modes
#define BRAKE_AND_THROTTLE_SPEED_MODE_AMBLE                       0x00
#define BRAKE_AND_THROTTLE_SPEED_MODE_LEISURE                     0x01
#define BRAKE_AND_THROTTLE_SPEED_MODE_SPORTS                      0x02

/***** Direct Law max IQ percentage ********/
// Note: IQ 15750 is approximately 14000 milli-Amp.  @ 9600mAh -> 14000mA = 1.458C
// 50.6% IQmax = 10015 -> 9000 mA
// 56.3% IQmax = 11250 -> 10000 mA
// 61.9% IQmax = 12375 -> 11000 mA
// 67.5% IQmax = 13500 -> 12000 mA
// 73.1% IQmax = 14625 -> 13000 mA
#define BRAKE_AND_THROTTLE_DIRECT_MODE_REDUCTION_RATIO_AMBLE      45        //45%   IQmax = 9000 -> 8000 mA
#define BRAKE_AND_THROTTLE_DIRECT_MODE_REDUCTION_RATIO_LEISURE    50        //50%   IQmax = 10000 -> 8888.9 mA
#define BRAKE_AND_THROTTLE_DIRECT_MODE_REDUCTION_RATIO_SPORTS     55        //55%   IQmax = 11000 -> 9777.8 mA

/**** Normal Law TORQUEIQ reduction ratio - IQmax = 20000  ****/
#define BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_AMBLE       86        //90%   IQmax = 18000 -> 16028 mA RMS
#define BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_LEISURE     86        //85%   IQmax = 17000 -> 15138 mA RMS
#define BRAKE_AND_THROTTLE_SPEED_MODE_REDUCTION_RATIO_SPORTS      86        //

//Speed mode ramp rate (acceleration) in milliseconds
#define BRAKE_AND_THROTTLE_RAMPRATE_AMBLE                         4000      // default: RAMPRATEs are set but not used.
#define BRAKE_AND_THROTTLE_RAMPRATE_LEISURE                       3000
#define BRAKE_AND_THROTTLE_RAMPRATE_SPORTS                        2000

//Brake and Throttle dynamic threshold in percentage - used to trigger brake status and throttle status during dynamic conditions
#define BRAKEPERCENTTHRESHOLD                                     30    //%
#define THROTTLEPERCENTTHRESHOLD                                  30    //%

//Throttle calibration values = value range the throttle ADC is conditioned to be within
#define THROTTLE_ADC_CALIBRATE_H                                  2150  // @ 3.32V
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
// This set of data is for PID motor control
typedef struct bat_params{
    uint8_t     motor_state;
    uint8_t     brakeStatus;
    uint8_t     chargingStatus;
    uint16_t    throttle_rpm;             // throttle demand rpm
    uint16_t    instant_rpm;              // current rpm
    uint16_t    speedModeIQmax;
    uint16_t    allowableRPM;
    uint16_t    IQ_value;
    float       IQ_increment_factor;
    float       acceleration;
    float       accelerationThreshold;
    float       K_volt;
}bat_params_t;

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
