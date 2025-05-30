/*
 * gGo_debug_config.h
 * Note:  This header file is created to contain all constants and parameters
 *        for debugging and testing purposes.
 *
 * Parameters and Constants
 *      MOTOR_CONNECT = connected to MCU and retrieving data from MCU.
 *                      if not defined, Dummy data is used.
 *      MOTOR_1RPM_START_MODE = allow 1 rpm motor start
 *                      if not defined, motor is only instructed to start at or above the minimum rpm.
 *
 *  Created on: 5 Nov 2024
 *      Author: Chee
 */

#ifndef HARDWARE_GGO_DEBUG_CONFIG_H_
#define HARDWARE_GGO_DEBUG_CONFIG_H_


#ifdef _cplusplus
extern "C"
{
#endif

/*********************************************************************
* Libraries
*/
#include <stdio.h>
#include <stdint.h>

/*********************************************************************
* CONSTANTS
*/
/**     Define MOTOR_CONNECT when connected to Motor Control Unit
 *      Note:   If MOTOR_CONNECT is "NOT" defined -> dummy data becomes active
 *              If MOTOR_CONNECT = 1 -> Receiving MCU data = MCU_DATA_ON
 **/
#define MOTOR_CONNECT           1         // MOTOR_CONNECT = 1 = Receiving MCU data = MCU_DATA_ON
#ifdef MOTOR_CONNECT
#define ADC_CORRECTION_FACTOR   1       //  ADC_CORRECTION_FACTOR corrects ADC value due to VCC difference between LaunchPad and MOTOR CONNECT
#endif // MOTOR_CONNECT
#ifndef MOTOR_CONNECT
#define ADC_CORRECTION_FACTOR   1.03    //  ADC_CORRECTION_FACTOR corrects ADC value due to VCC difference between LaunchPad and MOTOR CONNECT
#endif // MOTOR_CONNECT


#ifdef _cplusplus
}
#endif

#endif /* HARDWARE_GGO_DEBUG_CONFIG_H_ */
