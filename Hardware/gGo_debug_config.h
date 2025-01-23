/*
 * gGo_debug_config.h
 * Note:  This header file is created to contain all constants and parameters
 *        for debugging and testing purposes.
 *
 * Parameters and Constants
 *      MOTOR_CONNECT = connected to MCU and retrieving data from MCU.
 *                      if not defined, Dummy data is used.
 *      MOTOR_0RPM_START_MODE = allow 0 rpm motor start
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


/** Note: If MOTOR_0RPM_START_MODE is NOT defined -> Minimum IQ speed, i.e. REG_MINP_RPM, is active
 *          otherwise, minimum IQ speed is zero
 ***/
//#define MOTOR_0RPM_START_MODE          1


#ifdef _cplusplus
}
#endif

#endif /* HARDWARE_GGO_DEBUG_CONFIG_H_ */
