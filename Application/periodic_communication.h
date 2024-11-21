/*
 * periodic_communication.h
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */

#ifndef APPLICATION_PERIODIC_COMMUNICATION_H_
#define APPLICATION_PERIODIC_COMMUNICATION_H_

#ifdef _cplusplus
extern "C"
{
#endif
/*********************************************************************
 * INCLUDES
 */
#include <stdint.h>
#include <stddef.h>
#include <math.h>

#include "Hardware/gGo_device_params.h"
#include "Hardware/gGo_debug_config.h"

#include "Application/simple_peripheral.h"

#include "Application/motor_control.h"

/*********************************************************************
 * CONSTANTS
 */
//#define PERIODIC_COMMUNICATION_ACTIVATE              0x01
//#define PERIODIC_COMMUNICATION_DEACTIVATE            0x00

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * FUNCTIONS
 */
extern void periodic_communication_MCUArrayRegister(MCUD_t *ptrMCUDArray);
extern void periodic_communication_MCUSampling();
extern void periodic_communication_MCUSamplingRPM();
extern uint8_t periodic_communication_getxhf();
extern void periodic_communication_init( void );        // added  20241110


#ifdef _cplusplus
}
#endif


#endif /* APPLICATION_PERIODIC_COMMUNICATION_H_ */
