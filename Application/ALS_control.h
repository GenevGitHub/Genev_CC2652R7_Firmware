/*
 * ALS_control.h
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */

#ifndef APPLICATION_ALS_CONTROL_H_
#define APPLICATION_ALS_CONTROL_H_

#ifdef _cplusplus
extern "C"
{
#endif
/*********************************************************************
 * INCLUDES
 */
#include "Hardware/gGo_device_params.h"

/*********************************************************************
* CONSTANTS
*/
/** #define the ambient light sensor chip that is in use **/
#define veml6030            1
//#define veml3235            1

#define ALS_NUMSAMPLES      5    // The number of samples used for light intensity evaluation --> must be 8


/* ********************************************************************
 * API FUNCTIONS
 */
extern uint8_t ALS_control_init();
extern void* ALS_control_flagbRegister(void);
extern uint8_t ALS_control_calculateLux();

#ifdef veml6030

extern uint8_t ALS_control_getIntR();

#endif

#ifdef _cplusplus
}
#endif



#endif /* APPLICATION_ALS_CONTROL_H_ */
