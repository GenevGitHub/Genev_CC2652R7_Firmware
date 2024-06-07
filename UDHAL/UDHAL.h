/*
 * UDHAL.h
 *
 *  Created on: 4 May 2024
 *      Author: Chee
 */

#ifndef UDHAL_UDHAL_H_
#define UDHAL_UDHAL_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <stdlib.h>
#include <stdint.h>

#include "UDHAL/UDHAL_ADC.h"
#include "UDHAL/UDHAL_GPIO.h"
#include "UDHAL/UDHAL_I2C.h"
#include "UDHAL/UDHAL_UART.h"
#include "UDHAL/UDHAL_PWM.h"

#include "UDHAL/UDHAL_Timer2.h"
#include "UDHAL/UDHAL_Timer3.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */
#define UDHAL_SUCCESS         0
#define UDHAL_FAILURE         1

/* TYPEDEFS
*/
// This set of data is stored in ram. It is used to store variables from Motor Controller Unit
typedef struct sysFatalError{
    uint8_t     UARTfailure;
    uint8_t     PWMfailure;
    uint8_t     I2Cfailure;
    uint8_t     ADCfailure;
    uint8_t     Otherfailure;
}sysFatalError_t;

/*********************************************************************
 * FUNCTIONS
 */
extern uint8_t UDHAL_init();
extern void* UDHAL_sysFatalErrorRegister();
extern void UDHAL_Boot();

#ifdef __cplusplus
}
#endif



#endif /* UDHAL_UDHAL_H_ */
