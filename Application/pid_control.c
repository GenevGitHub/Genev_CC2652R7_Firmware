/*
 * pid_control.c
 *
 *  Created on: 11 Jun 2025
 *      Author: yuche
 */
/*********************************************************************
 * INCLUDES
 */

#include <stdint.h>
#include <math.h>

#include "Application/pid_control.h"


/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 *********************************************************************/
/*
 */
bat_params_t     *ptr_bat_params;
static uint16_t         IQ_input = 0;

/*********************************************************************
 * @fun    PID_speedControl
 *
 * @brief   Normal Law Algorithm - use PID control algorithm
 *
 * @param   Nil
 *
 * @return  Nil
 *********************************************************************/

extern void PID_control_init(){

}

/**** obtain/register the pointer to BAT_params   ****/
extern void pid_control_batParamsRegister(bat_params_t *ptrBatParams)
{
    ptr_bat_params = ptrBatParams;
}


/*********************************************************************
 * @fun    PID_speedControl
 *
 * @brief   brake_and_throttle_PID_speedControl could be moved here for better organization
 *
 *          Adaptive Speed Control Algorithm - a variable biased PID concept
 *          Too great of change in IQ can be harmful for the motor controller, motor, hardware and battery
 *          Therefore, change to throttleRPMApplied (IQ) should be progressive or incremental or decremental when reducing.
 *          Sudden large jumps from a high value to a low value or to zero instantly must be avoided.
 *          In general, always provide a soft landing to throttleRPMApplied (IQ) from non-zero value to zero.
 *
 * @param   Nil
 *
 * @return  Nil
 *********************************************************************/
extern uint16_t PID_speedControl() {

    return (IQ_input);
}


//move static void throttleErrorHandler() here
