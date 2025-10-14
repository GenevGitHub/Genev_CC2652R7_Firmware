/*
 * pid_control.h
 *
 *  Created on: 11 Jun 2025
 *      Author: yuche
 */

#ifndef APPLICATION_PID_CONTROL_H_
#define APPLICATION_PID_CONTROL_H_


#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <stdint.h>
#include "Hardware/gGo_device_params.h"
#include "Hardware/gGo_debug_config.h"

#include "Application/brake_and_throttle.h"
#include "Application/general_purpose_timer.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/
/*********************************************************************
 * CONSTANTS
 */
//PID CONSTANTS FOR POSITIVE ERROR
#define PID_FPO         0.08//0.03
#define PID_TCI         0.03//0.05
#define PID_TCD         2.5
#define PID_TCD_A       3.5

//PID CONSTANTS FOR NEGATIVE ERROR
#define PID_FNE         0.08//0.01
#define PID_TFI         1
#define PID_TFD         3

#define KP_EXP          1//0.75
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Typedef
 */
// This set of data is for PID control algorithm
typedef struct pid_params{
    float kp;                   // proportional gain
    float ki;                   // integral gain
    float kd;                   // derivative gain
    float pid_error;            // error
    float pid_error0;           // previous error
    float pid_biasKS;           // pid bias action Kick Start
    float pid_biasBS;           // pid bias action Brake Start
    float pid_biasTS;           // pid bias action Throttle Start
    float pid_p;                // pid p action
    float pid_i;                // pid i action
    float pid_d;                // pid d action
    float pid_total;            // pid_total = pid_bias + pid_p + pid_i + pid_d
    float pid_total0;           // previous pid_total
    uint16_t rpm;               // rpm
    uint16_t rpm0;              // previous rpm
}pid_params_t;

/*********************************************************************
 * FUNCTIONS
 */
/*
 * Task creation function.
 */
extern void PID_control_init();

/*********************************************************************
*********************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_PID_CONTROL_H_ */
