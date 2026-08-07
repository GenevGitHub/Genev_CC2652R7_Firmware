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
#define PID_FPO         0.16//0.1
#define PID_TCI         5//5
#define PID_TCD_S       4.3//4.3
#define PID_TCD_L       4.5//4.5
#define PID_TCD_A       4.4//4

//PID CONSTANTS FOR NEGATIVE ERROR
#define PID_FNE         0.025//0.01
#define PID_TFI         1
#define PID_TFD         2

#define RPMMIN_FACTOR   0.5    // must be greater than 0.3
#define CLAMPACC_S      -25
#define CLAMPACC_L      -15
#define CLAMPACC_A      -5

// Increment factor enables the IQ increment size of each mode to be individually tuned
#define IQ_INCREMENT_FACTOR_AMBLE                                 0.82//0.83
#define IQ_INCREMENT_FACTOR_LEISURE                               0.90//0.9
#define IQ_INCREMENT_FACTOR_SPORTS                                1.12//1.10

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Typedef
 */
// This set of data is for PID control algorithm
typedef struct pid_params{
    uint16_t rpm;               // rpm
    uint16_t rpm0;              // previous rpm
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
}pid_params_t;

/*********************************************************************
 * FUNCTIONS
 */
/*
 * Task creation function.
 */
extern void PID_control_init();
extern void pid_control_batParamsRegister(bat_params_t *ptrBatParams);
extern uint16_t PID_speedControl();

/*********************************************************************
*********************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_PID_CONTROL_H_ */
