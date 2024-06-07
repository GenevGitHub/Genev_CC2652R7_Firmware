/*
 * motor_control.h
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */

#ifndef APPLICATION_MOTOR_CONTROL_H_
#define APPLICATION_MOTOR_CONTROL_H_


#ifdef _cplusplus
extern "C"
{
#endif
/*********************************************************************
 * INCLUDES
 */
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <icall.h>

#include "Hardware/gGo_device_params.h"

#include "simple_peripheral.h"

/*********************************************************************
* CONSTANTS
*/
#define SBP_MC_GATT_EVT                        0x0020
#define SBP_MC_ADV_EVT                         0x0040   // appears not used
#define PWR_MGNT_STACK_SIZE                    128      // appears not used
#define PWR_MGNT_PRIORITY                      5        // appears not used

/* ********************************************************************
 * TYPEDEFS
*/
// This set of data is stored in ram. It is used to store variables from Motor Controller Unit
typedef struct mcuData{
        uint16_t    bat_voltage_mV;
        uint16_t    bat_current_mA;
        uint16_t    phase_voltage_mV;
        uint16_t    phase_current_mA;
        uint16_t    speed_rpm;
        uint8_t     heatSinkTempOffset50_Celcius;
        uint8_t     motorTempOffset50_Celcius;
        uint8_t     count_hf;
}MCUD_t;

/* ********************************************************************
* MACROS
*/

/* ********************************************************************
 * API FUNCTIONS
 */
extern uint8_t Boot();
extern void motor_control_init(void);
extern void motorcontrol_registerCB(simplePeripheral_bleCBs_t *obj);

extern void motor_control_speedModeChgCB(uint16_t torqueIQ, uint16_t allowableSpeed, uint16_t rampRate);
//extern void motor_control_dataAnalysis_sampling(uint8_t x_hf);
/*********************************************************************
*********************************************************************/

#ifdef _cplusplus
}
#endif

#endif /* APPLICATION_MOTOR_CONTROL_H_ */
