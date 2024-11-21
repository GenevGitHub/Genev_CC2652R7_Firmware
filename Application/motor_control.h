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
#include "Hardware/gGo_debug_config.h"

#include "simple_peripheral.h"

/*********************************************************************
* CONSTANTS
*/

#define SBP_MC_GATT_EVT                        0x0020
#define SBP_MC_ADV_EVT                         0x0040   // appears not used
#define PWR_MGNT_STACK_SIZE                    128      // appears not used
#define PWR_MGNT_PRIORITY                      5        // appears not used
#define ABOVE_MIN_SPEED                        0x01     // Above the minimum speed
#define BELOW_MIN_SPEED                        0x00     // Below the minimum speed
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
        uint8_t     rpm_status;
        uint8_t     heatSinkTempOffset50_Celcius;
        uint8_t     motorTempOffset50_Celcius;
        uint8_t     count_hf;
}MCUD_t;

// This set of data is stored in ram. It contains variables that commands the Motor Controller Unit
typedef struct STM32MCPData{
        uint16_t    allowable_rpm;
        uint16_t    speed_mode_IQmax;
        uint16_t    IQ_value;
        uint16_t    ramp_rate;
        uint16_t    brake_percent;
        uint8_t     error_msg;
        uint8_t     brake_status;
        uint8_t     tail_light_status;
        uint8_t     speed_mode;
}STM32MCPD_t;
/* ********************************************************************
* MACROS
*/

/* ********************************************************************
 * API FUNCTIONS
 */
extern uint8_t Boot();
extern void motor_control_init(void);
extern uint8_t motor_control_minSpeed(void);
extern void motor_control_setIQvalue();
//extern void motor_control_speedModeParamsChg();
extern void motor_control_changeSpeedMode();
extern void motor_control_brakeStatusChg();
extern void motor_control_taillightStatusChg();

/*********************************************************************
*********************************************************************/

#ifdef _cplusplus
}
#endif

#endif /* APPLICATION_MOTOR_CONTROL_H_ */
