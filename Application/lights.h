/*
 * lights.h
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */

#ifndef APPLICATION_LIGHTS_H_
#define APPLICATION_LIGHTS_H_
#ifdef __cplusplus
extern "C"
{
#endif
//
/*********************************************************************
 * INCLUDES
 */
#include<stdio.h>
#include<stdint.h>

#include "Hardware/gGo_device_params.h"
#include "Application/motor_control.h"

/*********************************************************************
* CONSTANTS
*/
#define NUM_PERIODIC_COMMUNICATION_HF_2_ALS             2    // in milliseconds, the time between ALS sampling = PERIODIC_COMMUNICATION_HF_SAMPLING_TIME x N_ALS, i.e. ALS_SAMPLING_TIME

// there is actually no need to sample light level too frequently.
// Perhaps frequency of 1 second is sufficient.
// lightControl timer is timer 8
// For example, total time interval between AUTO light ON/OFF is (ALS_NUMSPLES - 1) * ALS_SAMPLING_TIME => (8-1) * 250 = 1750 milliseconds
// For example, total time interval between AUTO light ON/OFF is (ALS_NUMSPLES - 1) * ALS_SAMPLING_TIME => (8-1) * 300 = 2100 milliseconds
// For example, total time interval between AUTO light ON/OFF is (ALS_NUMSPLES - 1) * ALS_SAMPLING_TIME => (3-1) * 1000 = 2000 milliseconds
#define LIGHT_MODE_OFF                                  0x00
#define LIGHT_MODE_ON                                   0x01
#define LIGHT_MODE_AUTO                                 0x02
#define LIGHT_MODE_INITIAL                              LIGHT_MODE_AUTO

#define LIGHT_STATUS_OFF                                0x00
#define LIGHT_STATUS_ON                                 0x01
#define LIGHT_STATUS_INITIAL                            LIGHT_STATUS_OFF

#define LIGHT_PWM_DUTY                                  75     //  in percentage, 50%


//ALS is abbreviation for Ambient Light Sensor
/* ********************************************************************
 * TYPEDEFS
*/


/* ********************************************************************
* MACROS
*/

/* ********************************************************************
 * API FUNCTIONS
 */
extern void* lights_lightModeRegister();
extern void* lights_lightStatusRegister(void);
extern void lights_STM32MCPDArrayRegister(STM32MCPD_t *ptrSTM32MCDArray);

static void lights_MODE_AUTO( void );
static void lights_MODE_OFF( void );
static void lights_MODE_ON( void );

extern void lights_init( uint8_t lights_i2cOpenStatus, uint8_t uart2ErrorStatus, uint8_t lightmodeInit );
extern void lights_ALSFxn();
extern void lights_statusChg(void);
extern uint8_t lights_getLightStatus();
extern uint8_t lights_lightModeChange( void );

#ifdef _cplusplus
}
#endif


#endif /* APPLICATION_LIGHTS_H_ */
