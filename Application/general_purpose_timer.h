/*
 * general_purpose_timer.h
 *
 *  Created on: 30 Apr 2024
 *      Author: Chee
 */

#ifndef APPLICATION_GENERAL_PURPOSE_TIMER_H_
#define APPLICATION_GENERAL_PURPOSE_TIMER_H_

#ifdef _cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
/* Library Header files */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <ti/sysbios/knl/Task.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* Application Header files */
#include "Hardware/gGo_device_params.h"

/*********************************************************************************************
 *  Constants
 *********************************************************************************************/
// Task configuration
#define GPT_TASK_PRIORITY           4
#ifndef GPT_TASK_STACK_SIZE
#define GPT_TASK_STACK_SIZE         1024    // Stack size must be multiples of 8
#endif

/** GPT_TIME is the smallest time interval unit in General Purpose Timer **/
#define GPT_TIME                    50      // in milli-seconds
#define GPT_INACTIVE                0
#define GPT_ACTIVE                  1
#define EXECUTION_INTERVAL_2        150     // in milli-seconds
#define EXECUTION_INTERVAL_3        500     // in milli-seconds
/*********************************************************************
 * FUNCTIONS
 *********************************************************************/
extern void GeneralPurposeTimer_createTask(void);
extern void gpt_InitComplFlagRegister(uint8_t *ptr_initComplete_flag);
extern void gpt_powerOnRegister(bool *ptrpowerOn);
extern void gpt_registeropcode(uint8_t *ptr_opcode, uint8_t *ptr_advertiseFlag);
extern void* gpt_snvWriteFlageRegister();

#ifdef _cplusplus
}
#endif



#endif /* APPLICATION_GENERAL_PURPOSE_TIMER_H_ */
