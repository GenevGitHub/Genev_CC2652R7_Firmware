/*
 * power_on_time.h
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */

#ifndef APPLICATION_POWER_ON_TIME_H_
#define APPLICATION_POWER_ON_TIME_H_

#ifdef __cplusplus
extern "C"
{
#endif
//
/*********************************************************************
 * INCLUDES
 */
/* Library Header files */
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <ti/sysbios/knl/Task.h>

/* Driver Header files */

/* Driver configuration */

/* Application Header files */
#include "Hardware/gGo_device_params.h"

#include "Profiles/dashboard_profile.h"

/*********************************************************************************************
 *  Constants
 *********************************************************************************************/
// Task configuration
#define POT_TASK_PRIORITY           2
#define POT_TASK_STACK_SIZE         512         // Stack size must be multiples of 8

#define POT_TIME                    1000        // milli-seconds
#define MINUTE_IN_HOUR              60          // 60 minutes in an hour
#define POWERONTIME_MINUTE_TIME     60000       // milli-seconds in one minute


/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
*********************************************************************/
extern void power_on_time_init();
extern void power_on_time_createTask(void);
extern uint16_t power_on_time_getPowerOnTime();
extern void pot_powerOnRegister(bool *ptr_powerOn);
extern void pot_setDeviceUpTime(uint32_t uptimeMinutes);
extern void pot_InitComplFlagRegister(uint8_t *ptr_initComplete_flag);
extern void* pot_uptimeMinute();
extern void pot_snvWriteCompleteFlag_register(uint8_t *ptr_snvWriteComplete_flag);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_POWER_ON_TIME_H_ */
