/*
 * UDHAL_PWM.h
 *
 *  Created on: 10 May 2024
 *      Author: Chee
 */

#ifndef UDHAL_UDHAL_PWM_H_
#define UDHAL_UDHAL_PWM_H_

#ifdef _cplusplus
extern "C"
{
#endif

/*  ======== pwmled2.c ========
 *************************************/
#include <stddef.h>
#include <stdint.h>
/* Driver Header files */
#include <ti/drivers/PWM.h>
/* Driver configuration */
#include <ti_drivers_config.h>

#include "Hardware/gGo_device_params.h"

#define LIGHT_PWM_PERIOD                                1000    // 1000 microsecond => 1kHz
#define BUZZER_PWM_FREQUENCY                            4200
/* Auxiliary Light  */
#ifdef AUXILIARY_LIGHT
#define AUXILIARY_PWM_PERIOD                            1000    // 1000 microsecond => 1kHz
#endif //AUXILIARY_LIGHT

extern void UDHAL_PWM_init();
extern uint8_t UDHAL_PWM_paramInit();
extern void UDHAL_PWM_setHLDutyAndPeriod(uint8_t dutyPercent);
extern void UDHAL_PWM_setBUZDutyAndPeriod(uint8_t dutyPercent, uint32_t pwmBUZFrequency);
#ifdef AUXILIARY_LIGHT
extern void UDHAL_PWM_setALDutyAndPeriod(uint8_t dutyPercent);
#endif // AUXILIARY_LIGHT

#ifdef _cplusplus
}
#endif


#endif /* UDHAL_UDHAL_PWM_H_ */
