/*
 * UDHAL_ADC.h
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */

#ifndef UDHAL_UDHAL_ADC_H_
#define UDHAL_UDHAL_ADC_H_

#ifdef _cplusplus
extern "C"
{
#endif
//
/*********************************************************************
 * INCLUDES
 */
#include <stdint.h>
#include <stddef.h>

/* Driver configuration */
#include <ti_drivers_config.h>
#include <ti/drivers/ADC.h>

extern void UDHAL_ADC_init();
extern uint8_t UDHAL_ADC_throttleOpen();
extern uint8_t UDHAL_ADC_brakeOpen();
extern void UDHAL_ADC_Convert();
extern void UDHAL_ADC_close(ADC_Handle adcHandle);
extern void UDHAL_ADC_ptrADCValues(uint16_t (*ptrADCvalues)[]);

#ifdef _cplusplus
}
#endif

#endif /* UDHAL_UDHAL_ADC_H_ */
