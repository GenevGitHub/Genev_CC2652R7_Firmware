/*
 * UDHAL_I2C.h
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */

#ifndef UDHAL_UDHAL_I2C_H_
#define UDHAL_UDHAL_I2C_H_

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
#include <ti/drivers/I2C.h>

extern void UDHAL_I2C_init();
extern uint8_t UDHAL_I2C_paramInit();
extern I2C_Handle UDHAL_I2C_getI2CStatus();
extern uint8_t UDHAL_I2C_getI2COpenStatus();

#ifdef _cplusplus
}
#endif

#endif /* UDHAL_UDHAL_I2C_H_ */
