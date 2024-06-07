/*
 * UDHAL_GPIO.h
 *
 *  Created on: 4 May 2024
 *      Author: Chee
 */

#ifndef UDHAL_UDHAL_GPIO_H_
#define UDHAL_UDHAL_GPIO_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <stdlib.h>
#include <stdint.h>
/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */
/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * FUNCTIONS
 */
extern void UDHAL_GPIO_init();
extern void UDHAL_GPIO_params_init();
extern uint_fast8_t UDHAL_GPIO_read(uint_least8_t index);
extern void UDHAL_GPIO_write(uint_least8_t index, uint8_t value);
extern void UDHAL_GPIO_InterruptFxn(uint_least8_t index);  //GPIO interrupt callback, change it to meet your device requirement

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif




#endif /* UDHAL_UDHAL_GPIO_H_ */
