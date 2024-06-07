/*
 * UDHAL_TIM2.h
 *  This library is used for STM32MCP/STM32MCP.h to counter the heartbeat duration
 *  Created on:   26 Jan 2021 by Siu Yeung Yik
 *  Last Updated: 26 Jan 2021 by Siu Yeung Yik
 */
#ifndef UDHAL_TIM2_UDHAL_TIM2_H_
#define UDHAL_TIM2_UDHAL_TIM2_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "Hardware/gGo_device_params.h"

#include "Hardware/STM32MCP.h"

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
extern void UDHAL_TIMER2_init();
extern void UDHAL_TIMER2_params_init();
extern STM32MCP_timerManager_t* UDHAL_TIMER2_timerRegister();

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* UDHAL_TIM2_UDHAL_TIM2_H_ */
