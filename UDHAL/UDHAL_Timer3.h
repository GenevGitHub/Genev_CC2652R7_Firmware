/*
 * UDHAL_Timer3.h
 * This library is used for singleButton.h to counter the button press duration
 *  Created on: 4 May 2024
 *      Author: Chee
 */

#ifndef UDHAL_UDHAL_TIMER3_H_
#define UDHAL_UDHAL_TIMER3_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "Application/multi_purpose_button.h"

/*********************************************************************
 * FUNCTIONS
 */
extern void UDHAL_TIMER3_init();
extern void UDHAL_TIMER3_params_init();
extern void UDHAL_TIMER3_clockDelete();
extern mpb_timerManager_t* UDHAL_TIMER3_mpbTimerRegister();

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* UDHAL_UDHAL_TIMER3_H_ */
