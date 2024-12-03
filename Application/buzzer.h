/*
 * buzzer.h
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */

#ifndef APPLICATION_BUZZER_H_
#define APPLICATION_BUZZER_H_

#ifdef _cplusplus
extern "C"
{
#endif
/*********************************************************************
 * INCLUDES
 */
#include <stdint.h>
#include <stddef.h>
#include "Hardware/gGo_device_params.h"

/*********************************************************************
* CONSTANTS
*/
// constants for buzzerStatus
#define BUZZER_NO_BEEP                              0x00
#define BUZZER_SINGLE_SHORT_BEEP                    0x01
#define BUZZER_SINGLE_LONG_BEEP                     0x02
#define BUZZER_WARNING_BEEP                         0x03

// Warning beeps starts after the following duration (in milliseconds)
#define BUZZER_WARNING_FREQUENCY           3200     //  BUZZER frequency
#define BUZZER_WARNING_DUTYPERCENT         5       // in percentage -> control sound volume
#define BUZZER_WARNING_BEEP_PERIOD         30000    //  BUZZER repeat beeps at the following period (in milliseconds) -
                                                    //Values should be in multiples of GPT_TIME
// singleButton beeps
#define BUZZER_MPB_FREQUENCY               4200
#define BUZZER_MPB_DUTYPERCENT             5     // in percentage -> controls sound volume
/* ********************************************************************
 * TYPEDEFS
*/


/* ********************************************************************
* MACROS
*/

/* ********************************************************************
 * API FUNCTIONS
 */
extern void buzzer_init();
extern void buzzer_buzzerStatusRegister(uint8_t *ptrbuzzerStatus);
extern void buzzer_ErrorHandler( void );
extern void buzzer_buttonHandler(uint8_t buttonStatus);


#ifdef _cplusplus
}
#endif

#endif /* APPLICATION_BUZZER_H_ */
