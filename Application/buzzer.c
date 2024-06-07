/*
 * buzzer.c
 *
 *  Created on: 7 May 2024
 *      Author: Chee
 */
/*********************************************************************
* INCLUDES
*/
#include <stdlib.h>
/* Driver Header files */
//#include <ti/drivers/PWM.h>
/* Example/Board Header files */
//#include <Board.h>
#include "UDHAL/UDHAL_PWM.h"

#include "Application/buzzer.h"


/*********************************************************************
* LOCAL VARIABLES
*/

/*********************************************************************
* LOCAL FUNCTIONS
*/
/*********************************************************************
 * @fn      buzzer_Init
 *
 * @brief   It is used to initialize the library
 *
 * @param   none
 *
 * @return  none
 */
//uint8_t   buzzer_pwmOpenStatus = 0;

void buzzer_init()
{

}

/*********************************************************************
 * @fn      buzzer_buttonHandler
 *
 * @brief   buzzer execution when single button is pressed
 *
 * @param   buttonStatus
 *
 * @return  none
 */
uint8_t buzzerStart = 0;
uint8_t buzzer_buttonStatus = 0;
void buzzer_buttonHandler(uint8_t buttonStatus)
{
    /* Start buzzer if Buzzer is OFF */
    buzzer_buttonStatus = buttonStatus; // if buzzer_buttonStatus is not 0, sound the buzzer
    if ((buttonStatus != 0) && (buzzerStart == 0)){
        UDHAL_PWM_setBUZDutyAndPeriod(BUZZER_MPB_DUTYPERCENT, BUZZER_MPB_FREQUENCY);
        buzzerStart = 2;                   // buzzerStart = 2 means buzzer is activated due to falling edge of single button press
    }
    /* Stop buzzer if buzzer is ON */
    if (buttonStatus == 0){
        if (buzzerStart >= 2){
            UDHAL_PWM_setBUZDutyAndPeriod(0, BUZZER_MPB_FREQUENCY);   // set duty to 0
            buzzerStart = 0;
        }
    }

}

/*********************************************************************
 * @fn      buzzer_errorHandler
 *
 * @brief   buzzer execution when system error is present
 *
 * @param   warningDutyPercent, warningPeriod
 *
 * @return  none
 */
void buzzer_errorHandle(uint8_t warningDutyPercent, uint32_t warningPeriod)
{
    UDHAL_PWM_setBUZDutyAndPeriod(warningDutyPercent, warningPeriod);
}
